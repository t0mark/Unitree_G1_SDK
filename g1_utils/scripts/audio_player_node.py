#!/usr/bin/env python3
"""
Audio Player Node for Unitree G1 Robot
This node loads audio files from the sounds folder and plays them through the robot's speaker.
"""

import os
import sys
import wave
import io
import rclpy
from pydub import AudioSegment
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

# Add unitree SDK to path
sdk_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..',
                        'unitree_sdk', 'unitree_sdk2_python')
sys.path.insert(0, os.path.abspath(sdk_path))

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient


class AudioPlayerNode(Node):
    """ROS2 Node for playing audio files on Unitree G1 robot."""

    def __init__(self):
        super().__init__('audio_player_node')

        # Declare parameters
        self.declare_parameter('network_interface', 'eth0')
        self.declare_parameter('sounds_folder', '')
        self.declare_parameter('default_volume', 50)
        self.declare_parameter('app_name', 'audio_player')
        self.declare_parameter('sound_file', '')

        # Get parameters
        self.network_interface = self.get_parameter('network_interface').get_parameter_value().string_value
        sounds_folder_param = self.get_parameter('sounds_folder').get_parameter_value().string_value
        self.default_volume = self.get_parameter('default_volume').get_parameter_value().integer_value
        self.app_name = self.get_parameter('app_name').get_parameter_value().string_value
        self.sound_file = self.get_parameter('sound_file').get_parameter_value().string_value

        # Set sounds folder path
        if sounds_folder_param:
            self.sounds_folder = sounds_folder_param
        else:
            package_share = get_package_share_directory('g1_utils')
            self.sounds_folder = os.path.join(package_share, 'sounds')

        self.get_logger().info(f'Sounds folder: {self.sounds_folder}')
        self.get_logger().info(f'Network interface: {self.network_interface}')

        # Initialize Unitree SDK channel
        self._init_unitree_sdk()

        # Initialize audio client
        self.audio_client = AudioClient()
        self.audio_client.SetTimeout(3.0)
        self.audio_client.Init()

        # Set default volume
        self._set_volume(self.default_volume)

        # Stream counter for unique stream IDs
        self.stream_counter = 0

        # Create ROS2 interfaces
        self._create_ros_interfaces()

        self.get_logger().info('Audio Player Node initialized successfully')

        # Play startup sound if specified
        if self.sound_file:
            self.get_logger().info(f'Playing startup sound: {self.sound_file}')
            self.play_audio_file(self.sound_file)

    def _init_unitree_sdk(self):
        """Initialize Unitree SDK communication channel."""
        ChannelFactoryInitialize(0, self.network_interface)

    def _create_ros_interfaces(self):
        """Create ROS2 subscribers, publishers, and services."""
        # Subscriber to play audio by filename
        self.play_audio_sub = self.create_subscription(
            String,
            'play_audio',
            self._play_audio_callback,
            10
        )

        # Service to list available sounds
        self.list_sounds_srv = self.create_service(
            Trigger,
            'list_sounds',
            self._list_sounds_callback
        )

        # Service to stop audio playback
        self.stop_audio_srv = self.create_service(
            Trigger,
            'stop_audio',
            self._stop_audio_callback
        )

        # Publisher for playback status
        self.status_pub = self.create_publisher(String, 'audio_status', 10)

    def _play_audio_callback(self, msg: String):
        """Callback to play audio file by filename."""
        filename = msg.data
        self.get_logger().info(f'Received request to play: {filename}')

        success = self.play_audio_file(filename)

        status_msg = String()
        if success:
            status_msg.data = f'Playing: {filename}'
        else:
            status_msg.data = f'Failed to play: {filename}'
        self.status_pub.publish(status_msg)

    def _list_sounds_callback(self, request, response):
        """Service callback to list available sound files."""
        try:
            if os.path.exists(self.sounds_folder):
                files = [f for f in os.listdir(self.sounds_folder)
                        if f.lower().endswith(('.wav', '.mp3'))]
                response.success = True
                response.message = ', '.join(files) if files else 'No sound files found'
            else:
                response.success = False
                response.message = f'Sounds folder not found: {self.sounds_folder}'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _stop_audio_callback(self, request, response):
        """Service callback to stop audio playback."""
        try:
            self.audio_client.PlayStop(self.app_name)
            response.success = True
            response.message = 'Audio playback stopped'
            self.get_logger().info('Audio playback stopped')
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'Failed to stop audio: {e}')
        return response

    def _set_volume(self, volume: int):
        """Set the robot's audio volume (0-100)."""
        volume = max(0, min(100, volume))
        code = self.audio_client.SetVolume(volume)
        if code == 0:
            self.get_logger().info(f'Volume set to {volume}')
        else:
            self.get_logger().warning(f'Failed to set volume, error code: {code}')
        return code == 0

    def _load_audio_as_pcm(self, filepath: str):
        """
        Load audio file and convert to PCM data.
        Supports WAV and MP3 formats.

        Returns:
            tuple: (pcm_data, channels, sample_width, framerate) or None if failed
        """
        ext = os.path.splitext(filepath)[1].lower()

        if ext == '.wav':
            with wave.open(filepath, 'rb') as wav_file:
                channels = wav_file.getnchannels()
                sample_width = wav_file.getsampwidth()
                framerate = wav_file.getframerate()
                n_frames = wav_file.getnframes()
                pcm_data = wav_file.readframes(n_frames)
                return pcm_data, channels, sample_width, framerate

        elif ext == '.mp3':
            audio = AudioSegment.from_mp3(filepath)
            channels = audio.channels
            sample_width = audio.sample_width
            framerate = audio.frame_rate
            pcm_data = audio.raw_data
            return pcm_data, channels, sample_width, framerate

        else:
            return None

    def play_audio_file(self, filename: str) -> bool:
        """
        Play an audio file from the sounds folder.

        Args:
            filename: Name of the audio file (supports .wav and .mp3)

        Returns:
            True if playback started successfully, False otherwise
        """
        # Check if filename has supported extension
        if not filename.lower().endswith(('.wav', '.mp3')):
            # Try to find file with supported extension
            for ext in ['.wav', '.mp3']:
                test_path = os.path.join(self.sounds_folder, filename + ext)
                if os.path.exists(test_path):
                    filename = filename + ext
                    break
            else:
                filename = filename + '.wav'

        filepath = os.path.join(self.sounds_folder, filename)

        if not os.path.exists(filepath):
            self.get_logger().error(f'Audio file not found: {filepath}')
            return False

        try:
            # Load audio file and convert to PCM
            result = self._load_audio_as_pcm(filepath)
            if result is None:
                self.get_logger().error(f'Unsupported audio format: {filepath}')
                return False

            pcm_data, channels, sample_width, framerate = result

            self.get_logger().info(
                f'Playing {filename}: {channels}ch, {sample_width*8}bit, {framerate}Hz'
            )

            # Generate unique stream ID
            self.stream_counter += 1
            stream_id = f'stream_{self.stream_counter}'

            # Play through Unitree audio client
            code = self.audio_client.PlayStream(self.app_name, stream_id, pcm_data)

            if code == 0:
                self.get_logger().info(f'Successfully started playing: {filename}')
                return True
            else:
                self.get_logger().error(f'Failed to play audio, error code: {code}')
                return False

        except Exception as e:
            self.get_logger().error(f'Error playing audio file: {e}')
            return False

    def destroy_node(self):
        """Clean up resources when node is destroyed."""
        try:
            self.audio_client.PlayStop(self.app_name)
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = AudioPlayerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
