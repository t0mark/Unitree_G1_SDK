import os
import wave
import rclpy
from rclpy.node import Node
from yolo_msgs.msg import DetectionArray
from std_msgs.msg import String
from unitree_api.msg import Request
from unitree_hg.msg import SportModeState
import json
import time
from pydub import AudioSegment
from ament_index_python.packages import get_package_share_directory

# API IDs for ROS2 communication
ROBOT_API_ID_ARM_ACTION_EXECUTE_ACTION = 7106
ROBOT_API_ID_AUDIO_START_PLAY = 1003
ROBOT_API_ID_AUDIO_STOP_PLAY = 1004
ROBOT_API_ID_AUDIO_SET_VOLUME = 1006

# Action IDs
ACTION_HIGH_WAVE = 26  # high wave action id


class PersonEvent(Node):
    def __init__(self):
        super().__init__('person_event')

        # Parameters
        self.declare_parameter('detection_duration', 2.0)  # 사람이 몇 초 이상 감지되어야 인사할지
        self.declare_parameter('greeting_count', -1)  # -1이면 무한, 양수면 그 횟수만큼만
        self.declare_parameter('greeting_interval', 5.0)  # 인사 후 다음 인사까지 최소 대기 시간
        self.declare_parameter('greeting_sound', 'hello.mp3')  # 인사 음성 파일
        self.declare_parameter('audio_volume', 80)  # 음성 볼륨 (0-100)

        self.detection_duration = self.get_parameter('detection_duration').value
        self.greeting_count = self.get_parameter('greeting_count').value
        self.greeting_interval = self.get_parameter('greeting_interval').value
        self.greeting_sound = self.get_parameter('greeting_sound').value
        self.audio_volume = self.get_parameter('audio_volume').value

        # Sounds folder path
        package_share = get_package_share_directory('person_event')
        self.sounds_folder = os.path.join(package_share, 'sounds')

        # ROS2 Publishers for robot API
        self.arm_request_pub = self.create_publisher(Request, '/api/arm/request', 10)
        self.voice_request_pub = self.create_publisher(Request, '/api/voice/request', 10)

        # Detection subscriber
        self.sub = self.create_subscription(
            DetectionArray,
            '/yolo/tracking',
            self.cb,
            10
        )
        self.arm_state_sub = self.create_subscription(
            String,
            '/arm/action/state',
            self.arm_state_cb,
            10
        )

        # FSM state subscriber
        self.sport_mode_sub = self.create_subscription(
            SportModeState,
            '/sportmodestate',
            self.sport_mode_cb,
            10
        )

        # State tracking
        self.arm_state = None
        self.fsm_id = None
        self.person_detected_time = None
        self.greeting_executed_count = 0
        self.last_greeting_time = None
        self.greeting_in_progress = False
        self.stream_counter = 0
        self.app_name = 'person_event'

        # Set initial volume
        self._set_volume(self.audio_volume)

        self.get_logger().info(f'Person event node initialized (ROS2 mode)')
        self.get_logger().info(f'Detection duration: {self.detection_duration}s')
        self.get_logger().info(f'Greeting count: {self.greeting_count} (-1 = unlimited)')
        self.get_logger().info(f'Greeting interval: {self.greeting_interval}s')
        self.get_logger().info(f'Greeting sound: {self.greeting_sound}')
        self.get_logger().info(f'Audio volume: {self.audio_volume}')

    def _generate_request_id(self):
        """Generate unique request ID."""
        return int(time.time_ns())

    def _create_request(self, api_id, parameter='', binary=None):
        """Create a Request message."""
        req = Request()
        req.header.identity.id = self._generate_request_id()
        req.header.identity.api_id = api_id
        req.parameter = parameter
        if binary is not None:
            req.binary = list(binary)
        return req

    def sport_mode_cb(self, msg: SportModeState):
        """Handle sport mode state updates."""
        self.fsm_id = msg.fsm_id

    def arm_state_cb(self, msg: String):
        """Arm action state callback"""
        try:
            self.arm_state = int(msg.data)
        except:
            self.arm_state = None

    def _set_volume(self, volume: int):
        """Set the robot's audio volume (0-100)."""
        volume = max(0, min(100, volume))
        param = json.dumps({'volume': volume})
        req = self._create_request(ROBOT_API_ID_AUDIO_SET_VOLUME, param)
        self.voice_request_pub.publish(req)
        self.get_logger().info(f'Volume set to {volume}')

    def _load_audio_as_pcm(self, filepath: str):
        """Load audio file and convert to PCM data."""
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
            # Convert to 16kHz mono for robot speaker
            audio = audio.set_frame_rate(16000).set_channels(1)
            channels = audio.channels
            sample_width = audio.sample_width
            framerate = audio.frame_rate
            pcm_data = audio.raw_data
            return pcm_data, channels, sample_width, framerate

        else:
            return None

    def play_greeting_sound(self):
        """Play greeting sound file."""
        filename = self.greeting_sound

        # Check if filename has supported extension
        if not filename.lower().endswith(('.wav', '.mp3')):
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
            result = self._load_audio_as_pcm(filepath)
            if result is None:
                self.get_logger().error(f'Unsupported audio format: {filepath}')
                return False

            pcm_data, channels, sample_width, framerate = result

            self.get_logger().info(
                f'Playing {filename}: {channels}ch, {sample_width*8}bit, {framerate}Hz'
            )

            self.stream_counter += 1
            stream_id = f'stream_{self.stream_counter}'

            param = json.dumps({
                'app_name': self.app_name,
                'stream_id': stream_id
            })
            req = self._create_request(ROBOT_API_ID_AUDIO_START_PLAY, param, pcm_data)
            self.voice_request_pub.publish(req)
            self.get_logger().info(f'Started playing: {filename}')
            return True

        except Exception as e:
            self.get_logger().error(f'Error playing audio file: {e}')
            return False

    def check_robot_fsm_state(self):
        """Check robot FSM state (should be 801)"""
        if self.fsm_id is None:
            self.get_logger().warn('FSM state not yet received')
            return False
        return self.fsm_id == 801

    def execute_arm_action(self, action_id):
        """Execute arm action via ROS2."""
        param = json.dumps({'data': action_id})
        req = self._create_request(ROBOT_API_ID_ARM_ACTION_EXECUTE_ACTION, param)
        self.arm_request_pub.publish(req)
        self.get_logger().info(f'Arm action {action_id} sent')

    def execute_greeting(self):
        """Execute hand wave greeting"""
        if self.greeting_in_progress:
            return

        # Check if greeting count limit reached
        if self.greeting_count >= 0 and self.greeting_executed_count >= self.greeting_count:
            self.get_logger().info(f'Greeting count limit reached ({self.greeting_count})')
            return

        # Check arm state (should be 0) - only if state is available
        if self.arm_state is not None and self.arm_state != 0:
            self.get_logger().warn(f'Arm state is {self.arm_state}, expected 0. Skipping greeting.')
            return

        # Check robot FSM state (should be 801)
        if not self.check_robot_fsm_state():
            self.get_logger().warn('Robot FSM state is not 801. Skipping greeting.')
            return

        self.greeting_in_progress = True
        try:
            self.get_logger().info('Executing hand wave greeting with voice...')

            # Play greeting sound and arm action together
            self.play_greeting_sound()
            self.execute_arm_action(ACTION_HIGH_WAVE)

            self.greeting_executed_count += 1
            self.last_greeting_time = time.time()
            self.get_logger().info(f'Greeting executed ({self.greeting_executed_count}/{self.greeting_count if self.greeting_count >= 0 else "unlimited"})')
        except Exception as e:
            self.get_logger().error(f'Failed to execute greeting: {e}')
        finally:
            self.greeting_in_progress = False

    def cb(self, msg: DetectionArray):
        """Detection callback - tracking_node already filters for person"""
        current_time = time.time()

        if len(msg.detections) > 0:
            # Person detected
            if self.person_detected_time is None:
                self.person_detected_time = current_time
                self.get_logger().info('Person detected, tracking duration...')

            # Check if detection duration threshold met
            detection_duration = current_time - self.person_detected_time
            if detection_duration >= self.detection_duration:
                # Check if enough time passed since last greeting
                if self.last_greeting_time is None or (current_time - self.last_greeting_time) >= self.greeting_interval:
                    self.execute_greeting()
                    self.person_detected_time = None  # Reset detection time after greeting
        else:
            # No person detected, reset timer
            if self.person_detected_time is not None:
                self.get_logger().info('Person no longer detected, resetting timer')
            self.person_detected_time = None

def main():
    rclpy.init()
    node = PersonEvent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
