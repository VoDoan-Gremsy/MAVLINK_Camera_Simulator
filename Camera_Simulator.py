import time
import threading
from pymavlink import mavutil
from pymavlink.dialects.v20.ardupilotmega import (
    MAVLink_camera_information_message,
    MAVLink_camera_settings_message
)

class MavlinkCamera:
    def __init__(self, port='com10', baud=115200):
        """Initialize MAVLink camera connection."""
        self.connection = mavutil.mavlink_connection(
            port, baud=baud,
            source_system=1, 
            source_component=100,
            dialect='common'
        )
        self.start_time = time.time()
        self.current_mode_id = 0
        self.running = False

    def get_time_boot_ms(self):
        """Calculate time since program start in milliseconds."""
        return int((time.time() - self.start_time) * 1000)

    def wait_for_heartbeat(self, timeout=10):
        """Wait for initial heartbeat with timeout."""
        print("Waiting for heartbeat...")
        end_time = time.time() + timeout
        
        while time.time() < end_time:
            msg = self.connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg:
                print(f"[RX] Heartbeat received from system {msg.get_srcSystem()}, "
                      f"component {msg.get_srcComponent()}")
                return True
            time.sleep(0.1)
        
        print("No heartbeat received within timeout")
        return False

    def send_camera_info(self):
        """Send camera information message."""
        cam_info = {
            'time_boot_ms': self.get_time_boot_ms(),
            'vendor_name': b"ExampleVendor".ljust(32, b'\0'),
            'model_name': b"ExampleModel".ljust(32, b'\0'),  # Fixed: Changed to Model
            'firmware_version': 1,
            'focal_length': 5.0,
            'sensor_size_h': 0.0,
            'sensor_size_v': 0.0,
            'resolution_h': 1920,
            'resolution_v': 1080,
            'lens_id': 0,
            'flags': 0,
            'cam_definition_version': 1,
            'cam_definition_uri': b"https://raw.githubusercontent.com/mavlink/mavlink-devguide/master/en/services/camera_definition_example.xml".ljust(140, b'\0'),
            'gimbal_device_id': 0
        }
        
        msg = MAVLink_camera_information_message(**cam_info)
        self.connection.mav.send(msg)
        print("[TX] Sent CAMERA_INFORMATION")
    
    def send_camera_settings(self):
        """Send current camera settings."""
        settings = {
            'time_boot_ms': self.get_time_boot_ms(),
            'mode_id': self.current_mode_id,
            'zoomLevel': 0.0,
            'focusLevel': 0.0
        }
        
        msg = MAVLink_camera_settings_message(**settings)
        self.connection.mav.send(msg)
        print("[TX] Sent CAMERA_SETTINGS")

    def send_heartbeat(self):
        """Continuously send heartbeat messages."""
        while self.running:
            self.connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_CAMERA,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, mavutil.mavlink.MAV_STATE_ACTIVE
            )
            print("[TX] Sent Heartbeat")
            time.sleep(1)

    def receive_messages(self):
        """Handle incoming messages (heartbeats and commands)."""
        while self.running:
            # Process all available messages
            while True:
                msg = self.connection.recv_match(blocking=False)
                if not msg:
                    break
                    
                if msg.get_type() == 'HEARTBEAT':
                    print(f"[RX] Heartbeat from system {msg.get_srcSystem()}, "
                          f"component {msg.get_srcComponent()}")
                elif msg.get_type() == 'COMMAND_LONG':
                    self.handle_command(msg)
            
            time.sleep(0.1)

    def handle_command(self, msg):
        """Handle incoming MAVLink commands."""
        if msg.command == mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE:
            if msg.param1 == 259:  # CAMERA_INFORMATION
                print("[RX] Received MAV_CMD_REQUEST_MESSAGE for CAMERA_INFORMATION")
                self.connection.mav.command_ack_send(
                    command=msg.command,
                    result=mavutil.mavlink.MAV_RESULT_ACCEPTED
                )
                print("[TX] Sent COMMAND_ACK (MAV_RESULT_ACCEPTED)")
                self.send_camera_info()
            if msg.param1 == 260:  # CAMERA_SETTINGS
                print("[RX] Received MAV_CMD_REQUEST_MESSAGE for CAMERA_SETTINGS")
                self.connection.mav.command_ack_send(
                    command=msg.command,
                    result=mavutil.mavlink.MAV_RESULT_ACCEPTED
                )
                print("[TX] Sent COMMAND_ACK (MAV_RESULT_ACCEPTED)")
                self.send_camera_settings()

    def run(self):
        """Main execution method."""
        if not self.wait_for_heartbeat():
            print("Exiting due to no heartbeat")
            return

        self.running = True
        
        # Start threads
        tx_thread = threading.Thread(target=self.send_heartbeat, daemon=True)
        rx_thread = threading.Thread(target=self.receive_messages, daemon=True)

        tx_thread.start()
        rx_thread.start()

        try:
            while self.running:
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("Stopping...")
            self.running = False
            tx_thread.join()
            rx_thread.join()

if __name__ == "__main__":
    camera = MavlinkCamera()
    camera.run()