import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class CmdVelToArduino(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_arduino')
        self.subscription = self.create_subscription(
            Twist,
            # '/diff_cont/cmd_vel_unstamped',
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Arduino와의 시리얼 연결 설정
        # self.serial_port = serial.Serial("/dev/ttyUSB2", 115200, timeout=1.0)
        # self.serial_port.open()
        # self.ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        # self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        time.sleep(2)
        
        try:
            self.ser.open()
        except serial.SerialException as e:
            self.get_logger().error("Connect Failed.")
            return
        
        if self.ser.is_open:
            self.get_logger().info("Connected!")
        else:
            return

    def cmd_vel_callback(self, msg):
        # 선형 및 각속도를 문자열 형식으로 변환
        linear = msg.linear.x
        angular = msg.angular.z
        serial_command = f'{linear} {angular}\n'

        if self.ser.is_open:
            # Arduino로 명령어 전송
            self.ser.write(serial_command.encode('utf-8'))
            self.get_logger().info(f'Sent: {serial_command}')
        else:
            self.get_logger().error('Failed to get log ...')        

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToArduino()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
