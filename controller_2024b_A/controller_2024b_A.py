import rclpy
from rclpy.node import Node
from drobo_interfaces.msg import PointDrive, DiffDrive

import pigpio
import struct

I2C_ADDR = 0x14
WRITE_DATA_SIZE = 16

class Controller2024b_A(Node):
    def __init__(self):
        self.is_subscribed = False
        self.pi = pigpio.pi()
        self.i2c_handle = self.pi.bb_i2c_open(10, 11, 100000)
        
        super().__init__('controller_2024b_A')
        self.point_2_2_sub = self.create_subscription(
            PointDrive,
            '/point_2_2',
            self.point_2_2_callback,
            10
        )
        self.point_2_2_msg = PointDrive()
        self.diff_2_2_sub = self.create_subscription(
            DiffDrive,
            '/diff_2_2',
            self.diff_2_2_callback,
            10
        )
        self.diff_2_2_msg = DiffDrive()
        self.timer = self.create_timer(0.01, self.i2c_send)

    def i2c_send(self):
        packet = struct.pack(
            "BBBBBHhhbbbbhhhBB",
            0x04,
            I2C_ADDR,
            0x02,
            0x07,
            WRITE_DATA_SIZE,
            2,
            self.point_2_2_msg.md0,
            self.point_2_2_msg.md1,
            self.point_2_2_msg.md2,
            self.point_2_2_msg.md3,
            self.point_2_2_msg.md4,
            self.point_2_2_msg.md5,
            self.diff_2_2_msg.left,
            self.diff_2_2_msg.right,
            0,
            0x03,
            0x00
        )
        self.pi.bb_i2c_zip(10, packet)
    
    def point_2_2_callback(self, msg):
        self.point_2_2_msg = msg
        self.is_subscribed = True

    def diff_2_2_callback(self, msg):
        self.diff_2_2_msg = msg
        self.is_subscribed = True
        
def main(args=None):
    rclpy.init(args=args)
    controller_2024b_A = Controller2024b_A()
    rclpy.spin(controller_2024b_A)
    controller_2024b_A.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
