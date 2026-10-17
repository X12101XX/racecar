"""Run in the Humble container: native rclpy and generated messages, no stubs."""
import sys
sys.path.insert(0,'/workspace/ros2_ws/src/racecar_mission')
import math
import unittest
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, Imu, LaserScan
from nav_msgs.msg import Odometry
from racecar_mission.gateway import Gateway
from racecar_mission.mission_node import MissionNode
from racecar_mission.ros_io import image_to_rgb

class NativeContract(unittest.TestCase):
    def setUp(self):rclpy.init(args=['--ros-args','-p','config_dir:=/workspace/config','-p','tts:=none'])
    def tearDown(self):rclpy.shutdown()
    def test_unarmed_gateway_never_advertises_chassis(self):
        n=Gateway()
        try:
            self.assertIsNone(n.pub)
            for _ in range(3):n.tick()
            self.assertEqual(n.count_publishers('/car_cmd_vel'),0)
            self.assertEqual(n.count_publishers('/cmd_vel'),0)
        finally:n.destroy_node()
    def test_default_mission_has_no_command_publisher(self):
        n=MissionNode()
        try:
            self.assertFalse(n.publish_cmd);self.assertIsNone(n.io.cmd_pub)
            n._on_control()
            self.assertEqual(n.count_publishers('/car_cmd_vel'),0)
            self.assertEqual(n.count_publishers('/crossfire/desired_cmd'),0)
        finally:n.destroy_node()
    def test_stale_and_invalid_sensors_do_not_refresh(self):
        n=Gateway()
        try:
            m=LaserScan();m.angle_increment=.01;m.range_min=.1;m.range_max=12.;m.ranges=[2.]*720
            n.on_scan(m);self.assertNotIn('scan',n.last)
            m.header.stamp=n.get_clock().now().to_msg();m.ranges=[float('nan')]*720
            n.on_scan(m);self.assertNotIn('scan',n.last)
            m.ranges=[2.]*720;n.on_scan(m);self.assertIn('scan',n.last)
            i=Imu();i.header.stamp=n.get_clock().now().to_msg();i.angular_velocity.z=float('nan')
            n.on_sensor('imu',i);self.assertNotIn('imu',n.last)
            t=Twist();t.linear.x=float('inf');n.on_cmd(t);self.assertTrue(n.estop)
        finally:n.destroy_node()
    def test_real_image_with_padded_row(self):
        m=Image();m.width=2;m.height=2;m.encoding='rgb8';m.step=8
        m.data=bytes([1,2,3,4,5,6,99,99,7,8,9,10,11,12,99,99])
        a=image_to_rgb(m)
        self.assertIsNotNone(a)
        self.assertEqual(a.tolist(),[[[1,2,3],[4,5,6]],[[7,8,9],[10,11,12]]])

if __name__=='__main__':unittest.main(verbosity=2)
