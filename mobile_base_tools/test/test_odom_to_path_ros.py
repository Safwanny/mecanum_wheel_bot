import os
import time

from mobile_base_tools.odom_to_path import OdomToPath
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Empty


def spin_until(executor, predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not predicate():
        executor.spin_once(timeout_sec=0.05)
    assert predicate()


def test_path_topic_is_latched_and_resettable():
    os.environ['ROS_LOG_DIR'] = '/tmp/mobile_base_tools_test_logs'
    rclpy.init()
    trajectory = OdomToPath()
    test_node = Node('test_odom_to_path')
    executor = SingleThreadedExecutor()
    executor.add_node(trajectory)
    executor.add_node(test_node)
    try:
        odom_publisher = test_node.create_publisher(
            Odometry, '/mobile_base_controller/odometry', 10)
        message = Odometry()
        message.header.frame_id = 'odom'
        message.header.stamp.sec = 1
        message.pose.pose.position.x = 1.25
        odom_publisher.publish(message)
        spin_until(executor, lambda: len(trajectory.accumulator.samples) == 1)

        received = []
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        subscription = test_node.create_subscription(
            Path, '/mobile_base/trajectory', received.append, qos)
        spin_until(executor, lambda: received)
        assert received[-1].header.frame_id == 'odom'
        assert [pose.pose.position.x for pose in received[-1].poses] == [1.25]

        client = test_node.create_client(
            Empty, '/mobile_base/trajectory/reset')
        spin_until(executor, client.service_is_ready)
        future = client.call_async(Empty.Request())
        spin_until(executor, future.done)
        spin_until(executor, lambda: received and not received[-1].poses)
        test_node.destroy_subscription(subscription)
    finally:
        executor.shutdown()
        trajectory.destroy_node()
        test_node.destroy_node()
        rclpy.shutdown()
