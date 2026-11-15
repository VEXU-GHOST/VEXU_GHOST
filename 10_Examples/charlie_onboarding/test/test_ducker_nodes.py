import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import rclpy
from std_msgs.msg import String


PUBLISHER_TOPIC = '/ducker_publisher_test'
SUBSCRIBER_TOPIC = '/ducker_subscriber_test'


def generate_test_description():
    talker = launch_ros.actions.Node(
        package='charlie_onboarding',
        executable='talker',
        name='ducker_test_talker',
        remappings=[('topic', PUBLISHER_TOPIC)],
        output='screen',
    )
    listener = launch_ros.actions.Node(
        package='charlie_onboarding',
        executable='listener',
        name='ducker_test_listener',
        remappings=[('topic', SUBSCRIBER_TOPIC)],
        output='screen',
    )
    return (
        launch.LaunchDescription([
            talker,
            listener,
            launch.actions.TimerAction(
                period=1.0,
                actions=[launch_testing.actions.ReadyToTest()],
            ),
        ]),
        {},
    )


class TestDuckerNodes(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('ducker_test_client')

    def tearDown(self):
        self.node.destroy_node()

    def spin_until(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        return predicate()

    def test_publisher_emits_growing_artwork(self):
        messages = []
        self.node.create_subscription(
            String, PUBLISHER_TOPIC, lambda msg: messages.append(msg.data), 10
        )
        self.assertTrue(self.spin_until(lambda: len(messages) >= 2))

        segment = '\n⠀⠸⣿⣧⡀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⠃⠀⠀'
        first = messages[0]
        legs = first.find('\n⠀⠀⠛⢿⣿⣿⣿⣿⣇')
        self.assertNotEqual(legs, -1)
        self.assertEqual(messages[1], first[:legs] + segment + segment + first[legs:])

    def test_subscriber_logs_messages(self, proc_output):
        publisher = self.node.create_publisher(String, SUBSCRIBER_TOPIC, 10)
        self.assertTrue(
            self.spin_until(lambda: publisher.get_subscription_count() > 0)
        )

        for index in range(3):
            message = String()
            message.data = f'test message {index}'
            publisher.publish(message)
            proc_output.assertWaitFor(
                f"I heard: '{message.data}'",
                timeout=5,
            )
