#!/usr/bin/env python
import unittest
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached, ProxyActionClient, ProxyServiceCaller

from std_msgs.msg import String
from std_srvs.srv import Trigger
from flexbe_msgs.action import BehaviorExecution

class TestProxies(unittest.TestCase):
    @classmethod
    def setUp(self):
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.executor = MultiThreadedExecutor(context=self.context)
        self.node = rclpy.create_node('TestProxies', context=self.context)

        # rclpy.spin(self.node, self.executor)

    @classmethod
    def tearDown(self):
        # self.node.destroy_node()
        # self.executor.shutdown()
        rclpy.shutdown(context=self.context)

    def test_publish_subscribe(self):
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=5)
        ProxyPublisher._initialize(self.node)
        ProxySubscriberCached._initialize(self.node)

        t1 = '/pubsub_1'
        t2 = '/pubsub_2'
        pub = ProxyPublisher({t1: String})
        pub = ProxyPublisher({t2: String})
        sub = ProxySubscriberCached({t1: String})

        self.assertTrue(pub.is_available(t1))

        self.assertTrue(pub.wait_for_any(t1))
        # self.assertFalse(pub.wait_for_any(t2))

        msg1 = String()
        msg1.data = '1'
        msg2 = String()
        msg2.data = '2'

        pub.publish(t1, msg1)
        pub.publish(t2, msg2)

        # time.sleep(0.5) # make sure latched message is sent before subscriber is added
        # sub = ProxySubscriberCached({t2: String})
        sub.subscribe(t2, String)
        time.sleep(0.5) # make sure latched message can be received before checking

        self.assertTrue(sub.has_msg(t1))
        self.assertEqual(sub.get_last_msg(t1).data, '1')
        sub.remove_last_msg(t1)

        self.assertFalse(sub.has_msg(t1))
        self.assertIsNone(sub.get_last_msg(t1))

        self.assertTrue(sub.has_msg(t2))
        self.assertEqual(sub.get_last_msg(t2).data, '2')

    def test_subscribe_buffer(self):
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=5)
        ProxyPublisher._initialize(self.node)
        ProxySubscriberCached._initialize(self.node)

        t1 = '/buffered_1'
        pub = ProxyPublisher({t1: String})
        sub = ProxySubscriberCached({t1: String})
        sub.enable_buffer(t1)
        self.assertTrue(pub.wait_for_any(t1))

        msg1 = String()
        msg1.data = '1'
        msg2 = String()
        msg2.data = '2'

        pub.publish(t1, msg1)
        pub.publish(t1, msg2)
        time.sleep(0.5) # make sure messages can be received

        self.assertTrue(sub.has_msg(t1))
        self.assertTrue(sub.has_buffered(t1))
        self.assertEqual(sub.get_from_buffer(t1).data, '1')

        pub.publish(t1, String('3'))
        time.sleep(0.5) # make sure messages can be received

        self.assertEqual(sub.get_from_buffer(t1).data, '2')
        self.assertEqual(sub.get_from_buffer(t1).data, '3')
        self.assertIsNone(sub.get_from_buffer(t1))
        self.assertFalse(sub.has_buffered(t1))

    def test_service_caller(self):
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=5)
        ProxyServiceCaller._initialize(self.node)

        t1 = '/service_1'

        def server_callback(request, response):
            self.node.get_logger().info("Starting service callback")
            response.success = True
            response.message = "ok"

            self.node.get_logger().info("Sent service message")
            return response

        self.node.create_service(Trigger, t1, server_callback)

        srv = ProxyServiceCaller({t1: Trigger})

        result = srv.call(t1, Trigger.Request())

        self.node.get_logger().info("Called service request")

        self.assertIsNotNone(result)
        self.assertTrue(result.success)
        self.assertEqual(result.message, 'ok')

        self.assertFalse(srv.is_available('/not_there'))
        srv = ProxyServiceCaller({'/invalid': Trigger}, wait_duration=.1)
        self.assertFalse(srv.is_available('/invalid'))

    def test_action_client(self):
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=5)
        t1 = '/action_1'
        server = None

        def execute_cb(goal_handle):
            time.sleep(0.1)
            if server.is_cancel_requested:
                # server.set_preempted()
                goal_handle.canceled()
            else:
                goal_handle.succeed()
                # server.set_succeeded(BehaviorExecution.Result(outcome='ok'))

        # server = actionlib.SimpleActionServer(t1, BehaviorExecution, execute_cb, auto_start=False)
        server = ActionServer(self.node, BehaviorExecution, t1, execute_cb)
        # server.start()

        ProxyActionClient._initialize(self.node)
        client = ProxyActionClient({t1: BehaviorExecution})
        self.assertFalse(client.has_result(t1))
        client.send_goal(t1, BehaviorExecution.Goal())

        rate = self.node.create_rate(20, self.node.get_clock())
        for i in range(20):
            self.assertTrue(client.is_active(t1) or client.has_result(t1))
            # rate.sleep()
        self.assertTrue(client.has_result(t1))

        result = client.get_result(t1)
        self.assertEqual(result.outcome, 'ok')

        client.send_goal(t1, BehaviorExecution.Goal())
        time.sleep(0.1)

        client.cancel(t1)
        time.sleep(0.3)

        self.assertFalse(client.is_active(t1))

        self.assertFalse(client.is_available('/not_there'))
        client = ProxyActionClient({'/invalid': BehaviorExecution}, wait_duration=.1)
        self.assertFalse(client.is_available('/invalid'))


if __name__ == '__main__':
    # rospy.init_node('test_flexbe_proxies')
    # import rostest
    # rostest.rosrun('flexbe_core', 'test_flexbe_proxies', TestProxies)
    unittest.main()
