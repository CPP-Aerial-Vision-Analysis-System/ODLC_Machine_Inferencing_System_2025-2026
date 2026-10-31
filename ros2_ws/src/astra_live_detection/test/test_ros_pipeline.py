"""Real ROS pub/sub and service smoke test; requires the actual SAHI model stack.

Run in the built/sourced workspace with ASTRA_TEST_MODEL=/absolute/model.pt.
No mocked ROS objects or fake detections are used.
"""
import os
import time
import unittest

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_srvs.srv import SetBool, Trigger
    from vision_msgs.msg import Detection2DArray
    AVAILABLE = True
except ImportError:
    AVAILABLE = False


@unittest.skipUnless(AVAILABLE and os.environ.get('ASTRA_TEST_MODEL'),
                     'requires ROS, SAHI/YOLO and ASTRA_TEST_MODEL')
class RosPipelineTest(unittest.TestCase):
    def test_real_inference_header_results_and_services(self):
        from astra_live_detection.live_node import LiveDetection
        rclpy.init(args=['--ros-args', '-p',
                        'model_path:=' + os.environ['ASTRA_TEST_MODEL'], '-p', 'device:=cpu'])
        detector = peer = executor = None
        try:
            detector = LiveDetection()
            peer = Node('astra_pipeline_smoke')
            executor = SingleThreadedExecutor()
            executor.add_node(peer)
            executor.add_node(detector)
            results, images = [], []
            subscriptions = [
                peer.create_subscription(Detection2DArray, 'detection/results', results.append, 1),
                peer.create_subscription(Image, 'detection/annotated', images.append, 1)]
            publisher = peer.create_publisher(Image, 'zr10/image_raw', 1)
            enable = peer.create_client(SetBool, 'detection/set_enabled')
            stats = peer.create_client(Trigger, 'detection/get_statistics')
            health = peer.create_client(Trigger, 'detection/get_health')

            def spin_until(predicate, timeout=60):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    if predicate():
                        return
                    executor.spin_once(timeout_sec=.05)
                self.fail('timed out waiting for ROS graph or inference')

            def call(client, request):
                self.assertTrue(client.wait_for_service(timeout_sec=5))
                future = client.call_async(request)
                spin_until(future.done, 5)
                return future.result()

            spin_until(lambda: publisher.get_subscription_count() > 0, 10)
            message = Image()
            message.header.frame_id = 'smoke_camera'
            message.header.stamp.sec = 123
            message.header.stamp.nanosec = 456
            message.width = message.height = 640
            message.encoding, message.step = 'rgb8', 1920
            message.data = bytes(640 * 640 * 3)
            publisher.publish(message)
            spin_until(lambda: results and images)
            self.assertEqual(results[-1].header, message.header)
            self.assertEqual(images[-1].header, message.header)
            self.assertEqual(images[-1].encoding, 'bgr8')
            self.assertTrue(call(stats, Trigger.Request()).success)
            # Send fresh input before the health request (CPU inference can be slow).
            publisher.publish(message)
            spin_until(lambda: detector.last_received is not None and
                       time.monotonic() - detector.last_received < 1, 5)
            self.assertTrue(call(health, Trigger.Request()).success)
            request = SetBool.Request()
            request.data = False
            self.assertTrue(call(enable, request).success)
            # Drain anything published before the pause response.
            deadline = time.monotonic() + .3
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.02)
            count = len(results)
            message.header.stamp.sec = 124
            publisher.publish(message)
            deadline = time.monotonic() + .3
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.02)
            self.assertEqual(len(results), count)
            request.data = True
            self.assertTrue(call(enable, request).success)
            publisher.publish(message)
            spin_until(lambda: any(result.header.stamp.sec == 124 for result in results))
        finally:
            if detector is not None:
                detector.worker.close()
            if executor is not None:
                executor.shutdown()
            for node in (detector, peer):
                if node is not None:
                    node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
