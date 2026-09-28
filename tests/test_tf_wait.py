"""Sensor messages wait for TF at their stamp instead of being dropped."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'njord_sim'))
from njord_sim.tf_wait import TfWaitQueue


class TfWaitQueueTests(unittest.TestCase):
    def test_message_waits_for_its_transform_and_keeps_order(self):
        queue, processed, tf_until = TfWaitQueue(0.5), [], [0.0]

        def process(stamp):  # TF exists up to tf_until[0]
            if stamp > tf_until[0]:
                return False
            processed.append(stamp)
            return True
        for stamp in (10.0, 10.1, 10.2):
            queue.push(stamp, stamp)
        self.assertEqual(queue.release(10.25, process), 0)  # no TF yet: all wait
        tf_until[0] = 10.1
        self.assertEqual(queue.release(10.25, process), 2)
        self.assertEqual(processed, [10.0, 10.1])
        tf_until[0] = 10.3
        queue.release(10.3, process)
        self.assertEqual(processed, [10.0, 10.1, 10.2])

    def test_old_future_and_out_of_order_messages_are_dropped(self):
        queue, processed = TfWaitQueue(0.5), []

        def process(stamp):  # TF always available
            processed.append(stamp)
            return True
        queue.push(10.0, 10.0)
        self.assertFalse(queue.push(9.9, 9.9))  # older than the newest queued
        queue.push(10.4, 10.4)
        queue.release(10.6, process)  # 10.0 is 0.6 s old: dropped, 10.4 processed
        self.assertEqual(processed, [10.4])
        queue.push(20.0, 20.0)
        queue.release(5.0, process)  # clock went backwards: the future message goes
        self.assertEqual(processed, [10.4])
        self.assertFalse(queue.items)

    def test_capacity_drops_the_oldest(self):
        queue = TfWaitQueue(10.0, capacity=2)
        for stamp in (1.0, 2.0, 3.0):
            queue.push(stamp, stamp)
        self.assertEqual([s for s, _ in queue.items], [2.0, 3.0])
        with self.assertRaises(ValueError):
            TfWaitQueue(0.0)


if __name__ == '__main__':
    unittest.main()
