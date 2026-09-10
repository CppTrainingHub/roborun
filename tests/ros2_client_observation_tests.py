"""Exercise the DDS client's wait logic without requiring ROS on the test host."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest


class TerminalTopicsTest(unittest.TestCase):
    def run_wait(self, events, seconds=1.0):
        path = Path(__file__).resolve().parents[1] / 'ros2/roborun_ros2_bridge/test/dss_mock_client.py'
        tree = ast.parse(path.read_text())
        function = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
                        and n.name == 'wait_for_terminal_topics')
        node = SimpleNamespace(statuses=[], io_states=[], alarm_states=[], joints=[])
        elapsed = [0.0]
        queue = iter(events)

        def spin_once(current, timeout_sec):
            elapsed[0] += timeout_sec
            event = next(queue, None)
            if event is not None:
                field, sequence = event
                getattr(current, field).append(SimpleNamespace(
                    terminal=True, snapshot_sequence=sequence))

        namespace = {
            'time': SimpleNamespace(monotonic=lambda: elapsed[0]),
            'rclpy': SimpleNamespace(spin_once=spin_once),
        }
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), 'exec'), namespace)
        result = namespace['wait_for_terminal_topics'](node, 9, seconds)
        return node, result

    def test_terminal_status_can_arrive_before_other_topics(self):
        node, result = self.run_wait([
            ('statuses', 9), ('io_states', 9), ('alarm_states', 9), ('joints', 0)])
        self.assertEqual(result.snapshot_sequence, 9)
        self.assertEqual(len(node.joints), 1)

    def test_old_authoritative_samples_do_not_complete_the_wait(self):
        node, _ = self.run_wait([
            ('io_states', 8), ('alarm_states', 8), ('joints', 0),
            ('statuses', 9), ('io_states', 9), ('alarm_states', 9)])
        self.assertEqual(node.io_states[-1].snapshot_sequence, 9)
        self.assertEqual(node.alarm_states[-1].snapshot_sequence, 9)

    def test_missing_topic_times_out(self):
        with self.assertRaises(RuntimeError):
            self.run_wait([('statuses', 9), ('io_states', 9), ('alarm_states', 9)])


if __name__ == '__main__':
    unittest.main()
