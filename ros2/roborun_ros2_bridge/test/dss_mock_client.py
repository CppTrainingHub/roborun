#!/usr/bin/env python3

import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from roborun_interfaces.action import ExecuteProgram
from roborun_interfaces.msg import AlarmState, IoState, RuntimeStatus
from roborun_interfaces.srv import ControlRuntime


class MockCheckClient(Node):
    def __init__(self):
        super().__init__('roborun_ros2_mock_check_client')
        authoritative_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.action = ActionClient(self, ExecuteProgram, 'execute_program')
        self.control = self.create_client(ControlRuntime, 'control_runtime')
        self.statuses = []
        self.io_states = []
        self.alarm_states = []
        self.joints = []
        self.create_subscription(RuntimeStatus, 'runtime_status', self.statuses.append,
                                 authoritative_qos)
        self.create_subscription(IoState, 'io_state', self.io_states.append, authoritative_qos)
        self.create_subscription(AlarmState, 'alarm_state', self.alarm_states.append,
                                 authoritative_qos)
        sensor_qos = QoSProfile(
            depth=5,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(JointState, '/joint_states', self.joints.append, sensor_qos)


def wait_for(node, future, seconds):
    rclpy.spin_until_future_complete(node, future, timeout_sec=seconds)
    if not future.done():
        raise RuntimeError('timed out waiting for the ROS 2 Action')
    return future.result()


def submit_control(node, control):
    if not node.control.wait_for_service(timeout_sec=5.0):
        raise RuntimeError('Control service did not become available')
    request = ControlRuntime.Request()
    request.control = control
    request.source = 'ros2-docker-dds'
    response = wait_for(node, node.control.call_async(request), 5.0)
    if response.status != 'queued':
        raise RuntimeError(f'{control} service was not queued: {response.message}')


def wait_for_lifecycle(node, lifecycle, after_sequence):
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if (node.statuses and node.statuses[-1].snapshot_sequence > after_sequence and
                node.statuses[-1].lifecycle == lifecycle):
            return node.statuses[-1]
    latest = (f'{node.statuses[-1].lifecycle}@{node.statuses[-1].snapshot_sequence}'
              if node.statuses else 'no status')
    raise RuntimeError(f'Runtime did not reach lifecycle {lifecycle}; latest={latest}')


def run_goal(node, goal):
    goal_handle = wait_for(node, node.action.send_goal_async(goal), 10.0)
    if not goal_handle.accepted:
        raise RuntimeError('Bridge rejected a valid Action Goal')
    wrapped_result = wait_for(node, goal_handle.get_result_async(), 10.0)
    if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
        raise RuntimeError(f'Action did not succeed: status={wrapped_result.status}')
    if not wrapped_result.result.succeeded:
        raise RuntimeError(f'Runtime result was not successful: {wrapped_result.result.outcome}')
    return wrapped_result


def check_parse_rejection(node):
    if not node.statuses:
        raise RuntimeError('No Runtime status before malformed Goal')
    before = node.statuses[-1]
    goal = ExecuteProgram.Goal()
    goal.program_text = 'NOT_A_COMMAND\n'
    goal.source_name = 'parse-rejection.task'
    handle = wait_for(node, node.action.send_goal_async(goal), 5.0)
    if not handle.accepted:
        raise RuntimeError('Nonempty malformed task should reach the parser')
    wrapped = wait_for(node, handle.get_result_async(), 5.0)
    result = wrapped.result
    if wrapped.status != GoalStatus.STATUS_ABORTED or result.succeeded:
        raise RuntimeError('Malformed program did not abort its Action')
    if result.outcome != 'rejected' or not result.diagnostic_code:
        raise RuntimeError('Parse rejection is missing its outcome or diagnostic')
    if (result.lifecycle != before.lifecycle or
            result.final_snapshot_sequence != before.snapshot_sequence):
        raise RuntimeError('Parse rejection fabricated a Runtime state transition')
    deadline = time.monotonic() + 0.3
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    after = node.statuses[-1]
    if (after.lifecycle != result.lifecycle or
            after.snapshot_sequence != result.final_snapshot_sequence):
        raise RuntimeError('Rejected Action and Runtime status Topic disagree')
    print(f'PARSE_REJECTION status=passed lifecycle={result.lifecycle}')


def main():
    rclpy.init()
    node = MockCheckClient()
    try:
        if not node.action.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('Bridge Action server did not become available')
        discovery_deadline = time.monotonic() + 1.0
        while time.monotonic() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        check_parse_rejection(node)
        goal = ExecuteProgram.Goal()
        goal.program_text = 'SERVO_ON\nMOVEJ 0 0 0 0 0 0\nSERVO_OFF\nSTOP\n'
        goal.source_name = 'ros2-docker-dds.task'
        wrapped_result = run_goal(node, goal)

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.statuses and node.statuses[-1].terminal:
                break
        if not node.statuses or not node.io_states or not node.alarm_states or not node.joints:
            raise RuntimeError('Bridge did not publish every required state Topic')
        terminal = node.statuses[-1]
        if not terminal.terminal or terminal.outcome != 'program_completed':
            raise RuntimeError('Runtime status Topic did not publish the terminal Runtime outcome')
        if len(node.joints[-1].name) != 6 or len(node.joints[-1].position) != 6:
            raise RuntimeError('JointState did not contain one synchronized six-joint sample')
        if node.joints[-1].velocity or node.joints[-1].effort:
            raise RuntimeError('JointState fabricated unavailable velocity or effort')
        if terminal.snapshot_sequence != wrapped_result.result.final_snapshot_sequence:
            raise RuntimeError('terminal Action result and status Topic use different snapshot sequences')
        check_parse_rejection(node)
        submit_control(node, 'ESTOP')
        emergency = wait_for_lifecycle(node, 'emergency_stopped', terminal.snapshot_sequence)
        check_parse_rejection(node)
        submit_control(node, 'RESET')
        idle = wait_for_lifecycle(node, 'idle', emergency.snapshot_sequence)
        resumed_result = run_goal(node, goal)
        if resumed_result.result.final_snapshot_sequence <= idle.snapshot_sequence:
            raise RuntimeError('resubmitted Action did not publish a newer terminal snapshot')
        print('RESULT status=passed phase=ros2_mock_dds snapshot_sequence='
              f'{resumed_result.result.final_snapshot_sequence}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print(f'ERROR {error}', file=sys.stderr)
        sys.exit(1)
