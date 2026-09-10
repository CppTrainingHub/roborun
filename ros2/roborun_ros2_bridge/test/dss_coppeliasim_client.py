#!/usr/bin/env python3

import argparse
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


PICK_AND_PLACE = '''SERVO_ON
SET_DO CLAMP_ENABLE ON
WAIT_DI PART_READY ON TIMEOUT 500
MOVEJ PICK SPEED 100 TIMEOUT 1000
SET_TOOL GRIPPER CLOSED TIMEOUT 500
WAIT_DI GRIP_OK ON TIMEOUT 500
MOVEJ PLACE SPEED 100 TIMEOUT 1000
SET_TOOL GRIPPER OPEN TIMEOUT 500
MOVEJ HOME SPEED 100 TIMEOUT 1000
STOP
'''

LONG_MOVE = 'SERVO_ON\nMOVEJ PICK SPEED 1 TIMEOUT 30000\nSTOP\n'


class CoppeliaSimCheckClient(Node):
    def __init__(self):
        super().__init__('roborun_ros2_coppeliasim_check_client')
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
        self.move_feedback_seen = False
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

    def feedback(self, message):
        if message.feedback.active_command_type == 'MOVEJ':
            self.move_feedback_seen = True


def wait_for(node, future, seconds):
    rclpy.spin_until_future_complete(node, future, timeout_sec=seconds)
    if not future.done():
        raise RuntimeError('timed out waiting for a ROS 2 request')
    return future.result()


def wait_until(node, condition, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if condition():
            return
    raise RuntimeError('timed out waiting for mid-motion feedback')


def send_goal(node, text, source):
    goal = ExecuteProgram.Goal()
    goal.program_text = text
    goal.source_name = source
    goal_handle = wait_for(node, node.action.send_goal_async(goal, feedback_callback=node.feedback),
                           10.0)
    if not goal_handle.accepted:
        raise RuntimeError('Bridge rejected a valid Action Goal')
    return goal_handle


def check_topics(node, result):
    wait_until(node, lambda: bool(node.statuses and node.statuses[-1].terminal), 2.0)
    if not node.statuses or not node.io_states or not node.alarm_states or not node.joints:
        raise RuntimeError('Bridge did not publish every required state Topic')
    terminal = node.statuses[-1]
    if not terminal.terminal:
        raise RuntimeError('Runtime status Topic did not publish a terminal state')
    if len(node.joints[-1].name) != 6 or len(node.joints[-1].position) != 6:
        raise RuntimeError('JointState did not contain one synchronized six-joint sample')
    if node.joints[-1].velocity or node.joints[-1].effort:
        raise RuntimeError('JointState fabricated unavailable velocity or effort')
    if terminal.snapshot_sequence != result.final_snapshot_sequence:
        raise RuntimeError('terminal Action result and status Topic use different snapshot sequences')
    return terminal


def run_normal(node):
    goal_handle = send_goal(node, PICK_AND_PLACE, 'ros2-coppeliasim-pick-place.task')
    wrapped = wait_for(node, goal_handle.get_result_async(), 30.0)
    if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not wrapped.result.succeeded:
        raise RuntimeError(f'pick-and-place failed: {wrapped.result.outcome}')
    terminal = check_topics(node, wrapped.result)
    if terminal.outcome != 'program_completed':
        raise RuntimeError(f'unexpected normal outcome: {terminal.outcome}')
    return terminal


def run_cancel(node):
    goal_handle = send_goal(node, LONG_MOVE, 'ros2-coppeliasim-cancel.task')
    wait_until(node, lambda: node.move_feedback_seen, 10.0)
    cancellation = wait_for(node, goal_handle.cancel_goal_async(), 10.0)
    if not cancellation.goals_canceling:
        raise RuntimeError('Bridge rejected mid-motion Action cancellation')
    wrapped = wait_for(node, goal_handle.get_result_async(), 15.0)
    if wrapped.status != GoalStatus.STATUS_CANCELED or wrapped.result.outcome != 'operator_stopped':
        raise RuntimeError(f'Cancel did not stop the active motion: {wrapped.result.outcome}')
    return check_topics(node, wrapped.result)


def run_estop(node):
    goal_handle = send_goal(node, LONG_MOVE, 'ros2-coppeliasim-estop.task')
    wait_until(node, lambda: node.move_feedback_seen, 10.0)
    if not node.control.wait_for_service(timeout_sec=5.0):
        raise RuntimeError('Control service did not become available')
    request = ControlRuntime.Request()
    request.control = 'ESTOP'
    request.source = 'ros2-coppeliasim-check'
    response = wait_for(node, node.control.call_async(request), 5.0)
    if response.status != 'queued' or response.request_id == 0:
        raise RuntimeError(f'ESTOP service was not queued: {response.message}')
    wrapped = wait_for(node, goal_handle.get_result_async(), 15.0)
    if wrapped.status != GoalStatus.STATUS_ABORTED or wrapped.result.outcome != 'emergency_stopped':
        raise RuntimeError(f'ESTOP did not abort the active motion: {wrapped.result.outcome}')
    return check_topics(node, wrapped.result)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=('normal', 'cancel', 'estop'), required=True)
    mode = parser.parse_args().mode
    rclpy.init()
    node = CoppeliaSimCheckClient()
    try:
        if not node.action.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('Bridge Action server did not become available')
        terminal = {'normal': run_normal, 'cancel': run_cancel, 'estop': run_estop}[mode](node)
        print(f'RESULT status=passed phase=ros2_coppeliasim mode={mode} '
              f'snapshot_sequence={terminal.snapshot_sequence}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print(f'ERROR {error}', file=sys.stderr)
        sys.exit(1)
