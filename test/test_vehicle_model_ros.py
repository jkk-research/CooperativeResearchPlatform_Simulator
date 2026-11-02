import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from autoware_control_msgs.msg import Control
from crp_msgs.msg import Ego, PowertrainTelemetry, Scenario


def run_model(executable, scenario_file, model, acceleration_mode=False):
    node = rclpy.create_node('vehicle_model_regression_' + model)
    states = []
    powertrain_states = []
    subscription = node.create_subscription(Ego, '/ego', states.append, 10)
    powertrain_subscription = node.create_subscription(
        PowertrainTelemetry, '/simulator/powertrain_telemetry', powertrain_states.append, 10)
    publisher = node.create_publisher(Control, '/control/command/control_cmd', 10)
    process = subprocess.Popen([
        executable, '--ros-args', '-p', 'scenario_file:=' + scenario_file,
        '-p', 'vehicle_model:=' + model, '-p', 'use_internal_controller:=false',
        '-p', 'use_road_gradient:=false', '-p', 'road_pitch:=0.0',
        '-p', 'initial_speed:=0.0',
        '-p', 'use_scenario_speed_when_uncommanded:=false', '-p', 'command_timeout:=0.25',
        '-p', 'command_mode:=' + ('acceleration' if acceleration_mode else 'speed'),
    ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    try:
        deadline = time.monotonic() + 20.0
        command_end = None
        next_command = 0.0
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError('Simulator exited during ' + model + ' test')
            rclpy.spin_once(node, timeout_sec=0.02)
            current = time.monotonic()
            if len(states) >= 3 and publisher.get_subscription_count() > 0 and command_end is None:
                assert states[-1].twist.twist.linear.x == 0.0, 'Standstill before commands'
                command_end = current + 1.2
            if command_end is not None and current < command_end and current >= next_command:
                command = Control()
                command.longitudinal.velocity = 3.0
                command.longitudinal.acceleration = 3.0
                command.lateral.steering_tire_angle = 0.12
                publisher.publish(command)
                next_command = current + 0.05
            if command_end is not None and current > command_end + 1.2:
                break
        assert command_end is not None, 'No ego/control discovery for ' + model
        peak_speed = max((state.twist.twist.linear.x for state in states), default=0.0)
        assert peak_speed > 2.0, model + ': peak speed was ' + str(peak_speed)
        assert powertrain_states and all(math.isfinite(message.efficiency) and
            math.isfinite(message.electrical_power_w) for message in powertrain_states)
        assert max(message.energy_consumed_j for message in powertrain_states) > 0.0
        torque_steps = [abs(current.motor_torque_nm - previous.motor_torque_nm)
            for previous, current in zip(powertrain_states, powertrain_states[1:])]
        motor_speed_steps = [abs(current.motor_speed_rad_s - previous.motor_speed_rad_s)
            for previous, current in zip(powertrain_states, powertrain_states[1:])]
        assert max(torque_steps, default=0.0) < 120.0, model + ': motor torque changed too sharply'
        if not acceleration_mode:
            peak_speed = max(state.twist.twist.linear.x for state in states)
            assert peak_speed < 3.01, 'Speed feedback does not overshoot the flat-road target'
        assert any(abs(state.twist.twist.angular.z) > 0.01 for state in states)
        assert any(abs(state.accel.accel.linear.y) > 0.01 for state in states)
        assert any(abs(state.accel.accel.angular.z) > 0.01 for state in states)
        assert states[-1].twist.twist.linear.x == 0.0, 'Stale commands brake to rest'
        assert states[-1].twist.twist.linear.y == 0.0, 'No lateral drift at rest'
        assert states[-1].twist.twist.angular.z == 0.0, 'No turn at rest'
        if model == 'dynamic':
            assert any(abs(state.twist.twist.linear.y) > 0.01 for state in states)
        else:
            assert all(state.twist.twist.linear.y == 0.0 for state in states)
        for state in states:
            assert state.twist.twist.linear.x >= 0.0
            for value in (
                    state.pose.pose.position.x, state.pose.pose.position.y,
                    state.pose.pose.orientation.z, state.twist.twist.linear.x,
                    state.twist.twist.linear.y, state.twist.twist.angular.z,
                    state.accel.accel.linear.x, state.accel.accel.linear.y,
                    state.accel.accel.angular.z):
                assert math.isfinite(value)
        print(model + ': live acceleration, steering, lateral/yaw states and watchdog braking passed; '
            'max motor torque step=' + str(max(torque_steps, default=0.0)) + ' Nm, '
            'max motor speed step=' + str(max(motor_speed_steps, default=0.0)) + ' rad/s')
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            output = process.communicate(timeout=10)[0]
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output = process.communicate()[0]
        if process.returncode not in (0, -signal.SIGINT):
            print(output)
        node.destroy_subscription(subscription)
        node.destroy_subscription(powertrain_subscription)
        node.destroy_node()


def run_occlusion(executable, scenario_file):
    node = rclpy.create_node('occlusion_regression')
    scenarios = []
    subscription = node.create_subscription(Scenario, '/scenario', scenarios.append, 10)
    process = subprocess.Popen([
        executable, '--ros-args', '-p', 'scenario_file:=' + scenario_file,
        '-p', 'initial_x:=65.0', '-p', 'initial_y:=0.0', '-p', 'initial_yaw:=0.0',
        '-p', 'use_scenario_speed_when_uncommanded:=false',
    ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    try:
        deadline = time.monotonic() + 15.0
        while len(scenarios) < 5 and time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError('Simulator exited during occlusion test')
            rclpy.spin_once(node, timeout_sec=0.05)
        assert len(scenarios) >= 5, 'No live occlusion scenario stream'
        assert all(message.occluded_regions for message in scenarios), 'Obstacle shadows are published'
        crossing_pedestrians = 0
        for message in scenarios:
            virtual_ids = {bytes(identifier.uuid) for identifier in message.virtual_object_ids}
            found_virtual_ids = set()
            for actor in message.local_moving_objects.objects + message.local_obstacles.objects:
                identifier = bytes(actor.object_id.uuid)
                if identifier not in virtual_ids:
                    assert actor.existence_probability == 1.0, 'Visible detections are certain'
                    continue
                found_virtual_ids.add(identifier)
                branches = actor.kinematics.predicted_paths
                assert len(branches) == 2
                assert abs(sum(branch.confidence for branch in branches) - 1.0) < 1e-6
                assert all(1 < len(branch.path) <= 100 for branch in branches)
                if actor.classification[0].label == 7 and abs(actor.existence_probability - 0.5) < 1e-6:
                    crossing_pedestrians += 1
                    assert abs(branches[0].confidence - 0.9) < 1e-6
            assert virtual_ids == found_virtual_ids, 'Every virtual ID identifies a published actor'
        assert crossing_pedestrians > 0, 'Occluded crossing creates possible walking/running pedestrians'
        print('Live shadows, virtual UUIDs, crossing priors and conditional predictions passed')
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            output = process.communicate(timeout=10)[0]
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output = process.communicate()[0]
        if process.returncode not in (0, -signal.SIGINT):
            print(output)
        node.destroy_subscription(subscription)
        node.destroy_node()


def run_initial_speed_hold(executable, scenario_file):
    node = rclpy.create_node('initial_speed_regression')
    states = []
    subscription = node.create_subscription(Ego, '/ego', states.append, 10)
    publisher = node.create_publisher(Control, '/control/command/control_cmd', 10)
    process = subprocess.Popen([executable, '--ros-args', '-p', 'scenario_file:=' + scenario_file,
        '-p', 'vehicle_model:=dynamic', '-p', 'command_mode:=acceleration'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    try:
        deadline = time.monotonic() + 10.
        while len(states) < 6 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        assert len(states) >= 6
        assert all(abs(state.twist.twist.linear.x - 50 / 3.6) < 1e-9 for state in states)
        assert all(state.pose.pose.position.x == states[0].pose.pose.position.x for state in states)
        initial_x = states[-1].pose.pose.position.x
        deadline = time.monotonic() + .5
        while time.monotonic() < deadline:
            command = Control()
            command.longitudinal.acceleration = 0.
            publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=.02)
        assert abs(states[-1].pose.pose.position.x - initial_x) > 1.
        print('50 km/h initial state is held until the first acceleration command')
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        process.communicate(timeout=10)
        node.destroy_subscription(subscription)
        node.destroy_node()


def main():
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    rclpy.init()
    try:
        if sys.argv[1] == '--occlusion':
            run_occlusion(sys.argv[2], sys.argv[3])
            return
        for model in ('kinematic', 'dynamic'):
            run_model(sys.argv[1], sys.argv[2], model)
        run_model(sys.argv[1], sys.argv[2], 'dynamic', acceleration_mode=True)
        run_initial_speed_hold(sys.argv[1], sys.argv[2])
        invalid = subprocess.run([
            sys.argv[1], '--ros-args', '-p', 'scenario_file:=' + sys.argv[2],
            '-p', 'vehicle_model:=invalid',
        ], capture_output=True, text=True, timeout=10)
        assert invalid.returncode != 0
        assert 'vehicle_model must be kinematic or dynamic' in invalid.stdout + invalid.stderr
        print('Invalid model selection rejected')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()