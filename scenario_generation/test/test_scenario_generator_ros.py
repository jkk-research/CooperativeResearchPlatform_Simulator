import math
import os
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from crp_msgs.msg import Ego, Scenario, ScenarioVisualization
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


def test_heading_fixtures(binary):
    from pathlib import Path
    import numpy as np
    from scipy.io import savemat

    route = np.array([[0., 0., 0.], [50., 0., 0.]])
    lanes = np.empty((1, 1), dtype=object)
    lanes[0, 0] = route
    data = {'time': np.array([0., 0.05, 0.1]), 'speedLimit': 8.0,
            'targetPath_qg': route, 'drivableSurface_qg': route, 'lanes': lanes}
    data['objects'] = {'position': {'x': np.array([5., 6., 7.]), 'y': np.zeros(3), 'z': np.zeros(3)},
                       'velocity': 1.0, 'size': np.array([4.5, 1.8, 1.5])}
    obstacles = np.empty((1, 3), dtype=[('position', object), ('size', object), ('heading', object)])
    for index in range(3):
        obstacles[0, index]['position'] = {'x': np.full(3, 10.0 + index * 10),
                                          'y': np.full(3, 2.0 + index * 2), 'z': np.zeros(3)}
        obstacles[0, index]['size'] = np.array([10., 2., 1.5])
    obstacles[0, 0]['heading'] = np.pi / 2
    obstacles[0, 2]['heading'] = np.empty(0)
    data['obstacles'] = obstacles
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / 'heading_fixture.mat'
        for shape in ((1, 3), (3, 1)):
            obstacles[0, 1]['heading'] = np.array([0., np.pi / 4, np.pi / 2]).reshape(shape)
            savemat(path, data)
            subprocess.run([binary, '--headings', str(path)], check=True)
        for invalid in (np.array([0., 1.]), np.ones((2, 2)), np.nan, np.inf, 'invalid'):
            obstacles[0, 1]['heading'] = invalid
            savemat(path, data)
            result = subprocess.run([binary, '--load', str(path)], capture_output=True, text=True)
            assert result.returncode != 0, f'Invalid heading accepted: {invalid}'
    print('Scalar/vector headings, legacy fallback, ego transforms, predictions and malformed inputs passed')


def main():
    os.environ['ROS_DOMAIN_ID'] = str(200 + os.getpid() % 30)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    rclpy.init()
    node = rclpy.create_node('scenario_generator_alignment_test')
    transforms, dynamic_transforms, scenarios, visualizations = [], [], [], []
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        node.create_subscription(TFMessage, '/tf_static', transforms.append, qos),
        node.create_subscription(TFMessage, '/tf', dynamic_transforms.append, 10),
        node.create_subscription(Scenario, '/scenario', scenarios.append, 10),
        node.create_subscription(ScenarioVisualization, '/scenario_visualization', visualizations.append, 10),
    ]
    ego_publisher = node.create_publisher(Ego, '/ego', 10)
    process = subprocess.Popen([
        sys.argv[1], '--ros-args', '-p', 'scenario_file:=' + sys.argv[2],
        '-p', 'initial_x:=5.0', '-p', 'initial_y:=10.0', '-p', 'initial_yaw:=0.0',
    ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    try:
        yaw = math.pi / 2.0
        ego = Ego()
        ego.pose.pose.position.x = 100.0
        ego.pose.pose.position.y = 200.0
        ego.pose.pose.orientation.z = math.sin(yaw / 2.0)
        ego.pose.pose.orientation.w = math.cos(yaw / 2.0)
        deadline = time.monotonic() + 10.0
        index = 0
        alignment = None
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError('Scenario generator exited before producing output: ' + process.communicate()[0])
            ego.header.stamp.sec = index // 20
            ego.header.stamp.nanosec = (index % 20) * 50_000_000
            ego_publisher.publish(ego)
            rclpy.spin_once(node, timeout_sec=0.02)
            for message in transforms:
                alignment = next((transform for transform in message.transforms
                    if transform.header.frame_id == 'map' and transform.child_frame_id == 'map_local'), alignment)
            if alignment is not None and dynamic_transforms and scenarios and visualizations:
                break
            index += 1
        assert alignment is not None, 'No fixed map to map_local transform was published'
        assert abs(alignment.transform.translation.x - 110.0) < 1e-8
        assert abs(alignment.transform.translation.y - 195.0) < 1e-8
        actual_yaw = math.atan2(2.0 * alignment.transform.rotation.w * alignment.transform.rotation.z,
                                1.0 - 2.0 * alignment.transform.rotation.z**2)
        assert abs(actual_yaw - yaw) < 1e-8
        ego_transform = next(transform for message in dynamic_transforms for transform in message.transforms
            if transform.header.frame_id == 'map' and transform.child_frame_id == 'base_link')
        assert abs(ego_transform.transform.translation.x - 100.0) < 1e-8
        assert abs(ego_transform.transform.translation.y - 200.0) < 1e-8
        assert scenarios[-1].header.frame_id == 'base_link'
        assert visualizations[-1].header.frame_id == 'base_link'
        assert visualizations[-1].scenario.header.frame_id == 'base_link'
        print('Global ego initialization, static planar alignment and scenario output passed')
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
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_publisher(ego_publisher)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) == 3 and sys.argv[1] == '--headings':
        test_heading_fixtures(sys.argv[2])
    else:
        main()