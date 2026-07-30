"""Launch-logic tests for fail-fast controller sequencing."""

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

from launch import LaunchDescription
from launch.actions import EmitEvent, ExecuteProcess
from launch.events import Shutdown
import launch_testing


_SIMULATION_LAUNCH = (
    Path(__file__).resolve().parents[1] / 'launch' / 'simulation.launch.py'
)
_SPEC = importlib.util.spec_from_file_location(
    'ackermann_simulation_launch',
    _SIMULATION_LAUNCH,
)
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)


def generate_test_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                sys.executable,
                '-c',
                'import time; time.sleep(1.0)',
            ],
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestControllerSequence(unittest.TestCase):

    def _assert_failure_shutdown(self, label):
        marker = object()
        callback = _MODULE.next_only_on_success(marker, label)
        actions = callback(SimpleNamespace(returncode=23), None)
        self.assertEqual(len(actions), 2)
        self.assertIsInstance(actions[0], _MODULE.LogError)
        self.assertIsInstance(actions[1], EmitEvent)
        self.assertIsInstance(actions[1].event, Shutdown)
        self.assertIn(label, actions[1].event.reason)
        self.assertNotIn(marker, actions)

    def test_normal_spawn_continues_to_jsb(self):
        jsb = object()
        callback = _MODULE.next_only_on_success(jsb, 'robot spawn')
        self.assertEqual(
            callback(SimpleNamespace(returncode=0), None),
            [jsb],
        )

    def test_normal_jsb_continues_to_ackermann_controller(self):
        ackermann = object()
        callback = _MODULE.next_only_on_success(
            ackermann,
            'joint_state_broadcaster spawner',
        )
        self.assertEqual(
            callback(SimpleNamespace(returncode=0), None),
            [ackermann],
        )

    def test_normal_final_spawner_exits_without_followup(self):
        callback = _MODULE.next_only_on_success(
            None,
            'ackermann_steering_controller spawner',
        )
        self.assertEqual(callback(SimpleNamespace(returncode=0), None), [])

    def test_spawn_failure_injection_shuts_down(self):
        self._assert_failure_shutdown('robot spawn')

    def test_jsb_failure_injection_shuts_down(self):
        self._assert_failure_shutdown('joint_state_broadcaster spawner')

    def test_ackermann_spawner_failure_injection_shuts_down(self):
        self._assert_failure_shutdown(
            'ackermann_steering_controller spawner'
        )

    def test_simulation_consumes_generated_runtime_files(self):
        source = _SIMULATION_LAUNCH.read_text(encoding='utf-8')
        self.assertIn("'config', 'controllers.yaml'", source)
        self.assertIn("'config', 'command_guard.yaml'", source)
        self.assertIn(
            "'package://ackermann_car_description/meshes'",
            source,
        )
        self.assertIn("'--param-file'", source)
        self.assertIn("'use_sim_time': True", source)
        self.assertIn("package='ackermann_command_guard'", source)

    def test_mapping_and_localization_each_include_one_simulation(self):
        launch_dir = _SIMULATION_LAUNCH.parent
        mapping = (launch_dir / 'slam.launch.py').read_text(encoding='utf-8')
        localization = (
            launch_dir / 'localization.launch.py'
        ).read_text(encoding='utf-8')
        self.assertEqual(mapping.count("'simulation.launch.py'"), 1)
        self.assertEqual(localization.count("'simulation.launch.py'"), 1)
        self.assertIn("'online_async_launch.py'", mapping)
        self.assertIn("'map_file_name'", localization)
