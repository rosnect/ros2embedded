import pathlib
import subprocess
import unittest
from unittest.mock import Mock, patch

import yaml
from rclpy.node import Node
from bridge_manager.manager_node import BridgeManager, node_name, validate_identity


class ManagerTest(unittest.TestCase):
    def setUp(self):
        self.manager = BridgeManager.__new__(BridgeManager)
        self.manager._bridges = {}
        self.manager.get_logger = lambda: Mock()
        self.spawn_patch = patch('bridge_manager.manager_node.subprocess.Popen')
        self.spawn = self.spawn_patch.start()
        self.prefix_patch = patch('bridge_manager.manager_node.get_package_prefix', return_value='/tmp/test-prefix')
        self.prefix_patch.start()
        self.addCleanup(self.spawn_patch.stop)
        self.addCleanup(self.prefix_patch.stop)
        self.addCleanup(self.cleanup_bridges)

    def cleanup_bridges(self):
        for key in list(self.manager._bridges):
            self.manager.stop_bridge(key)

    def start(self, process):
        self.spawn.return_value = process
        self.manager.start_bridge('phone_1', 'mqtt', 7, [('config_file', '/tmp/example.json'),
            ('mqtt_password', 'secret'), ('domain_id', 99)])

    def test_identity_and_name_isolation(self):
        for value in [-1, 0x100000000]:
            with self.assertRaises(ValueError):
                validate_identity('phone', 'mqtt', value)
        with self.assertRaises(ValueError):
            validate_identity('phone', 'redis', 7)
        self.assertNotEqual(node_name('a-b', 7), node_name('a_2db', 7))
        self.assertNotEqual(node_name('phone', 7), node_name('phone', 8))

    def test_direct_child_private_parameters_and_cleanup(self):
        process = Mock()
        process.poll.return_value = None
        self.start(process)
        args = self.spawn.call_args.args[0]
        self.assertEqual(args[0], '/tmp/test-prefix/lib/bridge_node/bridge_node')
        self.assertNotIn('secret', ' '.join(args))
        path = pathlib.Path(self.manager._bridges['phone_1'].params_file)
        self.assertEqual(path.stat().st_mode & 0o777, 0o600)
        params = yaml.safe_load(path.read_text())['/**']['ros__parameters']
        self.assertEqual(params['domain_id'], 7)
        self.assertEqual(params['mqtt_password'], 'secret')
        self.assertEqual(params['payload_encoding'], 'json')
        with patch.object(Node, 'destroy_node', return_value=True):
            self.manager.destroy_node()
        process.terminate.assert_called_once()
        process.wait.assert_called_once_with(timeout=3)
        self.assertFalse(path.exists())

    def test_running_duplicate_and_exited_restart(self):
        first = Mock()
        first.poll.return_value = None
        self.start(first)
        first_file = pathlib.Path(self.manager._bridges['phone_1'].params_file)
        self.start(first)
        self.assertEqual(self.spawn.call_count, 1)
        first.poll.return_value = 1
        second = Mock()
        second.poll.return_value = None
        self.start(second)
        self.assertEqual(self.spawn.call_count, 2)
        self.assertFalse(first_file.exists())
        first.wait.assert_called_once_with()

    def test_forced_shutdown_after_timeout(self):
        process = Mock()
        process.poll.return_value = None
        process.wait.side_effect = [subprocess.TimeoutExpired('bridge', 3), 0]
        self.start(process)
        self.manager.stop_bridge('phone_1')
        process.kill.assert_called_once()
        self.assertFalse(self.manager._bridges)

    def test_mqtt_device_configuration_needs_no_local_file(self):
        process = Mock()
        process.poll.return_value = None
        self.spawn.return_value = process
        self.manager.start_bridge('phone_1', 'mqtt', 7, [])
        path = pathlib.Path(self.manager._bridges['phone_1'].params_file)
        params = yaml.safe_load(path.read_text())['/**']['ros__parameters']
        self.assertEqual(params['config_source'], 'device')
        self.assertNotIn('config_file', params)

    def test_serial_still_requires_file_and_device_mode_cannot_mix_file(self):
        with self.assertRaises(ValueError):
            self.manager.start_bridge('phone_1', 'serial', 7, [])
        with self.assertRaises(ValueError):
            self.manager.start_bridge('phone_1', 'mqtt', 7,
                [('config_file', '/tmp/example.json'), ('config_source', 'device')])
        self.spawn.assert_not_called()

    def test_spawn_failure_removes_parameter_file(self):
        self.spawn.side_effect = OSError('spawn failed')
        with self.assertRaises(OSError):
            self.manager.start_bridge('phone_1', 'mqtt', 7, [('config_file', '/tmp/example.json')])
        path = self.spawn.call_args.args[0][3]
        self.assertFalse(pathlib.Path(path).exists())


if __name__ == '__main__':
    unittest.main()
