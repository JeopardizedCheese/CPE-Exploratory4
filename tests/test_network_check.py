"""Only loopback sockets: these checks never contact a physical robot."""
import contextlib
import io
import json
import socket
import threading
import unittest
from unittest.mock import patch

import network_check


@contextlib.contextmanager
def responder(valid=True):
    packets, stopped = [], threading.Event()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1', 0))
        sock.settimeout(.05)
        def serve():
            while not stopped.is_set():
                try:
                    data, addr = sock.recvfrom(2048)
                except socket.timeout:
                    continue
                p = json.loads(data)
                packets.append(p)
                sock.sendto(b'not-json', addr)
                sock.sendto(b'[]', addr)
                sock.sendto(json.dumps({'state': 'IDLE', 'session': 'other-session'}).encode(), addr)
                if valid:
                    sock.sendto(json.dumps({'state': 'IDLE', 'session': p['s'], 'rssi': -55}).encode(), addr)
        thread = threading.Thread(target=serve)
        thread.start()
        try:
            yield sock.getsockname()[1], packets
        finally:
            stopped.set()
            thread.join(timeout=1)


class NetworkCheckTests(unittest.TestCase):
    def test_receives_status_while_sending_only_ping(self):
        with responder() as (port, packets):
            result = network_check.probe('127.0.0.1', port, .35)
        self.assertGreaterEqual(result['replies'], 2)
        self.assertEqual(result['status']['state'], 'IDLE')
        self.assertEqual({p['c'] for p in packets}, {'ping'})
        self.assertTrue(all(set(p) == {'v', 's', 'q', 'c'} for p in packets))
        self.assertEqual([p['q'] for p in packets], list(range(1, len(packets)+1)))

    def test_invalid_and_wrong_session_responses_do_not_confirm_connection(self):
        with responder(valid=False) as (port, _):
            result = network_check.probe('127.0.0.1', port, .15)
        self.assertEqual(result['replies'], 0)
        self.assertIsNone(result['status'])

    def test_network_check_reports_clamped_close_configuration(self):
        result = {'replies': 3, 'ip': '127.0.0.1', 'port': 4211, 'last_status_age_s': .01,
                  'max_status_gap_s': .2, 'status': {'state': 'IDLE', 'grip_close_deg': 70,
                                                  'servo_min_deg': 0, 'servo_max_deg': 55}}
        with patch('sys.argv', ['network_check.py', '127.0.0.1']), patch.object(network_check, 'probe', return_value=result), contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(SystemExit, 'Gripper configuration mismatch'):
                network_check.main()

    def test_invalid_destination_and_duration_open_no_socket(self):
        with patch.object(network_check.socket, 'socket') as sock:
            for ip, seconds in [('255.255.255.255', 1), ('0.0.0.0', 1), ('224.0.0.1', 1),
                                ('127.0.0.1', float('nan')), ('127.0.0.1', 0)]:
                with self.assertRaises(ValueError):
                    network_check.probe(ip, seconds=seconds)
            sock.assert_not_called()


if __name__ == '__main__':
    unittest.main()
