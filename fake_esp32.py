"""Read-only UDP diagnostic receiver. Never controls hardware."""
import json
import socket
import time


def main():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1', 4210))
        sock.settimeout(.3)
        print('Listening on localhost:4210. Ctrl+C exits.')
        while True:
            try:
                raw, address = sock.recvfrom(2048)
                packet = json.loads(raw)
                print(time.strftime('%H:%M:%S'), address, packet)
            except socket.timeout:
                print('No packet: target expired')
            except (ValueError, UnicodeError):
                print('Invalid packet')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
