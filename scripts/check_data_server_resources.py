"""Check server resources without opening cameras or starting ROS nodes."""
import argparse
import socket
import subprocess


CAMERA_PROCESS_MARKERS = (
    '/lib/franka_realsense_camera_publisher/realsense_camera_publisher',
    'ros2 launch franka_realsense_camera_publisher cameras.launch.py',
)


def find_camera_owners(process_table: str) -> list[str]:
    return [
        line.strip()
        for line in process_table.splitlines()
        if any(marker in line for marker in CAMERA_PROCESS_MARKERS)
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--port', required=True, type=int)
    parser.add_argument('--cache-port', required=True, type=int)
    parser.add_argument('--hand-port', type=int, default=0)
    args = parser.parse_args()
    endpoints = [('sample', args.host, args.port), ('camera cache', '127.0.0.1', args.cache_port)]
    if args.hand_port:
        endpoints.append(('hand telemetry', args.host, args.hand_port))
    failed = False
    sockets = []
    try:
        for name, host, port in endpoints:
            if not 1 <= port <= 65535:
                print(f'Error: invalid {name} port {port}', flush=True)
                failed = True
                continue
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sockets.append(sock)
            try:
                sock.bind((host, port))
                print(f'Ready: {name} endpoint {host}:{port} is available', flush=True)
            except OSError as error:
                print(f'Error: {name} endpoint {host}:{port} unavailable: {error}', flush=True)
                failed = True
    finally:
        for sock in sockets:
            sock.close()
    # Starting another publisher risks opening devices already used by another workspace.
    # Report the owner; never stop it or silently switch camera configuration.
    processes = subprocess.check_output(['ps', '-eo', 'pid=,args='], text=True)
    owners = find_camera_owners(processes)
    if owners:
        print('Error: existing RealSense launch/publisher processes detected; do not start duplicate camera owners:', flush=True)
        for owner in owners:
            print(f'  {owner}', flush=True)
        print('Arrange camera ownership with the existing task before starting this full camera stack.', flush=True)
        failed = True
    return 1 if failed else 0


if __name__ == '__main__':
    raise SystemExit(main())
