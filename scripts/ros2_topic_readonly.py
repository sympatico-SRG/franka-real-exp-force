"""Bounded read-only CLI with a narrowly scoped discovery warning filter."""
import os
import re
import subprocess
import sys

args = sys.argv[1:]
raw = '--raw-warnings' in args
args = [arg for arg in args if arg != '--raw-warnings']
if len(args) < 2 or args[0] not in ('info', 'echo') or not args[1].startswith('/'):
    sys.exit('Usage: ros2_topic_readonly.sh info|echo /TOPIC [ros2 options] [--raw-warnings]')
if '--no-daemon' not in args:
    args.append('--no-daemon')
if args[0] == 'echo' and '--once' not in args:
    args.append('--once')
ros2 = f"/opt/ros/{os.environ['ROS_DISTRO']}/bin/ros2"
pattern = re.compile(r"^\[WARN\]\s+\[[^\]]+\]\s+\[rmw_cyclonedds_cpp\]: Failed to parse type hash for topic .* from USER_DATA '\(null\)'\.\s*$")
if not raw:
    print('Filtering missing type-hash discovery warnings only; metadata remains unchanged. Use --raw-warnings for originals.', file=sys.stderr, flush=True)
# stdout is inherited so YAML and topic info remain untouched. Only stderr is filtered.
process = subprocess.Popen(
    ['timeout', '--signal=INT', '--kill-after=2s', '12s', sys.executable, ros2, 'topic', *args],
    stderr=subprocess.PIPE, text=True,
)
try:
    for line in process.stderr:
        if raw or not pattern.match(line):
            print(line, end='', file=sys.stderr, flush=True)
    result = process.wait()
except KeyboardInterrupt:
    process.wait()
    result = 130
if result == 124:
    print('Timed out after 12 seconds; receipt was not verified. No robot was started.', file=sys.stderr)
sys.exit(result)
