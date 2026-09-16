"""Build/run production control code on the host; requires configured H723 headers."""
import argparse
import json
from pathlib import Path
import re
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--cxx', default='c++', help='Native C++ compiler, or zig executable')
    parser.add_argument('--zig', action='store_true', help='Use the zig c++ subcommand')
    parser.add_argument('--build-dir', type=Path)
    args = parser.parse_args()
    root = args.root.resolve()
    build = args.build_dir or root / 'build/host-control'
    build.mkdir(parents=True, exist_ok=True)
    database = json.loads((root / 'build/h723-debug/compile_commands.json').read_text())
    entry = next(e for e in database if e['file'].replace('\\', '/').endswith('/app/chassis/chassis.cpp'))
    includes = [a or b for a, b in re.findall(r'-I(?:"([^"]+)"|(\S+))', entry['command'])]
    sources = [
        'tests/host/control_semantics.cpp', 'app/chassis/chassis.cpp',
        'app/chassis/pendulum.cpp', 'app/chassis/vmc.cpp', 'app/chassis/odometry.cpp',
        'app/chassis/lqr.cpp', 'app/command/command.cpp', 'app/robot_debug.cpp',
        'pnx_libs/control/src/pid.cpp', 'pnx_libs/math/src/constrain.cpp',
    ]
    compiler = [args.cxx, 'c++'] if args.zig else [args.cxx]
    exe = build / 'control_semantics.exe'
    log_path = build / 'compile.log'
    with log_path.open('w', encoding='utf-8') as log:
        result = subprocess.run(compiler + ['-std=c++17', '-O0', '-g', '-Wall', '-Wextra'] +
                                [f'-I{p}' for p in includes] + [str(root / s) for s in sources] +
                                ['-o', str(exe)], stdout=log, stderr=subprocess.STDOUT)
    if result.returncode:
        raise SystemExit(f'Compilation failed; see {log_path}')
    subprocess.run([str(exe.resolve())], check=True)


if __name__ == '__main__':
    main()
