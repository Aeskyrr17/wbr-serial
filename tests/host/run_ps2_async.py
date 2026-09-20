"""Compile the production PS2 sequencer against deterministic GPIO/timer/RTOS fakes."""
import argparse
from pathlib import Path
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('--cxx', required=True)
parser.add_argument('--zig', action='store_true')
parser.add_argument('--build-dir', type=Path, required=True)
args = parser.parse_args()
root = Path(__file__).resolve().parents[2]
args.build_dir.mkdir(parents=True, exist_ok=True)
exe = args.build_dir / 'ps2_async.exe'
cmd = [args.cxx] + (['c++'] if args.zig else [])
cmd += ['-std=c++17', '-Wall', '-Wextra', '-Werror', '-O0', '-g']
for include in ['tests/host/ps2_async_stubs', 'pnx_modules/remoter/include',
                'pnx_bsp/timer/include', 'pnx_libs/common/include']:
    cmd += ['-I', str(root / include)]
cmd += [str(root / 'tests/host/ps2_async.cpp'),
        str(root / 'pnx_modules/remoter/src/ps2_gpio_async.cpp'), '-o', str(exe)]
subprocess.run(cmd, check=True)
subprocess.run([str(exe)], check=True)
