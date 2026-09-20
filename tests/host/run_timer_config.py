"""Check timer ownership/timing failures and optional PS2 transport generation."""
import argparse
import copy
import json
from pathlib import Path
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('--cmake', required=True)
parser.add_argument('--build-dir', type=Path, required=True)
args = parser.parse_args()
root = Path(__file__).resolve().parents[2]
board = json.loads((root / 'boards/h723_mc02/board.json').read_text())
params = json.loads((root / 'configs/boards/h723_mc02/params.json').read_text())
ioc = (root / 'boards/h723_mc02/h723_mc02.ioc').read_text()

def check(name, config, settings, hardware, error=None, transport=None):
    directory = args.build_dir / name
    directory.mkdir(parents=True, exist_ok=True)
    (directory / 'board.json').write_text(json.dumps(config))
    (directory / 'params.json').write_text(json.dumps(settings))
    (directory / 'board.ioc').write_text(hardware)
    cmd = [args.cmake, '-DPNX_BOARD_FAMILY=stm32h7',
           f'-DIOC={directory / "board.ioc"}', f'-DBOARD_CONFIG={directory / "board.json"}',
           f'-DPARAMS={directory / "params.json"}', f'-DOUT_DIR={directory / "generated"}',
           '-P', str(root / 'configs/cmake/generate_config.cmake')]
    result = subprocess.run(cmd, capture_output=True, text=True)
    output = result.stdout + result.stderr
    (directory / 'result.log').write_text(output)
    if error:
        assert result.returncode != 0 and error in output, output
    else:
        assert result.returncode == 0, output
        if transport:
            generated = (directory / 'generated/bsp_bindings.hpp').read_text()
            assert f'using ps2_transport = {transport};' in generated

check('async', board, params, ioc, transport='ps2_gpio_async')
check('counter', board, params, ioc.replace('TIM7.Prescaler=260-1', 'TIM7.Prescaler=130-1'),
      error='requires a 1 MHz IOC counter')
check('opm', board, params, ioc.replace('VP_TIM7_VS_OPM.Mode=OPM_bit', 'VP_TIM7_VS_OPM.Mode=disabled'),
      error='needs IOC OPM + IRQ')
check('irq', board, params, ioc.replace('NVIC.TIM7_IRQn=true', 'NVIC.TIM7_IRQn=false'),
      error='needs IOC OPM + IRQ')
check('timebase', board, params, ioc.replace('NVIC.TimeBaseIP=TIM23', 'NVIC.TimeBaseIP=TIM7'),
      error='is the HAL timebase')
conflict = copy.deepcopy(board)
conflict['bindings']['timers']['ps2_clock']['timer'] = 'tim6'
check('owner', conflict, params, ioc, error='already has another owner')
missing = copy.deepcopy(params)
missing['ps2']['timer'] = 'missing'
check('missing', board, missing, ioc, error='is not a board timer role')
legacy = copy.deepcopy(params)
del legacy['ps2']['timer']
check('legacy', board, legacy, ioc, transport='ps2_gpio')
empty = copy.deepcopy(board)
del empty['bindings']['timers']
check('empty', empty, legacy, ioc, transport='ps2_gpio')
print('Timer generation: 3 valid configurations and 6 invalid configurations passed')
