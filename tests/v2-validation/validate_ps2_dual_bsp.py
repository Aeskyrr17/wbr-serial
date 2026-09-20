"""Build one shared Module tree against both BSP families, with an isolated F4 PS2 board."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[2])
parser.add_argument('--f4-bsp', type=Path, required=True)
parser.add_argument('--cmake', required=True)
parser.add_argument('--cubemx', type=Path, required=True)
parser.add_argument('--java', required=True)
parser.add_argument('--prepare-only', action='store_true')
parser.add_argument('--skip-generate', action='store_true')
args = parser.parse_args()
root = args.root.resolve()
out = root / 'build/ps2-dual-bsp'
board = out / 'f4-board'
source = root / 'boards/f407_c_board'
out.mkdir(parents=True, exist_ok=True)

def run(command, log):
    with log.open('w', encoding='utf-8') as stream:
        result = subprocess.run([str(arg) for arg in command], cwd=root, stdout=stream,
                                stderr=subprocess.STDOUT)
    if result.returncode:
        raise RuntimeError(f'{log}\n{log.read_text(encoding="utf-8", errors="replace")[-4000:]}')

def module_hashes():
    return {p.relative_to(root / 'pnx_modules').as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (root / 'pnx_modules').rglob('*')
            if p.is_file() and p.suffix in ('.cpp', '.hpp', '.c', '.h')}

before = module_hashes()
if not args.skip_generate:
    shutil.copytree(source, board, dirs_exist_ok=True)
    # CubeMX CLI emits Src/Inc at the project root. Seed its USER CODE there.
    for directory in ('Src', 'Inc'):
        shutil.copytree(source / 'Core' / directory, board / directory, dirs_exist_ok=True)
    ioc = (source / 'f407_c_board.ioc').read_text(encoding='utf-8')
    ioc = ioc.replace('Mcu.IPNb=21', 'Mcu.IP21=TIM6\nMcu.IPNb=22')
    ioc = ioc.replace('Mcu.PinsNb=52', '''Mcu.Pin52=VP_TIM6_VS_ClockSourceINT
Mcu.Pin53=VP_TIM6_VS_OPM
Mcu.Pin54=PG2
Mcu.Pin55=PG3
Mcu.Pin56=PG4
Mcu.Pin57=PG5
Mcu.PinsNb=58''')
    ioc = ioc.replace('MxCube.Version=6.15.0', 'MxCube.Version=6.17.0')
    ioc = ioc.replace('MxDb.Version=DB.6.0.150', 'MxDb.Version=DB.6.0.170')
    ioc = ioc.replace('ProjectManager.AskForMigrate=true', 'ProjectManager.AskForMigrate=false')
    ioc = re.sub(r'(ProjectManager.functionlistsort=[^\n]+)',
                 r'\1,21-MX_TIM6_Init-TIM6-false-HAL-true', ioc)
    ioc += '''
TIM6.IPParameters=Prescaler,Period
TIM6.Prescaler=84-1
TIM6.Period=65535
VP_TIM6_VS_ClockSourceINT.Mode=Enable_Timer
VP_TIM6_VS_ClockSourceINT.Signal=TIM6_VS_ClockSourceINT
VP_TIM6_VS_OPM.Mode=OPM_bit
VP_TIM6_VS_OPM.Signal=TIM6_VS_OPM
NVIC.TIM6_DAC_IRQn=true\\:14\\:0\\:true\\:false\\:true\\:false\\:true\\:true\\:true
PG2.Locked=true
PG2.Signal=GPIO_Output
PG2.GPIOParameters=GPIO_PuPd,GPIO_Speed,PinState
PG2.GPIO_PuPd=GPIO_NOPULL
PG2.GPIO_Speed=GPIO_SPEED_FREQ_HIGH
PG2.PinState=GPIO_PIN_SET
PG3.Locked=true
PG3.Signal=GPIO_Input
PG3.GPIOParameters=GPIO_PuPd
PG3.GPIO_PuPd=GPIO_PULLUP
PG4.Locked=true
PG4.Signal=GPIO_Output
PG4.GPIOParameters=GPIO_PuPd,GPIO_Speed,PinState
PG4.GPIO_PuPd=GPIO_NOPULL
PG4.GPIO_Speed=GPIO_SPEED_FREQ_HIGH
PG4.PinState=GPIO_PIN_SET
PG5.Locked=true
PG5.Signal=GPIO_Output
PG5.GPIOParameters=GPIO_PuPd,GPIO_Speed,PinState
PG5.GPIO_PuPd=GPIO_NOPULL
PG5.GPIO_Speed=GPIO_SPEED_FREQ_HIGH
PG5.PinState=GPIO_PIN_SET
'''
    (board / 'f407_c_board.ioc').write_text(ioc, encoding='utf-8')
    script = out / 'generate.txt'
    script.write_text(f'config load "{board}/f407_c_board.ioc"\ngenerate code "{board}"\nexit\n')
    if args.prepare_only:
        print(script, flush=True)
        raise SystemExit(0)
    run([args.java, '-jar', args.cubemx, '-q', script], out / 'cubemx.log')

# Use actual CubeMX output, not hand-written peripheral initialization.
for name in ('Src/main.c', 'Src/tim.c', 'Src/gpio.c', 'Src/stm32f4xx_it.c',
             'Inc/main.h', 'Inc/tim.h', 'Inc/stm32f4xx_it.h'):
    shutil.copy2(board / name, board / 'Core' / name)
timer_source = (board / 'Core/Src/tim.c').read_text()
assert 'HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 14, 0)' in timer_source
assert 'htim6.Init.Prescaler = 84-1' in timer_source
assert 'bsp_timer_dispatch(htim)' in (board / 'Core/Src/main.c').read_text()

config = json.loads((source / 'board.json').read_text())
config['bindings']['timers'] = {'ps2_clock': {'timer': 'tim6'}}
(board / 'board.json').write_text(json.dumps(config, indent=2))
params = json.loads((root / 'configs/boards/f407_c_board/params.json').read_text())
params['remoter']['source'] = 'ps2'
params['remoter']['thread_priority'] = 9
params['test']['remoter'] = True
params['test']['usb'] = False
params['ps2'] = dict(backend='gpio', timer='ps2_clock', cmd='pg2', data='pg3', clk='pg4', cs='pg5')
(board / 'params.json').write_text(json.dumps(params, indent=2))
board_cmake = (source / 'board.cmake').read_text()
board_cmake = board_cmake.replace('${CMAKE_CURRENT_LIST_DIR}/../../configs/boards/f407_c_board/params.json',
                                 '${CMAKE_CURRENT_LIST_DIR}/params.json')
board_cmake = board_cmake.replace('${CMAKE_CURRENT_LIST_DIR}/../../configs/boards/f407_c_board/robot.json',
                                 (root / 'configs/boards/f407_c_board/robot.json').as_posix())
(board / 'board.cmake').write_text(board_cmake)

nm = shutil.which('arm-none-eabi-nm')
assert nm, 'arm-none-eabi-nm must be on PATH'
results = []
for family in ('h7', 'f4'):
    bsp = (root / 'pnx_bsp') if family == 'h7' else args.f4_bsp.resolve()
    profile = 'h723_mc02' if family == 'h7' else '../build/ps2-dual-bsp/f4-board'
    for mode in ('Debug', 'Release'):
        build = out / f'{family}-{mode.lower()}'
        run([args.cmake, '-S', root, '-B', build, '-G', 'Ninja', f'-DPNX_BOARD={profile}',
             f'-DPNX_BSP_SOURCE_DIR={bsp}', f'-DCMAKE_BUILD_TYPE={mode}'], out / f'{family}-{mode}-configure.log')
        run([args.cmake, '--build', build, '--parallel', '8'], out / f'{family}-{mode}-build.log')
        database = json.loads((build / 'compile_commands.json').read_text())
        for relative in ('remoter/src/ps2.cpp', 'remoter/src/ps2_gpio_async.cpp'):
            path = (root / 'pnx_modules' / relative).resolve()
            assert any(Path(entry['file']).resolve() == path for entry in database), path
        symbols = subprocess.check_output([nm, '-C', str(build / 'pnx_embedded.elf')], text=True)
        for name in ('remoter::ps2_gpio_async::transfer(', 'bsp::timer::arm(',
                     'TIM7_IRQHandler' if family == 'h7' else 'TIM6_DAC_IRQHandler'):
            assert name in symbols, name
        assert 'remoter::ps2_gpio::transfer(' not in symbols
        (out / f'{family}-{mode}-symbols.txt').write_text(symbols)
        results.append(dict(family=family, build_type=mode, passed=True,
                            board=profile, bsp=str(bsp), modules=str(root / 'pnx_modules')))
        print(f'PASS {family}/{mode}: shared PS2 modules + real IRQ/backend linked', flush=True)
assert before == module_hashes(), 'Shared Module tree changed during verification'
for name in ('bsp_timer.hpp', 'bsp_timer.h'):
    assert (root / 'pnx_bsp/timer/include' / name).read_bytes() == (args.f4_bsp / 'timer/include' / name).read_bytes()
(out / 'results.json').write_text(json.dumps(dict(builds=results, module_hashes=before,
    public_headers_identical=True, hardware_tested=False), indent=2))
print(f'Results: {out}/results.json')
