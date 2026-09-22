"""Build the actual command mapper and run stopping behavior regression."""
import argparse,json,re,subprocess
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--cxx',required=True);p.add_argument('--zig',action='store_true');a=p.parse_args()
r=Path(__file__).resolve().parents[2];b=r/'build/host-command-stopping';b.mkdir(parents=True,exist_ok=True)
db=json.loads((r/'build/h723-debug/compile_commands.json').read_text())
e=next(e for e in db if e['file'].replace('\\','/').endswith('/app/command/command.cpp'))
inc=[x or y for x,y in re.findall(r'-I(?:"([^"]+)"|(\S+))',e['command'])]
src=['tests/host/command_stopping.cpp','app/command/command.cpp','app/robot_debug.cpp']
cmd=[a.cxx]+(['c++'] if a.zig else [])+['-std=c++17']+['-I'+x for x in inc]+[str(r/x) for x in src]+['-o',str(b/'test.exe')]
subprocess.run(cmd,check=True);subprocess.run([str(b/'test.exe')],check=True)
