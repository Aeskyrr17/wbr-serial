"""Compile pre-change coordinate VMC and current VMC, compare the identical saved poses."""
import argparse,csv,json,subprocess
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--cxx',required=True);p.add_argument('--zig',action='store_true')
p.add_argument('--old-rev',default='HEAD');p.add_argument('--samples',type=Path,default=Path('build/spring_samples.csv'))
p.add_argument('--out',type=Path,default=Path('build/spring-force-comparison'))
a=p.parse_args();root=Path(__file__).resolve().parents[2];out=a.out.resolve();out.mkdir(parents=True,exist_ok=True)
rev=subprocess.check_output(['git','rev-parse',a.old_rev],cwd=root,text=True).strip()
old=subprocess.check_output(['git','show',rev+':app/chassis/vmc.cpp'],cwd=root).decode()
assert 'const float fsx' in old and 'state_.fs = 0.0f;' in old
header=(root/'app/chassis/vmc.hpp').read_text()
cls=header[header.index('class vmc'):header.rindex('} // namespace')]
cls=cls.replace('public:','public:\n    bool spring_protected = false;',1)
(out/'legacy.hpp').write_text('#pragma once\n#include "vmc.hpp"\nnamespace robot::legacy_chassis {\nusing namespace robot::chassis;\n'+cls+'\n}\n')
old=old.replace('#include "vmc.hpp"','#include "legacy.hpp"').replace('namespace robot::chassis','namespace robot::legacy_chassis')
old=old.replace('    using namespace model;','    spring_protected = false;\n    using namespace model;',1)
old=old.replace('        state_.fs = 0.0f;','        spring_protected = true;\n        state_.fs = 0.0f;')
(out/'legacy.cpp').write_text(old)
(out/'compare.cpp').write_text(r"""#include "vmc.hpp"
#include "legacy.hpp"
#include <cstdio>
#include <cmath>
int main() {
 std::puts("sample,phi1,phi4,len,phi,force,old_return,old_protected,old_valid,new_valid,new_error,fs_old,fs_new,abs_diff");
 unsigned id; float p1,p4;
 while(std::scanf("%u %f %f",&id,&p1,&p4)==3) {
  robot::legacy_chassis::vmc old;
  robot::chassis::vmc cur;
  bool ok=old.resolve(p1,p4), nk=cur.resolve(p1,p4);
  const auto& o=old.state();const auto& n=cur.state();
  bool ov=ok&&!old.spring_protected&&std::isfinite(o.fs);
  bool nv=nk&&n.spring_valid&&std::isfinite(n.fs);
  float diff=ov&&nv?std::fabs(o.fs-n.fs):NAN;
  std::printf("%u,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%d,%d,%d,%d,%.9g,%.9g,%.9g\n",id,p1,p4,n.len,n.phi,robot::chassis::model::spring_force,ok,old.spring_protected,ov,nv,int(n.error),o.fs,n.fs,diff);
 }
}
""")
cmd=[a.cxx]+(['c++'] if a.zig else [])+['-std=c++17','-I'+str(root/'app/chassis'),'-I'+str(out),str(out/'compare.cpp'),str(out/'legacy.cpp'),str(root/'app/chassis/vmc.cpp'),'-o',str(out/'compare.exe')]
subprocess.run(cmd,check=True)
rows=list(csv.DictReader(a.samples.open()))
poses=''.join(f"{i} {r['phi1']} {r['phi4']}\n" for i,r in enumerate(rows))
res=subprocess.run([str(out/'compare.exe')],input=poses,text=True,capture_output=True,check=True)
(out/'all.csv').write_text(res.stdout)
comp=list(csv.DictReader(res.stdout.splitlines()))
normal=[r for r in comp if r['old_valid']=='1' and r['new_valid']=='1']
diff=[r for r in comp if r['old_valid']!=r['new_valid']]
protected=[r for r in comp if r['old_protected']=='1']
for name,data in [('normal',normal),('validity_mismatch',diff),('old_protected',protected)]:
 with (out/(name+'.csv')).open('w',newline='') as f:
  w=csv.DictWriter(f,fieldnames=comp[0].keys());w.writeheader();w.writerows(data)
report={'old_revision':rev,'samples':len(comp),'both_formula_valid':len(normal),'validity_mismatch':len(diff),'old_protected_excluded':len(protected),'max_difference':max(normal,key=lambda r:float(r['abs_diff'])) if normal else None}
(out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
