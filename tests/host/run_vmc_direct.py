"""Compile current VMC against the saved pre-replacement production baseline."""
import argparse,csv,io,json,math,subprocess
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--cxx',required=True);p.add_argument('--zig',action='store_true');a=p.parse_args()
root=Path(__file__).resolve().parents[2];out=root/'build/vmc-direct';out.mkdir(parents=True,exist_ok=True)
base=out/'baseline';base.mkdir(exist_ok=True)
for src in (root/'tests/host/vmc_baseline').rglob('*'):
 if src.suffix not in ('.cpp','.hpp'): continue
 dst=base/src.relative_to(root/'tests/host/vmc_baseline');dst.parent.mkdir(parents=True,exist_ok=True)
 dst.write_text(src.read_text().replace('robot::chassis','robot::baseline_chassis'))
runner=r"""#include "vmc.hpp"
#include "baseline/vmc.hpp"
#include <cstdio>
#include <cmath>
#include <limits>
using robot::chassis::vmc;
float wrap(float x){return std::remainder(x,2*robot::chassis::model::pi);}
int main(){
 unsigned id;float p1,p4;
 while(std::scanf("%u %f %f",&id,&p1,&p4)==3){
 vmc n;robot::baseline_chassis::vmc o;
 bool nv=n.resolve(p1,p4),ov=o.resolve(p1,p4);auto ns=n.state();auto os=o.state();
 std::printf("%u,%d,%d,%d,%d",id,ov,nv,int(ns.geometry_error),int(os.error));
 auto emit=[](float x){std::printf(",%.9g",x);};
 emit(ns.len);emit(ns.phi);
 emit(nv&&ov?ns.len-os.len:NAN);emit(nv&&ov?wrap(ns.phi-os.phi):NAN);
 for(int i=0;i<4;++i)emit(nv&&ov?ns.J[i]-os.J[i]:NAN);
 for(int i=0;i<4;++i)emit(nv&&ov?ns.JT_inv[i]-os.JT_inv[i]:NAN);
 auto vn=n.velocity(.7f,-.4f);auto vo=o.velocity(.7f,-.4f);
 auto tn=n.joint_torque(83.0f,-2.3f);auto to=o.joint_torque(83.0f,-2.3f);
 auto fn=n.generalized_force(3.1f,-1.7f);auto fo=o.generalized_force(3.1f,-1.7f);
 for(int i=0;i<2;++i)emit(nv&&ov?vn[i]-vo[i]:NAN);
 for(int i=0;i<2;++i)emit(nv&&ov?tn[i]-to[i]:NAN);
 for(int i=0;i<2;++i)emit(nv&&ov?fn[i]-fo[i]:NAN);
 emit(nv&&ov?ns.spring_len-os.spring_len:NAN);emit(nv&&ov?ns.dls_dlen-os.dls_dlen:NAN);emit(nv&&ov?ns.fs-os.fs:NAN);
 for(float eps:{.002f,.001f,.0005f})for(int j=0;j<2;++j){
 vmc plus,minus;bool valid=nv&&plus.resolve(p1+(j==0?eps:0),p4+(j==1?eps:0))&&minus.resolve(p1-(j==0?eps:0),p4-(j==1?eps:0));
 emit(valid?(plus.state().len-minus.state().len)/(2*eps)-ns.J[j]:NAN);
 emit(valid?wrap(plus.state().phi-minus.state().phi)/(2*eps)-ns.J[j+2]:NAN);
 }
 auto restored=n.generalized_force(tn[0],tn[1]);emit(nv?restored[0]-83:NAN);emit(nv?restored[1]+2.3f:NAN);
 emit(nv?tn[0]*.7f+tn[1]*(-.4f)-(83*vn[0]-2.3f*vn[1]):NAN);
 float identity=0;
 for(int i=0;i<2;++i)for(int j=0;j<2;++j){float val=0;for(int k=0;k<2;++k)val+=ns.JT_inv[2*i+k]*ns.JT[2*k+j];identity=std::fmax(identity,std::fabs(val-(i==j?1:0)));}
 emit(nv?identity:NAN);
 // Failed solve must invalidate all current mappings and spring output.
 n.resolve(NAN,0);if(n.state().valid||n.state().spring_valid||!std::isnan(n.state().fs)||!std::isnan(n.velocity(1,1)[0]))return 2;
 std::puts("");
 }
}
"""
(out/'runner.cpp').write_text(runner)
cmd=[a.cxx]+(['c++'] if a.zig else [])+['-std=c++17','-I'+str(root/'app/chassis'),'-I'+str(out),str(out/'runner.cpp'),str(base/'vmc.cpp'),str(root/'app/chassis/vmc.cpp'),'-o',str(out/'runner.exe')]
subprocess.run(cmd,check=True)
poses=[]
def add(group,p1,p4,parent=-1): poses.append(dict(id=len(poses),group=group,phi1=p1,phi4=p4,parent=parent))
for r in csv.DictReader((root/'build/spring_samples.csv').open()):add('original5703',float(r['phi1']),float(r['phi4']))
# Extended startup/retraction/swing and phi crossing +/-pi, both joint wrap boundaries.
for phi in [-3.15,-math.pi,-3.13,-2.7,-1.57,-.8,0,.8,1.1,math.pi/2,2.7,3.13,math.pi,3.15]:
 for i in range(221):
  length=.045+i*.00195;eta=math.acos((.22**2+length**2-.26**2)/(2*.22*length))
  idx=len(poses);add('motion',phi+eta,phi-eta)
  for t1,t4 in [(1,0),(-1,0),(0,1),(0,-1),(1,-1)]:add('turns',phi+eta+t1*2*math.pi,phi-eta+t4*2*math.pi,idx)
for delta in [0,1e-7,1e-6,1e-5,1e-4,.001,.01,2*math.pi-.01,2*math.pi-.001,2*math.pi-1e-4,2*math.pi-1e-5,2*math.pi]:
 for phi in [-math.pi,-1.,0.,1.,math.pi]:add('boundary',phi+delta/2,phi-delta/2)
# The old half-angle coordinate chart rejects A+C close to zero.
for eta in [.2,.8,1.5,2.2,2.9]:
 chart=-math.asin(.22*math.sin(eta)/.26)
 for offset in [-1e-4,-1e-5,-1e-6,0,1e-6,1e-5,1e-4]:
  phi=chart+offset;add('chart',phi+eta,phi-eta)
for phi in [math.pi-.002,math.pi-.001,math.pi,math.pi+.001,math.pi+.002]:
 add('cross_pi',phi+1.4,phi-1.4)
for p1,p4 in [(float('nan'),0),(0,float('inf')),(-float('inf'),1)]:add('invalid',p1,p4)
text=''.join(f"{r['id']} {r['phi1']:.12g} {r['phi4']:.12g}\n" for r in poses)
res=subprocess.run([str(out/'runner.exe')],input=text,text=True,capture_output=True,check=True)
metrics=['len','phi']+[f'J{i}' for i in range(4)]+[f'JT_inv{i}' for i in range(4)]+['velocity_len','velocity_phi','torque1','torque4','force_len','force_phi','spring_len','dls_dlen','fs']
num=[f'diff_{eps}_{j}_{v}' for eps in ['.002','.001','.0005'] for j in range(2) for v in ['len','phi']]
header=['id','old_valid','new_valid','new_error','old_spring_error','new_len','new_phi']+metrics+num+['roundtrip_force','roundtrip_torque','power','identity']
rows=list(csv.DictReader(io.StringIO(res.stdout),fieldnames=header));assert len(rows)==len(poses)
for r,pose in zip(rows,poses):r.update(pose)
with (out/'all.csv').open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
mismatch=[r for r in rows if r['old_valid']!=r['new_valid']]
with (out/'validity_mismatch.csv').open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(mismatch)
def maxima(data,fields):
 result={}
 for field in fields:
  valid=[r for r in data if math.isfinite(float(r[field]))]
  if valid:
   r=max(valid,key=lambda r:abs(float(r[field])));result[field]={'abs_error':abs(float(r[field])),'id':r['id'],'group':r['group'],'phi1':r['phi1'],'phi4':r['phi4'],'len':r['new_len'],'phi':r['new_phi']}
 return result
summary={}
for group in ['original5703','motion','turns','boundary','chart','cross_pi','invalid']:
 data=[r for r in rows if r['group']==group];summary[group]={'count':len(data),'both_valid':sum(r['old_valid']==r['new_valid']=='1' for r in data),'mismatches':sum(r['old_valid']!=r['new_valid'] for r in data),'maxima':maxima(data,metrics)}
summary['independent']=maxima([r for r in rows if r['group'] in ['original5703','motion']],num+['roundtrip_force','roundtrip_torque','power','identity'])
turn_errors=[]
for r in rows:
 if r['parent']<0:continue
 b=rows[r['parent']]
 if r['new_valid']!=b['new_valid']:turn_errors.append({'id':r['id'],'reason':'validity'})
 elif r['new_valid']=='1':
  dl=abs(float(r['new_len'])-float(b['new_len']));dp=abs(math.remainder(float(r['new_phi'])-float(b['new_phi']),2*math.pi))
  if dl>2e-6 or dp>2e-6:turn_errors.append({'id':r['id'],'len':dl,'phi':dp})
summary['turn_failures']=turn_errors
cross=[r for r in rows if r['group']=='cross_pi'];unwrapped=[]
for r in cross:
 phi=float(r['new_phi']);unwrapped.append(phi if not unwrapped else unwrapped[-1]+math.remainder(phi-unwrapped[-1],2*math.pi))
summary['cross_pi_tracking']=unwrapped
assert not turn_errors
assert all(r['new_valid']=='0' for r in rows if r['group']=='invalid')
assert all(abs(b-a-.001)<2e-6 for a,b in zip(unwrapped,unwrapped[1:]))
# Assert acceptance only for the declared ordinary working set, not singular boundaries.
for key,value in summary['independent'].items():
 tol = (2e-4 if key.endswith('_len') else 4e-4) if key.startswith('diff_') else {'roundtrip_force':2e-5,'roundtrip_torque':2e-6,'power':2e-6,'identity':2e-7}[key]
 assert value['abs_error'] < tol, (key,value)
limits={'len':2e-6,'phi':2e-5,'fs':.002,'force_len':.002}
for key,tol in limits.items():assert summary['original5703']['maxima'][key]['abs_error']<tol

(out/'report.json').write_text(json.dumps(summary,indent=2))
print(json.dumps({g:{k:v for k,v in d.items() if k!='maxima'} for g,d in summary.items() if g not in ['independent','turn_failures','cross_pi_tracking']},indent=2));print('turn_failures',len(turn_errors));print('report',out/'report.json')
