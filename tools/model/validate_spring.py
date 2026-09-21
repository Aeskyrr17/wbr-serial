"""Validate production VMC CSV or captured samples; finite differences are offline only.
CSV: time,len,phi,spring_len,dls_dlen,fs,spring_valid; optional phi1,phi4 for geometry comparison.
Use one leg per file, same-cycle samples, spring force supplied explicitly.
"""
import argparse, csv, math
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('csv',type=Path)
p.add_argument('--force',type=float,required=True)
p.add_argument('--out',type=Path,required=True)
a=p.parse_args(); a.out.mkdir(parents=True,exist_ok=True)
rows=list(csv.DictReader(a.csv.open()))
rows=[{k:float(v) for k,v in r.items()} for r in rows]
L1=.22;L2=.26;d1=.03;d2=.05;gamma=.2164
geometry=[]
for r in rows:
 if r['spring_valid']!=1 or 'phi1' not in r: continue
 p1,p4=r['phi1'],r['phi4'];x=L1*(math.cos(p4)-math.cos(p1));y=L1*(math.sin(p4)-math.sin(p1))
 A=2*L2*x;B=2*L2*y;C=x*x+y*y
 u=2*math.atan2(B+math.sqrt(max(0,A*A+B*B-C*C)),A+C)
 old=math.hypot(L1*math.cos(p1)+d2*math.cos(u)-d1*math.cos(p1+gamma-math.pi/2),L1*math.sin(p1)+d2*math.sin(u)-d1*math.sin(p1+gamma-math.pi/2))
 geometry.append(abs(old-r['spring_len']))
results=[]; summary=[]
for stride in (5,10,20):
 errors=[]
 for i in range(stride,len(rows)-stride):
  segment=rows[i-stride:i+stride+1]; left,mid,right=rows[i-stride],rows[i],rows[i+stride]
  if any(r['spring_valid']!=1 or not all(math.isfinite(r[k]) for k in ('len','phi','spring_len','fs')) for r in segment): continue
  steps=[y['len']-x['len'] for x,y in zip(segment,segment[1:])]
  if not (all(d>1e-7 for d in steps) or all(d< -1e-7 for d in steps)): continue
  dl1=mid['len']-left['len'];dl2=right['len']-mid['len']
  if min(abs(dl1),abs(dl2))<.0002 or abs(dl1/dl2-1)>.1: continue
  if max(r['phi'] for r in segment)-min(r['phi'] for r in segment)>.01: continue
  if min(r['len'] for r in segment)<.05 or max(r['len'] for r in segment)>.47: continue
  fd=a.force*(right['spring_len']-left['spring_len'])/(right['len']-left['len'])
  errors.append(abs(fd-mid['fs']));results.append((stride,mid['len'],mid['fs'],fd))
 summary.append(f'stride={stride}: accepted={len(errors)}, max_abs_error_N={max(errors) if errors else None}')
with (a.out/'forces.csv').open('w',newline='') as f:
 w=csv.writer(f);w.writerow(['stride','len','fs_code','fs_check']);w.writerows(results)
# Standalone SVG plot, no plotting package needed.
if results:
 ys=[v for r in results for v in r[2:]];lo=min(ys);hi=max(ys);hi=max(hi,lo+1)
 def points(column):
  data=sorted((r for r in results if r[0]==10),key=lambda r:r[1])
  return ' '.join(f'{60+(r[1]-.16)/.19*700:.2f},{340-(r[column]-lo)/(hi-lo)*290:.2f}' for r in data)
 svg=f'<svg xmlns="http://www.w3.org/2000/svg" width="820" height="400"><rect width="100%" height="100%" fill="white"/><text x="60" y="20">Fs (N): code blue / finite difference red; Fg={a.force} N</text><text x="60" y="370">len (m): 0.16 to 0.35; Fs range: {lo:.3f} to {hi:.3f} N</text><polyline fill="none" stroke="blue" points="{points(2)}"/><polyline fill="none" stroke="red" stroke-dasharray="3 3" points="{points(3)}"/></svg>'
 (a.out/'forces.svg').write_text(svg)
summary.insert(0,f'geometry_samples={len(geometry)}, max_length_error_m={max(geometry) if geometry else None}; Fg={a.force} N')
(a.out/'report.txt').write_text('\n'.join(summary));print('\n'.join(summary))
