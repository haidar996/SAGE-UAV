import re,glob,math,statistics as st,collections,json,sys
world=sys.argv[1];globs=sys.argv[2:]
d=json.load(open(f'/home/haidar/SAGE-UAV/config/truth_{world}.json'))
def dseg(p,a,b):
    ax,ay=a;bx,by=b;px,py=p;dx,dy=bx-ax,by-ay
    t=max(0,min(1,((px-ax)*dx+(py-ay)*dy)/(dx*dx+dy*dy)))
    return math.hypot(px-ax-t*dx,py-ay-t*dy)
def label(p):
    c=[]
    for q in d['people']:
        pts=q['points']
        dist=math.hypot(p[0]-pts[0][0],p[1]-pts[0][1]) if len(pts)==1 else dseg(p,*pts)
        c.append((dist,'static' if q['type']=='static' else 'walker'))
    dd,l=min(c);return l if dd<1.5 else None
pat=re.compile(r'\[(\d+\.\d+)\].*Target (\d+) updated.*raw=\(([-0-9.]+), ([-0-9.]+)')
T=collections.defaultdict(list)
for g in globs:
  for f in glob.glob(g+'/world.log'):
    for l in open(f):
        m=pat.search(l)
        if m: T[(f,int(m[2]))].append((float(m[1]),float(m[3]),float(m[4])))
def ls(w):
    ts=[p[0] for p in w];tm=sum(ts)/len(ts);den=sum((t-tm)**2 for t in ts)
    if den<1e-6:return 0
    return math.hypot(sum((t-tm)*p[1] for t,p in zip(ts,w))/den,sum((t-tm)*p[2] for t,p in zip(ts,w))/den)
res={'static':[],'walker':[]}
for k,pts in T.items():
    pts.sort();i=0
    while i<len(pts):
        t0=pts[i][0];w=[p for p in pts[i:] if p[0]-t0<=8.0]
        if len(w)>=8 and w[-1][0]-t0>=6.5:
            lab=label((st.median(p[1] for p in w),st.median(p[2] for p in w)))
            if lab:res[lab].append(ls(w))
            i+=len(w)//2 or 1
        else:i+=1
def q(a,p):a=sorted(a);return a[int(p*(len(a)-1))]
for lab,a in res.items():
    if a:print(f'{world} {lab:6s} n={len(a):3d} p10={q(a,.1):.2f} p50={q(a,.5):.2f} p75={q(a,.75):.2f} p90={q(a,.9):.2f}  >0.3: {100*sum(x>0.3 for x in a)/len(a):.0f}%')
