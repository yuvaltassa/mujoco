import csv, glob, os
import numpy as np

SP = '/private/tmp/claude-501/-Users-yuval-git-mujoco/d6e61d91-ca96-4813-bab6-ba8bad1a4f85/scratchpad/conelogs'
data = {}
for path in sorted(glob.glob(os.path.join(SP, '*.csv'))):
    name = os.path.basename(path)[:-4]
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append([float(r[k]) for k in
                ('nv','nefc','ncone','ncc','nL','nJ','nH','F2','S_quad','S_cone',
                 'PATHA','EXACTU','t_stock','t_fold')])
    data[name] = np.array(rows)
COLS = dict(nv=0,nefc=1,ncone=2,ncc=3,nL=4,nJ=5,nH=6,F2=7,S_quad=8,S_cone=9,
            PATHA=10,EXACTU=11,ts=12,tf=13)
def C(a,k): return a[:,COLS[k]]

alla = np.vstack(list(data.values()))
ts, tf = C(alla,'ts'), C(alla,'tf')

# fitted-time comparison policy (5 constants) vs single-alpha
Xs = np.column_stack([np.ones(len(alla)), C(alla,'nL'), C(alla,'EXACTU')])
cs, *_ = np.linalg.lstsq(Xs, ts, rcond=None)
Xf = np.column_stack([np.ones(len(alla)), C(alla,'F2'), C(alla,'S_quad')+C(alla,'S_cone'),
                      C(alla,'nJ'), C(alla,'nH')])
cf, *_ = np.linalg.lstsq(Xf, tf, rcond=None)

def eval_policy(a, fold):
    t = np.where(fold, C(a,'tf'), C(a,'ts'))
    o = np.minimum(C(a,'ts'), C(a,'tf'))
    return t.sum(), o.sum()

tot_t = tot_o = 0
for a in data.values():
    ps = cs[0] + cs[1]*C(a,'nL') + cs[2]*C(a,'EXACTU')
    pf = cf[0] + cf[1]*C(a,'F2') + cf[2]*(C(a,'S_quad')+C(a,'S_cone')) + cf[3]*C(a,'nJ') + cf[4]*C(a,'nH')
    t, o = eval_policy(a, pf < ps)
    tot_t += t; tot_o += o
print(f"fitted-times policy: total regret {100*(tot_t-tot_o)/tot_o:.2f}%")

# finer alpha sweep around the flat bottom, and the integer form 4U > F2+2S
for alpha in [0.4, 0.45, 0.5, 0.55, 0.6, 0.65]:
    tot_t = tot_o = 0
    for a in data.values():
        fold = C(a,'EXACTU') > alpha*(C(a,'F2')/2 + C(a,'S_quad') + C(a,'S_cone'))
        t, o = eval_policy(a, fold)
        tot_t += t; tot_o += o
    print(f"alpha={alpha:<5} regret {100*(tot_t-tot_o)/tot_o:.2f}%")

# S_quad share of the fold-cost estimate: if J'DJ skipped quad rows, how much drops?
print(f"\n{'config':<18}{'S_quad/(F2/2+S_all)':>21}{'in tf terms':>12}")
for name, a in data.items():
    denom = C(a,'F2')/2 + C(a,'S_quad') + C(a,'S_cone')
    share = np.sum(C(a,'S_quad'))/np.sum(denom)
    tf_share = cf[2]*np.sum(C(a,'S_quad'))/np.sum(C(a,'tf'))
    print(f"{name:<18}{100*share:>20.1f}%{100*tf_share:>11.1f}%")

# where does the alpha=0.5 regret live? distribution of per-call regret
tot = 0; big = 0
for a in data.values():
    fold = C(a,'EXACTU') > 0.5*(C(a,'F2')/2 + C(a,'S_quad') + C(a,'S_cone'))
    t = np.where(fold, C(a,'tf'), C(a,'ts'))
    o = np.minimum(C(a,'ts'), C(a,'tf'))
    reg = t - o
    tot += reg.sum()
    big += reg[(t > 1.5*o) & (reg > 20)].sum()  # calls where the wrong choice cost >50% and >20us
print(f"\nalpha=0.5: total regret {tot/1000:.1f}ms; from badly-wrong calls (>1.5x and >20us): {big/1000:.1f}ms")
