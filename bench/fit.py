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
    a = np.array(rows)
    data[name] = a

COLS = dict(nv=0,nefc=1,ncone=2,ncc=3,nL=4,nJ=5,nH=6,F2=7,S_quad=8,S_cone=9,
            PATHA=10,EXACTU=11,ts=12,tf=13)
def C(a, k): return a[:, COLS[k]]

print(f"{'config':<18}{'calls':>7}{'ncone rng':>12}{'ts med':>9}{'tf med':>9}")
for name, a in data.items():
    print(f"{name:<18}{len(a):>7}{int(C(a,'ncone').min()):>6}-{int(C(a,'ncone').max()):<5}"
          f"{np.median(C(a,'ts')):>9.0f}{np.median(C(a,'tf')):>9.0f}")

# ---- 1. PATHA vs EXACTU quality ----
alla = np.vstack(list(data.values()))
pa, ex = C(alla,'PATHA'), C(alla,'EXACTU')
mask = ex > 0
r = pa[mask]/ex[mask]
print(f"\nPATHA/EXACTU: median {np.median(r):.3f}  p5 {np.percentile(r,5):.3f}  p95 {np.percentile(r,95):.3f}")
print(f"correlation(PATHA, EXACTU) = {np.corrcoef(pa[mask], ex[mask])[0,1]:.4f}")

# ---- 2. cost-model fits (least squares, global) ----
def fit(y, X, names):
    coef, res, *_ = np.linalg.lstsq(X, y, rcond=None)
    pred = X @ coef
    ss = 1 - np.sum((y-pred)**2)/np.sum((y-np.mean(y))**2)
    print('  ' + '  '.join(f'{n}={c:.4g}' for n, c in zip(names, coef)) + f'   R2={ss:.3f}')
    return coef

ts, tf = C(alla,'ts'), C(alla,'tf')
ones = np.ones(len(alla))
print("\nt_stock fit (us):")
cs = fit(ts, np.column_stack([ones, C(alla,'nL'), C(alla,'EXACTU')]), ['c','nL','EXACTU'])
print("t_stock fit with PATHA instead:")
csp = fit(ts, np.column_stack([ones, C(alla,'nL'), C(alla,'PATHA')]), ['c','nL','PATHA'])
print("t_fold fit (us):")
cf = fit(tf, np.column_stack([ones, C(alla,'F2'), C(alla,'S_quad')+C(alla,'S_cone'),
                              C(alla,'nJ'), C(alla,'nH')]), ['c','F2','S_all','nJ','nH'])

# ---- 3. decision policies, evaluated per config by regret vs oracle ----
def eval_policy(a, choose_fold):
    ts, tf = C(a,'ts'), C(a,'tf')
    t = np.where(choose_fold, tf, ts)
    oracle = np.minimum(ts, tf)
    return t.sum(), oracle.sum()

def policy_table(policies):
    print(f"\n{'config':<18}" + ''.join(f"{p:>11}" for p, _ in policies) + f"{'oracle ms':>11}")
    tot = {p: 0.0 for p, _ in policies}; tot_o = 0.0
    for name, a in data.items():
        row = f"{name:<18}"
        for p, fn in policies:
            t, o = eval_policy(a, fn(a))
            tot[p] += t
            regret = 100*(t-o)/o if o > 0 else 0
            row += f"{regret:>10.1f}%"
        _, o = eval_policy(a, fn(a)); tot_o += o
        row += f"{o/1000:>11.1f}"
        print(row)
    print(f"{'TOTAL regret':<18}" + ''.join(f"{100*(tot[p]-tot_o)/tot_o:>10.1f}%" for p, _ in policies))

# candidate alpha sweep for the flop-ratio rule: fold iff EXACTU > alpha*(F2/2 + S_all)
best = None
for alpha in [0.25, 0.35, 0.5, 0.7, 1.0, 1.4, 2.0]:
    tot_t = tot_o = 0
    for a in data.values():
        fold = C(a,'EXACTU') > alpha*(C(a,'F2')/2 + C(a,'S_quad') + C(a,'S_cone'))
        t, o = eval_policy(a, fold)
        tot_t += t; tot_o += o
    reg = 100*(tot_t-tot_o)/tot_o
    print(f"alpha={alpha:<5} EXACTU-rule total regret {reg:6.2f}%")
    if best is None or reg < best[1]: best = (alpha, reg)
al = best[0]
print(f"best alpha = {al}")

bestp = None
for alpha in [0.25, 0.35, 0.5, 0.7, 1.0, 1.4, 2.0]:
    tot_t = tot_o = 0
    for a in data.values():
        fold = C(a,'PATHA') > alpha*(C(a,'F2')/2 + C(a,'S_quad') + C(a,'S_cone'))
        t, o = eval_policy(a, fold)
        tot_t += t; tot_o += o
    reg = 100*(tot_t-tot_o)/tot_o
    print(f"alpha={alpha:<5} PATHA-rule  total regret {reg:6.2f}%")
    if bestp is None or reg < bestp[1]: bestp = (alpha, reg)
alp = bestp[0]

# best global ncone constant, for comparison
bestc = None
for K in [8, 16, 24, 32, 48, 64, 96, 128, 192]:
    tot_t = tot_o = 0
    for a in data.values():
        t, o = eval_policy(a, C(a,'ncone') > K)
        tot_t += t; tot_o += o
    reg = 100*(tot_t-tot_o)/tot_o
    if bestc is None or reg < bestc[1]: bestc = (K, reg)
print(f"best ncone constant K={bestc[0]} (regret {bestc[1]:.2f}%)")

policies = [
    ('stock',   lambda a: np.zeros(len(a), bool)),
    ('fold',    lambda a: np.ones(len(a), bool)),
    ('K=24',    lambda a: C(a,'ncone') > 24),
    (f'K={bestc[0]}', lambda a: C(a,'ncone') > bestc[0]),
    (f'E@{al}', lambda a: C(a,'EXACTU') > al*(C(a,'F2')/2 + C(a,'S_quad') + C(a,'S_cone'))),
    (f'P@{alp}', lambda a: C(a,'PATHA') > alp*(C(a,'F2')/2 + C(a,'S_quad') + C(a,'S_cone'))),
]
policy_table(policies)
