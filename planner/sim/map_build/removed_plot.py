import sys, numpy as np, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
def L(p):
    raw=open(p,"rb").read(); k=raw.index(b"DATA binary\n")+12
    return np.frombuffer(raw[k:],dtype="<f4").reshape(-1,4)
A=L(sys.argv[1]); B=L(sys.argv[2])
d,_=cKDTree(B[:,:3]).query(A[:,:3],distance_upper_bound=0.01); R=A[np.isinf(d)]
print("removed",len(R),"of",len(A))
fig,ax=plt.subplots(1,2,figsize=(24,12))
for a,P,tt in ((ax[0],B,"kept"),(ax[1],R,"removed")):
  m=(P[:,2]>0.3)&(P[:,2]<1.8); a.scatter(P[m,0],P[m,1],s=0.2,c=P[m,2],cmap="viridis",vmin=0.3,vmax=1.8); a.set_aspect("equal"); a.set_title(tt+" 0.3-1.8 m")
plt.tight_layout(); plt.savefig(sys.argv[3],dpi=55)
