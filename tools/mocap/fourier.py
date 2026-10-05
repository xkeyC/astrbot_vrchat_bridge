import numpy as np
B=np.load('bins.npy',allow_pickle=True).item()
ph=np.arange(100)/100
def hand(A,s):
    ax=A[s+'ax']; nm=A[s+'nm']; inward=1. if s=='l' else -1.
    ax=ax/np.linalg.norm(ax,axis=1,keepdims=True)
    pitch=np.degrees(np.arctan2(-ax[:,2],-ax[:,1]))
    inw=np.array([inward,0,0]); sg=1. if s=='l' else -1.
    tw=np.degrees(np.arctan2(sg*np.sum(np.cross(inw,nm)*ax,1),nm@inw))
    return pitch,tw
def four(x):
    m=x.mean(); out=[m]
    for k in (1,2):
        c=np.sum(x*np.exp(-2j*np.pi*k*ph))*2/100
        out+= [abs(c),(-np.angle(c)/(2*np.pi*k))%(1/k)]
    fit=m+out[1]*np.cos(2*np.pi*(ph-out[2]))+out[3]*np.cos(4*np.pi*(ph-out[4]))
    r2=1-np.var(x-fit)/max(np.var(x),1e-12)
    return out+[r2]
for k,(r,A) in B.items():
    lp,lt=hand(A,'l'); rp,rt=hand(A,'r')
    sig={'headY_cm':A['head'][:,1]*100,'headX_cm':A['head'][:,0]*100,'headZ_cm':A['head'][:,2]*100,
         'headPitch':A['hang'][:,0]-A['hang'][:,0].mean(),'headYaw':A['hang'][:,1]-A['hang'][:,1].mean(),'headRoll':A['hang'][:,2]-A['hang'][:,2].mean(),
         'lwFwd_cm':-A['lw'][:,2]*100,'lwY_cm':A['lw'][:,1]*100,'lwX_cm':A['lw'][:,0]*100,'lwPitch':lp,'lwTwist':lt,
         'rwFwd_cm':-A['rw'][:,2]*100,'rwPitch':rp,'rwTwist':rt}
    print('==',k,'v=%.2f'%r['speed'])
    for n,x in sig.items():
        m,a1,p1,a2,p2,r2=four(x)
        print('  %-10s mean %7.1f  A1 %5.1f @%.2f  A2 %5.1f @%.2f  R2 %.2f  min %.1f max %.1f'%(n,m,a1,p1,a2,p2,r2,x.min(),x.max()))
