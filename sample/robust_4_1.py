import numpy as np
import matplotlib.pyplot as plt
from control.matlab import *


m1 = 0.8
m2 = 0.2
k1 = 100

c1 = 1
c2 = 0.3
Ks = 100

M = np.matrix([[m1, 0],[0, m2]])
C= np.matrix([[c1+c2, -c2],[-c2, c2]])
F = np.matrix([[Ks],[0]])

iM = np.linalg.inv(M)
Bp = np.concatenate([np.zeros((2,1)), iM*F])
Cp = [0,1,0,0]
Dp = 0


plt.figure(1)
plt.subplot(2,1,1)
plt.ylim(-60, 40)
plt.yticks(np.arange(-60, 41, 20))
plt.xlim(1e0, 1e2)
plt.subplot(2,1,2)
plt.ylim(-360, 0)
plt.yticks(np.arange(-360, 1, 90))
plt.xlim(1e0, 1e2)
fig1, fig2 = plt.gcf().axes # subplotのハンドラを取得

delta = np.arange(-1, 1, 0.1)
for i in range(len(delta)):
    k2 = 300+300*0.2*delta[i]
    K = np.matrix([[k1+k2, -k2],[-k2, k2]])
    Ap = np.concatenate([np.concatenate([np.zeros((2,2,)), np.eye(2)], axis = 1), np.concatenate([-iM*K, -iM*C], axis = 1)])
    P = ss(Ap, Bp, Cp, Dp)
    mag, phase, om = bode(P, plot = False)
    plt.sca(fig1)
    plt.semilogx(om, 20*np.log10(mag))
    plt.sca(fig2)
    plt.semilogx(om, phase*180/np.pi)
plt.sca(fig1)
plt.xticks([])
plt.show()
