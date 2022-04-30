"""
2021/04/10
@Yuya Shimizu

ロバスト制御器の設計
"""
import numpy as np
import matplotlib.pyplot as plt
from control import bode, tf
from control.matlab import logspace
from tf_arm import arm_tf
from for_plot import bodeplot_set
import numpy as np
import matplotlib.pyplot as plt
from control import bode, tf, mixsyn, ss2tf
from control.matlab import logspace, feedback, step
from tf_arm import arm_tf
from for_plot import plot_set


class Mode():
    def __init__(self, freq_rad, damp, shape):
        self.freq_rad = freq_rad
        self.damp = damp
        self.shape = shape

def sat_func(modes:list):
    sat_tf = 0
    for mode in modes:
        print(sat_tf)
        sat_tf += tf([0,mode.shape**2],[1, 2*mode.damp*mode.freq_rad, mode.freq_rad**2])
    return sat_tf

mode1 = Mode(0.196*2*np.pi, 0.1, -1.46*10)
mode2 = Mode(0.834*2*np.pi, 0.1, -4.45)
mode3 = Mode(2.192*2*np.pi,0.1, 5.92)
mode4 = Mode(10.192*2*np.pi, 0.2, 12.92)

true_modes = [ mode1, mode2, mode3, mode4]
modified_modes = [ mode1]

P = sat_func(true_modes)
Pn = sat_func(modified_modes)
bode(P)
bode(Pn)
bode(tf([0,-4.45**2],[1,0.5*0.834*2*np.pi,(0.834*2*np.pi)**2]))

###不確かさ
delta = np.arange(-1, 1, 0.1)
WT = tf([0.3, 10], [1.3, 70])

fig, ax = plt.subplots(1, 2)

#不確かさをもつ制御対象
gain, _, w = bode(P, logspace(-3, 6), Plot = False)
ax[0].semilogx(w, 20*np.log10(gain), color='k', lw=0.3)



#乗法的不確かさ
DT = (Pn-P)/Pn
gain, _, w = bode(DT, logspace(-3, 6), Plot = False)
ax[1].semilogx(w, 20*np.log10(gain), color='k', lw=0.3)

gain, _, w = bode(Pn, logspace(-3, 6), Plot = False)
ax[0].semilogx(w, 20*np.log10(gain), color='r', lw=2)

gain, _, w = bode(WT, logspace(-3, 6), Plot = False)
ax[1].semilogx(w, 20*np.log10(gain), color='r', lw=2)

bodeplot_set(ax)
ax[0].set_xlabel('$\omega$ [rad/s]')
ax[0].set_ylabel('Gain of $P$ [dB]')
ax[1].set_xlabel('$\omega$ [rad/s]')
ax[1].set_ylabel('Gain of $\Delta W_T/P$ [dB]')

fig.suptitle("Gain of control target with Certainty")
plt.show()



###重み関数の定義
WS = tf([0, 1], [1, 1, 0.15])   #感度関数に対する重み関数
WU = tf(1, 1)
#WT = tf([10, 0], [1, 150])   #相補感度関数に対する重み関数
append = tf([1,0], [1, 1])

###混合感度問題
K, _, gamma = mixsyn(Pn, w1 = WS, w2 = WU, w3 = WT) #混合感度問題を解く
print('K=', ss2tf(K))
print('gamma=', gamma[0])

fig, ax = plt.subplots(1, 2)
###感度関数
Ssys = feedback(1, Pn*K)
gain, _, w = bode(Ssys, logspace(-3, 3), Plot = False)
ax[0].semilogx(w, 20*np.log10(gain), lw=2, label='$S$')
gain, _, w = bode(1/WS, logspace(-3, 3), Plot = False)
ax[0].semilogx(w, 20*np.log10(gain), ls='-.', label='$1/W_S$')

###相補感度関数
Tsys = feedback(Pn*K, 1)
gain, _, w = bode(Tsys, logspace(-3, 3), Plot = False)
ax[1].semilogx(w, 20*np.log10(gain), lw=2, label='$T$')
gain, _, w = bode(1/WT, logspace(-3, 3), Plot = False)
ax[1].semilogx(w, 20*np.log10(gain), ls='--', label='$1/W_T$')

for i in range(2):
    ax[i].set_ylim(-50, 50)
    ax[i].legend()
    ax[i].grid(which="both", ls=':')
    ax[i].set_ylabel('Gain [dB]')
    ax[i].set_xlabel('$\omega$ [rad/s]')

fig.tight_layout()
fig.suptitle("robust controller")
plt.show()

###設計した制御器の性能確認
fig, ax = plt.subplots()
ref = 0.01      #目標値30
#不確かさ
#不確かさ
delta = np.arange(-1, 1, 0.1)

#不確かさを有するモデルに対する性能
for i in range(len(delta)):
    #不確かさをもつ制御対象
    P = (1 + DT*delta[i])*Pn
    Gyr = feedback(P*K, 1)
    y, t = step(Gyr, np.arange(0, 5, 0.01))
    ax.plot(t, y*ref, color='k', lw=0.3)


#ノミナルモデル（不確かさなし）に対する性能
Gyr = feedback(Pn*K, 1)
y, t = step(Gyr, np.arange(0, 5, 0.01))
ax.plot(t, y*ref, color='r', lw=2, label="nominal model")
ax.legend()
plot_set(ax, 't', 'y')

ax.set_title("robust controller test in model")
plt.show()