import numpy as np
from scipy.integrate import odeint
import matplotlib
matplotlib.use('Qt4Agg')
import matplotlib.pyplot as plt

# v = 0.5

# define the ODE as a first order system
def system_v025_k2(d, t):
    # v = 0.25 + 0.1 * np.sin(t * 2)
    k1 = 1.0
    k2 = 2.0
    U = -k1 * d[2] - k2 * np.abs(v) * d[1] - k2 * np.power(v, 2) * d[0]
    return [d[1], d[2], U]    


def system_v05_k2(d, t):
    # v = 0.5 * 1 / (1 + np.exp(-t + 5))
    # v = 0.5 + 0.1 * np.sin(t * 2 )
    k1 = 1.0
    k2 = 2.0
    U = -k1 * d[2] - k2 * np.abs(v) * d[1] - k2 * np.power(v, 2) * d[0]
    return [d[1], d[2], U]


def system_v05_k1(d, t):
    # v = 0.5 * 1 / (1 + np.exp(-t + 5))
    # v = 0.5 + 0.1 * np.sin(t * 2 )
    k1 = 1.0
    k2 = 1.0
    U = -k1 * d[2] - k2 * np.abs(v) * d[1] - k2 * np.power(v, 2) * d[0]
    return [d[1], d[2], U]


if __name__ == '__main__':
    # initial values
    d0 = [1.0, 0.0, 0.0]
    t = np.linspace(0, 30, 401)


    # numerical integration
    # k1 = k2 = 2.0 ,v = 0.25
    d_v025_list = []
    v = 0.25
    for init_d in [1.0, 0.6, 0.2]:
        d0[0] = init_d
        d = odeint(system_v025_k2, d0, t)
        d_v025_list.append(d)

    # k1 = k2 = 2.0 ,v = 0.5
    d_v05_list = []
    v = 0.5
    for init_d in [1.0, 0.6, 0.2]:
        d0[0] = init_d
        d = odeint(system_v05_k2, d0, t)
        d_v05_list.append(d)

    # k1 = k2 = 1.0 ,v = 0.5    
    d_v05_k1_list = []
    v = 0.5
    for init_d in [1.0,]:
        d0[0] = init_d
        d = odeint(system_v05_k1, d0, t)
        d_v05_k1_list.append(d)


    # Visualization
    fig = plt.figure(figsize=(10, 4))
    ax =  fig.add_subplot(111)

    line_v025 = None
    for i, linestyle in enumerate(["-", "--", ":"]):
        line, = ax.plot(t, d_v025_list[i][:,0], color="red", alpha=0.9, linestyle=linestyle)
        if i == 0: line_v025 = line

    line_v05 = None
    for i, linestyle in enumerate(["-", "--", ":"]):
        line, = ax.plot(t, d_v05_list[i][:,0], color="blue", alpha=0.9, linestyle=linestyle)
        if i == 0: line_v05 = line

    line_v05_k1 = None
    for i, alpha in enumerate([0.9,]):
        line, = ax.plot(t, d_v05_k1_list[i][:,0], color="green", alpha=alpha)
        if i == 0: line_v05_k1 = line

    ax.legend([line_v025, line_v05, line_v05_k1], [r"$k_1= 1.0, k_2= 2.0$" + r"$, \bf{v= 0.25}$" + " m/s",
                                                   r"$k_1= 1.0, k_2= 2.0$" + r"$, v= 0.5$" + " m/s",
                                                   r"$k_1= 1.0, \bf{k_2}= 1.0$" + r"$, v= 0.5 $" + " m/s"], fontsize=13)

    # line_d10_v05_k2, = ax.plot(t, d_d10_v05_k2[:,0], color="blue", alpha=0.5)
    # line_d10_v025_k2, = ax.plot(t, d_d10_v025_k2[:,0], color="red", alpha=0.5)
    # cost, = ax.plot(t, np.cos(t * 2))

    # ax.legend([line_d10_v05_k2, line_d10_v025_k2], ["$v=0.5 + 0.1sin(2t)$", "$v=0.25 + 0.1sin(2t)$"])
    plt.xlabel("t (seconds)", fontsize=12)
    plt.ylabel("Cross-track error: $d(t)$", fontsize=14)
    ax.set_ylim([-0.6, 1.0])
    ax.set_yticks(np.linspace(-0.6, 1.0, 9))
    ax.set_xticks(np.linspace(0, 30, 7))
    plt.grid(True)
    plt.box(False)
    plt.subplots_adjust(bottom=0.15, left=0.08, right=0.97, top=0.97)
    plt.show()