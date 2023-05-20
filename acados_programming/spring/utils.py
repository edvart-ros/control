import os
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

def latexify_plot():
    params = {'backend': 'ps',
            'text.latex.preamble': r"\usepackage{gensymb} \usepackage{amsmath}",
            'axes.labelsize': 10,
            'axes.titlesize': 10,
            'legend.fontsize': 10,
            'xtick.labelsize': 10,
            'ytick.labelsize': 10,
            'text.usetex': True,
            'font.family': 'serif'
    }

    matplotlib.rcParams.update(params)

def plot_spring(shooting_nodes, u_max, U, X_true, latexify=False, plt_show=True, X_true_label=None):
    """
    Params:
        shooting_nodes: time values of the discretization
        u_max: maximum absolute value of u
        U: arrray with shape (N_sim-1, nu) or (N_sim, nu)
        X_true: arrray with shape (N_sim, nx)
        latexify: latex style plots
    """

    # latexify plot
    if latexify:
        latexify_plot()

    nx = X_true.shape[1]
    t = shooting_nodes


    plt.subplot(nx+1, 1, 1)
    line, = plt.step(t, np.append([U[0]], U))
    if X_true_label is not None:
        line.set_label(X_true_label)
    else:
        line.set_color('r')
    plt.title('closed-loop simulation')
    plt.ylabel('$u$')
    plt.xlabel('$t$')
    plt.hlines(u_max, t[0], t[-1], linestyles='dashed', alpha=0.7)
    plt.hlines(-u_max, t[0], t[-1], linestyles='dashed', alpha=0.7)
    plt.ylim([-1.2*u_max, 1.2*u_max])
    plt.grid()

    states_lables = ['$x_1$', '$v_1$']

    for i in range(nx):
        plt.subplot(nx+1, 1, i+2)
        line, = plt.plot(t, X_true[:, i], label='true')
        if X_true_label is not None:
            line.set_label(X_true_label)

        plt.ylabel(states_lables[i])
        plt.xlabel('$t$')
        plt.grid()
        plt.legend(loc=1)

    plt.subplots_adjust(left=None, bottom=None, right=None, top=None, hspace=0.4)

    # avoid plotting when running on Travis
    if os.environ.get('ACADOS_ON_CI') is None and plt_show:
        plt.show()