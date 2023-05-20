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

def plot_vessel(shooting_nodes, u_max, U, X_true, latexify=False, plt_show=True, X_true_label=None):
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
    nu = U.shape[1]
    t = shooting_nodes

    states_lables = ['$x$', '$y$', '$\psi$', '$u$', '$v$', '$r$']
    control_labels = [f'$u_{i}$' for i in range(nu)]
    
    fig, axs = plt.subplots(nu+2, 2, figsize=(14, 2*(nu+2)))  # modify figure size to 2 columns

    for i in range(nu):
        axs[i//2, i%2].step(t, np.append(U[0, i], U[:, i]))
        axs[i//2, i%2].set_title('vessel simulation')
        axs[i//2, i%2].set_ylabel(control_labels[i])
        axs[i//2, i%2].set_xlabel('$t$')
        axs[i//2, i%2].hlines(u_max, t[0], t[-1], linestyles='dashed', alpha=0.7)
        axs[i//2, i%2].hlines(-u_max, t[0], t[-1], linestyles='dashed', alpha=0.7)
        axs[i//2, i%2].set_ylim([-1.2*u_max, 1.2*u_max])
        axs[i//2, i%2].grid()

    for i in range(nx):
        axs[(nu+i)//2, (nu+i)%2].plot(t, X_true[:, i], label='true')
        axs[(nu+i)//2, (nu+i)%2].set_ylabel(states_lables[i])
        axs[(nu+i)//2, (nu+i)%2].set_xlabel('$t$')
        axs[(nu+i)//2, (nu+i)%2].grid()

    # XY position plot
    axs[-1, -1].plot(X_true[:, 0], X_true[:, 1], label='XY position')
    axs[-1, -1].set_xlabel('$x$')
    axs[-1, -1].set_ylabel('$y$')
    axs[-1, -1].grid()

    plt.tight_layout()  # this will take care of spacing in a grid layout

    # avoid plotting when running on Travis
    if os.environ.get('ACADOS_ON_CI') is None and plt_show:
        plt.show()

