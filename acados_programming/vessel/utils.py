import os 
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

def draw_boat(ax, x, y, psi, length=1.0):
    """Draw a boat represented as a rectangle and a triangle on the given axes."""
    boat_length = length
    boat_width = boat_length / 4

    # define rectangle and triangle representing the boat
    rectangle = plt.Rectangle((x - boat_length / 2, y - boat_width / 2), boat_length, boat_width, fill=True, color="blue")
    triangle = plt.Polygon(((x + boat_length / 2, y - boat_width / 2), (x + boat_length / 2 + boat_length / 3, y), (x + boat_length / 2, y + boat_width / 2)), closed=True, fill=True, color="blue")

    # create a transformation matrix for the boat's orientation and position
    t = matplotlib.transforms.Affine2D().rotate_deg_around(x, y, np.degrees(psi))

    # apply the transformation to the boat
    rectangle.set_transform(t + ax.transData)
    triangle.set_transform(t + ax.transData)

    # add the boat to the plot
    ax.add_patch(rectangle)
    ax.add_patch(triangle)



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
        ylim = u_max[0] * 1.2
        axs[i//2, i%2].set_ylim([-ylim, ylim])
        axs[i//2, i%2].grid()

    for i in range(nx):
        axs[(nu+i)//2, (nu+i)%2].plot(t, X_true[:, i], label='true')
        axs[(nu+i)//2, (nu+i)%2].set_ylabel(states_lables[i])
        axs[(nu+i)//2, (nu+i)%2].set_xlabel('$t$')
        axs[(nu+i)//2, (nu+i)%2].grid()

    # XY position plot
    ax = axs[-1, -1]
    ax.plot(X_true[:, 0], X_true[:, 1], label='XY position')

    # set equal aspect ratio
    ax.set_aspect('equal')

    # draw boats at various points along the trajectory
    for i in range(0, len(X_true), len(X_true)//5):  # draw 5 boats evenly spaced along the trajectory
        draw_boat(ax, X_true[i, 0], X_true[i, 1], X_true[i, 2])

    ax.set_xlabel('$x$')
    ax.set_ylabel('$y$')
    ax.grid()


    plt.tight_layout()  # this will take care of spacing in a grid layout

    # avoid plotting when running on Travis
    if os.environ.get('ACADOS_ON_CI') is None and plt_show:
        plt.show()

