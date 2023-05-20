import os

os.environ['ACADOS_LIB_PATH'] = '/home/edvart/software/acados/lib'
os.environ['ACADOS_INCLUDE_PATH'] = '/home/edvart/software/acados/include'


from acados_template import AcadosSim, AcadosSimSolver
from spring_model import export_spring_model
import numpy as np
from utils import plot_spring


def main():
    sim = AcadosSim()
    model = export_spring_model()

    # set model_name
    sim.model = model

    Tf = 0.1
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    N = 1000

    # set simulation time
    sim.solver_options.T = Tf
    # set options
    sim.solver_options.integrator_type = 'IRK'
    sim.solver_options.num_stages = 3
    sim.solver_options.num_steps = 3
    sim.solver_options.newton_iter = 3 # for implicit integrator
    sim.solver_options.collocation_type = "GAUSS_RADAU_IIA"

    # create
    acados_integrator = AcadosSimSolver(sim)

    simX = np.zeros((N+1, nx)) # create memory for solution
    x0 = np.array([1.0, 0.0]) # initial condition
    u0 = np.array([0.0]) # initial control
    acados_integrator.set("u", u0)

    simX[0,:] = x0 # inform simulator of initial condition

    for i in range(N):
        # set initial state
        acados_integrator.set("x", simX[i,:])
        # initialize IRK
        if sim.solver_options.integrator_type == 'IRK':
            acados_integrator.set("xdot", np.zeros((nx,)))

        # solve
        status = acados_integrator.solve()
        # get solution
        simX[i+1,:] = acados_integrator.get("x")

    if status != 0:
        raise Exception(f'acados returned status {status}.')

    S_forw = acados_integrator.get("S_forw")
    print("S_forw, sensitivities of simulation result wrt x,u:\n", S_forw)

    # plot results
    #plot_pendulum(np.linspace(0, N*Tf, N+1), 10, np.repeat(u0, N), simX, latexify=False)
    plot_spring(np.linspace(0, N*Tf, N+1), 10, np.repeat(u0, N), simX, latexify=False)


if __name__ == "__main__":
    main()