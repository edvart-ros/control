import os
from acados_template import AcadosSim, AcadosSimSolver
from vessel_mpc_model import export_vessel_mpc_model
import numpy as np
from utils import plot_vessel
import time


def main():
    sim = AcadosSim()
    model = export_vessel_mpc_model()

    # set model_name
    sim.model = model

    Tf = 0.01
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
    x0 = np.zeros((nx,)) # initial condition
    u0 = np.array([2000, 0.0, 250]) # initial control
    U = np.tile(u0, (N,1)) # constant control input
    acados_integrator.set("u", u0)

    simX[0,:] = x0 # inform simulator of initial condition

    tik = time.time()

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
    
    print(f"Elapsed time: {time.time()-tik:.3f} s")
    # plot results
    plot_vessel(np.linspace(0, N*Tf, N+1), 10, U, simX, latexify=False)

if __name__ == '__main__':
    main()