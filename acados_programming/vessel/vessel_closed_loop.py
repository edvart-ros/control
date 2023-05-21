from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver, AcadosSim
from vessel_mpc_model import export_vessel_mpc_model
from utils import plot_vessel
import numpy as np
import scipy.linalg
import time




def main():
    # create optimal control problem object
    ocp = AcadosOcp()
    model = export_vessel_mpc_model()
    ocp.model = model

    Tf = 0.1 # prediction horizon
    nx = model.x.size()[0] # number of states
    nu = model.u.size()[0] # number of control inputs
    ny = nx + nu # number of parameters in the stage cost function
    ny_e = nx # number of parameters in the terminal cost function
    N_horizon = 20 # prediction horizon steps

    # set dimensions
    ocp.dims.N = N_horizon

    # set cost module, here we use the simple and effective linear least squares
    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'

    #formulate cost function
    ocp.cost.Vx = np.vstack((np.eye(nx), np.zeros((nu,nx))))
    ocp.cost.Vx_e = np.eye(nx)
    ocp.cost.Vu = np.vstack((np.zeros((nx,nu)), np.eye(nu)))

    #define stage and terminal reference
    x_setpoint = np.zeros((nx)) #drive states to zero
    u_setpoint = np.zeros((nu)) #drive control inputs to zero
    ocp.cost.yref = np.hstack((x_setpoint, u_setpoint))
    ocp.cost.yref_e = x_setpoint

    # Cost function weights
    # states 
    Qx, Qy, Qpsi, Qu, Qv, Qr = 10, 10, 0, 1, 1, 1
    Q = np.diag([Qx, Qy, Qpsi, Qu,Qv, Qr])
    # control inputs
    R_u, R_v, R_r = 1e-7, 1e-2, 1e-7
    R = np.diag([R_u, R_v, R_r])

    # stage and terminal cost matrices
    W = scipy.linalg.block_diag(Q, R)
    ocp.cost.W = 100000*W #seems like there are numerical issues if cost coefficients are too small
    ocp.cost.W_e = 10000*Q

    # set constraints
    u_max = np.array([200, 200, 200])
    u_min = np.array([-200, -200, -200])
    x0 = np.array([10, -5, 1.57, 0, 0, 0])
    ocp.constraints.constr_type = 'BGH'
    ocp.constraints.lbu = u_min
    ocp.constraints.ubu = u_max
    ocp.constraints.idxbu = np.array([0, 1, 2])
    ocp.constraints.x0 = x0

    # set QP solver and integration
    ocp.solver_options.tf = Tf # prediction horizon
    ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP'
    ocp.solver_options.qp_solver_cond_N = N_horizon

    solver_json = 'acados_ocp_' + model.name + '.json'
    ocp_solver = AcadosOcpSolver(ocp, json_file=solver_json)


    # create simulator
    sim = AcadosSim()

    # set model_name
    sim.model = model

    Tf = 0.01
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    Nsim = 1000

    # set simulation time
    sim.solver_options.T = Tf
    # set options
    sim.solver_options.integrator_type = 'ERK'
    sim.solver_options.num_stages = 3
    sim.solver_options.num_steps = 3
    sim.solver_options.newton_iter = 3 # for implicit integrator
    sim.solver_options.collocation_type = "GAUSS_RADAU_IIA"

    # create
    integrator = AcadosSimSolver(sim)

    simX = np.zeros((Nsim+1, nx)) # create memory for solution
    simU = np.zeros((Nsim, nu)) # create memory for control inputs
    
    simX[0,:] = x0 # inform simulator of initial condition

    flag = True
    # simulate closed-loop
    tik = time.time()
    for i in range(Nsim):
        # solve ocp and get next control input
        simU[i,:] = ocp_solver.solve_for_x0(x0_bar = simX[i, :])
        # simulate system
        simX[i+1, :] = integrator.simulate(x=simX[i, :], u=simU[i,:])
    print("time per iteration: ", (time.time()-tik)/Nsim)
    plot_vessel(np.linspace(0, Tf/N_horizon*Nsim, Nsim+1), u_max, simU, simX)



if __name__ == '__main__':
    main()