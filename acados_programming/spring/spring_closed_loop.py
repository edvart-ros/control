from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from spring_model import export_spring_model
from utils import plot_spring
import numpy as np
import scipy.linalg

#using the example for pendulum_closed_loop to make a spring_closed_loop

def main():
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()

    # set model
    model = export_spring_model()
    ocp.model = model

    Tf = 1.0
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu 
    ny_e = nx 
    N_horizon = 20

    # set dimensions
    ocp.dims.N = N_horizon

    # set cost module
    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'

    Q_mat = 2*np.diag([1e3, 1e3]) #punishing the cost of the spring position and velocity
    R_mat = 2*np.diag([1e-2]) #punishing the control effort

    ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat) 

    ocp.cost.W_e = Q_mat 

    ocp.cost.Vx = np.zeros((ny, nx)) 
    ocp.cost.Vx[:nx,:nx] = np.eye(nx)

    Vu = np.zeros((ny, nu))
    Vu[2,0] = 1.0
    ocp.cost.Vu = Vu

    ocp.cost.Vx_e = np.eye(nx)

    position_setpoint = 2
    velocity_setpoint = 0
    ocp.cost.yref = np.array([position_setpoint, velocity_setpoint, 0]) # cost
    ocp.cost.yref_e = np.array([position_setpoint, velocity_setpoint])  # Terminal cost

    # set constraints
    Fmax = 100
    x0 = np.array([0.0, 0.0])
    ocp.constraints.constr_type = 'BGH'
    ocp.constraints.lbu = np.array([-Fmax])
    ocp.constraints.ubu = np.array([+Fmax])
    ocp.constraints.x0 = x0
    ocp.constraints.idxbu = np.array([0])

    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM' # FULL_CONDENSING_QPOASES
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP' # SQP_RTI

    ocp.solver_options.qp_solver_cond_N = N_horizon

    # set prediction horizon
    ocp.solver_options.tf = Tf

    solver_json = 'acados_ocp_' + model.name + '.json'
    acados_ocp_solver = AcadosOcpSolver(ocp, json_file = solver_json)

    # create an integrator with the same settings as used in the OCP solver.
    acados_integrator = AcadosSimSolver(ocp, json_file = solver_json)

    Nsim = 200
    simX = np.ndarray((Nsim+1, nx))
    simU = np.ndarray((Nsim, nu))

    simX[0,:] = x0

    # closed loop
    for i in range(Nsim):

        # solve ocp and get next control input
        simU[i,:] = acados_ocp_solver.solve_for_x0(x0_bar = simX[i, :])

        # simulate system
        simX[i+1, :] = acados_integrator.simulate(x=simX[i, :], u=simU[i,:])

    # plot results
    plot_spring(np.linspace(0, Tf/N_horizon*Nsim, Nsim+1), Fmax, simU, simX)


if __name__ == '__main__':
    main()