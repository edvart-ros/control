from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, blockcat
from scipy.linalg import block_diag

def export_vessel_mpc_model() -> AcadosModel:
    # making a model for control of a vessel
    model_name = 'vessel_ode'

    #intertia matrix entries
    m_11 = 50.05
    m_22 = 84.36
    m_33 = 17.21

    #damper matrix entries
    Xu = 151.57
    Yv = 132.5
    Nr = 34.5

    # constructing intertia matrix
    M_inv = SX.zeros(3,3)
    M_inv[0,0], M_inv[1,1], M_inv[2,2] = 1/m_11, 1/m_22, 1/m_33

    # Constructing damping matrix
    D = SX.zeros(3,3)
    D[0,0], D[1,1], D[2,2] = Xu, Yv, Nr

    # set up states & controls
    x_      = SX.sym('x_1')
    y_      = SX.sym('y_1')
    psi_    = SX.sym('psi_1')
    u_      = SX.sym('u_1')
    v_      = SX.sym('v_1')
    r_      = SX.sym('r_1')

    x_dot      = SX.sym('x_dot')
    y_dot      = SX.sym('y_dot')
    psi_dot    = SX.sym('psi_dot')
    u_dot      = SX.sym('u_dot')
    v_dot      = SX.sym('v_dot')
    r_dot      = SX.sym('r_dot')

    xdot = vertcat(x_dot, y_dot, psi_dot, u_dot, v_dot, r_dot)

    x = vertcat(x_, y_, psi_, u_, v_, r_)
    tau_u = SX.sym('tau_u')
    tau_v = SX.sym('tau_v')
    tau_r = SX.sym('tau_r')
    tau = vertcat(tau_u, tau_v, tau_r)

    # construct rotation matrix from body to inertial frame
    J = SX.zeros(3,3)
    J[0,0], J[0,1], J[0,2] =  cos(psi_), -sin(psi_), 0
    J[1,0], J[1,1], J[1,2] =  sin(psi_), cos(psi_),  0
    J[2,0], J[2,1], J[2,2] =  0,            0,       1

    # dynamics
    A = blockcat(SX.zeros(3,3), J, -SX.zeros(3, 3), -M_inv@D)
    B = vertcat(SX.zeros(3,3), M_inv)

    # model in non-linear state space form
    f_expl = A@x + B@tau
    f_impl = xdot - f_expl
    model = AcadosModel()
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl

    model.x = x
    model.xdot = xdot
    model.u = tau
    model.name = model_name

    return model

if __name__ == "__main__":
    export_vessel_mpc_model()