from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos

def export_spring_model() -> AcadosModel:
    #making a spring model
    model_name = 'spring_ode'

    # constants
    m = 20 # mass
    k = 2 # spring constant
    c = 4 # damper constant

    # set up states & controls
    x_1      = SX.sym('x')
    v_1      = SX.sym('v')

    x = vertcat(x_1, v_1)

    u = SX.sym('u')
    u = vertcat(u)

    # xdot
    x_dot      = SX.sym('x_dot')
    v_dot      = SX.sym('v_dot')

    xdot = vertcat(x_dot, v_dot)

    # dynamics
    f_expl = vertcat(v_1, -(k/m)*x_1 - (c/m)*v_1 + u/m)
    f_impl = xdot - f_expl # equal to 0
    model = AcadosModel()
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl

    model.x = x
    model.xdot = xdot
    model.u = u
    model.name = model_name

    return model

if __name__ == "__main__":
    export_spring_model()