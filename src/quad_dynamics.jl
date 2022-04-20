const quad_mass = 2.0
const quad_inertia = [0.01566089 0.00000318037 0; 0.00000318037 0.01562078 0; 0 0 0.02226868]
const quad_motor_kf = 0.0244101
const quad_motor_bf = -30.48576
const quad_motor_km = 0.00029958
const quad_motor_bm = -0.367697
const quad_arm_len = 0.28
const quad_min_throttle = 1148.0
const quad_max_throttle = 1832.0

"""
* `x` - Quadrotor state
* `u` - Motor PWM commands
"""
function forces(x, u)
    q = Rotations.UnitQuaternion(x[4:7])
    kf = quad_motor_kf
    bf = quad_motor_bf

    Kf = [0.0  0   0   0;
          0.0  0   0   0;
          kf   kf  kf  kf];
    Bf = [0; 0; 4*bf];
    g = [0; 0; -9.81 * quad_mass]

    force = Kf * u + Bf + q' * g
    return force
end

function moments(x, u)
    km = quad_motor_km
    bm = quad_motor_bm
    kf = quad_motor_kf
    bf = quad_motor_bf

    Km = [0.0  0  0   0;
          0.0  0  0   0;
          km  -km km -km];
    # Bm = [0; 0; 4*bm];
    # torque = Km * u + Bm
    torque = Km * u

    ss = normalize.([[1.;1;0], [1.;-1;0], [-1.;-1;0], [-1.;1;0]])
    for (si, ui) in zip(ss, u)
        torque += cross(quad_arm_len * si, [0; 0; kf * ui + bf])
    end

    return torque
end

function cont_dynamics(x, u)
    p = x[1:3]
    q = Rotations.UnitQuaternion(x[4:7])
    v = x[8:10]
    ω = x[11:13]
    m = quad_mass
    J = quad_inertia

    dp = q * v
    dq = Rotations.kinematics(q, ω)
    dv = 1/m * forces(x, u) - cross(ω, v)
    dω = J \ (moments(x, u) - cross(ω, J * ω))

    return [dp; dq; dv; dω]
end

function dynamics_rk4(x, u, dt)
    k1 = cont_dynamics(x, u)
    k2 = cont_dynamics(x + 0.5 * dt * k1, u)
    k3 = cont_dynamics(x + 0.5 * dt * k2, u)
    k4 = cont_dynamics(x + dt * k3, u)
    tmp = x + (dt/6.0) * (k1 + 2*k2 + 2*k3 + k4)

    tmp[4:7] .= Rotations.params(Rotations.UnitQuaternion(tmp[4:7]))

    return tmp
end
