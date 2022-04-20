"""Error between states x1 and x2"""
function state_error(x2, x1)
    p1 = x1[1:3]
    p2 = x2[1:3]
    q1 = Rotations.UnitQuaternion(x1[4:7])
    q2 = Rotations.UnitQuaternion(x2[4:7])
    all1 = x1[8:end]
    all2 = x2[8:end]

    ori_er = Rotations.rotation_error(q2, q1, Rotations.CayleyMap())

    dx =[p2-p1; ori_er; all2-all1]
    return dx
end

"""Map from state to error state"""
function error_state_jacobian(x)
    # Get various compoents
    q = Rotations.UnitQuaternion(x[4:7])
    # Build error state to state jacobian
    J = zeros(13, 12)
    J[1:3, 1:3] .= I(3)
    J[4:7, 4:6] .= Rotations.∇differential(q)
    J[8:end, 7:end] .= I(6)

    return J
end

function trim_controls()
    kf, bf = quad_motor_kf, quad_motor_bf
    g = 9.81
    m = quad_mass

    thrust = (g * m) / 4.0
    pwm = (thrust - bf) / kf

    return [pwm for _ in 1:4]
end

function lqr(A, B, Q, R; max_iters=200, tol=1e-6)
    P = copy(Q)
    n,m = size(B)
    K = zeros(m,n)
    K_prev = copy(K)

    P = copy(Q)
    for k = 1:max_iters
        K .= (R + B' * P * B) \ (B' * P * A)
        P .= Q + A' * P * A - A' * P * B * K

        if maximum(K - K_prev) < tol
            return K
        end

        K_prev .= K
    end

    return K
end

function generate_LQR_hover_gains(xhover, uhover, dt;
    Qd = 1*ones(12),
    Rd = fill(0.1, 4),
)
    # Setting x₀ hover state
    xhover = copy(xhover)
    uhover = copy(uhover)

    A = ForwardDiff.jacobian(_x->dynamics_rk4(_x, uhover, dt), xhover)
    B = ForwardDiff.jacobian(_u->dynamics_rk4(xhover, _u, dt), uhover)

    E2 = error_state_jacobian(xhover)
    Ã = E2' * A * E2
    B̃ = E2' * B

    # Cost weights
    Q = Array(Diagonal(Qd))
    R = Array(Diagonal(Rd))

    # LQR Gain
    K = lqr(Ã, B̃, Q, R; max_iters=200, tol=1e-6)

    return K, uhover
end

"""Prints out the gain matrix in a the form to initializate an Eigen matrix """
function print_gains(K)
    print("K_gains  << ")
    for i in 1:(size(K)[1] - 1)
        for j in 1:(size(K)[2])
            @printf("%.5f, ", K[i,j])
        end
        print("\n\t")
    end
    for j in 1:(size(K)[2] - 1)
        @printf("%.5f, ", K[end,j])
    end
    @printf("%.5f;", K[end,end])
end
