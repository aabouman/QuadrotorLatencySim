struct ImuState
    bias::AbstractVector
    bias_dist::MvNormal
    measurement_dist::MvNormal
end

function ImuState(bias = randn(6))
    # Accelerometer and gyroscope Allan std devs
    bias_σ = [0.006, 0.006, 0.006,
              0.006, 0.006, 0.006]
    bias_dist = MvNormal(zeros(6), Diagonal(bias_σ .^ 2))
    measurement_σ =  [0.042, 0.042, 0.042,
                      0.042, 0.042, 0.042]
    measurement_dist = MvNormal(zeros(6), Diagonal(measurement_σ .^ 2))

    return ImuState(bias, bias_dist, measurement_dist)
end

"""IMU Process model"""
function imu_process!(imu_state::ImuState, quad_state::T, quad_input::T) where {T <: AbstractVector}
    x_dot = cont_dynamics(quad_state, quad_input)
    v_dot_true = x_dot[8:10]
    ω_true = quad_state[11:13]

    # TODO add bias and noise on the measurement
    imu_measurement = [v_dot_true; ω_true] + imu_state.bias + rand(imu_state.measurement_dist)

    # Bias random walk
    imu_state.bias .+= rand(imu_state.bias_dist)

    return imu_measurement
end

"""Cache struct for IMU Integrator State Estimator"""
struct ImuIntegrationEst


end

"""Simple IMU Integration State Estimator"""
function imu_integration()


end
