# # %% Examining Stability of Quad with injected delay on all elements of the state
# dt = 0.001
# Nt = 20000
# K, _ = generate_LQR_hover_gains(HOVER_STATE, HOVER_INPUT, dt;
#                                 Qd=([10,10,10, 1,1,1, 1,1,1, 1,1,1]))
# # Vector of latencies we will simulate with
# latencies = [0.001, 0.003, 0.01, 0.03, 0.1]
# times = 0:dt:((Nt-1)*dt)
# xhist = zeros(length(latencies), Nt, length(HOVER_STATE))
# # Initialize from starting point of the quadrotor
# for i in 1:length(latencies)
#     xhist[i,1,:] .= [+0.0266,  +0.1500, +0.8081,
#                      -0.9983,  +0.0106, +0.0028, -0.0565,
#                      +0.0468,  +0.0188, -0.0305,
#                      -0.0019,  +0.0113, -0.0210]
# end

# # Run a simulation for each latency duration
# for (i, latency) in enumerate(latencies)
#     u = HOVER_INPUT
#     # Circular buffer for delay backlog of size latency duration/step length
#     delay_xhist = CircularBuffer{Vector}(convert(Int, floor(latency / dt)))
#     pushfirst!(delay_xhist, xhist[i,1,:])

#     # Simulate for
#     for k = 1:(Nt-1)
#         if isfull(delay_xhist) # We can start up the delayed input
#             delayed_xhist = pop!(delay_xhist)
#             u = clamp.(HOVER_INPUT - K * state_error(delayed_xhist, HOVER_STATE),
#                        quad_min_throttle, quad_max_throttle)
#         else # No input due to inital delay
#             u = fill(quad_min_throttle, 4)
#             # u =
#         end

#         xhist[i,k+1,:] .= dynamics_rk4(xhist[i,k,:], u, dt)
#         pushfirst!(delay_xhist, xhist[i,k+1,:])
#     end
# end

# function simulate(get_control::Function, x0::AbstractVector, u_default::AbstractVector=zeros(4);
#                   dt::Float64=0.01, num_steps::Int=10000,
#                   latency::float=0*dt, # Default no latency in the measurement
#                   )
#     X_hist = [copy(x0) for i in 1:n+1]
#     U_hist = [zeros(length(x0)) for i in 1:n]

#     # Initialize delay buffer
#     delay_xhist = CircularBuffer{Vector}(Int(floor(latency / dt)) + 1)

#     # Simulate for
#     for k = 1:num_steps
#         pushfirst!(delay_xhist, X_hist[k, :])

#         if isfull(delay_xhist) # We can start up the delayed input
#             delayed_xhist = pop!(delay_xhist)
#             u = get_control(delayed_xhist)
#         else # No input due to inital delay
#             u = u_default
#         end
#         clamp!(u, quad_min_throttle, quad_max_throttle)

#         U_hist[k] = u
#         X_hist[k+1,:] .= dynamics_rk4(X_hist[k,:], u, dt)
#     end
# end


