# %%
using Revise
using DelimitedFiles
import QuadrotorLatencySim as QLS

# %%
start_time = time()
data_file = joinpath(@__DIR__, "data", "imu2.txt")
QLS.record_data(data_file; serial_port_name="/dev/tty.usbmodem92225501", data_points=100000)
@info time() - start_time

# %%
imu_data = readdlm(data_file, ',', Float64; skipstart=1)
times = imu_data[:, 1]

acc_x = imu_data[:, 2]
acc_y = imu_data[:, 3]
acc_z = imu_data[:, 4]

gyr_x = imu_data[:, 5]
gyr_y = imu_data[:, 6]
gyr_z = imu_data[:, 7]

ave_freq = 1 / (sum(diff(times)) / length(times))

# %%
using Plots

taus, all_var_gyr_x = QLS.computeAllanDev(ave_freq, gyr_x)
plt = plot(taus, all_var_gyr_x, xscale=:log10, yscale=:log10,
           label="Gyr x", title="Allan Std Deviation of Gyro",
           xlabel="Observation Period (sec)", ylabel="Std Dev (rad / sec)"
           )
taus, all_var_gyr_y = QLS.computeAllanDev(ave_freq, gyr_y)
plot!(plt, taus, all_var_gyr_y, xscale=:log10, yscale=:log10, label="Gyr y")
taus, all_var_gyr_z = QLS.computeAllanDev(ave_freq, gyr_z)
plot!(plt, taus, all_var_gyr_z, xscale=:log10, yscale=:log10, label="Gyr z")
savefig(plt, joinpath(@__DIR__, "graphics", "Allan_std_dev_gyro.png"))

# %%

imu_data = readdlm(joinpath(@__DIR__, "imu_sys_id", "delim_file.csv"), ',', Float64; skipstart=1)

times = imu_data[:, 1]

acc_x = imu_data[:, 2]
acc_y = imu_data[:, 3]
acc_z = imu_data[:, 4]

gyr_x = imu_data[:, 5]
gyr_y = imu_data[:, 6]
gyr_z = imu_data[:, 7]

# %%
using Plots

taus, all_var_gyr_x = computeAllanDev(177.0, gyr_x)
plt = plot(taus, all_var_gyr_x, xscale=:log10, yscale=:log10,
           label="Gyr x", title="Allan Std Deviation of Gyro",
           xlabel="Observation Period (sec)", ylabel="Std Dev (rad / sec)"
           )
taus, all_var_gyr_y = computeAllanDev(177.0, gyr_y)
plot!(plt, taus, all_var_gyr_y, xscale=:log10, yscale=:log10, label="Gyr y")
taus, all_var_gyr_z = computeAllanDev(177.0, gyr_z)
plot!(plt, taus, all_var_gyr_z, xscale=:log10, yscale=:log10, label="Gyr z")
savefig(plt, joinpath(@__DIR__, "graphics", "Allan_std_dev_gyro.png"))

# %%
taus, all_var_acc_x = computeAllanDev(177.0, acc_x)
plt = plot(taus, all_var_acc_x, xscale=:log10, yscale=:log10,
           label="Acc x", title="Allan Std Deviation of Accelerometer",
           xlabel="Observation Period (sec)", ylabel="Std Dev (rad / sec)"
           )
taus, all_var_acc_y = computeAllanDev(177.0, acc_y)
plot!(plt, taus, all_var_acc_y, xscale=:log10, yscale=:log10, label="Acc y")
taus, all_var_acc_z = computeAllanDev(177.0, acc_z)
plot!(plt, taus, all_var_acc_z, xscale=:log10, yscale=:log10, label="Acc z")
savefig(plt, joinpath(@__DIR__, "graphics", "Allan_std_dev_accel.png"))

# %%
tau1_ind = argmin(abs.(taus .- 1))
σ_bias = [
    all_var_acc_x[tau1_ind],
    all_var_acc_y[tau1_ind],
    all_var_acc_z[tau1_ind],
    all_var_gyr_x[tau1_ind],
    all_var_gyr_y[tau1_ind],
    all_var_gyr_z[tau1_ind]
]