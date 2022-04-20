using LibSerialPort
using DelimitedFiles

function record_data(filename; serial_port_name="/dev/cu.usbmodem92225501", data_points=30000)
    # Settup serial port
    sp = LibSerialPort.open(serial_port_name, 115200)
    close(sp)
    dataBuffer = zeros(UInt8, 4 * 6 * data_points)

    # Record data into data buffer
    start_time = time()
    open(sp)
    read!(sp, dataBuffer)
    close(sp)
    total_time = time() - start_time

    # Reinterpret
    tmp = split(String(dataBuffer), "End")
    num_measurements = length(tmp)
    float_data = map(_x->reinterpret(Float32, Vector{UInt8}(_x)), tmp[2:end-1])
    times = Vector(range(0, total_time; length=num_measurements))[2:end-1]

    # Write IMU data to file
    open(joinpath(@__DIR__, filename), "w") do io
        writedlm(io, [["Time", "Acc_X", "Acc_Y", "Acc_Z", "GYR_X", "GYR_Y", "GYR_Z"]], ",\t")

        for i in 1:length(times)
            writedlm(io, [[times[i]; float_data[i]]], ",\t")
        end
    end

    # return append!.(times, float_data)
end

logrange(x1, x2, n; base=10.0) = (base^y for y in range(x1, x2, length=n))

function computeAllanDev(freq::Float64, time_series::AbstractVector)
    dt = 1/freq
    N = length(time_series)
    M_max = 2^floor(log2(N / 2))
    M = logrange(log10(1), log10(M_max), 100)
    M = ceil.(M)
    M = unique(M)
    taus = M * dt
    allan_variance = zeros(length(M))

    for (i, mi) in enumerate(M)
        mi = Int(mi)
        twoMi = 2 * mi

        allan_variance[i] = sum((time_series[twoMi:end] - (2.0 * time_series[mi:(end-mi)]) + time_series[1:(end-(twoMi-1))]).^2)
    end

    allan_variance .= allan_variance ./ ((2.0 * taus.^2) .* (N .- 2.0 * M))
    return (taus, sqrt.(allan_variance))
end
