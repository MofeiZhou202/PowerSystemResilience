#!/usr/bin/env julia

# Independent reduced-model oracle for the case33bw AC/DC propagation driver.
# It intentionally uses only Julia Base: no HySim code or C++ formula is called.
# Literature/model scope is documented in the dynamics and reliability manuals.

length(ARGS) == 1 || error("usage: validate_case33bw_acdc_dc_fault_oracle.jl <study-executable>")
study = ARGS[1]

function read_rows(path)
    lines = readlines(path)
    isempty(lines) && error("empty trajectory CSV")
    header = split(lines[1], ',')
    column = Dict(name => index for (index, name) in enumerate(header))
    required = [
        "scenario", "dt_s", "time_s", "ac18_voltage_pu", "vsc2_p_mw",
        "vsc2_vdc_link_pu", "vsc2_dc_active_power_scale",
        "vsc2_dc_undervoltage_timer_s", "vsc2_dc_undervoltage_blocked",
    ]
    all(haskey(column, name) for name in required) || error("trajectory CSV schema mismatch")
    rows = Vector{Vector{SubString{String}}}()
    for line in lines[2:end]
        fields = split(line, ',')
        length(fields) == length(header) || error("malformed trajectory CSV row")
        push!(rows, fields)
    end
    return rows, column
end

value(row, column, name) = parse(Float64, row[column[name]])
scenario(row, column) = String(row[column["scenario"]])

function active_power_scale(vdc; block=0.75, derate=0.95)
    vdc <= block && return 0.0
    vdc >= derate && return 1.0
    return (vdc - block) / (derate - block)
end

mktempdir() do directory
    report = joinpath(directory, "report.json")
    csv = joinpath(directory, "trajectories.csv")
    run(`$study --smoke --output $report --csv $csv`)
    rows, column = read_rows(csv)

    baseline = filter(row -> scenario(row, column) == "no_fault_baseline", rows)
    legacy = filter(row -> scenario(row, column) == "dc_bus_2_fault_legacy", rows)
    length(baseline) == length(legacy) || error("legacy and baseline grids differ")
    legacy_power_error = maximum(abs(
        value(legacy[k], column, "vsc2_p_mw") -
        value(baseline[k], column, "vsc2_p_mw")) for k in eachindex(legacy))
    legacy_power_error <= 1e-6 || error("legacy identity failed: $legacy_power_error MW")

    block_times = Float64[]
    maximum_scale_error = 0.0
    enhanced_power_deviation = 0.0
    enhanced_voltage_deviation = 0.0
    for dt in (0.001, 0.002, 0.005)
        enhanced = filter(row ->
            scenario(row, column) == "dc_bus_2_fault_enhanced" &&
            isapprox(value(row, column, "dt_s"), dt; atol=1e-12), rows)
        isempty(enhanced) && error("missing enhanced trajectory at dt=$dt")

        first_timed = nothing
        first_blocked = nothing
        for row in enhanced
            vdc = value(row, column, "vsc2_vdc_link_pu")
            scale = value(row, column, "vsc2_dc_active_power_scale")
            timer = value(row, column, "vsc2_dc_undervoltage_timer_s")
            blocked = value(row, column, "vsc2_dc_undervoltage_blocked") > 0.5
            expected = blocked ? 0.0 : active_power_scale(vdc)
            maximum_scale_error = max(maximum_scale_error, abs(scale - expected))
            if first_timed === nothing && timer > 0.0 && !blocked
                first_timed = row
            end
            if first_blocked === nothing && blocked
                first_blocked = row
            end
        end
        first_timed === nothing && error("undervoltage timer never picked up at dt=$dt")
        first_blocked === nothing && error("undervoltage block never operated at dt=$dt")

        pickup_time = value(first_timed, column, "time_s") -
                      value(first_timed, column, "vsc2_dc_undervoltage_timer_s")
        predicted_block_time = pickup_time + 0.02
        observed_block_time = value(first_blocked, column, "time_s")
        abs(observed_block_time - predicted_block_time) <= dt + 1e-9 ||
            error("block timer mismatch at dt=$dt")
        push!(block_times, observed_block_time)

        if isapprox(dt, 0.002; atol=1e-12)
            for row in enhanced
                t = value(row, column, "time_s")
                index = findfirst(base ->
                    isapprox(value(base, column, "time_s"), t; atol=1e-9), baseline)
                index === nothing && continue
                enhanced_power_deviation = max(enhanced_power_deviation, abs(
                    value(row, column, "vsc2_p_mw") -
                    value(baseline[index], column, "vsc2_p_mw")))
                enhanced_voltage_deviation = max(enhanced_voltage_deviation, abs(
                    value(row, column, "ac18_voltage_pu") -
                    value(baseline[index], column, "ac18_voltage_pu")))
            end
        end
    end

    maximum_scale_error <= 1e-12 || error("active-power envelope mismatch: $maximum_scale_error")
    maximum(block_times) - minimum(block_times) <= 0.003 ||
        error("block-time spread exceeds 3 ms")
    enhanced_power_deviation >= 0.05 || error("enhanced AC power response is too small")
    enhanced_voltage_deviation >= 1e-4 || error("enhanced AC voltage response is too small")

    println("Julia oracle passed")
    println("legacy_power_error_mw=", legacy_power_error)
    println("maximum_scale_error=", maximum_scale_error)
    println("block_time_spread_s=", maximum(block_times) - minimum(block_times))
    println("enhanced_power_deviation_mw=", enhanced_power_deviation)
    println("enhanced_voltage_deviation_pu=", enhanced_voltage_deviation)
end
