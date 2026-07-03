#!/usr/bin/env julia

if length(ARGS) < 4
    error("usage: export_trace.jl <PowerSimulationsDynamics.jl repo> <case> <output.csv> <signal>")
end

const PSD_REPO = abspath(ARGS[1])
const CASE_NAME = ARGS[2]
const OUT_CSV = ARGS[3]
const SIGNAL = ARGS[4]
const TEST_FILES_DIR = joinpath(PSD_REPO, "test")

import Pkg
Pkg.activate(TEST_FILES_DIR; io = devnull)
Pkg.develop(Pkg.PackageSpec(path = PSD_REPO); io = devnull)
Pkg.instantiate(; io = devnull)

pushfirst!(LOAD_PATH, TEST_FILES_DIR)

using PowerSimulationsDynamics
using PowerSystems
using PowerSystemCaseBuilder
using Sundials
using DelimitedFiles
import LinearAlgebra
import PowerNetworkMatrices

const PSID = PowerSimulationsDynamics
const PSY = PowerSystems
const PNM = PowerNetworkMatrices

include(joinpath(TEST_FILES_DIR, "utils", "get_results.jl"))
include(joinpath(TEST_FILES_DIR, "utils", "data_utils.jl"))
include(joinpath(TEST_FILES_DIR, "data_tests", "dynamic_test_data.jl"))

function write_series(path, t, y)
    open(path, "w") do io
        println(io, "time_s,value")
        for i in eachindex(t)
            println(io, string(t[i], ",", y[i]))
        end
    end
end

function export_signal(results, signal::String)
    parts = split(signal, ":")
    length(parts) == 2 || error("signal must be '<device-or-bus>:<quantity>', got $(signal)")
    ref = String(parts[1])
    quantity = String(parts[2])
    if startswith(ref, "bus")
        bus_number = parse(Int, replace(ref, "bus" => ""))
        if quantity == "voltage_mag"
            return get_voltage_magnitude_series(results, bus_number)
        elseif quantity == "voltage_angle"
            return get_voltage_angle_series(results, bus_number)
        end
    else
        if quantity == "delta_rad"
            return get_state_series(results, (ref, :δ))
        elseif quantity == "delta_deg"
            t, y = get_state_series(results, (ref, :δ))
            return t, y .* 180.0 ./ pi
        elseif quantity == "omega_pu"
            return get_state_series(results, (ref, :ω))
        elseif quantity == "frequency_pu"
            return get_frequency_series(results, ref)
        elseif quantity == "p_pu"
            return get_activepower_series(results, ref)
        elseif quantity == "q_pu"
            return get_reactivepower_series(results, ref)
        elseif quantity == "p_oc"
            return get_state_series(results, (ref, :p_oc))
        end
    end
    error("unsupported PSD export signal $(signal)")
end

function run_genrou()
    raw_file = joinpath(TEST_FILES_DIR, "benchmarks", "psse", "GENROU", "ThreeBusMulti.raw")
    dyr_file = joinpath(TEST_FILES_DIR, "benchmarks", "psse", "GENROU", "ThreeBus_GENROU.dyr")
    sys = System(raw_file, dyr_file)
    for l in get_components(PSY.StandardLoad, sys)
        transform_load_to_constant_impedance(l)
    end
    work = mktempdir()
    try
        sim = Simulation!(
            ResidualModel,
            sys,
            work,
            (0.0, 2.0),
            BranchTrip(1.0, Line, "BUS 1-BUS 2-i_1"),
        )
        status = execute!(sim, IDA(); dtmax = 0.005, saveat = 0.005)
        status == PSID.SIMULATION_FINALIZED || error("PSD simulation did not finalize: $(status)")
        return read_results(sim)
    finally
        rm(work; force = true, recursive = true)
    end
end

function run_zip_constant_power()
    raw_file = joinpath(TEST_FILES_DIR, "benchmarks", "psse", "LOAD", "ThreeBusMulti.raw")
    dyr_file = joinpath(TEST_FILES_DIR, "benchmarks", "psse", "LOAD", "ThreeBus_GENROU.dyr")
    sys = System(raw_file, dyr_file)
    for l in get_components(PSY.StandardLoad, sys)
        transform_load_to_constant_power(l)
    end
    work = mktempdir()
    try
        sim = Simulation!(
            ResidualModel,
            sys,
            work,
            (0.0, 2.0),
            BranchTrip(1.0, Line, "BUS 1-BUS 2-i_1"),
        )
        status = execute!(sim, IDA(); abstol = 1e-9, saveat = 0.005)
        status == PSID.SIMULATION_FINALIZED || error("PSD simulation did not finalize: $(status)")
        return read_results(sim)
    finally
        rm(work; force = true, recursive = true)
    end
end

function run_gridfollowing(case_name::String)
    if case_name == "test24" || case_name == "gridfollowing_reduced"
        include(joinpath(TEST_FILES_DIR, "data_tests", "test24.jl"))
    elseif case_name == "test51" || case_name == "gridfollowing_kaura"
        include(joinpath(TEST_FILES_DIR, "data_tests", "test51.jl"))
    else
        error("unsupported grid-following PSD case: $(case_name)")
    end
    case_inv = collect(PSY.get_components(PSY.DynamicInjection, omib_sys))[1]
    pref_change = ControlReferenceChange(1.0, case_inv, :P_ref, 0.7)
    work = mktempdir()
    try
        sim = Simulation!(
            ResidualModel,
            omib_sys,
            work,
            (0.0, 2.0),
            pref_change,
        )
        status = execute!(sim, IDA(); dtmax = 0.001, saveat = 0.005)
        status == PSID.SIMULATION_FINALIZED || error("PSD simulation did not finalize: $(status)")
        return read_results(sim)
    finally
        rm(work; force = true, recursive = true)
    end
end

results =
    if CASE_NAME == "genrou"
        run_genrou()
    elseif CASE_NAME == "zip_constant_power"
        run_zip_constant_power()
    elseif CASE_NAME in ("test24", "gridfollowing_reduced", "test51", "gridfollowing_kaura")
        run_gridfollowing(CASE_NAME)
    else
        error("unsupported PSD validation case: $(CASE_NAME)")
    end

t, y = export_signal(results, SIGNAL)
write_series(OUT_CSV, t, y)
