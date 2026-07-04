#!/usr/bin/env julia

if length(ARGS) < 2
    error(
        "usage: export_trace.jl <PowerSimulationsDynamics.jl repo> <case> <output.csv> <signal>\n" *
        "   or: export_trace.jl <PowerSimulationsDynamics.jl repo> --batch <case>|<signal>=<output.csv>...",
    )
end

const PSD_REPO = abspath(ARGS[1])
const TEST_FILES_DIR = joinpath(PSD_REPO, "test")

import Pkg
Pkg.activate(TEST_FILES_DIR; io = devnull)
Pkg.develop(Pkg.PackageSpec(path = PSD_REPO); io = devnull)
Pkg.instantiate(; io = devnull)

pushfirst!(LOAD_PATH, TEST_FILES_DIR)

using PowerSimulationsDynamics
using PowerSystems
using PowerSystemCaseBuilder
using PowerFlows
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
        elseif quantity == "eq_p"
            return get_state_series(results, (ref, :eq_p))
        elseif quantity == "ed_p"
            return get_state_series(results, (ref, :ed_p))
        elseif quantity == "eq_pp"
            return get_state_series(results, (ref, :eq_pp))
        elseif quantity == "ed_pp"
            return get_state_series(results, (ref, :ed_pp))
        elseif quantity == "psi_kd"
            return get_state_series(results, (ref, :ψ_kd))
        elseif quantity == "psi_kq"
            return get_state_series(results, (ref, :ψ_kq))
        elseif quantity == "psiq_pp" || quantity == "psi_q_pp"
            return get_state_series(results, (ref, :ψq_pp))
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

function one_done_q_fault_ybus(sys)
    fault_branches =
        filter(x -> get_name(x) != "BUS 1-BUS 3-i_1", collect(get_components(Branch, sys)))
    sorted_buses = sort(collect(get_components(ACBus, sys)); by = x -> get_number(x))
    return PNM.Ybus(fault_branches, sorted_buses)[:, :]
end

function run_onedoneq()
    sys = build_system(PSIDTestSystems, "psid_test_threebus_oneDoneQ")
    pf = ACPowerFlow()
    solve_powerflow!(pf, sys)
    ybus_fault = one_done_q_fault_ybus(sys)
    ybus_change = NetworkSwitch(1.0, ybus_fault)
    work = mktempdir()
    try
        sim = Simulation(
            ResidualModel,
            sys,
            work,
            (0.0, 2.0),
            ybus_change,
        )
        status = execute!(sim, IDA(); dtmax = 0.005, saveat = 0.005)
        status == PSID.SIMULATION_FINALIZED || error("PSD simulation did not finalize: $(status)")
        return read_results(sim)
    finally
        rm(work; force = true, recursive = true)
    end
end

function run_simple_marconato()
    sys = build_system(PSIDTestSystems, "psid_test_threebus_simple_marconato")
    pf = ACPowerFlow()
    solve_powerflow!(pf, sys)
    ybus_fault = one_done_q_fault_ybus(sys)
    ybus_change = NetworkSwitch(1.0, ybus_fault)
    work = mktempdir()
    try
        sim = Simulation!(
            ResidualModel,
            sys,
            work,
            (0.0, 2.0),
            ybus_change,
        )
        status = execute!(sim, IDA(); dtmax = 0.005, saveat = 0.005)
        status == PSID.SIMULATION_FINALIZED || error("PSD simulation did not finalize: $(status)")
        return read_results(sim)
    finally
        rm(work; force = true, recursive = true)
    end
end

function run_psse_machine_case(folder::String, dyr_name::String)
    raw_file = joinpath(TEST_FILES_DIR, "benchmarks", "psse", folder, "ThreeBusMulti.raw")
    dyr_file = joinpath(TEST_FILES_DIR, "benchmarks", "psse", folder, dyr_name)
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

function run_genrou()
    return run_psse_machine_case("GENROU", "ThreeBus_GENROU.dyr")
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

function run_case(case_name::String)
    if case_name == "onedoneq" || case_name == "test02"
        run_onedoneq()
    elseif case_name == "simple_marconato" || case_name == "test03"
        run_simple_marconato()
    elseif case_name == "genrou"
        run_genrou()
    elseif case_name == "genroe" || case_name == "test16"
        run_psse_machine_case("GENROE", "ThreeBus_GENROE.dyr")
    elseif case_name == "genroe_high_sat" || case_name == "test16_high_sat"
        run_psse_machine_case("GENROE", "ThreeBus_GENROE_HIGH_SAT.dyr")
    elseif case_name == "gensal" || case_name == "test18"
        run_psse_machine_case("GENSAL", "ThreeBus_GENSAL.dyr")
    elseif case_name == "gensae" || case_name == "test19"
        run_psse_machine_case("GENSAE", "ThreeBus_GENSAE.dyr")
    elseif case_name == "zip_constant_power"
        run_zip_constant_power()
    elseif case_name in ("test24", "gridfollowing_reduced", "test51", "gridfollowing_kaura")
        run_gridfollowing(case_name)
    else
        error("unsupported PSD validation case: $(case_name)")
    end
end

function split_once(value::String, sep::String)
    parts = split(value, sep; limit = 2)
    length(parts) == 2 || error("expected '<left>$(sep)<right>', got $(value)")
    return String(parts[1]), String(parts[2])
end

function export_case_signals(case_name::String, requests::Vector{Tuple{String, String}})
    results = run_case(case_name)
    for (signal, out_csv) in requests
        t, y = export_signal(results, signal)
        write_series(out_csv, t, y)
    end
end

function export_batch(args)
    batches = Dict{String, Vector{Tuple{String, String}}}()
    case_order = String[]
    for item in args
        lhs, out_csv = split_once(String(item), "=")
        case_name, signal = split_once(lhs, "|")
        if !haskey(batches, case_name)
            batches[case_name] = Tuple{String, String}[]
            push!(case_order, case_name)
        end
        push!(batches[case_name], (signal, out_csv))
    end
    for case_name in case_order
        export_case_signals(case_name, batches[case_name])
    end
end

if ARGS[2] == "--batch"
    length(ARGS) >= 3 ||
        error("batch mode requires at least one '<case>|<signal>=<output.csv>' request")
    export_batch(ARGS[3:end])
elseif length(ARGS) >= 4 && ARGS[3] == "--batch"
    case_name = ARGS[2]
    requests = Tuple{String, String}[]
    for item in ARGS[4:end]
        signal, out_csv = split_once(String(item), "=")
        push!(requests, (signal, out_csv))
    end
    export_case_signals(case_name, requests)
elseif length(ARGS) >= 4
    results = run_case(ARGS[2])
    t, y = export_signal(results, ARGS[4])
    write_series(ARGS[3], t, y)
else
    error("usage: export_trace.jl <PowerSimulationsDynamics.jl repo> <case> <output.csv> <signal>")
end
