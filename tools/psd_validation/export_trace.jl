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
        elseif quantity == "psi_q"
            return get_state_series(results, (ref, :ψq))
        elseif quantity == "psi_d"
            return get_state_series(results, (ref, :ψd))
        elseif quantity == "psi_kd"
            return get_state_series(results, (ref, :ψ_kd))
        elseif quantity == "psi_kq"
            return get_state_series(results, (ref, :ψ_kq))
        elseif quantity == "psiq_pp" || quantity == "psi_q_pp"
            return get_state_series(results, (ref, :ψq_pp))
        elseif quantity == "frequency_pu"
            return get_frequency_series(results, ref)
        elseif quantity == "omega_oc_pu"
            return get_state_series(results, (ref, :ω_oc))
        elseif quantity == "theta_oc_rad"
            return get_state_series(results, (ref, :θ_oc))
        elseif quantity == "E_oc" || quantity == "E_oc_pu"
            return get_state_series(results, (ref, :E_oc))
        elseif quantity == "p_pu"
            return get_activepower_series(results, ref)
        elseif quantity == "q_pu"
            return get_reactivepower_series(results, ref)
        elseif quantity == "p_oc"
            return get_state_series(results, (ref, :p_oc))
        elseif quantity == "Vt" || quantity == "internal_voltage_pu"
            return get_state_series(results, (ref, :Vt))
        elseif quantity == "theta_rad" || quantity == "theta_t_rad"
            return get_state_series(results, (ref, :θt))
        elseif quantity == "field_voltage_pu" || quantity == "vf_pu"
            return get_field_voltage_series(results, ref)
        elseif quantity == "mechanical_torque_pu" || quantity == "tau_m_pu"
            return get_mechanical_torque_series(results, ref)
        elseif quantity == "pss_output_pu" || quantity == "vs_pu"
            return get_pss_output_series(results, ref)
        elseif quantity == "delta_hp_rad"
            return get_state_series(results, (ref, :δ_hp))
        elseif quantity == "delta_ip_rad"
            return get_state_series(results, (ref, :δ_ip))
        elseif quantity == "delta_ex_rad"
            return get_state_series(results, (ref, :δ_ex))
        elseif quantity == "omega_hp_pu"
            return get_state_series(results, (ref, :ω_hp))
        elseif quantity == "omega_ip_pu"
            return get_state_series(results, (ref, :ω_ip))
        elseif quantity == "omega_ex_pu"
            return get_state_series(results, (ref, :ω_ex))
        end
    end
    error("unsupported PSD export signal $(signal)")
end

function run_periodic_variable_source()
    include(joinpath(TEST_FILES_DIR, "data_tests", "test28.jl"))
    work = mktempdir()
    try
        sim = Simulation!(
            ResidualModel,
            sys,
            work,
            (0.0, 1.0),
        )
        status = execute!(sim, IDA(); dtmax = 0.01, saveat = 0.01)
        status == PSID.SIMULATION_FINALIZED || error("PSD simulation did not finalize: $(status)")
        return read_results(sim)
    finally
        rm(work; force = true, recursive = true)
    end
end

function run_test01_omib()
    sys = build_system(PSIDTestSystems, "psid_test_omib")
    fault_branch = deepcopy(collect(get_components(Branch, sys))[1])
    fault_branch.r = 0.0
    fault_branch.x = 0.1
    ybus_fault = PNM.Ybus([fault_branch], collect(get_components(ACBus, sys)))[:, :]
    ybus_change = NetworkSwitch(1.0, ybus_fault)
    work = mktempdir()
    try
        sim = Simulation(
            ResidualModel,
            sys,
            work,
            (0.0, 20.0),
            ybus_change,
        )
        status = execute!(sim, IDA(); dtmax = 0.005, saveat = 0.005)
        status == PSID.SIMULATION_FINALIZED || error("PSD simulation did not finalize: $(status)")
        return read_results(sim)
    finally
        rm(work; force = true, recursive = true)
    end
end

function run_test41_stab1()
    raw_file = joinpath(TEST_FILES_DIR, "benchmarks", "psse", "STAB1", "OMIB_SSS.raw")
    dyr_file = joinpath(TEST_FILES_DIR, "benchmarks", "psse", "STAB1", "OMIB_SSS.dyr")
    sys = System(raw_file, dyr_file)
    for l in get_components(PSY.StandardLoad, sys)
        transform_load_to_constant_impedance(l)
    end
    gen = first(get_components(Generator, sys))
    dynamic_injector = get_dynamic_injector(gen)
    for g in get_components(Generator, sys)
        if get_number(get_bus(g)) == 1
            gen = g
            dynamic_injector = get_dynamic_injector(g)
        end
    end
    perturbation = ControlReferenceChange(1.0, dynamic_injector, :V_ref, 1.0472)
    work = mktempdir()
    try
        sim = Simulation(
            ResidualModel,
            sys,
            work,
            (0.0, 20.0),
            perturbation,
        )
        status = execute!(sim, IDA(); dtmax = 0.005, saveat = 0.005)
        status == PSID.SIMULATION_FINALIZED || error("PSD simulation did not finalize: $(status)")
        return read_results(sim)
    finally
        rm(work; force = true, recursive = true)
    end
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

function run_marconato()
    sys = build_system(PSIDTestSystems, "psid_test_threebus_marconato")
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

function run_simple_af()
    sys = build_system(PSIDTestSystems, "psid_test_threebus_simple_anderson")
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

function run_anderson_fouad()
    sys = build_system(PSIDTestSystems, "psid_test_threebus_anderson")
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

function run_five_mass_shaft()
    sys = build_system(PSIDTestSystems, "psid_test_threebus_5shaft")
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
        status = execute!(sim, IDA(); dtmax = 0.001, saveat = 0.005)
        status == PSID.SIMULATION_FINALIZED || error("PSD simulation did not finalize: $(status)")
        return read_results(sim)
    finally
        rm(work; force = true, recursive = true)
    end
end

function run_test13_avrs()
    include(joinpath(TEST_FILES_DIR, "data_tests", "test13.jl"))
    ybus_change = NetworkSwitch(1.0, Ybus_fault)
    work = mktempdir()
    try
        sim = Simulation(
            ResidualModel,
            threebus_sys,
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

function run_test12_multimachine()
    include(joinpath(TEST_FILES_DIR, "data_tests", "test12.jl"))
    ybus_change = NetworkSwitch(1.0, Ybus_fault)
    work = mktempdir()
    try
        sim = Simulation(
            ResidualModel,
            threebus_sys,
            work,
            (0.0, 5.0),
            ybus_change,
        )
        status = execute!(sim, IDA(); dtmax = 0.005, saveat = 0.005)
        status == PSID.SIMULATION_FINALIZED || error("PSD simulation did not finalize: $(status)")
        return read_results(sim)
    finally
        rm(work; force = true, recursive = true)
    end
end

function run_test17_avrtype1()
    include(joinpath(TEST_FILES_DIR, "data_tests", "test17.jl"))
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

function run_psse_machine_case(
    folder::String,
    dyr_name::String;
    t_end::Float64 = 2.0,
    dtmax::Float64 = 0.005,
    saveat::Float64 = 0.005,
)
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
            (0.0, t_end),
            BranchTrip(1.0, Line, "BUS 1-BUS 2-i_1"),
        )
        status = execute!(sim, IDA(); dtmax = dtmax, saveat = saveat)
        status == PSID.SIMULATION_FINALIZED || error("PSD simulation did not finalize: $(status)")
        return read_results(sim)
    finally
        rm(work; force = true, recursive = true)
    end
end

function genrou_machine_pss2()
    return PSY.RoundRotorQuadratic(;
        R = 0.0,
        Td0_p = 7.0,
        Td0_pp = 999.0,
        Tq0_p = 0.4,
        Tq0_pp = 999.0,
        Xd = 2.20,
        Xq = 2.20,
        Xd_p = 0.30,
        Xq_p = 0.30,
        Xd_pp = 0.30,
        Xl = 0.2,
        Se = (0.0, 0.0),
    )
end

function single_mass_shaft_pss2()
    return PSY.SingleMass(; H = 4.00, D = 0.00)
end

function sexs_avr_pss2()
    return PSY.SEXS(;
        Ta_Tb = 1.0,
        Tb = 1.0,
        K = 120.0,
        Te = 0.4,
        V_lim = (min = -10.0, max = 10.0),
    )
end

function pss2_model(kind::String)
    common = (;
        input_code_1 = 1,
        remote_bus_control_1 = 0,
        input_code_2 = 3,
        remote_bus_control_2 = 0,
        M_rtf = 5,
        N_rtf = 1,
        Tw1 = 1.5,
        Tw2 = 1.5,
        T6 = 0.0,
        Tw3 = 1.5,
        Tw4 = 0.0,
        T7 = 1.5,
        Ks2 = 0.1875,
        Ks3 = 1.0,
        T8 = 0.5,
        T9 = 0.1,
        Ks1 = 2.0,
        T1 = 0.59451,
        T2 = 0.0447,
        T3 = 0.59451,
        T4 = 0.0447,
    )
    if kind == "PSS2A"
        return PSY.PSS2A(; common..., Vst_lim = (-0.1, 0.1))
    elseif kind == "PSS2B"
        return PSY.PSS2B(;
            common...,
            T10 = 1.0,
            T11 = 1.0,
            Vs1_lim = (-0.00055, 0.00035),
            Vs2_lim = (0.895, 0.915),
            Vst_lim = (-0.1, 0.1),
        )
    elseif kind == "PSS2C"
        return PSY.PSS2C(;
            common...,
            T10 = 1.0,
            T11 = 1.0,
            Vs1_lim = (-0.00055, 0.00035),
            Vs2_lim = (0.895, 0.915),
            Vst_lim = (-0.1, 0.1),
            T12 = 1.0,
            T13 = 1.0,
            PSS_Hysteresis_param = (0.885, 0.895),
            Xcomp = 1.0,
            Tcomp = 1.0,
        )
    end
    error("unsupported PSS2 kind $(kind)")
end

function pss2_csv_folder(kind::String)
    if kind == "PSS2A"
        return "PSS2A"
    elseif kind == "PSS2B"
        return "PSS2B"
    elseif kind == "PSS2C"
        return "PSS2C"
    end
    error("unsupported PSS2 kind $(kind)")
end

function get_gen_by_bus_number(system, number)
    for gen in get_components(Generator, system)
        if get_number(get_bus(gen)) == number
            return gen
        end
    end
    error("no generator at bus $(number)")
end

function run_pss2_case(kind::String)
    raw_file = joinpath(TEST_FILES_DIR, "benchmarks", "psse", "PSS2A", "OMIB.raw")
    dyr_file = joinpath(TEST_FILES_DIR, "benchmarks", "psse", "PSS2A", "OMIB_GENCLS.dyr")
    sys = System(raw_file, dyr_file)
    for l in get_components(PSY.StandardLoad, sys)
        transform_load_to_constant_impedance(l)
    end
    g = get_gen_by_bus_number(sys, 1)
    dynamic_injector = get_dynamic_injector(g)
    remove_component!(sys, dynamic_injector)
    dyn_gen = DynamicGenerator(;
        name = get_name(g),
        machine = genrou_machine_pss2(),
        shaft = single_mass_shaft_pss2(),
        avr = sexs_avr_pss2(),
        prime_mover = PSY.TGFixed(; efficiency = 1.0),
        ω_ref = 1.0,
        pss = pss2_model(kind),
    )
    add_component!(sys, dyn_gen, g)
    perturbation = ControlReferenceChange(1.0, dyn_gen, :V_ref, 1.04691)
    work = mktempdir()
    try
        sim = Simulation(
            ResidualModel,
            sys,
            work,
            (0.0, 2.0),
            perturbation,
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

function run_vsm_inverter()
    sys = build_system(PSIDTestSystems, "psid_test_vsm_inverter")
    case_inv = collect(PSY.get_components(PSY.DynamicInjection, sys))[1]
    pref_change = ControlReferenceChange(1.0, case_inv, :P_ref, 0.7)
    work = mktempdir()
    try
        sim = Simulation(
            ResidualModel,
            sys,
            work,
            (0.0, 4.0),
            pref_change,
        )
        status = execute!(sim, IDA(); dtmax = 0.005, saveat = 0.005)
        status == PSID.SIMULATION_FINALIZED || error("PSD simulation did not finalize: $(status)")
        return read_results(sim)
    finally
        rm(work; force = true, recursive = true)
    end
end

function run_droop_inverter()
    sys = build_system(PSIDTestSystems, "psid_test_droop_inverter")
    case_inv = collect(PSY.get_components(PSY.DynamicInjection, sys))[1]
    pref_change = ControlReferenceChange(1.0, case_inv, :P_ref, 0.7)
    work = mktempdir()
    try
        sim = Simulation!(
            ResidualModel,
            sys,
            work,
            (0.0, 4.0),
            pref_change,
        )
        status = execute!(sim, IDA(); dtmax = 0.005, saveat = 0.005)
        status == PSID.SIMULATION_FINALIZED || error("PSD simulation did not finalize: $(status)")
        return read_results(sim)
    finally
        rm(work; force = true, recursive = true)
    end
end

function run_voc_inverter()
    include(joinpath(TEST_FILES_DIR, "data_tests", "test44.jl"))
    pref_change = ControlReferenceChange(1.0, case_inv, :P_ref, 0.7)
    work = mktempdir()
    try
        sim = Simulation(
            ResidualModel,
            omib_sys,
            work,
            (0.0, 4.0),
            pref_change,
        )
        status = execute!(sim, IDA(); dtmax = 0.005, saveat = 0.005)
        status == PSID.SIMULATION_FINALIZED || error("PSD simulation did not finalize: $(status)")
        return read_results(sim)
    finally
        rm(work; force = true, recursive = true)
    end
end

function run_test25_dynamic_lines()
    include(joinpath(TEST_FILES_DIR, "data_tests", "test25.jl"))
    gen2 = get_dynamic_injector(get_component(Generator, sys, "generator-102-1"))
    pref_change = ControlReferenceChange(1.0, gen2, :P_ref, 0.9)
    work = mktempdir()
    try
        sim = Simulation!(
            ResidualModel,
            sys,
            work,
            (0.0, 2.0),
            pref_change,
        )
        status = execute!(sim, IDA(); dtmax = 0.01, saveat = 0.01)
        status == PSID.SIMULATION_FINALIZED || error("PSD simulation did not finalize: $(status)")
        return read_results(sim)
    finally
        rm(work; force = true, recursive = true)
    end
end

function run_test49_csvgn1()
    raw_file = joinpath(TEST_FILES_DIR, "benchmarks", "psse", "CSVGN1", "3_BUS_System.raw")
    dyr_file = joinpath(TEST_FILES_DIR, "benchmarks", "psse", "CSVGN1", "3_BUS_System.dyr")
    sys = System(raw_file, dyr_file)
    for l in get_components(PSY.StandardLoad, sys)
        transform_load_to_constant_impedance(l)
    end

    bus_3 = first(b for b in get_components(ACBus, sys) if get_number(b) == 3)
    csvgn1_source = Source(;
        name = "CSVGN1",
        available = true,
        active_power = 0.0,
        reactive_power = 0.0,
        bus = bus_3,
        R_th = 0.0,
        X_th = 0.0,
    )
    add_component!(sys, csvgn1_source)
    set_bustype!(bus_3, 2)
    for source in get_components(Source, sys)
        if get_number(get_bus(source)) != 3
            continue
        end
        dynamic_injector = PSY.CSVGN1(;
            name = get_name(source),
            K = 20.0,
            T1 = 0.0,
            T2 = 1.0,
            T3 = 0.154833,
            T4 = 1.0,
            T5 = 0.005167,
            Rmin = 0.0,
            Vmax = 1.0,
            Vmin = 0.0,
            CBase = 60.0,
            base_power = 500.0,
        )
        set_dynamic_injector!(source, dynamic_injector)
    end

    load21 = first(l for l in get_components(PSY.StandardLoad, sys) if get_name(l) == "load21")
    load_change = LoadChange(0.005, load21, :P_ref_impedance, 400 / get_base_power(sys))
    work = mktempdir()
    try
        sim = Simulation(
            ResidualModel,
            sys,
            work,
            (0.0, 0.07),
            load_change,
        )
        status = execute!(sim, IDA(); dtmax = 0.0001, saveat = 0.0001)
        status == PSID.SIMULATION_FINALIZED || error("PSD simulation did not finalize: $(status)")
        return read_results(sim)
    finally
        rm(work; force = true, recursive = true)
    end
end

function run_case(case_name::String)
    if case_name == "omib" || case_name == "test01"
        run_test01_omib()
    elseif case_name == "onedoneq" || case_name == "test02"
        run_onedoneq()
    elseif case_name == "simple_marconato" || case_name == "test03"
        run_simple_marconato()
    elseif case_name == "marconato" || case_name == "test04"
        run_marconato()
    elseif case_name == "simple_af" || case_name == "simple_anderson" || case_name == "test05"
        run_simple_af()
    elseif case_name == "anderson_fouad" || case_name == "anderson" || case_name == "test06"
        run_anderson_fouad()
    elseif case_name == "five_mass_shaft" || case_name == "test07"
        run_five_mass_shaft()
    elseif case_name == "test12" || case_name == "multimachine_tgtype2"
        run_test12_multimachine()
    elseif case_name == "test13" || case_name == "onedoneq_avr_tg"
        run_test13_avrs()
    elseif case_name == "test17" || case_name == "genrou_avrtype1"
        run_test17_avrtype1()
    elseif case_name == "test20" || case_name == "esac1a" || case_name == "ac1a"
        run_psse_machine_case("AC1A", "ThreeBus_ESAC1A.dyr")
    elseif case_name == "test21" || case_name == "gast"
        run_psse_machine_case("GAST", "ThreeBus_GAST.dyr")
    elseif case_name == "test22" || case_name == "tgov1"
        run_psse_machine_case("TGOV1", "ThreeBus_TGOV1.dyr")
    elseif case_name == "test31" || case_name == "hygov"
        run_psse_machine_case("HYGOV", "ThreeBus_HYGOV.dyr")
    elseif case_name == "test47" || case_name == "scrx"
        run_psse_machine_case("SCRX", "ThreeBus_SCRX.dyr")
    elseif case_name == "test52" || case_name == "pss2a"
        run_pss2_case("PSS2A")
    elseif case_name == "test53" || case_name == "pss2b"
        run_pss2_case("PSS2B")
    elseif case_name == "test54" || case_name == "pss2c"
        run_pss2_case("PSS2C")
    elseif case_name == "genrou"
        run_genrou()
    elseif case_name == "sexs" || case_name == "test26"
        run_psse_machine_case("SEXS", "ThreeBus_SEXS.dyr")
    elseif case_name == "ieeest" || case_name == "test30"
        run_psse_machine_case("IEEEST", "ThreeBus_IEEEST_with_filter.dyr")
    elseif case_name == "stab1" || case_name == "test41"
        run_test41_stab1()
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
    elseif case_name == "test08" || case_name == "vsm_inverter"
        run_vsm_inverter()
    elseif case_name == "test23" || case_name == "droop_inverter"
        run_droop_inverter()
    elseif case_name == "test44" || case_name == "voc_inverter"
        run_voc_inverter()
    elseif case_name == "test25" || case_name == "dynamic_lines_test25"
        run_test25_dynamic_lines()
    elseif case_name == "test28" || case_name == "periodic_variable_source"
        run_periodic_variable_source()
    elseif case_name == "test49" || case_name == "csvgn1"
        run_test49_csvgn1()
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
