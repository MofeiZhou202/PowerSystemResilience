#!/usr/bin/env julia

if length(ARGS) < 3
    error("usage: export_gridfollowing_trace.jl <PowerSimulationsDynamics.jl repo> <test24|test51> <output.csv> [state_symbol]")
end

const PSD_REPO = abspath(ARGS[1])
const CASE_NAME = ARGS[2]
const OUT_CSV = ARGS[3]
const STATE_SYMBOL = length(ARGS) >= 4 ? Symbol(ARGS[4]) : :p_oc
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

const PSID = PowerSimulationsDynamics
const PSY = PowerSystems

include(joinpath(TEST_FILES_DIR, "utils", "get_results.jl"))
include(joinpath(TEST_FILES_DIR, "utils", "data_utils.jl"))
include(joinpath(TEST_FILES_DIR, "data_tests", "dynamic_test_data.jl"))

if CASE_NAME == "test24" || CASE_NAME == "gridfollowing_reduced"
    include(joinpath(TEST_FILES_DIR, "data_tests", "test24.jl"))
elseif CASE_NAME == "test51" || CASE_NAME == "gridfollowing_kaura"
    include(joinpath(TEST_FILES_DIR, "data_tests", "test51.jl"))
else
    error("unsupported PSD grid-following case: $(CASE_NAME)")
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
    status = execute!(sim, Sundials.IDA(); dtmax = 0.001, saveat = 0.005)
    status == PSID.SIMULATION_FINALIZED || error("PSD simulation did not finalize: $(status)")
    results = read_results(sim)
    t, y = get_state_series(results, ("generator-102-1", STATE_SYMBOL))
    open(OUT_CSV, "w") do io
        println(io, "time_s,value")
        for i in eachindex(t)
            println(io, string(t[i], ",", y[i]))
        end
    end
finally
    rm(work; force = true, recursive = true)
end
