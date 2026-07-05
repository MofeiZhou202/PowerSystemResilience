#!/usr/bin/env julia

if length(ARGS) != 3
    error(
        "usage: export_internal_diagnostics.jl " *
        "<PowerSimulationsDynamics.jl repo> <case> <output-dir>",
    )
end

const PSD_REPO = abspath(ARGS[1])
const CASE_NAME = ARGS[2]
const OUT_DIR = abspath(ARGS[3])
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
using LinearAlgebra
using SparseArrays
using Printf

const PSID = PowerSimulationsDynamics
const PSY = PowerSystems

include(joinpath(TEST_FILES_DIR, "utils", "get_results.jl"))
include(joinpath(TEST_FILES_DIR, "utils", "data_utils.jl"))
include(joinpath(TEST_FILES_DIR, "data_tests", "dynamic_test_data.jl"))

function build_case_system(case_name::String)
    if case_name == "simple_marconato"
        sys = build_system(PSIDTestSystems, "psid_test_threebus_simple_marconato")
        pf = ACPowerFlow()
        solve_powerflow!(pf, sys)
        return sys
    end
    error("unsupported PSD internal diagnostics case $(case_name)")
end

function state_name(name)
    raw = String(name)
    raw == "δ" && return "delta"
    raw == "ω" && return "omega"
    return raw
end

function sorted_bus_numbers(inputs)
    lookup = PSID.get_lookup(inputs)
    pairs = sort(collect(lookup); by = item -> item.second)
    return [item.first for item in pairs]
end

function sorted_state_labels(inputs)
    state_map = PSID.make_global_state_map(inputs)
    labels = Vector{String}(undef, PSID.get_variable_count(inputs))
    for (device, device_index) in state_map
        for (state, ix) in device_index
            labels[ix] = string(device, ":", state_name(state))
        end
    end
    buses = sorted_bus_numbers(inputs)
    for (k, bus) in enumerate(buses)
        labels[k] = string("bus", bus, ":Vr")
        labels[length(buses) + k] = string("bus", bus, ":Vi")
    end
    return labels
end

function write_metadata(path, pairs)
    open(path, "w") do io
        println(io, "key,value")
        for (key, value) in pairs
            println(io, string(key, ",", value))
        end
    end
end

function write_state_table(path, labels, x0, residual, mass_diag, dae_vector)
    open(path, "w") do io
        println(io, "index,label,value,residual,mass_diag,is_differential")
        for ix in eachindex(x0)
            @printf(
                io,
                "%d,%s,%.17g,%.17g,%.17g,%d\n",
                ix,
                labels[ix],
                x0[ix],
                residual[ix],
                mass_diag[ix],
                dae_vector[ix] ? 1 : 0,
            )
        end
    end
end

function write_vector(path, values)
    open(path, "w") do io
        println(io, "index,value")
        for ix in eachindex(values)
            @printf(io, "%d,%.17g\n", ix, values[ix])
        end
    end
end

function write_matrix(path, matrix)
    open(path, "w") do io
        for row in 1:size(matrix, 1)
            for col in 1:size(matrix, 2)
                col > 1 && print(io, ",")
                @printf(io, "%.17g", matrix[row, col])
            end
            println(io)
        end
    end
end

function write_eigenvalues(path, eigs)
    open(path, "w") do io
        println(io, "index,real,imag")
        for ix in eachindex(eigs)
            @printf(io, "%d,%.17g,%.17g\n", ix, real(eigs[ix]), imag(eigs[ix]))
        end
    end
end

function export_internal_diagnostics(case_name::String, out_dir::String)
    mkpath(out_dir)
    sys = build_case_system(case_name)
    work = mktempdir()
    try
        sim = Simulation!(ResidualModel, sys, work, (0.0, 2.0))
        inputs = PSID.get_simulation_inputs(sim)
        x0 = PSID.get_initial_conditions(sim)
        n = length(x0)
        residual = zeros(n)
        dx0 = zeros(n)
        model = ResidualModel(inputs, x0, PSID.SimCache)
        model(residual, dx0, x0, nothing, 0.0)

        mass_diag = collect(diag(PSID.get_mass_matrix(inputs)))
        dae_vector = PSID.get_DAE_vector(inputs)
        jacobian_wrapper = PSID.get_jacobian(ResidualModel, inputs, x0, 0)
        jacobian = Matrix(jacobian_wrapper.Jv)
        small_signal = small_signal_analysis(sim)
        reduced_jacobian = Matrix(small_signal.reduced_jacobian)
        eigs = small_signal.eigenvalues
        labels = sorted_state_labels(inputs)

        write_metadata(
            joinpath(out_dir, "metadata.csv"),
            [
                ("case", case_name),
                ("formulation", "ResidualModel"),
                ("variable_count", n),
                ("bus_count", PSID.get_bus_count(inputs)),
                ("algebraic_count", count(!, dae_vector)),
                ("differential_count", count(identity, dae_vector)),
                ("residual_inf_norm", norm(residual, Inf)),
                ("jacobian_rows", size(jacobian, 1)),
                ("jacobian_cols", size(jacobian, 2)),
                ("jacobian_inf_norm", norm(jacobian, Inf)),
                ("reduced_jacobian_rows", size(reduced_jacobian, 1)),
                ("reduced_jacobian_cols", size(reduced_jacobian, 2)),
                ("reduced_jacobian_inf_norm", norm(reduced_jacobian, Inf)),
                ("eigenvalue_count", length(eigs)),
                ("small_signal_stable", small_signal.stable ? 1 : 0),
            ],
        )
        write_state_table(joinpath(out_dir, "state_table.csv"),
                          labels,
                          x0,
                          residual,
                          mass_diag,
                          dae_vector)
        write_vector(joinpath(out_dir, "mass_diag.csv"), mass_diag)
        write_vector(joinpath(out_dir, "residual.csv"), residual)
        write_matrix(joinpath(out_dir, "jacobian.csv"), jacobian)
        write_matrix(joinpath(out_dir, "reduced_jacobian.csv"), reduced_jacobian)
        write_eigenvalues(joinpath(out_dir, "eigenvalues.csv"), eigs)
    finally
        rm(work; force = true, recursive = true)
    end
end

export_internal_diagnostics(CASE_NAME, OUT_DIR)
