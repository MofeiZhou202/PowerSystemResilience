#!/usr/bin/env julia

if length(ARGS) < 3
    error("usage: export_psd_snapshot.jl <PowerSimulationsDynamics.jl repo> <case|system.json|raw> <output.json> [dyr]")
end

const PSD_REPO = abspath(ARGS[1])
const INPUT_REF = ARGS[2]
const OUT_JSON = ARGS[3]
const OPTIONAL_DYR = length(ARGS) >= 4 ? ARGS[4] : ""
const TEST_FILES_DIR = joinpath(PSD_REPO, "test")

import Pkg
Pkg.activate(TEST_FILES_DIR; io = devnull)
Pkg.develop(Pkg.PackageSpec(path = PSD_REPO); io = devnull)
Pkg.instantiate(; io = devnull)

pushfirst!(LOAD_PATH, TEST_FILES_DIR)

using Dates
using PowerSimulationsDynamics
using PowerSystems
using PowerSystemCaseBuilder

const PSY = PowerSystems

include(joinpath(TEST_FILES_DIR, "utils", "data_utils.jl"))
include(joinpath(TEST_FILES_DIR, "data_tests", "dynamic_test_data.jl"))

short_type(x) = string(nameof(typeof(x)))

function json_escape(s::AbstractString)
    out = IOBuffer()
    for c in s
        if c == '"'
            print(out, "\\\"")
        elseif c == '\\'
            print(out, "\\\\")
        elseif c == '\b'
            print(out, "\\b")
        elseif c == '\f'
            print(out, "\\f")
        elseif c == '\n'
            print(out, "\\n")
        elseif c == '\r'
            print(out, "\\r")
        elseif c == '\t'
            print(out, "\\t")
        elseif Int(c) < 0x20
            print(out, "\\u", lpad(string(Int(c), base = 16), 4, '0'))
        else
            print(out, c)
        end
    end
    return String(take!(out))
end

function write_json_value(io::IO, x, indent::Int = 0)
    pad = repeat(" ", indent)
    next_pad = repeat(" ", indent + 2)
    if x === nothing
        print(io, "null")
    elseif x isa Bool
        print(io, x ? "true" : "false")
    elseif x isa Integer || x isa AbstractFloat
        print(io, isfinite(float(x)) ? string(x) : "null")
    elseif x isa AbstractString
        print(io, '"', json_escape(x), '"')
    elseif x isa Symbol
        print(io, '"', json_escape(String(x)), '"')
    elseif x isa AbstractVector || x isa Tuple
        if isempty(x)
            print(io, "[]")
        else
            println(io, "[")
            for (i, item) in enumerate(x)
                print(io, next_pad)
                write_json_value(io, item, indent + 2)
                i == length(x) || print(io, ",")
                println(io)
            end
            print(io, pad, "]")
        end
    elseif x isa AbstractDict
        keys_sorted = sort(collect(keys(x)); by = string)
        if isempty(keys_sorted)
            print(io, "{}")
        else
            println(io, "{")
            for (i, key) in enumerate(keys_sorted)
                print(io, next_pad, '"', json_escape(String(key)), "\": ")
                write_json_value(io, x[key], indent + 2)
                i == length(keys_sorted) || print(io, ",")
                println(io)
            end
            print(io, pad, "}")
        end
    else
        print(io, '"', json_escape(string(x)), '"')
    end
end

function safe_name(x)
    try
        return String(PSY.get_name(x))
    catch
        return nothing
    end
end

function safe_number(x)
    try
        return PSY.get_number(x)
    catch
        return nothing
    end
end

function safe_base_power(x)
    try
        return PSY.get_base_power(x)
    catch
        return nothing
    end
end

function safe_n_states(x)
    try
        return PSY.get_n_states(x)
    catch
        return nothing
    end
end

function safe_states(x)
    try
        return [String(s) for s in PSY.get_states(x)]
    catch
        return String[]
    end
end

function scalarize(x; depth = 0)
    x === nothing && return nothing
    if x isa Number || x isa Bool || x isa AbstractString
        return x
    elseif x isa Symbol
        return String(x)
    elseif x isa AbstractVector || x isa Tuple
        length(x) > 64 && return Dict("type" => string(typeof(x)), "length" => length(x))
        return [scalarize(v; depth = depth + 1) for v in x]
    elseif x isa AbstractDict
        return Dict(String(k) => scalarize(v; depth = depth + 1) for (k, v) in x)
    end

    name = safe_name(x)
    if name !== nothing && depth > 0
        item = Dict("ref_type" => short_type(x), "name" => name)
        number = safe_number(x)
        number !== nothing && (item["number"] = number)
        return item
    end

    if depth >= 2
        return string(x)
    end

    fields = fieldnames(typeof(x))
    isempty(fields) && return string(x)
    out = Dict{String, Any}("type" => short_type(x))
    for f in fields
        key = String(f)
        key in ("internal", "time_series_container") && continue
        value = try
            getfield(x, f)
        catch
            continue
        end
        out[key] = scalarize(value; depth = depth + 1)
    end
    return out
end

function field_map(x)
    out = Dict{String, Any}()
    for f in fieldnames(typeof(x))
        key = String(f)
        key in ("internal", "time_series_container") && continue
        value = try
            getfield(x, f)
        catch
            continue
        end
        out[key] = scalarize(value; depth = 1)
    end
    return out
end

function component_record(x)
    out = Dict{String, Any}(
        "type" => short_type(x),
        "fields" => field_map(x),
    )
    name = safe_name(x)
    name !== nothing && (out["name"] = name)
    number = safe_number(x)
    number !== nothing && (out["number"] = number)
    base_power = safe_base_power(x)
    base_power !== nothing && (out["base_power_mva"] = base_power)
    states = safe_states(x)
    !isempty(states) && (out["states"] = states)
    n_states = safe_n_states(x)
    n_states !== nothing && (out["n_states"] = n_states)
    return out
end

function maybe_slot(slot_name::String, getter, device)
    value = try
        getter(device)
    catch
        nothing
    end
    value === nothing && return nothing
    rec = component_record(value)
    rec["slot"] = slot_name
    return rec
end

function dynamic_slots(device)
    slots = Any[]
    for item in (
        maybe_slot("machine", PSY.get_machine, device),
        maybe_slot("shaft", PSY.get_shaft, device),
        maybe_slot("avr", PSY.get_avr, device),
        maybe_slot("governor", PSY.get_prime_mover, device),
        maybe_slot("pss", PSY.get_pss, device),
        maybe_slot("converter", PSY.get_converter, device),
        maybe_slot("outer_control", PSY.get_outer_control, device),
        maybe_slot("inner_control", PSY.get_inner_control, device),
        maybe_slot("dc_source", PSY.get_dc_source, device),
        maybe_slot("frequency_estimator", PSY.get_freq_estimator, device),
        maybe_slot("filter", PSY.get_filter, device),
        maybe_slot("limiter", PSY.get_limiter, device),
    )
        item === nothing || push!(slots, item)
    end
    return slots
end

function slot_type(slots, name::String)
    for slot in slots
        if get(slot, "slot", "") == name
            return get(slot, "type", "")
        end
    end
    return ""
end

function hacdcpf_candidate(device_type::String, slots)
    if device_type == "DynamicGenerator"
        machine = slot_type(slots, "machine")
        if machine == "BaseMachine"
            return Dict("model_name" => "ClassicalMachine", "confidence" => "subset")
        elseif occursin("RoundRotor", machine)
            return Dict("model_name" => "GENROU", "confidence" => "profile")
        end
    elseif device_type == "DynamicInverter"
        pll = slot_type(slots, "frequency_estimator")
        outer = slot_type(slots, "outer_control")
        if pll in ("ReducedOrderPLL", "KauraPLL", "FixedFrequency")
            return Dict(
                "model_name" => "REGC_REEC_GFL_Subset",
                "components" => [Dict("type" => "pll", "model" => pll)],
                "confidence" => "subset",
            )
        elseif occursin("Droop", outer)
            return Dict("model_name" => "GridFormingNortonDroop", "confidence" => "subset")
        end
    end
    return Dict("model_name" => "", "confidence" => "metadata-only")
end

function dynamic_record(device)
    rec = component_record(device)
    slots = dynamic_slots(device)
    rec["slots"] = slots
    rec["hacdcpf_dynamic_profile_candidate"] = hacdcpf_candidate(get(rec, "type", ""), slots)
    return rec
end

function collect_records(sys, component_type)
    try
        return [component_record(c) for c in PSY.get_components(component_type, sys)]
    catch
        return Any[]
    end
end

function collect_dynamic_records(sys)
    try
        return [dynamic_record(c) for c in PSY.get_components(PSY.DynamicInjection, sys)]
    catch
        return Any[]
    end
end

function build_named_case(case_name::String)
    if case_name == "genrou"
        raw_file = joinpath(TEST_FILES_DIR, "benchmarks", "psse", "GENROU", "ThreeBusMulti.raw")
        dyr_file = joinpath(TEST_FILES_DIR, "benchmarks", "psse", "GENROU", "ThreeBus_GENROU.dyr")
        sys = System(raw_file, dyr_file)
        for l in PSY.get_components(PSY.StandardLoad, sys)
            transform_load_to_constant_impedance(l)
        end
        return sys, Dict("kind" => "psd_test_case", "case" => case_name, "raw" => raw_file, "dyr" => dyr_file)
    elseif case_name == "zip_constant_power"
        raw_file = joinpath(TEST_FILES_DIR, "benchmarks", "psse", "LOAD", "ThreeBusMulti.raw")
        dyr_file = joinpath(TEST_FILES_DIR, "benchmarks", "psse", "LOAD", "ThreeBus_GENROU.dyr")
        sys = System(raw_file, dyr_file)
        for l in PSY.get_components(PSY.StandardLoad, sys)
            transform_load_to_constant_power(l)
        end
        return sys, Dict("kind" => "psd_test_case", "case" => case_name, "raw" => raw_file, "dyr" => dyr_file)
    elseif case_name == "test24" || case_name == "gridfollowing_reduced"
        include(joinpath(TEST_FILES_DIR, "data_tests", "test24.jl"))
        return omib_sys, Dict("kind" => "psd_test_case", "case" => case_name)
    elseif case_name == "test51" || case_name == "gridfollowing_kaura"
        include(joinpath(TEST_FILES_DIR, "data_tests", "test51.jl"))
        return omib_sys, Dict("kind" => "psd_test_case", "case" => case_name)
    end
    return nothing, nothing
end

function build_input_system(input_ref::String, optional_dyr::String)
    sys, source = build_named_case(input_ref)
    sys !== nothing && return sys, source

    input_path = abspath(input_ref)
    if !isfile(input_path)
        error("unknown PSD case or input file: $(input_ref)")
    end
    if !isempty(optional_dyr)
        dyr_path = abspath(optional_dyr)
        sys = System(input_path, dyr_path)
        return sys, Dict("kind" => "raw_dyr", "raw" => input_path, "dyr" => dyr_path)
    elseif endswith(lowercase(input_path), ".json")
        sys = System(input_path)
        return sys, Dict("kind" => "powersystems_json", "path" => input_path)
    else
        sys = System(input_path; runchecks = false)
        return sys, Dict("kind" => "static_network_file", "path" => input_path)
    end
end

function system_value(sys, field::Symbol)
    try
        return getfield(sys, field)
    catch
        return nothing
    end
end

sys, source = build_input_system(INPUT_REF, OPTIONAL_DYR)
dynamic_injections = collect_dynamic_records(sys)

components = Dict(
    "buses" => collect_records(sys, PSY.ACBus),
    "branches" => collect_records(sys, PSY.Branch),
    "static_injections" => collect_records(sys, PSY.StaticInjection),
    "dynamic_injections" => dynamic_injections,
    "loads" => collect_records(sys, PSY.StaticLoad),
)

snapshot = Dict(
    "format" => "hacdcpf_psd_snapshot.v1",
    "generated_at" => string(now(UTC)),
    "source" => source,
    "system" => Dict(
        "name" => try
            String(PSY.get_name(sys))
        catch
            string(system_value(sys, :metadata))
        end,
        "base_power_mva" => system_value(sys, :base_power),
        "frequency_hz" => system_value(sys, :frequency),
        "component_counts" => Dict(k => length(v) for (k, v) in components),
    ),
    "components" => components,
    "conversion_notes" => [
        "This snapshot preserves PSD/PowerSystems component and controller identity for HACDCPF comparison.",
        "It is not a full network import contract; use it as the dynamic-profile bridge and benchmark manifest.",
        "Use export_trace.jl for PSD output trace conversion to the common time_s,value CSV format.",
    ],
)

mkpath(dirname(abspath(OUT_JSON)))
open(OUT_JSON, "w") do io
    write_json_value(io, snapshot)
    write(io, "\n")
end
