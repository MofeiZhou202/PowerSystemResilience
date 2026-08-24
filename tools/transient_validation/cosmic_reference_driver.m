function cosmic_reference_driver(cosmic_root, output_json)
% Run the authors' COSMIC IEEE 9-bus relay examples without modifying COSMIC.
% Song et al., IEEE TPWRS 31(3), 2016, Fig. 2 and Sec. II-B/C.

matlab_dir = fullfile(cosmic_root, 'matlab');
data_dir = fullfile(cosmic_root, 'data');
numerics_dir = fullfile(cosmic_root, 'numerics');
required = {fullfile(matlab_dir, 'simgrid.m'), ...
            fullfile(data_dir, 'case9_ps.m'), ...
            fullfile(numerics_dir, 'solve_dae.m')};
for i = 1:numel(required)
    if ~isfile(required{i})
        error('COSMIC source file not found: %s', required{i});
    end
end
addpath(matlab_dir, data_dir, numerics_dir);

report.schema = 'hacdcpf.dynamics.cosmic_reference.v1';
report.cosmic_expected_commit = ...
    '6acc77e4d3f17925f1f4b79a93652eef0d1314cc';
report.reference = ...
    'Song et al., IEEE TPWRS 31(3), 2016, DOI 10.1109/TPWRS.2015.2439237';
report.public_example = run_case(6, 0.90, 15, 'cosmic_public_example');
report.paper_figure_2 = run_case(7, 0.92, 15, 'cosmic_paper_figure_2');

encoded = jsonencode(report, PrettyPrint=true);
fid = fopen(output_json, 'w');
if fid < 0
    error('Cannot open COSMIC report output: %s', output_json);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%s\n', encoded);
end

function result = run_case(initial_branch, uvls_limit, t_max, label)
C = psconstants;
ps = updateps(case9_ps);
ps.branch(:, C.br.tap) = 1;
ps.shunt(:, C.sh.frac_S) = 1;
ps.shunt(:, C.sh.frac_E) = 0;
ps.shunt(:, C.sh.frac_Z) = 0;
ps.shunt(:, C.sh.gamma) = 0.08;
rateB_rateA = ps.branch(:, C.br.rateB) ./ ps.branch(:, C.br.rateA);
rateC_rateA = ps.branch(:, C.br.rateC) ./ ps.branch(:, C.br.rateA);
ps.branch(rateB_rateA == 1, C.br.rateB) = ...
    1.1 * ps.branch(rateB_rateA == 1, C.br.rateA);
ps.branch(rateC_rateA == 1, C.br.rateC) = ...
    1.5 * ps.branch(rateC_rateA == 1, C.br.rateA);

opt = psoptions;
opt.sim.integration_scheme = 1;
opt.sim.dt_default = 1 / 30;
opt.nr.use_fsolve = true;
opt.verbose = false;
opt.sim.draw = false;
opt.sim.writelog = true;
opt.sim.gen_control = 1;
opt.sim.angle_ref = 0;
opt.sim.COI_weight = 0;
opt.sim.uvls_limit = uvls_limit;
opt.sim.uvls_tdelay_ini = 0.5;
opt.sim.ufls_tdelay_ini = 0.5;
opt.sim.dist_tdelay_ini = 0.5;
opt.sim.temp_tdelay_ini = 0;

ps = newpf(ps, opt);
[ps.Ybus, ps.Yf, ps.Yt] = getYbus(ps, false);
ps = update_load_freq_source(ps);
[ps.mac, ps.exc, ps.gov] = get_mac_state(ps, 'salient');
ps.relay = get_relays(ps, 'all', opt);

global t_delay t_prev_check dist2threshold state_a
n = size(ps.bus, 1);
ng = size(ps.mac, 1);
m = size(ps.branch, 1);
n_sh = size(ps.shunt, 1);
ix = get_indices(n, ng, m, n_sh, opt);
t_delay = inf(size(ps.relay, 1), 1);
t_delay([ix.re.uvls]) = opt.sim.uvls_tdelay_ini;
t_delay([ix.re.ufls]) = opt.sim.ufls_tdelay_ini;
t_delay([ix.re.dist]) = opt.sim.dist_tdelay_ini;
t_delay([ix.re.temp]) = opt.sim.temp_tdelay_ini;
t_prev_check = nan(size(ps.relay, 1), 1);
dist2threshold = inf(size(ix.re.oc, 2) * 2, 1);
state_a = zeros(size(ix.re.oc, 2) * 2, 1);

event = zeros(3, C.ev.cols);
event(1, [C.ev.time C.ev.type]) = [0 C.ev.start];
event(2, [C.ev.time C.ev.type]) = [10 C.ev.trip_branch];
event(2, C.ev.branch_loc) = initial_branch;
event(3, [C.ev.time C.ev.type]) = [t_max C.ev.finish];

[outputs, ps] = simgrid(ps, event, label, opt);
[t, ~, ~, ~, ~, Vmag, theta] = read_outfile(outputs.outfilename, ps, opt);
V = Vmag .* exp(1i * theta);
If = (ps.Yf * V.').';
from = ps.bus_i(ps.branch(:, C.br.from));
y_apparent = abs(If) ./ Vmag(:, from);
distance_threshold = ps.relay(ix.re.dist, C.re.threshold);

result.success = logical(outputs.success);
result.initial_branch = initial_branch;
result.uvls_limit_pu = uvls_limit;
result.event_record = outputs.event_record;
result.demand_lost_mw = outputs.demand_lost;
result.bus5_min_voltage_pu = min(Vmag(:, 5));
result.branch6_max_distance_pickup_ratio = ...
    max(y_apparent(:, 6)) / distance_threshold(6);
result.sample_count = numel(t);
delete(outputs.outfilename);
end
