function matpower_variant_probe(caselist_file, out_dir)
%MATPOWER_VARIANT_PROBE Run runpf with Q-limit enforcement on/off per case.
%   Writes <case>.matpower_qon_bus.csv / <case>.matpower_qoff_bus.csv plus
%   <case>.matpower_variant_summary.csv rows with gen Q limit hit counts.
%   Used to diagnose HySim vs MATPOWER solution differences.

if ~exist(out_dir, 'dir'); mkdir(out_dir); end
txt = fileread(caselist_file);
paths = strsplit(strtrim(txt), '\n');
for k = 1:numel(paths)
    p = strtrim(paths{k});
    if isempty(p); continue; end
    [~, name, ~] = fileparts(p);
    mpc = loadcase(p);
    for mode = ["qon", "qoff"]
        opt = mpoption('verbose', 0, 'out.all', 0);
        if mode == "qon"
            opt = mpoption(opt, 'pf.enforce_q_lims', 1);
        end
        try
            r = runpf(mpc, opt);
            bus = r.bus;
            bfid = fopen(fullfile(out_dir, ...
                [name '.matpower_' char(mode) '_bus.csv']), 'w');
            fprintf(bfid, 'bus_i,vm,va_deg\n');
            for i = 1:size(bus, 1)
                fprintf(bfid, '%d,%.12g,%.12g\n', bus(i,1), bus(i,8), bus(i,9));
            end
            fclose(bfid);
            % count gens sitting at Q limits
            g = r.gen; nq = 0;
            for i = 1:size(g, 1)
                qg = g(i,3); qmin = g(i,5); qmax = g(i,4);
                if abs(qg - qmax) < 1e-3 || abs(qg - qmin) < 1e-3
                    nq = nq + 1;
                end
            end
            fprintf('%s %s success=%d gens_at_qlim=%d\n', ...
                name, mode, r.success, nq);
        catch ME
            fprintf('%s %s error: %s\n', name, mode, ME.message);
        end
    end
end
end
