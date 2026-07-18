function matpower_benchmark(caselist_file, out_dir)
%MATPOWER_BENCHMARK Run MATPOWER runpf over a list of case files.
%   For each case: times loadcase and runpf, records success flag, and
%   writes per-case bus voltages (bus_i, vm, va[deg]) plus a summary CSV.
%   Used by tools/matpower_benchmark.py for cross-tool accuracy/speed checks.

if ~exist(out_dir, 'dir'); mkdir(out_dir); end
summary_path = fullfile(out_dir, 'matpower_summary.csv');
sfid = fopen(summary_path, 'w');
fprintf(sfid, 'case,nbus,nbranch,success,loadcase_s,runpf_et_s,runpf_wall_s,note\n');

txt = fileread(caselist_file);
paths = strsplit(strtrim(txt), '\n');
for k = 1:numel(paths)
    p = strtrim(paths{k});
    if isempty(p); continue; end
    [~, name, ~] = fileparts(p);
    note = 'ok'; success = 0; nbus = 0; nbranch = 0;
    load_s = NaN; et_s = NaN; wall_s = NaN;
    try
        t0 = tic; mpc = loadcase(p); load_s = toc(t0);
        nbus = size(mpc.bus, 1); nbranch = size(mpc.branch, 1);
        mpopt = mpoption('verbose', 0, 'out.all', 0);
        t1 = tic; r = runpf(mpc, mpopt); wall_s = toc(t1);
        et_s = r.et;
        success = r.success;
        if success
            bus = r.bus;
            bfid = fopen(fullfile(out_dir, [name '.matpower_bus.csv']), 'w');
            fprintf(bfid, 'bus_i,vm,va_deg\n');
            for i = 1:size(bus, 1)
                fprintf(bfid, '%d,%.12g,%.12g\n', bus(i,1), bus(i,8), bus(i,9));
            end
            fclose(bfid);
        else
            note = 'runpf_not_converged';
        end
    catch ME
        note = ['error: ' regexprep(ME.message, '[\n,]', ' ')];
    end
    fprintf(sfid, '%s,%d,%d,%d,%.6f,%.6f,%.6f,"%s"\n', ...
        name, nbus, nbranch, success, load_s, et_s, wall_s, note);
    fprintf('[%d/%d] %s success=%d et=%.3fs\n', k, numel(paths), name, success, et_s);
end
fclose(sfid);
end
