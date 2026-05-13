function print_constraint_diagnostics(name, C_log, tk, tol)

if nargin < 4
    tol = 1e-6;
end

C_log = C_log(:);
tk = tk(:);

[max_C, max_idx] = max(C_log);
max_time = tk(max_idx);
violation_mask = C_log > tol;

fprintf('%-8s max = %12.6f at k = %4d, t = %8.3f s', ...
    name, max_C, max_idx, max_time);

if ~any(violation_mask)
    fprintf(' | violations: none\n');
    return;
end

idx = find(violation_mask);
breaks = find(diff(idx) > 1);
range_starts = idx([1; breaks + 1]);
range_ends = idx([breaks; numel(idx)]);

fprintf(' | violations: ');
for i = 1:numel(range_starts)
    if i > 1
        fprintf(', ');
    end
    fprintf('k %d-%d (t %.3f-%.3f s)', ...
        range_starts(i), range_ends(i), tk(range_starts(i)), tk(range_ends(i)));
end
fprintf('\n');

end
