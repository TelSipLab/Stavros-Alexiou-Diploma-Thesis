function [obstacles, obst_params] = obstacle_params(ref_fun)

% obstacle locations defined by time instants on the reference trajectory
[~, traj_params] = ref_fun(0);
t_obst1 = traj_params.Tstr/2;
t_obst2 = traj_params.Tstr + traj_params.Tt/2;
t_obst3 = traj_params.Tstr + traj_params.Tt + traj_params.Tstr/2;
t_obst4 = traj_params.Tstr + traj_params.Tt + traj_params.Tstr + traj_params.Tt/2;
t_obst = [t_obst1; t_obst2; t_obst3; t_obst4];

% obstacle avoidance parameters
obst_params.r_obst = 2;   % obstacle radius
obst_params.r_min = 3;    % minimum avoidance distance
obst_params.dd = 200;     % detection distance

% struct prealloc for obstacles
obstacles = repmat(struct( ...
    'pos', zeros(3,1), ...
    't', 0), 1, numel(t_obst));

% obstacle values
for i = 1:numel(t_obst)
    ref = ref_fun(t_obst(i));
    obstacles(i).pos = ref(1:3);
    obstacles(i).t = t_obst(i);
end
end
