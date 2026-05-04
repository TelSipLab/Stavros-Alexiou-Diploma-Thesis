function [obstacles, obst_params] = obstacle_params(ref_fun)

% obstacles and uav geometry
obst_params.r_uav = 1;     % UAV collision radius
obst_params.r_obst = 2;    % obstacle radius
obst_params.safety_margin = 0.1;  % safety margin
obst_params.r_min = obst_params.r_uav + obst_params.r_obst ... 
    + obst_params.safety_margin;  % minimum avoidance distance

% barrier functions parameters
obst_params.ddet = 200;    % detection distance
obst_params.d0_bar = 200;  % obstacle influence distance
obst_params.Qo_APF = 1;    % artificial potential field weight
obst_params.Qo_BF = 30;    % barrier function weight
obst_params.Ko_BF = 0.01;  % contractive barrier function weight

% obstacle locations defined by time instants on the reference trajectory
[~, traj_params] = ref_fun(0);
t_obst1 = traj_params.Tstr/2;
t_obst2 = traj_params.Tstr + traj_params.Tt/2;
t_obst3 = traj_params.Tstr + traj_params.Tt + traj_params.Tstr/2;
t_obst4 = traj_params.Tstr + traj_params.Tt + traj_params.Tstr + traj_params.Tt/2;
t_obst = [t_obst1; t_obst2; t_obst3; t_obst4];

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
