function [obstacles, obst_params] = obstacle_params(ref_fun)

% obstacles and uav geometry
obst_params.r_uav = 1;     % UAV collision radius
obst_params.r_obst = 2;    % obstacle radius
obst_params.safety_margin = 0.1;  % safety margin
obst_params.r_min = obst_params.r_uav + obst_params.r_obst ... 
    + obst_params.safety_margin;  % minimum avoidance distance

% barrier functions parameters
obst_params.ddet = 200;    % detection distance [m]
obst_params.d0_bar = 200;  % obstacle influence distance [m]
obst_params.Qo_BF = 30;    % barrier function weight (not used)
obst_params.Qo_APF = 0.05; % artificial potential field weight
obst_params.Ko_BF = 1;     % contractive barrier function weight
obst_params.obstacles_active_until = 220; % obstacle avoidance active time [s]

% obstacle locations defined by time instants on the reference trajectory
[~, traj_params] = ref_fun(0);
t_obst1 = traj_params.Tstr/2;
t_obst2 = traj_params.Tstr + traj_params.Tt/2;
t_obst3 = traj_params.Tstr + traj_params.Tt + traj_params.Tstr/2;
t_obst4 = traj_params.Tstr + traj_params.Tt + traj_params.Tstr + traj_params.Tt/2;
t_obst = [t_obst1; t_obst2; t_obst3; t_obst4];
active_obstacle_ids = [1 2 3 4];

% obstacle offset from reference trajectory
obstacle_offsets = zeros(3, numel(t_obst));
obstacle_offsets(:,1) = [0; -1; -0.5];
obstacle_offsets(:,2) = [0; 0; 0];
obstacle_offsets(:,3) = [0; -2; -1.5];
obstacle_offsets(:,4) = [0; 0; 0];

% struct prealloc for obstacles
obstacles = repmat(struct( ...
    'pos', zeros(3,1), ...
    't', 0, ...
    'id', 0), 1, numel(t_obst));

% obstacle values
for i = 1:numel(t_obst)
    ref = ref_fun(t_obst(i));
    obstacles(i).pos = ref(1:3) + obstacle_offsets(:,i);
    obstacles(i).t = t_obst(i);
    obstacles(i).id = i;
end

obstacles = obstacles(active_obstacle_ids);
end
