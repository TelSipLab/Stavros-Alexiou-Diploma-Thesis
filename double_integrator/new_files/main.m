clc; clear; close all;

%% System Parameters

% MPC Setup
T = 10;
Ts = 0.1;            
N = T / Ts;          
Hp = 10;             
Hc = 5;            

% Discrete-time model (Forward Euler Approximation)
A = [1 Ts; 0 1];
B = [0; Ts];

% Cost weights
Q = eye(2);          
R = 1;               

% Constraints
umin = -1;
umax = 1;

% Bounds for optimization (fmincon)
lb = umin * ones(Hc, 1); % lower bound
ub = umax * ones(Hc, 1); % upper bound

% Initial and target state
x0 = [0; 0]; % initial state
x_target = [10; 0]; % target state

%% System Predictions Test (F.E.)

% Input Control
U = ones(1,Hp);

% Forward Euler Loop (creating the X matrix --> state predictions)
X = zeros(length(x0), length(U)+1);
X(:,1) = x0;
for k = 1:Hp
    X(:, k+1) = A*X(:,k) + B*U(k);
end