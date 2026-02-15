function CS = mpc_state_prediction(U, cs0, A, B, Hp, Hc)

% prealloc control state vector
nx = size(A,1); % nx = 6
CS = zeros(nx, Hp+1); % S: 6x11

% reshape control state vector
nu = size(B,2); % nu = 3
U = reshape(U, nu, Hc); % U: 3x10

% mpc control state prediction
CS(:,1) = cs0;
for k = 1:Hp
    if k <= Hc 
        u_k = U(:,k);
    else       
        u_k = U(:,Hc);
    end % if end
    CS(:,k+1) = A*CS(:,k) + B*u_k;
end % for end
end % function end