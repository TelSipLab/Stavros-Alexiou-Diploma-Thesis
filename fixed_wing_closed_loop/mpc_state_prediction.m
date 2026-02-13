function S = mpc_state_prediction(U, cs0, A, B, Hp, Hc)

% prealloc state vector
nx = size(A,1); % nx = 6
S = zeros(nx, Hp+1); % S: 6x11

% reshape control vector
nu = size(B,2); % nu = 3
U = reshape(U, nu, Hc); % U: 3x10

% mpc state prediction
S(:,1) = cs0;
for k = 1:Hp
    if k <= Hc 
        u_k = U(:,k);
    else       
        u_k = U(:,Hc);
    end % if end
    S(:,k+1) = A*S(:,k) + B*u_k;
end % for end
end % function end
