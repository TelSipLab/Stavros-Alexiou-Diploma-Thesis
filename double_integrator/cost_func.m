function J = cost_func(U, A, B, Q, R, Hp, Hc, x0, x_target)

  X = zeros(length(x0), length(U)+1);
  X(:,1) = x0;
  X_TARGET = repmat(x_target, 1, 10);

  % calculate state predictions (X Matrix) using F.E.
  for k = 1:Hp
   if k <= Hc
    X(:, k+1) = A*X(:,k) + B*U(k);
   else
    X(:, k+1) = A*X(:,k) + B*U(Hc);
   end
  end

  % ipologismos J
  E = X(:, 2:end) - X_TARGET;
  tracking_error = trace(E'*Q*E);
  control_effort = U(1:Hc)' * R * U(1:Hc);
  J = tracking_error + control_effort;

end