function metrics = performance_metrics(t_total, Ts, logs)

% MSE (Mean Squared Error) & MAE (Mean Absolute Error)
E = logs.E;
MSE_x = mean(E(:,1).^2); metrics.MSE_x = MSE_x;
MSE_y = mean(E(:,2).^2); metrics.MSE_y = MSE_y;
MSE_h = mean(E(:,3).^2); metrics.MSE_h = MSE_h;
MSE_total = mean([MSE_x MSE_y MSE_h]); metrics.MSE_total = MSE_total;
MAE_x = mean(abs(E(:,1))); metrics.MAE_x = MAE_x;
MAE_y = mean(abs(E(:,2))); metrics.MAE_y = MAE_y;
MAE_h = mean(abs(E(:,3))); metrics.MAE_h = MAE_h;
MAE_total = mean([MAE_x MAE_y MAE_h]); metrics.MAE_total = MAE_total;

% MED (Mean Euclidian Distance)
p = [logs.x logs.y logs.h];
r = logs.ref_cont;
ED = sqrt(sum((p-r).^2,2)); metrics.ED = ED;
MED = mean(ED); metrics.MED = MED;

% norm2 of controller outputs + total control effort (Ju)
U_d = logs.U_d;
Ux = U_d(:,1); Uy = U_d(:,2); Uh = U_d(:,3);
norm2_Ux = sqrt(sum((Ux.^2).*Ts)); metrics.norm2_Ux = norm2_Ux;
norm2_Uy = sqrt(sum((Uy.^2).*Ts)); metrics.norm2_Uy = norm2_Uy;
norm2_Uh = sqrt(sum((Uh.^2).*Ts)); metrics.norm2_Uh = norm2_Uh;
Ju = sqrt(norm2_Ux^2 + norm2_Uy^2 + norm2_Uh^2); metrics.Ju = Ju;

% norm2 of real controller outputs
dt = diff(t_total);
ng = logs.ng; phib = logs.phib; Th = logs.Th;
norm2_ng = sqrt(sum((ng(1:end-1).^2).*dt)); metrics.norm2_ng = norm2_ng;
norm2_phib = sqrt(sum((phib(1:end-1).^2).*dt)); metrics.norm2_phib = norm2_phib;
norm2_Th = sqrt(sum((Th(1:end-1).^2).*dt)); metrics.norm2_Th = norm2_Th;

end