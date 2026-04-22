function [ax, ay, ah] = sf_controller(cs, r, Kx, Ky, Kh)

% extract current UAV di control state x(t)
x = cs(1); y = cs(2); h = cs(3);
dx = cs(4); dy = cs(5); dh = cs(6);

% extract reference state r(t)
rx = r(1); ry = r(2); rh = r(3);
rdx = r(4); rdy = r(5); rdh = r(6);
rd2x = r(7); rd2y = r(8); rd2h = r(9);

% calculate error signals
ex1 = x - rx; ex2 = dx - rdx;
ey1 = y - ry; ey2 = dy - rdy;
eh1 = h - rh; eh2 = dh - rdh;

% calculate controller output
ax = -Kx * [ex1; ex2] + rd2x;
ay = -Ky * [ey1; ey2] + rd2y;
ah = -Kh * [eh1; eh2] + rd2h;

end