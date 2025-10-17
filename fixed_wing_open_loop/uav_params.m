function params = uav_params()
    params.m = 20; % weight of the UAV (kg)
    params.ga = 9.81; % gravitional coeficient (kg/m^2)
    params.Area = 1.37; % wing area (m^2)
    params.CD0 = 0.02; % zero lift drag coeficient
    params.Kn = 1; % load factor effectivenes
    params.Kd = 0.1; % induced drag coeficient
    params.p = 1.225; % atmospheric density (kg/m^3)
    params.Vmw = 4; % mean wind speed altitude of 80m
end
