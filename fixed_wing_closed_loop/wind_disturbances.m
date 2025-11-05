function Vw = wind_disturbances(s, params)

    % current altitude
    h = s(3);

    % parameters extractrion
    Vmw = params.Vmw;

    % gust model (wind speed calculation from paper)
    Vw_normal = 0.215 * Vmw * log10(h) + 0.285 * Vmw;
    Vw_tan = 0.09 * Vmw * randn;
    Vw = Vw_normal + Vw_tan;

end