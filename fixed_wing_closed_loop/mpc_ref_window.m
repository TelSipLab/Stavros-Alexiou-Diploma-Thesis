function R = mpc_ref_window(t, Hp, Ts, ref_fun)

    R = zeros(9, Hp); % 9 x Hp

    for i = 1:Hp
        t_i = t + i*Ts;
        R(:,i) = ref_fun(t_i);
    end

end