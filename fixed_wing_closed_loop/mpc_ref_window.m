function R = mpc_ref_window(t, Hp, Ts, ref_fun)

    R = zeros(6, Hp); % 6 x Hp

    for i = 1:Hp
        t_i = t + i*Ts;
        s_ref = ref_fun(t_i);
        R(:,i) = s_ref(1:6);
    end

end