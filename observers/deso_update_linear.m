function deso_l_new = deso_update_linear(i, l, followers, leaders, topo, cfg, dt)

    c1 = 2 * cfg.deso.linear_omega;
    c2 = cfg.deso.linear_omega ^ 2;

    % 当前 follower i 对 leader l 的估计
    p_hat_il = followers(i).deso(l).p_hat;
    v_hat_il = followers(i).deso(l).v_hat;

    % 一致性误差项
    e_il = zeros(cfg.dim,1);

    % follower-follower 邻居一致性 (仅在启用 DESO 时启用)
    if cfg.deso.enable
        for j = 1:topo.Nf
            aij = topo.Af(i,j);
            if aij ~= 0
                p_hat_jl = followers(j).deso(l).p_hat;
                e_il = e_il + aij * (p_hat_il - p_hat_jl);
            end
        end
    end

    % 如果 follower i 直接连接到 leader l，则引入真实leader位置
    bil = topo.Bobs(i,l);
    if bil ~= 0
        e_il = e_il + bil * (p_hat_il - leaders(l).p);
    end

    % 2阶分布式观测器 (不再估计扰动 d)
    p_hat_dot = v_hat_il - c1 * e_il;
    v_hat_dot = - c2 * e_il;

    deso_l_new.p_hat = p_hat_il + dt * p_hat_dot;
    deso_l_new.v_hat = v_hat_il + dt * v_hat_dot;
end