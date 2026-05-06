function u0 = inclusion_pd_control(i, followers, leaders, topo, cfg)

    kp = cfg.controller.kp;
    kd = cfg.controller.kd;

    pi = followers(i).p;
    vi = followers(i).obs.v_hat;

    ep = zeros(cfg.dim,1);
    ev = zeros(cfg.dim,1);

    %% follower-follower 项：带相对位移偏置
    for j = 1:topo.Nf
        aij = topo.Af(i,j);
        if aij ~= 0
            pij_des = cfg.formation.delta{i,j};
            ep = ep + aij * ((pi - followers(j).p) - pij_des);
            ev = ev + aij * (vi - followers(j).obs.v_hat);
        end
    end

    %% follower-leader 包容项：改成用 DESO 估计值
    for l = 1:topo.Nl
        bil = topo.Bc(i,l);

        % 如果未启用 DESO，则只对该 follower 能直接观测到的 leader (Bobs) 产生吸引力
        % 否则，由于无法通过邻居获取其他 leader 信息，估计值会停留在初始位置导致控制偏差
        is_observable = cfg.deso.enable || (topo.Bobs(i,l) ~= 0);

        if bil ~= 0 && is_observable
            p_l_hat = followers(i).deso(l).p_hat;
            v_l_hat = followers(i).deso(l).v_hat;

            ep = ep + bil * (pi - p_l_hat);
            ev = ev + bil * (vi - v_l_hat);
        end
    end

    u0 = -kp * ep - kd * ev;
end