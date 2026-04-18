function u0 = inclusion_pd_control(i, followers, leaders, topo, cfg)

    kp = cfg.controller.kp;
    kd  = cfg.controller.kd;

    pi = followers(i).p;
    vi = followers(i).v;

    ep = zeros(cfg.dim,1);
    ev = zeros(cfg.dim,1);

    %% follower-follower 项：带相对位移偏置
    for j = 1:topo.Nf
        aij = topo.Af(i,j);
        if aij ~= 0
            pij_des = cfg.formation.delta{i,j};   % 若无则设为 0
            ep = ep + aij * ((pi - followers(j).p) - pij_des);
            ev = ev + aij * (vi - followers(j).v);
        end
    end

    %% follower-leader 包容项
    for l = 1:topo.Nl
        bil = topo.B(i,l);
        if bil ~= 0
            ep = ep + bil * (pi - leaders(l).p);
            ev = ev + bil * (vi - leaders(l).v);
        end
    end

    u0 = -kp * ep - kd * ev;
end