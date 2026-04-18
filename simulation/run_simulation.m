function history = run_simulation(cfg)

    %% 初始化
    [leaders, followers, topo] = initialize_system(cfg);

    N   = cfg.N;
    dt  = cfg.dt;
    Nf  = cfg.numFollowers;
    Nl  = cfg.numLeaders;
    dim = cfg.dim;

    %% 历史数据
    history.t = zeros(1, N);

    history.leader_p   = zeros(dim, Nl, N);
    history.follower_p = zeros(dim, Nf, N);
    history.follower_v = zeros(dim, Nf, N);
    history.u0         = zeros(dim, Nf, N);
    history.u          = zeros(dim, Nf, N);
    history.d          = zeros(dim, Nf, N);
    history.d_hat      = zeros(dim, Nf, N);
    history.p_hat      = zeros(dim, Nf, N);
    history.v_hat      = zeros(dim, Nf, N);

    %% 主循环
    for k = 1:N
        t = (k-1) * dt;
        history.t(k) = t;

        %% 跟随者更新
        for i = 1:Nf

            % 1) 控制通道扰动
            followers(i).d = local_disturbance(i, t, cfg);

            % 2) 名义控制 a0
            u0 = inclusion_pd_control(i, followers, leaders, topo, cfg);

            % 3) 补偿控制 a = a0 - d_hat
            u_cmd = u0 - followers(i).d_hat;

            % 4) ESO 用 a_cmd = a0
            [followers(i).obs, followers(i).d_hat] = ...
                observer_update(followers(i), u_cmd, cfg, dt);

            % 5) 推进真实动力学
            followers(i) = update_agent(followers(i), u_cmd, dt);

            % 记录中间量
            history.u0(:, i, k)    = u0;
            history.u(:, i, k)     = followers(i).u;
            history.d(:, i, k)     = followers(i).d;
            history.d_hat(:, i, k) = followers(i).d_hat;
            history.p_hat(:, i, k) = followers(i).obs.p_hat;
            history.v_hat(:, i, k) = followers(i).obs.v_hat;
        end

        %% 记录领导者
        for l = 1:Nl
            history.leader_p(:, l, k) = leaders(l).p;
        end

        %% 记录跟随者
        for i = 1:Nf
            history.follower_p(:, i, k) = followers(i).p;
            history.follower_v(:, i, k) = followers(i).v;
        end
    end
end

function d = local_disturbance(i, t, cfg)

    switch cfg.disturbance.type
        case 'none'
            d = zeros(cfg.dim,1);

        case 'step'
            if t >= cfg.disturbance.step_time
                d = cfg.disturbance.step_value{i};
            else
                d = zeros(cfg.dim,1);
            end

        otherwise
            error('未知扰动类型: %s', cfg.disturbance.type);
    end
end