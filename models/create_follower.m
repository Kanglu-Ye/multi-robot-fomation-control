function follower = create_follower(id, p0, v0, euler0, omega0, cfg)

    follower.id    = id;
    follower.type  = 'follower';
    follower.dim   = cfg.dim;

    follower.p     = p0(:);
    follower.v     = v0(:);
    follower.euler = euler0(:);
    follower.omega = omega0(:);

    follower.u     = zeros(cfg.dim,1);
    follower.d     = zeros(cfg.dim,1);
    follower.d_hat = zeros(cfg.dim,1);

    % 四轴相关扩展字段 (用于 update_agent_quad)
    follower.T      = 0;
    follower.tau    = zeros(3,1);
    follower.a_real = zeros(3,1);
    follower.Rd     = eye(3);

    follower.obs.p_hat = p0(:);
    follower.obs.v_hat = v0(:);
    follower.obs.d_hat = zeros(cfg.dim,1);
end