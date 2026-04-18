function follower = create_follower(id, p0, v0, cfg)

    follower.id    = id;
    follower.type  = 'follower';
    follower.dim   = cfg.dim;

    follower.p     = p0(:);
    follower.v     = v0(:);

    follower.u     = zeros(cfg.dim,1);
    follower.d     = zeros(cfg.dim,1);
    follower.d_hat = zeros(cfg.dim,1);

    follower.obs.p_hat = p0(:);
    follower.obs.v_hat = v0(:);
    follower.obs.d_hat = zeros(cfg.dim,1);
end