function leader = create_leader(id, p0, v0, cfg)

    leader.id    = id;
    leader.type  = 'leader';
    leader.dim   = cfg.dim;

    leader.p     = p0(:);
    leader.v     = v0(:);

    leader.u     = zeros(cfg.dim,1);
    leader.d     = zeros(cfg.dim,1);
    leader.d_hat = zeros(cfg.dim,1);

    leader.obs   = struct();
end