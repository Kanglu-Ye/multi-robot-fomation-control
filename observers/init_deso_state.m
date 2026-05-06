function deso = init_deso_state(cfg)

    for l = 1:cfg.numLeaders
        deso(l).p_hat = zeros(cfg.dim,1);
        deso(l).v_hat = zeros(cfg.dim,1);
    end
end