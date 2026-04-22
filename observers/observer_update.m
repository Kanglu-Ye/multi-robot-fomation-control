function [obs_new, d_hat] = observer_update(agent, u_cmd, cfg, dt)

    switch cfg.observer.type
        case 'None'
            obs_new = agent.obs;
            d_hat   = zeros(agent.dim,1);

        case 'LinearESO'
            [obs_new, d_hat] = eso_linear(agent, u_cmd, cfg, dt);

        case 'FxESO'
            [obs_new, d_hat] = eso_FxT(agent, u_cmd, cfg, dt);

        otherwise
            error('未知观测器类型: %s', cfg.observer.type);
    end
end