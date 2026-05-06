function followers = deso_update_all(followers, leaders, topo, cfg, dt)

    followers_old = followers;  % 同步更新，避免顺序污染

    for i = 1:topo.Nf
        for l = 1:topo.Nl
            switch cfg.deso.type
                case 'linear'
                    followers(i).deso(l) = deso_update_linear(i, l, followers_old, leaders, topo, cfg, dt);
                case 'FxT'
                    followers(i).deso(l) = deso_update_fxt(i, l, followers_old, leaders, topo, cfg, dt);
                otherwise
                    error('未知的 DESO 类型: %s', cfg.deso.type);
            end
        end
    end
end