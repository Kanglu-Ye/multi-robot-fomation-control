function plot_control_input(history, cfg)

    t = history.t;
    Nf = cfg.numFollowers;

    for i = 1:Nf
        figure;
        for k = 1:cfg.dim
            subplot(cfg.dim,1,k);
            plot(t, squeeze(history.u0(k,i,:)), 'LineWidth', 1.8); hold on;
            plot(t, squeeze(history.u(k,i,:)), '--', 'LineWidth', 1.8);
            grid on;
            ylabel(sprintf('dim %d', k));
            if k == 1
                title(sprintf('Follower %d 控制输入', i));
            end
            if k == cfg.dim
                xlabel('t (s)');
            end
            legend('u0', 'u');
        end
    end
end