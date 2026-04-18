function plot_disturbance_est(history, cfg)

    t = history.t;
    Nf = cfg.numFollowers;

    for i = 1:Nf
        figure;
        for k = 1:cfg.dim
            subplot(cfg.dim,1,k);
            plot(t, squeeze(history.d(k,i,:)), 'LineWidth', 1.8); hold on;
            plot(t, squeeze(history.d_hat(k,i,:)), '--', 'LineWidth', 1.8);
            grid on;
            ylabel(sprintf('dim %d', k));
            if k == 1
                title(sprintf('Follower %d 扰动与估计', i));
            end
            if k == cfg.dim
                xlabel('t (s)');
            end
            legend('d', 'd\_hat');
        end
    end
end