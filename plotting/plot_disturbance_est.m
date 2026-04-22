function plot_disturbance_est(history, cfg)

    t = history.t;
    Nf = cfg.numFollowers;

    for i = 1:Nf
        figure('Name', sprintf('Follower %d Disturbance Estimation', i));
        for k = 1:cfg.dim
            % 1. 扰动及其估计
            subplot(cfg.dim, 2, 2*k-1);
            plot(t, squeeze(history.d(k,i,:)), 'LineWidth', 1.5); hold on;
            plot(t, squeeze(history.d_hat(k,i,:)), '--', 'LineWidth', 1.5);
            grid on;
            ylabel(sprintf('Dim %d (m/s^2)', k));
            if k == 1, title('Disturbance & Est'); end
            if k == cfg.dim, xlabel('t (s)'); end
            legend('d', 'd\_hat');

            % 2. 估计误差 d - d_hat
            subplot(cfg.dim, 2, 2*k);
            error_d = squeeze(history.d(k,i,:)) - squeeze(history.d_hat(k,i,:));
            plot(t, error_d, 'r', 'LineWidth', 1.5);
            grid on;
            ylabel('Error');
            if k == 1, title('Estimation Error (d - d\_hat)'); end
            if k == cfg.dim, xlabel('t (s)'); end
        end
    end
end