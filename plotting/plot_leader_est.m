function plot_leader_est(history, cfg)
% 绘制跟随者对领导者位置的估计误差

    t = history.t;
    Nf = cfg.numFollowers;
    Nl = cfg.numLeaders;
    dim = cfg.dim;

    figure('Name', 'Leader Position Estimation Error', 'Color', 'w');
    
    for i = 1:Nf
        subplot(Nf, 1, i);
        hold on; grid on;
        
        colors = lines(Nl);
        for l = 1:Nl
            % 计算误差模长: ||p_l - p_il_hat||
            p_l = squeeze(history.leader_p(:, l, :)); % [dim, N]
            p_il_hat = squeeze(history.leader_p_hat(:, i, l, :)); % [dim, N]
            
            error = p_l - p_il_hat;
            error_norm = sqrt(sum(error.^2, 1));
            
            plot(t, error_norm, 'Color', colors(l,:), 'LineWidth', 1.5, ...
                'DisplayName', sprintf('Leader %d', l));
        end
        
        ylabel(sprintf('Follower %d Error (m)', i));
        if i == 1
            title('Leader Position Estimation Error Norm');
        end
        if i == Nf
            xlabel('Time (s)');
        end
        legend('show', 'Location', 'northeast');
    end
end
