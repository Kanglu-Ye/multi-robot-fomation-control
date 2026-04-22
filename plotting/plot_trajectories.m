function plot_trajectories(history, cfg)

    figure;
    hold on; grid on; axis equal;
    view(3);

    Nl = cfg.numLeaders;
    Nf = cfg.numFollowers;

    %% 画领导者
    L_final = zeros(3, Nl);
    for l = 1:Nl
        px = squeeze(history.leader_p(1,l,:));
        py = squeeze(history.leader_p(2,l,:));
        pz = squeeze(history.leader_p(3,l,:));

        plot3(px, py, pz, 'rs', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', sprintf('Leader %d', l));
        text(px(1)+0.1, py(1)+0.1, pz(1)+0.1, sprintf('Leader %d', l));

        % 记录最终位置
        L_final(:, l) = [px(end); py(end); pz(end)];
    end

    % 连接领导者最终位置，形成包容区域示意
    if Nl == 4
        % 特殊处理 4 个领导者：绘制四面体的 6 条边和 4 个面
        edges = [1,2; 1,3; 1,4; 2,3; 2,4; 3,4];
        for k = 1:size(edges, 1)
            p1 = L_final(:, edges(k,1));
            p2 = L_final(:, edges(k,2));
            
            % HandleVisibility 必须是 'on' 或 'off' 字符串
            if k == 1
                hv = 'on';
            else
                hv = 'off';
            end

            plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], ...
                'r--', 'LineWidth', 1.5, 'HandleVisibility', hv);
        end
        % 修改第一个 plot3 的 DisplayName，避免出现 6 个图例
        h = findobj(gca, 'Type', 'line', 'LineStyle', '--');
        if ~isempty(h), set(h(1), 'DisplayName', 'Containment Region (Tetrahedron)'); end

        % 绘制四面体的 4 个面
        faces = [1,2,3; 1,2,4; 1,3,4; 2,3,4];
        patch('Faces', faces, 'Vertices', L_final', 'FaceColor', 'r', ...
            'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');

    end

    %% 画跟随者轨迹
    for i = 1:Nf
        px = squeeze(history.follower_p(1,i,:));
        py = squeeze(history.follower_p(2,i,:));
        pz = squeeze(history.follower_p(3,i,:));

        plot3(px, py, pz, 'LineWidth', 2, 'DisplayName', sprintf('Follower %d', i));
        plot3(px(1), py(1), pz(1), 'ko', 'MarkerSize', 8, 'LineWidth', 1.5, 'HandleVisibility', 'off');
        plot3(px(end), py(end), pz(end), 'bx', 'MarkerSize', 10, 'LineWidth', 2, 'HandleVisibility', 'off');
    end

    xlabel('x');
    ylabel('y');
    zlabel('z');
    title('三维多智能体编队包容控制轨迹');

    legend('Location', 'northeast');
end