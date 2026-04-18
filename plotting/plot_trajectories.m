function plot_trajectories(history, cfg)

    figure;
    hold on; grid on; axis equal;
    view(3);

    Nl = cfg.numLeaders;
    Nf = cfg.numFollowers;

    %% 画领导者
    for l = 1:Nl
        px = squeeze(history.leader_p(1,l,:));
        py = squeeze(history.leader_p(2,l,:));
        pz = squeeze(history.leader_p(3,l,:));

        plot3(px, py, pz, 'rs', 'MarkerSize', 10, 'LineWidth', 2);
        text(px(1)+0.1, py(1)+0.1, pz(1)+0.1, sprintf('Leader %d', l));
    end

    %% 画跟随者轨迹
    for i = 1:Nf
        px = squeeze(history.follower_p(1,i,:));
        py = squeeze(history.follower_p(2,i,:));
        pz = squeeze(history.follower_p(3,i,:));

        plot3(px, py, pz, 'LineWidth', 2);
        plot3(px(1), py(1), pz(1), 'ko', 'MarkerSize', 8, 'LineWidth', 1.5);
        plot3(px(end), py(end), pz(end), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
    end

    xlabel('x');
    ylabel('y');
    zlabel('z');
    title('三维多智能体编队包容控制轨迹');

    legend('Leader 1','Leader 2','Leader 3','Follower 1','Follower 2');
end