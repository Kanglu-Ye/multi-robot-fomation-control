function cfg = default_config()

    %% 基本参数
    cfg.dim = 3;
    cfg.dt  = 0.01;
    cfg.T   = 20;
    cfg.N   = round(cfg.T / cfg.dt);

    %% 智能体数量
    cfg.numLeaders   = 3;
    cfg.numFollowers = 2;

    %% 控制器参数
    cfg.controller.type = 'PD';
    cfg.controller.kp = 5;
    cfg.controller.kd  = 2;

    %% 编队偏置项
    % 含义：每个跟随者相对“包容目标”的期望偏移
    cfg.formation.delta{1,2} = [-1;0;0];
    cfg.formation.delta{2,1} = [ 1;0;0];

    %% 观测器参数
    cfg.observer.type = 'LinearESO';

    cfg.observer.beta1 = 15;
    cfg.observer.beta2 = 75;
    cfg.observer.beta3 = 125;   % 先给一个值，后续可调

    % d_hat 微分项符号：+1 或 -1
    % 如果你确认 Simulink 里 beta3 块就是负号，就设成 -1
    cfg.observer.beta3_sign = +1;

    %% 扰动设置：三维阶跃向量，作用于控制通道
    cfg.disturbance.type = 'step';

    cfg.disturbance.step_time = 5.0;

    cfg.disturbance.step_value{1} = [ 0.8; -0.5; 0.6];
    cfg.disturbance.step_value{2} = [-0.6;  0.7; -0.4];

    %% 三个领导者固定位置
    cfg.leaders.p0{1} = [0; 0; 0];
    cfg.leaders.p0{2} = [4; 0; 0];
    cfg.leaders.p0{3} = [2; 3; 0];

    cfg.leaders.v0{1} = [0; 0; 0];
    cfg.leaders.v0{2} = [0; 0; 0];
    cfg.leaders.v0{3} = [0; 0; 0];

    %% 两个跟随者初始状态
    cfg.followers.p0{1} = [-3; -2; 1];
    cfg.followers.p0{2} = [ 6; -3; 2];

    cfg.followers.v0{1} = [0; 0; 0];
    cfg.followers.v0{2} = [0; 0; 0];

    %% 跟随者-跟随者邻接矩阵
    cfg.topology.Af = [0 1;
                       1 0];

    %% 跟随者-领导者权重矩阵
    % 这里就体现“不同跟随者有不同的领导者权重”
    cfg.topology.B  = [1.0 0.8 0.6;
                       0.5 1.0 0.9];
end