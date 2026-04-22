function cfg = default_config()

    %% 基本参数
    cfg.dim = 3;
    cfg.dt  = 0.01;
    cfg.T   = 20;
    cfg.N   = round(cfg.T / cfg.dt);

    %% 智能体数量
    cfg.numLeaders   = 4;
    cfg.numFollowers = 2;

    %% 控制器参数
    cfg.controller.type = 'PD';
    cfg.controller.kp = 5;
    cfg.controller.kd  = 2;

    %% 编队偏置项
    % 含义：每个跟随者相对“包容目标”的期望偏移
    cfg.formation.delta{1,2} = [-1;0;0];
    cfg.formation.delta{2,1} = [ 1;0;0];

    %% 观测器参数 'LinearESO', 'FxESO'
    cfg.observer.type = 'FxESO';

    %线性ESO参数:omega
    cfg.observer.linear_omega = 5;

    %FxESO参数
    cfg.observer.Fx_alpha = [0.6; 0.6; 0.6];
    cfg.observer.Fx_beta = [1.5; 1.5; 1.5];
    cfg.observer.Fx_k = [3; 1; 5; 2; 4; 1];


    % d_hat 微分项符号：+1 或 -1
    % 如果你确认 Simulink 里 beta3 块就是负号，就设成 -1
    cfg.observer.beta3_sign = +1;

    %% 扰动设置：使用 enable 字段选择启用的扰动类型（可叠加）
    % 阶跃扰动
    cfg.disturbance.step.enable = false;
    cfg.disturbance.step.step_time = 5.0;
    cfg.disturbance.step.step_value{1} = [ 0.8; -0.5; 0.6];
    cfg.disturbance.step.step_value{2} = [-0.6;  0.7; -0.4];

    % 风场扰动
    % 模型: d = A + A1*sin(2*pi*f1*t) + A2*sin(2*pi*f2*t)
    cfg.disturbance.wind.enable = false;
    cfg.disturbance.wind.A = [4.0; 3.0; 2.0];  % 三轴偏置项 (可选)
    cfg.disturbance.wind.A1 = [0.5; 0.4; 0.3];  % 三轴低频幅值
    cfg.disturbance.wind.f1 = [0.2; 0.2; 0.2];  % 三轴低频频率 (Hz)
    cfg.disturbance.wind.A2 = [0.2; 0.2; 0.2];  % 三轴高频幅值
    cfg.disturbance.wind.f2 = [2.0; 2.1; 1.9];  % 三轴高频频率 (Hz)

    % 模型失配扰动
    % 模型: d = -kv * v - knl * v.^2
    cfg.disturbance.model.enable = true;
    cfg.disturbance.model.kv  = 0.3;   % 线性阻尼系数
    cfg.disturbance.model.knl = 0.1;   % 非线性摩擦系数

    % 未建模动态扰动
    cfg.disturbance.unmodel.enable = false;
    cfg.disturbance.unmodel.noise_std = 0.15;

    %% 四个领导者固定位置
    cfg.leaders.p0{1} = [0; 0; 3];
    cfg.leaders.p0{2} = [4; 0; 3];
    cfg.leaders.p0{3} = [2; 3; 3];
    cfg.leaders.p0{4} = [3; 1; 5];

    cfg.leaders.v0{1} = [0; 0; 0];
    cfg.leaders.v0{2} = [0; 0; 0];
    cfg.leaders.v0{3} = [0; 0; 0];
    cfg.leaders.v0{4} = [0; 0; 0];

    %% 两个跟随者初始状态
    cfg.followers.p0{1} = [-3; -2; 0];
    cfg.followers.v0{1} = [0; 0; 0];
    cfg.followers.euler0{1} = [0; 0; 0];
    cfg.followers.omega0{1} = [0; 0; 0];
    
    cfg.followers.p0{2} = [ 6; -3; 0];
    cfg.followers.v0{2} = [0; 0; 0];
    cfg.followers.euler0{2} = [0; 0; 0];
    cfg.followers.omega0{2} = [0; 0; 0];   
    

    %% 跟随者-跟随者邻接矩阵
    cfg.topology.Af = [0 1;
                       1 0];

    %% 跟随者-领导者权重矩阵
    % 这里就体现“不同跟随者有不同的领导者权重”
    cfg.topology.B  = [1.0 0.8 0.6 0.4;
                       0.5 1.0 0.9 0.7];
end