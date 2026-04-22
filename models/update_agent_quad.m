function agent = update_agent_quad(agent, a_cmd, dt, quad)
% 离散四旋翼闭环动力学更新
%
% 输入:
%   agent.p      [3x1] 位置, 世界系
%   agent.v      [3x1] 速度, 世界系
%   agent.euler  [3x1] 欧拉角 [phi; theta; psi]
%   agent.omega  [3x1] 角速度 [p; q; r], 机体系
%   agent.d      [3x1] 外扰等效平动加速度, 世界系
%
%   a_cmd        [3x1] 来自包容控制协议的期望平动加速度, 世界系
%   dt           标量, 采样时间
%   quad         参数结构体
%
% 输出:
%   agent        更新后的状态结构体
%
% 说明:
%   1) 该模型是“带飞控的四旋翼闭环对象”
%   2) 输入不是推力/力矩，而是平动加速度指令 a_cmd
%   3) 内部自动完成:
%        a_cmd -> 期望姿态/总推力 -> 力矩 -> 四旋翼刚体动力学
%   4) 这是比二阶积分器真实很多、但仍适合多智能体仿真的版本

    % ---------- 读取状态 ----------
    p     = agent.p(:);
    v     = agent.v(:);
    euler = agent.euler(:);   % [phi; theta; psi]
    omega = agent.omega(:);   % [p; q; r]
    
    if isfield(agent, 'd')
        d = agent.d(:);
    else
        d = zeros(3,1);
    end

    % ---------- 参数 ----------
    m  = quad.m;
    J  = quad.J;
    g  = quad.g;
    psi_ref = quad.psi_ref;   % 固定偏航参考，可设为0

    % ---------- 当前姿态矩阵 ----------
    R = eulZYX_to_R(euler);

    % ============================================================
    % 1) 外环: 平动加速度指令 -> 期望姿态 Rd 和总推力 T
    % ============================================================
    % 四旋翼需要产生的总“比力”方向
    acc_total = a_cmd + [0;0;g];

    % 防止分母过小
    norm_acc_total = norm(acc_total);
    if norm_acc_total < 1e-6
        % 极端情况：退化为悬停方向
        b3d = [0;0;1];
        T = 0;
    else
        b3d = acc_total / norm_acc_total;
        % 推力大小：T = m * (acc_total 在当前/期望机体z轴方向上的需求)
        T = m * norm_acc_total;
    end

    % 限幅：总推力
    T = min(max(T, quad.T_min), quad.T_max);

    % 根据期望偏航构造期望姿态
    % 先定义期望机体 x 轴在水平面的参考方向
    b1c = [cos(psi_ref); sin(psi_ref); 0];

    % 用 b3d 和 b1c 构造正交基
    b2d = cross(b3d, b1c);
    if norm(b2d) < 1e-6
        % 若退化，给个备选
        b1c = [1;0;0];
        b2d = cross(b3d, b1c);
    end
    b2d = b2d / norm(b2d);

    b1d = cross(b2d, b3d);
    b1d = b1d / norm(b1d);

    Rd = [b1d, b2d, b3d];

    % ============================================================
    % 2) 内环: 姿态 PD -> 力矩 tau
    % ============================================================
    % 姿态误差（SO(3) 常用形式）
    eR_mat = 0.5 * (Rd' * R - R' * Rd);
    eR = vee(eR_mat);

    % 角速度参考先取 0
    omega_d = zeros(3,1);
    eOmega = omega - R' * Rd * omega_d;

    tau = -quad.KR * eR - quad.KOmega * eOmega + cross(omega, J * omega);

    % 力矩限幅
    tau = min(max(tau, -quad.tau_max), quad.tau_max);

    % ============================================================
    % 3) 四旋翼真实动力学
    % ============================================================
    % 平动动力学
    a_real = -[0;0;g] + (T / m) * (R * [0;0;1]) + d;

    % 转动动力学
    omega_dot = J \ (tau - cross(omega, J * omega));

    % 欧拉角运动学
    euler_dot = bodyrate_to_eulerdot(euler, omega);

    % ============================================================
    % 4) 离散积分 (Euler)
    % ============================================================
    p_new     = p + dt * v;
    v_new     = v + dt * a_real;
    euler_new = euler + dt * euler_dot;
    omega_new = omega + dt * omega_dot;

    % 欧拉角简单限幅，避免 pitch 接近奇异
    euler_new(2) = min(max(euler_new(2), -quad.theta_limit), quad.theta_limit);
    euler_new(1) = min(max(euler_new(1), -quad.phi_limit),   quad.phi_limit);

    % 偏航角 wrap 到 [-pi, pi]
    euler_new(3) = wrapToPi_local(euler_new(3));

    % ---------- 写回 ----------
    agent.p = p_new;
    agent.v = v_new;
    agent.euler = euler_new;
    agent.omega = omega_new;

    % 为了和你原先接口习惯保持一致，这里记录“协议输入”
    agent.u = a_cmd;

    % 记录内部控制量，便于画图分析
    agent.T   = T;
    agent.tau = tau;
    agent.a_real = a_real;
    agent.Rd = Rd;
end


% ========================= 子函数 =========================

function R = eulZYX_to_R(euler)
% euler = [phi; theta; psi]
    phi = euler(1);
    theta = euler(2);
    psi = euler(3);

    cphi = cos(phi);   sphi = sin(phi);
    cth  = cos(theta); sth  = sin(theta);
    cpsi = cos(psi);   spsi = sin(psi);

    % ZYX: R = Rz(psi) * Ry(theta) * Rx(phi)
    R = [ cpsi*cth,  cpsi*sth*sphi - spsi*cphi,  cpsi*sth*cphi + spsi*sphi;
          spsi*cth,  spsi*sth*sphi + cpsi*cphi,  spsi*sth*cphi - cpsi*sphi;
          -sth,      cth*sphi,                    cth*cphi ];
end

function euler_dot = bodyrate_to_eulerdot(euler, omega)
% omega = [p; q; r], body frame
% euler = [phi; theta; psi]
    phi   = euler(1);
    theta = euler(2);

    p = omega(1);
    q = omega(2);
    r = omega(3);

    cphi = cos(phi); sphi = sin(phi);
    cth  = cos(theta);
    tth  = tan(theta);

    % 避免奇异
    if abs(cth) < 1e-4
        cth = sign(cth) * 1e-4;
    end

    E = [1, sphi*tth,  cphi*tth;
         0, cphi,     -sphi;
         0, sphi/cth,  cphi/cth];

    euler_dot = E * [p; q; r];
end

function v = vee(S)
% 从反对称矩阵取向量
    v = [S(3,2); S(1,3); S(2,1)];
end

function ang = wrapToPi_local(ang)
    ang = mod(ang + pi, 2*pi) - pi;
end