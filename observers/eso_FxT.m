function [obs_new, d_hat] = eso_FxT(agent, u, cfg, dt)
% 固定时间ESO
%
% 状态定义：
%   x1_hat = p_hat
%   x2_hat = v_hat
%   d_hat  = disturbance estimate
%
% 观测器形式：
%   p_hat_dot = v_hat + k1*sig(e)^a1 + k2*sig(e)^b1
%   v_hat_dot = u + d_hat + k3*sig(e)^a2 + k4*sig(e)^b2
%   d_hat_dot = k5*sig(e)^a3 + k6*sig(e)^b3
%
% 其中：
%   e = p - p_hat
%   sig(e)^a := |e|^a .* sign(e)
%
% 输入：
%   agent.p         真实位置测量
%   agent.obs.p_hat 当前位置估计
%   agent.obs.v_hat 当前速度估计
%   agent.obs.d_hat 当前扰动估计
%   u               实际施加到plant的控制输入
%   cfg             参数结构体
%   dt              步长
%
% 输出：
%   obs_new         更新后的观测器状态
%   d_hat           更新后的扰动估计

    % ===== 读取参数 =====
    k1 = cfg.observer.Fx_k(1);
    k2 = cfg.observer.Fx_k(2);
    k3 = cfg.observer.Fx_k(3);
    k4 = cfg.observer.Fx_k(4);
    k5 = cfg.observer.Fx_k(5);
    k6 = cfg.observer.Fx_k(6);

    a1 = cfg.observer.Fx_alpha(1);
    a2 = cfg.observer.Fx_alpha(2);
    a3 = cfg.observer.Fx_alpha(3);

    b1 = cfg.observer.Fx_beta(1);
    b2 = cfg.observer.Fx_beta(2);
    b3 = cfg.observer.Fx_beta(3);

    % ===== 真实测量 =====
    p = agent.p;

    % ===== 当前观测器状态 =====
    p_hat = agent.obs.p_hat;
    v_hat = agent.obs.v_hat;
    d_hat0 = agent.obs.d_hat;

    % ===== 观测误差 =====
    e = p - p_hat;

    % ===== 非线性注入项 =====
    phi1 = sig_pow(e, a1);
    phi2 = sig_pow(e, b1);

    phi3 = sig_pow(e, a2);
    phi4 = sig_pow(e, b2);

    phi5 = sig_pow(e, a3);
    phi6 = sig_pow(e, b3);

    % ===== 观测器微分方程 =====
    p_hat_dot = v_hat + k1 * phi1 + k2 * phi2;
    v_hat_dot = u + d_hat0 + k3 * phi3 + k4 * phi4;
    d_hat_dot = k5 * phi5 + k6 * phi6;

    % ===== Euler离散 =====
    p_hat_new = p_hat + dt * p_hat_dot;
    v_hat_new = v_hat + dt * v_hat_dot;
    d_hat_new = d_hat0 + dt * d_hat_dot;

    % ===== 输出 =====
    obs_new.p_hat = p_hat_new;
    obs_new.v_hat = v_hat_new;
    obs_new.d_hat = d_hat_new;

    d_hat = d_hat_new;
end

function y = sig_pow(x, a)
% 逐元素实现 |x|^a * sign(x)
    y = abs(x).^a .* sign(x);
end