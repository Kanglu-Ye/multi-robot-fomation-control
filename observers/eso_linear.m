function [obs_new, d_hat] = eso_linear(agent, u_cmd, cfg, dt)

    beta1 = 3 * cfg.observer.linear_omega;
    beta2 = 3 * cfg.observer.linear_omega ^ 2;
    beta3 = cfg.observer.linear_omega ^ 3;

    % 测量输出：真实位置
    p = agent.p;

    % 当前观测器状态
    p_hat = agent.obs.p_hat;
    v_hat = agent.obs.v_hat;
    d_hat0 = agent.obs.d_hat;

    % 观测误差
    e = p - p_hat;

    % ESO 微分方程
    p_hat_dot = v_hat + beta1 * e;
    v_hat_dot = u_cmd + d_hat0 + beta2 * e;
    d_hat_dot = beta3 * e;

    % Euler 更新
    p_hat_new = p_hat + dt * p_hat_dot;
    v_hat_new = v_hat + dt * v_hat_dot;
    d_hat_new = d_hat0 + dt * d_hat_dot;

    % 输出
    obs_new.p_hat = p_hat_new;
    obs_new.v_hat = v_hat_new;
    obs_new.d_hat = d_hat_new;

    d_hat = d_hat_new;
end