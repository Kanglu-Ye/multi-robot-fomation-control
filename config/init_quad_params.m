function quad = init_quad_params()

    quad.m = 1.2;   % kg
    quad.g = 9.81;

    % 转动惯量
    quad.J = diag([0.02, 0.02, 0.04]);

    % 姿态环增益
    quad.KR = diag([8, 8, 4]);
    quad.KOmega = diag([2.5, 2.5, 1.5]);

    % 推力和力矩限幅
    quad.T_min = 0.0;
    quad.T_max = 2.5 * quad.m * quad.g;

    quad.tau_max = [0.8; 0.8; 0.4];

    % 姿态角限幅
    quad.phi_limit   = deg2rad(35);
    quad.theta_limit = deg2rad(35);

    % 固定偏航角参考
    quad.psi_ref = 0.0;
end