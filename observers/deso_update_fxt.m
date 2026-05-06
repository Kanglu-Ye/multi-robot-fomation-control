function deso_l_new = deso_update_fxt(i, l, followers, leaders, topo, cfg, dt)
% 固定时间 2阶分布式观测器 (FxT-DESO)
%
% 状态定义:
%   p_hat_il: follower i 对 leader l 的位置估计
%   v_hat_il: follower i 对 leader l 的速度估计
%
% 动力学方程:
%   p_dot = v_hat + l1*sig^alpha(e_obs) + l2*sig^beta(e_obs) + sum(aij*(p_hat_j - p_hat_i))
%   v_dot = l3*sig^alpha(e_obs) + l4*sig^beta(e_obs) + sum(aij*(v_hat_j - v_hat_i))
%
% 其中 e_obs = p_leader - p_hat (仅当 bil > 0 时存在)

    % ===== 读取参数 =====
    alpha = cfg.deso.fxt_alpha;
    beta  = cfg.deso.fxt_beta;
    l1 = cfg.deso.fxt_l(1);
    l2 = cfg.deso.fxt_l(2);
    l3 = cfg.deso.fxt_l(3);
    l4 = cfg.deso.fxt_l(4);

    % 当前状态
    p_hat_il = followers(i).deso(l).p_hat;
    v_hat_il = followers(i).deso(l).v_hat;

    % ===== 1. 本地观测项 (Local Observation) =====
    bil = topo.Bobs(i,l);
    term_local_p = zeros(cfg.dim, 1);
    term_local_v = zeros(cfg.dim, 1);
    
    if bil ~= 0
        % 误差: e = p_leader - p_hat
        e_obs = leaders(l).p - p_hat_il;
        
        term_local_p = l1 * sig_pow(e_obs, alpha) + l2 * sig_pow(e_obs, beta);
        term_local_v = l3 * sig_pow(e_obs, alpha) + l4 * sig_pow(e_obs, beta);
    end

    % ===== 2. 邻居交互项 (Neighbor Interaction) =====
    term_inter_p = zeros(cfg.dim, 1);
    term_inter_v = zeros(cfg.dim, 1);

    % 仅在启用分布式功能时计算邻居项
    if cfg.deso.enable
        for j = 1:topo.Nf
            aij = topo.Af(i,j);
            if aij ~= 0
                p_hat_jl = followers(j).deso(l).p_hat;
                v_hat_jl = followers(j).deso(l).v_hat;
                
                term_inter_p = term_inter_p + aij * (p_hat_jl - p_hat_il);
                term_inter_v = term_inter_v + aij * (v_hat_jl - v_hat_il);
            end
        end
    end

    % ===== 观测器微分方程 =====
    p_hat_dot = v_hat_il + term_local_p + term_inter_p;
    v_hat_dot = term_local_v + term_inter_v;

    % ===== Euler 离散 =====
    deso_l_new.p_hat = p_hat_il + dt * p_hat_dot;
    deso_l_new.v_hat = v_hat_il + dt * v_hat_dot;
end

function y = sig_pow(x, a)
    y = abs(x).^a .* sign(x);
end
