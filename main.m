clc; clear; close all;

addpath(genpath(pwd));

cfg = default_config();

history = run_simulation(cfg);

plot_trajectories(history, cfg);
plot_disturbance_est(history, cfg);
plot_control_input(history, cfg);
plot_leader_est(history, cfg);