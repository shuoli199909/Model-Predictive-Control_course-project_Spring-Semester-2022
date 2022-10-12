% Shuo Li - shuoli@student.ethz.ch 
% Compute the parameters of a tube MPC 
clear; close all; clc; 
Q = diag([248,0]); 
R = 1; 
N = 50; 
p = [0.1, 0.5]; 
params_z = generate_params_z(generate_params());
K_tube = compute_tube_controller(p,params_z); 
[H_tube,h_tube,n_iter] = compute_minRPI(K_tube,params_z); 
[H_N,h_N] = lqr_maxPI(Q,R,params_z);
params_z_tube = compute_tightening(K_tube,H_tube,h_tube,params_z); 
obj_MPC_TUBE = MPC_TUBE(Q,R,N,H_N,h_N,H_tube,h_tube,K_tube,params_z_tube);
save('MPC_TUBE_params.mat','p','K_tube','H_tube','h_tube','H_N','h_N','params_z_tube');
