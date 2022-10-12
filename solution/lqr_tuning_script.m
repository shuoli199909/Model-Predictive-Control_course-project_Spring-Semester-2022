%% Parameter study of LQR controller
clear; close all; clc;
%% Implementation
params = generate_params();
constraints = params.constraints; 
[A, B] = ndgrid(logspace(-1.5, 2, 6), 1:1:5);
Q = A.*B; 
[~, i_opt] = lqr_tuning(params.model.InitialConditionA, Q, params); 
q = Q(:, i_opt); 
[tuning_struct, i_opt] = lqr_tuning(params.model.InitialConditionA, q, params); 
saving_struct = struct("q", q, "tuning_struct", tuning_struct); 
save('lqr_tuning_script.mat', 'saving_struct', '-mat'); 