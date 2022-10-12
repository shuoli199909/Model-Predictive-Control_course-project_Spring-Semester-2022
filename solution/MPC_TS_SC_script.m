% Shuo Li - shuoli@student.ethz.ch 
% Compute the parameters of a tube MPC 
clear; close all; clc; 
params = generate_params(); 
Q = diag([91.5,0.0924,248,0,0,0]); 
R = eye(params.model.nu); 
N = 30; 
S = 1000000*eye(6); 
v = 1000000; 
[H, h] = lqr_maxPI(Q,R,params);
obj_MPC_TS = MPC_TS(Q,R,N,H,h,params); 
obj_MPC_TS_SC = MPC_TS_SC(Q,R,N,H,h,S,v,params); 
X_0 = params.model.InitialConditionB; 
% The solutions (input values) should be the same. 
U_0_TS = obj_MPC_TS.eval(X_0);
U_0_TS_SC = obj_MPC_TS.eval(X_0); 
save('MPC_TS_SC_params.mat','S','v');