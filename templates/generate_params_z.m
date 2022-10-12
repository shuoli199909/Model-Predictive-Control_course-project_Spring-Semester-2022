%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [params_z] = generate_params_z(params)
% initialize params_z
params_z = params;

% add initial condition of z-subsystem
params_z.model = rmfield(params_z.model,{'InitialConditionA', ...
                             'InitialConditionB', ...
                             'InitialConditionC'});
params_z.model.InitialConditionA_z = [-0.07;0.04]; % TODO: tune

% define projection matrices
T_z_x = [0 0 1 0 0 0;
         0 0 0 0 0 1];
T_z_u = [0 0 1];

% projected system dynamics
params_z.model.A = T_z_x*params.model.A*T_z_x';
params_z.model.B = T_z_x*params.model.B*T_z_u';
params_z.model.nx = 2;
params_z.model.nu = 1;

% state constraints
Hx = params.constraints.StateMatrix * T_z_x';
hx = params.constraints.StateRHS;
X = Polyhedron('A',Hx,'b',hx);
X.minHRep();

params_z.constraints.StateMatrix = X.A;
params_z.constraints.StateRHS = X.b;

% input constraints
Hu = params.constraints.InputMatrix * T_z_u';
hu = params.constraints.InputRHS;
U = Polyhedron('A',Hu,'b',hu);

params_z.constraints.InputMatrix = U.A; 
params_z.constraints.InputRHS = U.b;

% disturbance constraints
max_disturbance = 1e-4; % TODO: chose
params_z.constraints.MaxAbsDisturbance = max_disturbance;
H_w = kron(eye(params_z.model.nx),[1;-1]);
h_w = ones(2*params_z.model.nx, 1) * max_disturbance;

params_z.constraints.DisturbanceMatrix = H_w;
params_z.constraints.DisturbanceRHS = h_w;
end
