%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TUBE
    properties
        yalmip_optimizer
        K_tube
    end

    methods
        function obj = MPC_TUBE(Q,R,N,H_N,h_N,H_tube,h_tube,K_tube,params)
            obj.K_tube = K_tube; 
            % YOUR CODE HERE
            nu = params.model.nu;
            nx = params.model.nx;
            % Nominal inputs 
            V = sdpvar(repmat(nu,1,N),ones(1,N),'full');
            X0 = sdpvar(nx,1,'full'); 
            obj_LQR = LQR(Q,R,params); 
            Z = sdpvar(repmat(nx,1,N+1),ones(1,N+1),'full');
            objective = 0; 
            % Tightened state constraints
            H_z = params.constraints.StateMatrix; 
            h_z = params.constraints.StateRHS; 
            % Tightened input constraints
            H_v = params.constraints.InputMatrix; 
            h_v = params.constraints.InputRHS; 
            % Nominal state constraints 
            constraints = [H_tube * (- X0 + Z{1}) <= h_tube]; 
            % Set constraints 
            for k = 1:1:N-1
                constraints = [constraints, Z{k+1} == params.model.A * Z{k} + params.model.B * V{k}];
                constraints = [constraints, H_z * Z{k} <= h_z];
                constraints = [constraints, H_v * V{k} <= h_v];
                objective = objective + Z{k}' * Q * Z{k} + V{k}' * R * V{k};
            end
            constraints = [constraints, Z{N+1} == params.model.A * Z{N} + params.model.B * V{N}]; 
            constraints = [constraints, H_z * Z{N} <= h_z, H_z * Z{N+1} <= h_z];
            constraints = [constraints, H_v * V{N} <= h_v];
            constraints = [constraints, H_N * Z{N+1} <= h_N]; % terminal constraint
            objective = objective + Z{N}' * Q * Z{N} + V{N}' * R * V{N};
            objective = objective + Z{N+1}' * obj_LQR.P * Z{N+1}; % terminal cost 
            opts = sdpsettings('verbose',1,'solver','bnb');
            obj.yalmip_optimizer = optimizer(constraints,objective,opts,X0,{V{1} Z{1} objective});
        end

        function [u, u_info] = eval(obj,x)
            %% evaluate control action by solving MPC problem, e.g.
            tic;
            [optimizer_out,errorcode] = obj.yalmip_optimizer(x);
            solvetime = toc;
            % YOUR CODE HERE
            feasible = true;
            if (errorcode ~= 0)
                feasible = false; 
            end
            [v, z, objective] = optimizer_out{:};
            u = v + obj.K_tube * (x - z); 

            u_info = struct('ctrl_feas',feasible,'objective',objective,'solvetime',solvetime);
        end
    end
end