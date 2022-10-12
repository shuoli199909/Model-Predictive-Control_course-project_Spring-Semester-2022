%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TS_SC
    properties
        yalmip_optimizer
    end

    methods
        function obj = MPC_TS_SC(Q,R,N,H,h,S,v,params)            
            % YOUR CODE HERE
            nu = params.model.nu;
            nx = params.model.nx;
            % define optimization variables
            U = sdpvar(repmat(nu,1,N),ones(1,N),'full');
            X0 = sdpvar(nx,1,'full'); 
            e = sdpvar(repmat(length(params.constraints.StateRHS),1,N+1),ones(1,N+1),'full'); % slack variable e
            obj_LQR = LQR(Q,R,params); 
            X = sdpvar(repmat(nx,1,N+1),ones(1,N+1),'full');
            X{1} = X0;
            % State restriction 
            H_x = params.constraints.StateMatrix; 
            h_x = params.constraints.StateRHS; 
            % Input restriction
            H_u = params.constraints.InputMatrix; 
            h_u = params.constraints.InputRHS; 
            % Objective and constraints 
            objective = 0; 
            constraints = [e{1} >= 0, H_x * X{1} <= h_x + e{1}];
            
            for k = 1:1:N
                % Constraints 
                constraints = [constraints, X{k+1} == params.model.A * X{k} + params.model.B * U{k}];
                constraints = [constraints, H_x * X{k+1} <= h_x + e{k+1}];
                constraints = [constraints, H_u * U{k} <= h_u];
                constraints = [constraints, e{k+1} >= 0];
                objective = objective + X{k}' * Q * X{k} + U{k}' * R * U{k}; 
                objective = objective + e{k}' * S * e{k} + v * norm(e{k},Inf);
            end
            constraints = [constraints, (H * X{N+1}) <= h];
            objective = objective + e{N+1}' * S * e{N+1} + v * norm(e{N+1},Inf);
            objective = objective + X{N+1}'*obj_LQR.P*X{N+1};
            opts = sdpsettings('verbose',1,'solver','quadprog');
            obj.yalmip_optimizer = optimizer(constraints,objective,opts,X0,{U{1} objective});
        end

        function [u, u_info] = eval(obj,x)
            %% evaluate control action by solving MPC problem, e.g.
            tic;
            [optimizer_out,errorcode] = obj.yalmip_optimizer(x);
            solvetime = toc;
            [u, objective] = optimizer_out{:};

            feasible = true;
            if (errorcode ~= 0)
                feasible = false;
            end

            u_info = struct('ctrl_feas',feasible,'objective',objective,'solvetime',solvetime);
        end
    end
end