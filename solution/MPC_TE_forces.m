%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TE_forces
    properties
        forces_optimizer
    end

    methods
        function obj = MPC_TE_forces(Q,R,N,params)
            % YOUR CODE HERE
            nu = params.model.nu;
            nx = params.model.nx;
            % define optimization variables
            U = sdpvar(repmat(nu,1,N),ones(1,N),'full');
            X0 = sdpvar(nx,1,'full'); 
            X = sdpvar(repmat(nx,1,N+1),ones(1,N+1),'full');
            X{1} = X0;
            % State
            H_x = params.constraints.StateMatrix; 
            h_x = params.constraints.StateRHS; 
            % Input
            H_u = params.constraints.InputMatrix; 
            h_u = params.constraints.InputRHS; 
            % Set objective and constraints 
            objective = 0; 
            constraints = [H_x * X{1} <= h_x];
            for k = 1:1:N
                constraints = [constraints, X{k+1} == params.model.A * X{k} + params.model.B * U{k}];
                constraints = [constraints, H_x * X{k+1} <= h_x];
                constraints = [constraints, H_u * U{k} <= h_u];
                objective = objective + X{k}' * Q * X{k} + U{k}' * R * U{k};
            end
            constraints = [constraints, X{N+1} == zeros(size(X{N}))];
            opts = getOptions('forcesSolver');
            opts.printlevel = 0;
            obj.forces_optimizer = optimizerFORCES(constraints,objective,opts,X0,U{1});
        end

        function [u, u_info] = eval(obj,x)
            %% evaluate control action by solving MPC problem, e.g.
            [optimizer_out,errorcode,info] = obj.forces_optimizer(x);
            u = optimizer_out;
            objective = info.pobj;
            solvetime = info.solvetime;

            feasible = true;
            if any(errorcode ~= 1)
                feasible = false;
                warning('MPC infeasible');
            end
            
            u_info = struct('ctrl_feas',feasible,'objective',objective,'solvetime',solvetime);
        end
    end
end