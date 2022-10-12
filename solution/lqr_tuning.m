%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [tuning_struct, i_opt] = lqr_tuning(x0,Q,params)
    % YOUR CODE HERE
    R = eye(params.model.nu);
    i_opt = NaN;
    fuel_lowest = 0;
    tuning_struct = [];
    for i1 = 1:1:size(Q, 2)
        tuning_struct_tmp.InitialCondition = x0; 
        tuning_struct_tmp.Qdiag = Q(:,i1);
        ctrl = LQR(diag(Q(:,i1)), R, params);
        if sum(size(ctrl.K)) == 0
            continue;
        end
        [Xt,Ut,~] = simulate(x0, ctrl, params); 
        [s_max, y_max, u_max, J_u, df_max, vf_max, traj_feas] = traj_constraints(Xt,Ut,params);
        tuning_struct_tmp.MaxAbsPositionXZ = s_max;
        tuning_struct_tmp.MaxAbsPositionY = y_max;
        tuning_struct_tmp.MaxAbsThrust = u_max;
        tuning_struct_tmp.InputCost = J_u;
        tuning_struct_tmp.MaxFinalPosDiff = df_max;
        tuning_struct_tmp.MaxFinalVelDiff = vf_max;
        tuning_struct_tmp.TrajFeasible = traj_feas;
        if traj_feas == 1
            if fuel_lowest == 0 
                i_opt = i1;
                fuel_lowest = J_u;
            elseif fuel_lowest >= J_u
                i_opt = i1;
                fuel_lowest = J_u;
            end
        end
        tuning_struct = [tuning_struct;tuning_struct_tmp];
    end
    
end