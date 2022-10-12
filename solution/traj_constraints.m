%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [s_max, y_max, u_max, J_u, df_max, vf_max, traj_feas] = traj_constraints(x,u,params)
    % YOUR CODE HERE
    s_max = max([max(abs(x(1,:))),max(abs(x(3,:)))]);
    y_max = max(abs(x(2,:))); 
    u_max = max(max(abs(u)));
    %for i1 = 1:1:params.model.HorizonLength
    for i1 = 1:1:size(u,2)
        u_tmp = u(:,i1);
        if i1 == 1
            J_u = u_tmp'*u_tmp;
        else
            J_u = J_u + u_tmp'*u_tmp;
        end
    end
    df_max = sqrt((x(1,end))^2 +(x(2,end))^2 +(x(3,end))^2);
    vf_max = sqrt((x(4,end))^2 +(x(5,end))^2 +(x(6,end))^2);
    traj_feas = (s_max <= params.constraints.MaxAbsPositionXZ)...
        & (y_max <= params.constraints.MaxAbsPositionY)...
        & (u_max <= params.constraints.MaxAbsThrust)...
        & (df_max <= params.constraints.MaxFinalPosDiff)...
        & (vf_max <= params.constraints.MaxFinalVelDiff);
    
end

