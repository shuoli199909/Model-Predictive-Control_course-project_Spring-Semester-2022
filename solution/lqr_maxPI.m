%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% BRIEF:
%   Template for explicit invariant set computation. You MUST NOT change
%   the output.
% INPUT:
%   Q, R: State and input weighting matrix
% OUTPUT:
%   H, h: Describes polytopic X_LQR = {x | H * x <= h}

function [H, h] = lqr_maxPI(Q,R,params)
	% YOUR CODE HERE
    obj_ctrl = LQR(Q,R,params); 
    A = params.model.A;
    B = params.model.B;
    A_new = A + B*obj_ctrl.K;
    system = LTISystem('A', A_new);
    % State
    H_x = params.constraints.StateMatrix; 
    h_x = params.constraints.StateRHS; 
    x_min = -Inf([size(H_x,2),1]);
    x_max = Inf([size(H_x,2),1]);
    for i1 = 1:1:length(params.constraints.StateRHS)
        vec_tmp = H_x(i1,:)*h_x(i1);
        index_tmp = find(vec_tmp~=0);
        if sum(vec_tmp) > 0
            x_max(index_tmp) = sum(vec_tmp);
        else 
            x_min(index_tmp) = sum(vec_tmp);
        end
    end
    % Input
    H_u = params.constraints.InputMatrix; 
    h_u = params.constraints.InputRHS; 
    u_min = -Inf([size(H_u,2),1]);
    u_max = Inf([size(H_u,2),1]);
    for i1 = 1:1:length(params.constraints.InputRHS)
        vec_tmp = H_u(i1,:)*h_u(i1);
        index_tmp = find(vec_tmp~=0);
        if sum(vec_tmp) > 0
            u_max(index_tmp) = sum(vec_tmp);
        else 
            u_min(index_tmp) = sum(vec_tmp);
        end
    end
    X_trans = Polyhedron('A', ...
    [eye(size(obj_ctrl.P)); -eye(size(obj_ctrl.P)); obj_ctrl.K; -obj_ctrl.K], ...
        'b', ...
    [system.x.max;-system.x.min; u_max;-u_min]);
    system.x.with('setConstraint');
    system.x.setConstraint = X_trans;
    
    InvSetLQR = system.invariantSet();
    
    H = InvSetLQR.A;
    h = InvSetLQR.b;
end

