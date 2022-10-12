%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [H_u, h_u, H_x, h_x] = generate_constraints(params)
    % YOUR CODE HERE
    H_u = [eye(3);-eye(3)];
    h_u = params.constraints.MaxAbsThrust*ones(6,1); 
    H_x = [[eye(3);-eye(3)],zeros(6,3)];
    s_xz = params.constraints.MaxAbsPositionXZ; 
    s_y = params.constraints.MaxAbsPositionY;
    h_x = [s_xz;s_y;s_xz;s_xz;s_y;s_xz];
end