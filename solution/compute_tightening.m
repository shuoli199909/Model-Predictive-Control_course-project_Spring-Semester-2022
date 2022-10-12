%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function params = compute_tightening(K_tube,H_tube,h_tube,params)  
	% YOUR CODE HERE
    % x constraints
    H_x = params.constraints.StateMatrix;
    h_x = params.constraints.StateRHS;
    P_x = Polyhedron('A',H_x,'b',h_x);
    % u constraints
    H_u = params.constraints.InputMatrix;
    h_u = params.constraints.InputRHS;
    P_u = Polyhedron('A',H_u,'b',h_u);
    % epsilon constraints
    P_x_e = Polyhedron('A',H_tube,'B',h_tube); 
    P_u_e = affineMap(P_x_e,K_tube); 
    % Polyhedron minus 
    P_x_new = minus(P_x,P_x_e);
    P_u_new = minus(P_u,P_u_e);
    % constraints update 
    params.constraints.StateMatrix = P_x_new.A; 
    params.constraints.StateRHS = P_x_new.b; 
    params.constraints.InputMatrix = P_u_new.A; 
    params.constraints.InputRHS = P_u_new.b; 
end