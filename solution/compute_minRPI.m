%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [H_tube,h_tube,n_iter] = compute_minRPI(K_tube,params)
    % YOUR CODE HERE
    A = params.model.A; 
    B = params.model.B; 
    A_ = pinv(A + B*K_tube);
    H_w = params.constraints.DisturbanceMatrix; 
    h_w = params.constraints.DisturbanceRHS;
    P_w = Polyhedron('A',H_w,'b',h_w);
    P_w = P_w.minVRep();
    P_w = P_w.minHRep();
    P_a = P_w;
    n_iter = 0; 
    if_converge = false;
    while if_converge == false
        n_iter = n_iter + 1;
        P_b = Polyhedron('A',(P_a.A)*A_,'b',P_a.b);
        P_b = plus(P_b,P_w,'vrep');
        P_b = P_b.minVRep();
        P_b = P_b.minHRep();
        if eq(P_a,P_b) == 1
            if_converge = true;
        end
        P_a = P_b; 
    end
    H_tube = P_a.A;
    h_tube = P_a.b;
end