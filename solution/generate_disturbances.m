%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Wt = generate_disturbances(params)
    % YOUR CODE HERE
    H_w = params.constraints.DisturbanceMatrix; 
    h_w = params.constraints.DisturbanceRHS;
    P = Polyhedron('A',H_w,'b',h_w);
    N_t = params.model.HorizonLength; 
    Wt = [];
    for i1 = 1:1:N_t 
        W_tmp = P.randomPoint;
        Wt = [Wt,W_tmp];
    end
    
end