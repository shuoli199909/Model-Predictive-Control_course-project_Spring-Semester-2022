%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Ac, Bc] = generate_system_cont(params)
    % YOUR CODE HERE
    m = params.model.Mass; 
    mue = params.model.GravitationalParameter; 
    R = params.model.TargetRadius; 
    wn = sqrt(mue/(R)^3);
    Ac = zeros(6,6);
    Ac(1,4) = 1; 
    Ac(2,5) = 1;
    Ac(3,6) = 1;
    Ac(4,1) = 3*wn^2;
    Ac(4,5) = 2*wn; 
    Ac(5,4) = -2*wn; 
    Ac(6,3) = -wn^2;
    Bc = zeros(6,3);
    Bc(4,1) = 1/m; 
    Bc(5,2) = 1/m; 
    Bc(6,3) = 1/m; 
end