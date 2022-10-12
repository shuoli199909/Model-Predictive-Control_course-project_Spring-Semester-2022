%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function J_Nt = traj_cost(Xt,Ut,Q,R)
    % YOUR CODE HERE
    J_Nt = 0;
    for i1 = 1:1:width(Ut)
        Xt_tmp = Xt(:,i1);
        Ut_tmp = Ut(:,i1);
        J_Nt = J_Nt + Xt_tmp'*Q*Xt_tmp + Ut_tmp'*R*Ut_tmp;
    end
    
end