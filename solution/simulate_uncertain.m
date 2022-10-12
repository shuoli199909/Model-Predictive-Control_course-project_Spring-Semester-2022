%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Xt,Ut,u_info] = simulate_uncertain(x0, ctrl, Wt, params)
% YOUR CODE HERE
A = params.model.A; 
B = params.model.B; 
Nt = params.model.HorizonLength; 
Xt = x0;
Ut = [];
u_info = [];
for i1 = 1:1:Nt
    x_tmp_0 = Xt(:,end);
    [U_tmp,u_info_tmp] = ctrl.eval(x_tmp_0);
    x_tmp_1 = A*x_tmp_0 + B*U_tmp + Wt(:,i1);
    Xt = [Xt,x_tmp_1];
    Ut = [Ut,U_tmp];
    u_info = [u_info,u_info_tmp];
end

end