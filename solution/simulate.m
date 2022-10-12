%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Xt,Ut,u_info] = simulate(x0, ctrl, params)

% YOUR CODE HERE
% Hint: you can access the control command with ctrl.eval(x(:,i))
A = params.model.A; 
B = params.model.B; 
N_t = params.model.HorizonLength; 
Xt = x0;
Ut = [];
u_info = [];
for i1 = 1:1:N_t
    x_tmp_0 = Xt(:,end);
    [U_tmp,u_info_tmp] = ctrl.eval(x_tmp_0);
    x_tmp_1 = A*x_tmp_0 + B*U_tmp;
    Xt = [Xt,x_tmp_1];
    Ut = [Ut,U_tmp];
    u_info = [u_info,u_info_tmp];
end

end