%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef LQR
    properties
        P
        K
    end
    
    methods
        %constructor
        function obj = LQR(Q,R,params)
            % YOUR CODE HERE
            % obj.K = ... (save feedback matrix for use in eval function)
            A = params.model.A;
            B = params.model.B;
            [obj.P,obj.K,~] = idare(A,B,Q,R,zeros(size(B)),eye(size(A)));
            obj.K = -obj.K;
               
        end
        
        function [u, u_info] = eval(obj,x)
            % YOUR CODE HERE
            % u = ...
            if sum(size(obj.K)) == 0
                u = [];
                u_info = struct('ctrl_feas', false);
            else
                u = obj.K*x;
                u_info = struct('ctrl_feas',true);
            end
        end
    end
end