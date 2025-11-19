%% Always (Globally) Operator: □[a,b]φ
% The signal must satisfy φ at all time steps in the interval [k_now+a, k_now+b]
% Robust semantics: minimum robustness over the time interval
%
% Inputs:
%   k_now: current time step
%   interval_k: [a, b] time interval (in discrete steps)
%   sub_handle: function handle that returns [r, F] for subformula φ
%               r = robustness variable, F = constraints
%
% Outputs:
%   r_op: robustness variable for □[a,b]φ
%   F_op: YALMIP constraints for the always operator
%
function [r_op, F_op] = robust_always(k_now, interval_k, sub_handle)
    a = interval_k(1);
    b = interval_k(2);

    % Initialize constraints
    F_op = [];

    % Collect robustness values over the time interval
    r_array = [];

    for k = (k_now + a):(k_now + b)
        % Get robustness and constraints for subformula at time k
        [r_sub, F_sub] = sub_handle(k);

        % Accumulate constraints
        F_op = [F_op, F_sub];

        % Collect robustness values
        r_array = [r_array; r_sub];
    end

    % Robustness of always: minimum over the interval
    % For MILP encoding, use YALMIP's min function
    if length(r_array) == 1
        r_op = r_array;
    else
        r_op = sdpvar(1, 1);
        F_op = [F_op, r_op <= r_array];  % r_op is the minimum
        F_op = [F_op, r_op == min(r_array)];  % YALMIP will encode this as MILP
    end
end
