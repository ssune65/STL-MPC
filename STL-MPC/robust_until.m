%% UNTIL Operator: φ1 U[a,b] φ2
% φ1 must hold until φ2 becomes true within [a,b]
% Robust semantics: ρ(φ1 U[a,b] φ2) = max_{t∈[a,b]} min(ρ(φ2,t), min_{s∈[0,t)} ρ(φ1,s))
%
% Inputs:
%   k_now: current time step
%   interval_k: [a, b] time interval
%   handle_A: function handle for φ1 (must hold until)
%   handle_B: function handle for φ2 (eventually holds)
%
% Outputs:
%   r_op: robustness variable for φ1 U[a,b] φ2
%   F_op: YALMIP constraints for the until operator
%
function [r_op, F_op] = robust_until(k_now, interval_k, handle_A, handle_B)
    a = interval_k(1);
    b = interval_k(2);

    % Initialize constraints
    F_op = [];

    % Collect candidates for maximum
    candidates = [];

    for t = (k_now + a):(k_now + b)
        % Get robustness for φ2 at time t
        [r_B_t, F_B_t] = handle_B(t);
        F_op = [F_op, F_B_t];

        % Collect robustness for φ1 from k_now to t-1
        r_A_array = [];
        for s = k_now:(t-1)
            [r_A_s, F_A_s] = handle_A(s);
            F_op = [F_op, F_A_s];
            r_A_array = [r_A_array; r_A_s];
        end

        % min over φ1's robustness
        if isempty(r_A_array)
            r_A_min = sdpvar(1, 1);
            F_op = [F_op, r_A_min == inf];  % No constraint if empty
        else
            r_A_min = sdpvar(1, 1);
            F_op = [F_op, r_A_min <= r_A_array];
            F_op = [F_op, r_A_min == min(r_A_array)];
        end

        % min of (φ2 at t) and (φ1 until t)
        r_cand = sdpvar(1, 1);
        if isempty(r_A_array)
            F_op = [F_op, r_cand == r_B_t];
        else
            F_op = [F_op, r_cand <= r_B_t, r_cand <= r_A_min];
            F_op = [F_op, r_cand == min(r_B_t, r_A_min)];
        end

        candidates = [candidates; r_cand];
    end

    % Maximum over all candidates
    r_op = sdpvar(1, 1);
    F_op = [F_op, r_op >= candidates];
    F_op = [F_op, r_op == max(candidates)];
end
