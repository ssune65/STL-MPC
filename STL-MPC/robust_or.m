%% OR Operator (Disjunction): φ1 ∨ φ2
% At least one formula must be satisfied
% Robust semantics: maximum of the two robustness values
%
% Inputs:
%   handle_A: function handle for φ1 that returns [r1, F1]
%   handle_B: function handle for φ2 that returns [r2, F2]
%
% Outputs:
%   r_op: robustness variable for φ1 ∨ φ2
%   F_op: YALMIP constraints for the OR operator
%
function [r_op, F_op] = robust_or(handle_A, handle_B)
    % Get robustness and constraints for both subformulas
    [r_A, F_A] = handle_A();
    [r_B, F_B] = handle_B();

    % Robustness of OR: maximum of the two
    r_op = max(r_A, r_B);
    % F_op = [F_op, r_op >= r_A, r_op <= r_B];
    % F_op = [F_op, r_op == max(r_A, r_B)];  % YALMIP will encode this as MILP

    % Combine constraints
    F_op = [F_A, F_B];
end
