%% NOT Operator (Negation): ¬φ
% The formula must not be satisfied
% Robust semantics: negation of the robustness value
%
% Inputs:
%   handle_A: function handle for φ that returns [r, F]
%
% Outputs:
%   r_op: robustness variable for ¬φ
%   F_op: YALMIP constraints for the NOT operator
%
function [r_op, F_op] = robust_not(handle_A)
    % Get robustness and constraints for subformula
    [r_A, F_A] = handle_A();

    % Constraints remain the same
    F_op = F_A;

    % Robustness of NOT: negation
    r_op = -r_A;
end
