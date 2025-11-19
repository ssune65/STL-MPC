%% Atomic Predicate Helper
% Creates a predicate of the form: μ(x) = c - h'*x ≥ 0
% Robustness is simply: ρ = c - h'*x
%
% This is the standard STL robustness definition for atomic predicates.
%
% Examples:
%   x1 ≤ 5: Use h = [1; 0], c = 5 → ρ = 5 - x1
%   x1 ≥ 8: Use h = [-1; 0], c = -8 → ρ = -8 - (-x1) = x1 - 8
%   x1 + x2 ≤ 10: Use h = [1; 1], c = 10 → ρ = 10 - (x1 + x2)
%
% Inputs:
%   x: state/signal variable (sdpvar or numeric vector)
%   h: coefficient vector
%   c: constant threshold
%
% Outputs:
%   r: robustness variable (ρ = c - h'*x)
%   F: constraints (empty for simple atomic predicates)
%
function [r, F] = atomic_predicate(x, h, c)
    F = [];
    r = c - h' * x;
end
