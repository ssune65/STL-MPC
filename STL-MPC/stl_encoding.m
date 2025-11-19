%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%             STL Encoding Function             %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Reference: Raman et al., "Model Predictive Control with Signal
% Temporal Logic Specifications", CDC 2014
%
% This code implements STL robust semantics encoding for MPC using YALMIP
%
% Robust semantics:
%   - Always: ρ(□[a,b]φ) = min_{t∈[a,b]} ρ(φ,t)
%   - Eventually: ρ(◇[a,b]φ) = max_{t∈[a,b]} ρ(φ,t)
%   - And: ρ(φ1 ∧ φ2) = min(ρ(φ1), ρ(φ2))
%   - Or: ρ(φ1 ∨ φ2) = max(ρ(φ1), ρ(φ2))
%   - Not: ρ(¬φ) = -ρ(φ)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% 경로 설정
% clear all
%
% addpath(genpath('C:/path/YALMIP-master'))
% savepath

%% STL operations structure
% function ops = stl_ops()
%     ops.always = @robust_always;
%     ops.eventually = @robust_eventually;
%     ops.and = @robust_and;
%     ops.or = @robust_or;
%     ops.not = @robust_not;
%     ops.until = @robust_until;
% end

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

%% Eventually (Finally) Operator: ◇[a,b]φ
% The signal must satisfy φ at least once in the interval [k_now+a, k_now+b]
% Robust semantics: maximum robustness over the time interval
%
% Inputs:
%   k_now: current time step
%   interval_k: [a, b] time interval (in discrete steps)
%   sub_handle: function handle that returns [r, F] for subformula φ
%
% Outputs:
%   r_op: robustness variable for ◇[a,b]φ
%   F_op: YALMIP constraints for the eventually operator
%
function [r_op, F_op] = robust_eventually(k_now, interval_k, sub_handle)
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

    % Robustness of eventually: maximum over the interval
    % For MILP encoding, use YALMIP's max function
    if length(r_array) == 1
        r_op = r_array;
    else
        r_op = sdpvar(1, 1);
        F_op = [F_op, r_op >= r_array];  % r_op is the maximum
        F_op = [F_op, r_op == max(r_array)];  % YALMIP will encode this as MILP
    end
end

%% AND Operator (Conjunction): φ1 ∧ φ2
% Both formulas must be satisfied
% Robust semantics: minimum of the two robustness values
%
% Inputs:
%   handle_A: function handle for φ1 that returns [r1, F1]
%   handle_B: function handle for φ2 that returns [r2, F2]
%
% Outputs:
%   r_op: robustness variable for φ1 ∧ φ2
%   F_op: YALMIP constraints for the AND operator
%
function [r_op, F_op] = robust_and(handle_A, handle_B)
    % Get robustness and constraints for both subformulas
    [r_A, F_A] = handle_A();
    [r_B, F_B] = handle_B();

    % Combine constraints
    F_op = [F_A, F_B];

    % Robustness of AND: minimum of the two
    r_op = sdpvar(1, 1);
    F_op = [F_op, r_op <= r_A, r_op <= r_B];
    F_op = [F_op, r_op == min(r_A, r_B)];  % YALMIP will encode this as MILP
end

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

    % Combine constraints
    F_op = [F_A, F_B];

    % Robustness of OR: maximum of the two
    r_op = sdpvar(1, 1);
    F_op = [F_op, r_op >= r_A, r_op >= r_B];
    F_op = [F_op, r_op == max(r_A, r_B)];  % YALMIP will encode this as MILP
end

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

%% Atomic Predicate Helper
% Creates a predicate of the form: μ(x) = c - h'*x ≥ 0
% Robustness is simply: ρ = c - h'*x
%
% Examples:
%   x1 ≤ 5: Use h = [1; 0], c = 5 → ρ = 5 - x1
%   x1 ≥ 8: Use h = [-1; 0], c = -8 → ρ = -8 - (-x1) = x1 - 8
%
% Inputs:
%   x: state/signal variable (sdpvar)
%   h: coefficient vector
%   c: constant threshold
%
% Outputs:
%   r: robustness variable
%   F: constraints (can be empty for simple predicates)
%
function [r, F] = atomic_predicate(x, h, c)
    F = [];
    r = c - h' * x;
end

%% Example: Safety specification
% Example: □[0,T](x1 ≤ 5 ∧ x2 ≤ 5) - Always stay within bounds
%
function example_safety()
    % Time horizon
    T = 10;

    % State variables (2D state, T+1 time steps)
    x = sdpvar(2, T+1);

    % Define atomic predicates
    predicate_x1 = @(k) atomic_predicate(x(:,k+1), [1;0], 5);  % x1 ≤ 5
    predicate_x2 = @(k) atomic_predicate(x(:,k+1), [0;1], 5);  % x2 ≤ 5

    % Define AND of predicates
    and_handle = @(k) robust_and(@() predicate_x1(k), @() predicate_x2(k));

    % Define ALWAYS over time horizon
    [r_stl, F_stl] = robust_always(0, [0, T], and_handle);

    % STL constraint: robustness must be positive (satisfied)
    F_stl = [F_stl, r_stl >= 0];

    fprintf('Safety specification encoded successfully!\n');
    fprintf('Number of constraints: %d\n', length(F_stl));
end

%% Example: Reach-Avoid specification
% Example: ◇[0,T1](reach) ∧ □[0,T2](avoid) - Eventually reach goal and always avoid obstacle
%
function example_reach_avoid()
    % Time horizon
    T1 = 10;  % Eventually reach within T1
    T2 = 10;  % Always avoid within T2

    % State variables (2D state)
    x = sdpvar(2, T2+1);

    % Goal region: x1 ≥ 8 ∧ x2 ≥ 8
    goal_x1 = @(k) atomic_predicate(x(:,k+1), [-1;0], -8);  % -x1 + 8 ≤ 0 → x1 ≥ 8
    goal_x2 = @(k) atomic_predicate(x(:,k+1), [0;-1], -8);  % -x2 + 8 ≤ 0 → x2 ≥ 8
    goal_handle = @(k) robust_and(@() goal_x1(k), @() goal_x2(k));

    % Eventually reach goal
    [r_reach, F_reach] = robust_eventually(0, [0, T1], goal_handle);

    % Obstacle: ¬(x1 ∈ [4,6] ∧ x2 ∈ [4,6])
    obs_x1_lower = @(k) atomic_predicate(x(:,k+1), [-1;0], -4);  % x1 ≥ 4
    obs_x1_upper = @(k) atomic_predicate(x(:,k+1), [1;0], 6);    % x1 ≤ 6
    obs_x2_lower = @(k) atomic_predicate(x(:,k+1), [0;-1], -4);  % x2 ≥ 4
    obs_x2_upper = @(k) atomic_predicate(x(:,k+1), [0;1], 6);    % x2 ≤ 6

    obs_x1 = @(k) robust_and(@() obs_x1_lower(k), @() obs_x1_upper(k));
    obs_x2 = @(k) robust_and(@() obs_x2_lower(k), @() obs_x2_upper(k));
    obs_in = @(k) robust_and(@() obs_x1(k), @() obs_x2(k));
    avoid_handle = @(k) robust_not(@() obs_in(k));

    % Always avoid obstacle
    [r_avoid, F_avoid] = robust_always(0, [0, T2], avoid_handle);

    % Combine reach and avoid
    reach_avoid_handle = @() deal_return(robust_and(...
        @() deal(r_reach, F_reach), ...
        @() deal(r_avoid, F_avoid)));
    [r_stl, F_stl] = reach_avoid_handle();

    % STL constraint: robustness must be positive
    F_stl = [F_stl, r_stl >= 0];

    fprintf('Reach-avoid specification encoded successfully!\n');
    fprintf('Number of constraints: %d\n', length(F_stl));
end

% Helper function to handle [r,F] returns
function [r, F] = deal_return(varargin)
    if nargout == 2
        [r, F] = varargin{:};
    else
        r = varargin{1};
    end
end
