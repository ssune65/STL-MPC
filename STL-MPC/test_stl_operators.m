%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%          STL Operators Unit Tests             %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Individual tests for each STL operator
% Run this to verify your installation and understand each operator
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all

fprintf('====================================\n');
fprintf('STL Operators Unit Tests\n');
fprintf('====================================\n\n');

%% Test 1: Atomic Predicate
fprintf('Test 1: Atomic Predicate\n');
fprintf('------------------------\n');

% Create a simple variable
x = sdpvar(2, 1);

% Predicate: x1 <= 5 (i.e., 5 - x1 >= 0)
% atomic_predicate implements c - h'*x >= 0
% So for x1 <= 5: c - h'*x = 5 - x1, i.e., h = [1; 0], c = 5
h = [1; 0];
c = 5;
[r, F] = atomic_predicate(x, h, c);

% Test with x = [3; 2]
x_test = [3; 2];
r_value = c - h' * x_test;  % Should be 5 - 3 = 2
fprintf('Predicate: x1 <= 5\n');
fprintf('Test value: x = [3; 2]\n');
fprintf('Robustness: %.2f (should be 2, positive means satisfied)\n', r_value);

% Test with x = [7; 2]
x_test = [7; 2];
r_value = c - h' * x_test;  % Should be 5 - 7 = -2
fprintf('Test value: x = [7; 2]\n');
fprintf('Robustness: %.2f (should be -2, negative means not satisfied)\n\n', r_value);

%% Test 2: Always Operator
fprintf('Test 2: Always Operator\n');
fprintf('------------------------\n');

% Create trajectory
N = 10;
x_traj = sdpvar(1, N+1);

% Predicate: x <= 5 at each time step
predicate = @(k) deal(5 - x_traj(k+1), []);

% Always: □[0,5](x <= 5)
[r_always, F_always] = robust_always_simple(0, [0, 5], predicate);

% Create an optimization problem to test
F = [F_always];
F = [F, x_traj == [1, 2, 3, 4, 5, 6, 4, 3, 2, 1, 0]];  % Violates at k=5 (x=6)

% Evaluate robustness
assign(x_traj, [1, 2, 3, 4, 5, 6, 4, 3, 2, 1, 0]);
r_val = value(r_always);

fprintf('Specification: □[0,5](x <= 5)\n');
fprintf('Trajectory: [1, 2, 3, 4, 5, 6, ...]\n');
fprintf('Robustness: %.2f\n', r_val);
fprintf('Expected: min(5-1, 5-2, 5-3, 5-4, 5-5, 5-6) = min(4,3,2,1,0,-1) = -1\n');
fprintf('Interpretation: Violated because x=6 at k=5\n\n');

%% Test 3: Eventually Operator
fprintf('Test 3: Eventually Operator\n');
fprintf('---------------------------\n');

% Create trajectory
x_traj2 = sdpvar(1, N+1);

% Predicate: x >= 8
predicate2 = @(k) deal(x_traj2(k+1) - 8, []);

% Eventually: ◇[0,10](x >= 8)
[r_eventually, F_eventually] = robust_eventually_simple(0, [0, 10], predicate2);

% Test trajectory
F2 = [F_eventually];
F2 = [F2, x_traj2 == [1, 2, 3, 4, 5, 9, 7, 6, 5, 4, 3]];  % Reaches 9 at k=5

assign(x_traj2, [1, 2, 3, 4, 5, 9, 7, 6, 5, 4, 3]);
r_val2 = value(r_eventually);

fprintf('Specification: ◇[0,10](x >= 8)\n');
fprintf('Trajectory: [1, 2, 3, 4, 5, 9, 7, ...]\n');
fprintf('Robustness: %.2f\n', r_val2);
fprintf('Expected: max(1-8, 2-8, ..., 9-8, ...) = max(-7,-6,...,1,...) = 1\n');
fprintf('Interpretation: Satisfied because x=9 >= 8 at k=5\n\n');

%% Test 4: AND Operator
fprintf('Test 4: AND Operator\n');
fprintf('--------------------\n');

% Two conditions
r1_val = 2.0;   % First condition robustness
r2_val = 3.0;   % Second condition robustness

r_and = min(r1_val, r2_val);

fprintf('Condition 1 robustness: %.2f\n', r1_val);
fprintf('Condition 2 robustness: %.2f\n', r2_val);
fprintf('AND robustness: %.2f\n', r_and);
fprintf('Expected: min(2.0, 3.0) = 2.0\n');
fprintf('Interpretation: Limited by the weaker condition\n\n');

%% Test 5: OR Operator
fprintf('Test 5: OR Operator\n');
fprintf('-------------------\n');

% Two conditions
r1_val = -1.0;  % First condition not satisfied
r2_val = 2.0;   % Second condition satisfied

r_or = max(r1_val, r2_val);

fprintf('Condition 1 robustness: %.2f\n', r1_val);
fprintf('Condition 2 robustness: %.2f\n', r2_val);
fprintf('OR robustness: %.2f\n', r_or);
fprintf('Expected: max(-1.0, 2.0) = 2.0\n');
fprintf('Interpretation: Satisfied because at least one condition is satisfied\n\n');

%% Test 6: NOT Operator
fprintf('Test 6: NOT Operator\n');
fprintf('--------------------\n');

r_val = 3.0;
r_not = -r_val;

fprintf('Original robustness: %.2f (satisfied)\n', r_val);
fprintf('NOT robustness: %.2f (not satisfied)\n', r_not);
fprintf('Expected: -3.0\n');
fprintf('Interpretation: Negation flips the satisfaction\n\n');

%% Test 7: Complex Specification
fprintf('Test 7: Complex Specification\n');
fprintf('------------------------------\n');
fprintf('Specification: □[0,5](x <= 10) ∧ ◇[5,10](x >= 8)\n');
fprintf('Meaning: Always stay below 10 (first 5 steps) AND eventually reach 8 (steps 5-10)\n\n');

% Create trajectory
N2 = 15;
x_complex = sdpvar(1, N2+1);

% Subspec 1: □[0,5](x <= 10)
pred1 = @(k) deal(10 - x_complex(k+1), []);
[r1, F1] = robust_always_simple(0, [0, 5], pred1);

% Subspec 2: ◇[5,10](x >= 8)
pred2 = @(k) deal(x_complex(k+1) - 8, []);
[r2, F2] = robust_eventually_simple(0, [5, 10], pred2);

% Combined with AND
[r_combined, F_combined] = robust_and_simple(@() deal(r1, F1), @() deal(r2, F2));

% Test trajectory: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 8, 7, 6, 5, 4, 3]
traj = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 8, 7, 6, 5, 4, 3];
assign(x_complex, traj);

fprintf('Test trajectory: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 8, 7, ...]\n');
fprintf('Sub-spec 1 (□[0,5](x<=10)) robustness: %.2f\n', value(r1));
fprintf('Sub-spec 2 (◇[5,10](x>=8)) robustness: %.2f\n', value(r2));
fprintf('Combined (AND) robustness: %.2f\n', value(r_combined));
fprintf('Interpretation: Both conditions satisfied\n\n');

%% Summary
fprintf('====================================\n');
fprintf('All tests completed!\n');
fprintf('====================================\n');
fprintf('\nKey takeaways:\n');
fprintf('1. Positive robustness = specification satisfied\n');
fprintf('2. Negative robustness = specification violated\n');
fprintf('3. Larger robustness = more robust satisfaction\n');
fprintf('4. Always → minimum, Eventually → maximum\n');
fprintf('5. AND → minimum, OR → maximum, NOT → negation\n');

%% Helper functions

function [r, F] = atomic_predicate(x, h, c)
    F = [];
    r = c - h' * x;
end

function [r_op, F_op] = robust_always_simple(k_now, interval_k, sub_handle)
    a = interval_k(1);
    b = interval_k(2);
    F_op = [];
    r_array = [];

    for k = (k_now + a):(k_now + b)
        [r_sub, F_sub] = sub_handle(k);
        F_op = [F_op, F_sub];
        r_array = [r_array; r_sub];
    end

    if length(r_array) == 1
        r_op = r_array;
    else
        r_op = min(r_array);
    end
end

function [r_op, F_op] = robust_eventually_simple(k_now, interval_k, sub_handle)
    a = interval_k(1);
    b = interval_k(2);
    F_op = [];
    r_array = [];

    for k = (k_now + a):(k_now + b)
        [r_sub, F_sub] = sub_handle(k);
        F_op = [F_op, F_sub];
        r_array = [r_array; r_sub];
    end

    if length(r_array) == 1
        r_op = r_array;
    else
        r_op = max(r_array);
    end
end

function [r_op, F_op] = robust_and_simple(handle_A, handle_B)
    [r_A, F_A] = handle_A();
    [r_B, F_B] = handle_B();
    F_op = [F_A, F_B];
    r_op = min(r_A, r_B);
end
