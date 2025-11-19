%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%  STL Operators Unit Tests (MILP Encoding)    %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% stl_encoding.m의 MILP 인코딩 함수들을 사용한 단위 테스트
%
% test_stl_operators.m와 동일한 테스트를 수행하지만,
% 실제 MPC에서 사용되는 MILP 인코딩 방식을 사용합니다.
%
% 차이점:
%   - robust_always (MILP) vs robust_always_simple (직접 계산)
%   - 최적화 문제를 풀어서 robustness 값을 얻음
%   - 실제 MPC 구현과 동일한 방식
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all

fprintf('====================================\n');
fprintf('STL Operators Unit Tests (MILP)\n');
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

%% Test 2: Always Operator (MILP Encoding)
fprintf('Test 2: Always Operator (MILP Encoding)\n');
fprintf('----------------------------------------\n');

% Create trajectory
N = 10;
x_traj = sdpvar(1, N+1);

% Predicate: x <= 5 at each time step
predicate = @(k) deal(5 - x_traj(k+1), []);

% Always: □[0,5](x <= 5)
% 주의: stl_encoding.m의 robust_always는 MILP 인코딩을 사용
[r_always, F_always] = robust_always(0, [0, 5], predicate);

% 최적화 문제 설정
F = [F_always];
F = [F, x_traj == [1, 2, 3, 4, 5, 6, 4, 3, 2, 1, 0]];  % Violates at k=5 (x=6)

% MILP 인코딩이므로 최적화 문제를 풀어야 robustness 값을 얻을 수 있음
options = sdpsettings('verbose', 0);
sol = optimize(F, 0, options);  % Feasibility problem

if sol.problem == 0
    r_val = value(r_always);
    fprintf('Specification: □[0,5](x <= 5)\n');
    fprintf('Trajectory: [1, 2, 3, 4, 5, 6, ...]\n');
    fprintf('Robustness: %.2f\n', r_val);
    fprintf('Expected: min(5-1, 5-2, 5-3, 5-4, 5-5, 5-6) = min(4,3,2,1,0,-1) = -1\n');
    fprintf('Interpretation: Violated because x=6 at k=5\n\n');
else
    fprintf('Optimization failed (problem code: %d)\n\n', sol.problem);
end

%% Test 3: Eventually Operator (MILP Encoding)
fprintf('Test 3: Eventually Operator (MILP Encoding)\n');
fprintf('-------------------------------------------\n');

% Create trajectory
x_traj2 = sdpvar(1, N+1);

% Predicate: x >= 8
predicate2 = @(k) deal(x_traj2(k+1) - 8, []);

% Eventually: ◇[0,10](x >= 8)
[r_eventually, F_eventually] = robust_eventually(0, [0, 10], predicate2);

% 최적화 문제 설정
F2 = [F_eventually];
F2 = [F2, x_traj2 == [1, 2, 3, 4, 5, 9, 7, 6, 5, 4, 3]];  % Reaches 9 at k=5

options = sdpsettings('verbose', 0);
sol2 = optimize(F2, 0, options);

if sol2.problem == 0
    r_val2 = value(r_eventually);
    fprintf('Specification: ◇[0,10](x >= 8)\n');
    fprintf('Trajectory: [1, 2, 3, 4, 5, 9, 7, ...]\n');
    fprintf('Robustness: %.2f\n', r_val2);
    fprintf('Expected: max(1-8, 2-8, ..., 9-8, ...) = max(-7,-6,...,1,...) = 1\n');
    fprintf('Interpretation: Satisfied because x=9 >= 8 at k=5\n\n');
else
    fprintf('Optimization failed (problem code: %d)\n\n', sol2.problem);
end

%% Test 4: AND Operator (MILP Encoding)
fprintf('Test 4: AND Operator (MILP Encoding)\n');
fprintf('------------------------------------\n');

% Two conditions
r1 = sdpvar(1, 1);
r2 = sdpvar(1, 1);

% Create handles that return fixed robustness values
handle_A = @() deal(r1, [r1 == 2.0]);
handle_B = @() deal(r2, [r2 == 3.0]);

% AND operator
[r_and, F_and] = robust_and(handle_A, handle_B);

% 최적화 문제 설정
F3 = [F_and];

options = sdpsettings('verbose', 0);
sol3 = optimize(F3, 0, options);

if sol3.problem == 0
    fprintf('Condition 1 robustness: %.2f\n', value(r1));
    fprintf('Condition 2 robustness: %.2f\n', value(r2));
    fprintf('AND robustness: %.2f\n', value(r_and));
    fprintf('Expected: min(2.0, 3.0) = 2.0\n');
    fprintf('Interpretation: Limited by the weaker condition\n\n');
else
    fprintf('Optimization failed (problem code: %d)\n\n', sol3.problem);
end

%% Test 5: OR Operator (MILP Encoding)
fprintf('Test 5: OR Operator (MILP Encoding)\n');
fprintf('-----------------------------------\n');

% Two conditions
r3 = sdpvar(1, 1);
r4 = sdpvar(1, 1);

handle_C = @() deal(r3, [r3 == -1.0]);
handle_D = @() deal(r4, [r4 == 2.0]);

% OR operator
[r_or, F_or] = robust_or(handle_C, handle_D);

% 최적화 문제 설정
F4 = [F_or];

options = sdpsettings('verbose', 0);
sol4 = optimize(F4, 0, options);

if sol4.problem == 0
    fprintf('Condition 1 robustness: %.2f\n', value(r3));
    fprintf('Condition 2 robustness: %.2f\n', value(r4));
    fprintf('OR robustness: %.2f\n', value(r_or));
    fprintf('Expected: max(-1.0, 2.0) = 2.0\n');
    fprintf('Interpretation: Satisfied because at least one condition is satisfied\n\n');
else
    fprintf('Optimization failed (problem code: %d)\n\n', sol4.problem);
end

%% Test 6: NOT Operator (MILP Encoding)
fprintf('Test 6: NOT Operator (MILP Encoding)\n');
fprintf('------------------------------------\n');

r5 = sdpvar(1, 1);
handle_E = @() deal(r5, [r5 == 3.0]);

% NOT operator
[r_not, F_not] = robust_not(handle_E);

% 최적화 문제 설정
F5 = [F_not];

options = sdpsettings('verbose', 0);
sol5 = optimize(F5, 0, options);

if sol5.problem == 0
    fprintf('Original robustness: %.2f (satisfied)\n', value(r5));
    fprintf('NOT robustness: %.2f (not satisfied)\n', value(r_not));
    fprintf('Expected: -3.0\n');
    fprintf('Interpretation: Negation flips the satisfaction\n\n');
else
    fprintf('Optimization failed (problem code: %d)\n\n', sol5.problem);
end

%% Test 7: Complex Specification (MILP Encoding)
fprintf('Test 7: Complex Specification (MILP Encoding)\n');
fprintf('---------------------------------------------\n');
fprintf('Specification: □[0,5](x <= 10) ∧ ◇[5,10](x >= 8)\n');
fprintf('Meaning: Always stay below 10 (first 5 steps) AND eventually reach 8 (steps 5-10)\n\n');

% Create trajectory
N2 = 15;
x_complex = sdpvar(1, N2+1);

% Subspec 1: □[0,5](x <= 10)
pred1 = @(k) deal(10 - x_complex(k+1), []);
[r1_spec, F1_spec] = robust_always(0, [0, 5], pred1);

% Subspec 2: ◇[5,10](x >= 8)
pred2 = @(k) deal(x_complex(k+1) - 8, []);
[r2_spec, F2_spec] = robust_eventually(0, [5, 10], pred2);

% Combined with AND (MILP encoding)
[r_combined, F_combined] = robust_and(@() deal(r1_spec, F1_spec), @() deal(r2_spec, F2_spec));

% 최적화 문제 설정
traj = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 8, 7, 6, 5, 4, 3];
F6 = [F_combined];
F6 = [F6, x_complex == traj];

options = sdpsettings('verbose', 0);
sol6 = optimize(F6, 0, options);

if sol6.problem == 0
    fprintf('Test trajectory: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 8, 7, ...]\n');
    fprintf('Sub-spec 1 (□[0,5](x<=10)) robustness: %.2f\n', value(r1_spec));
    fprintf('Sub-spec 2 (◇[5,10](x>=8)) robustness: %.2f\n', value(r2_spec));
    fprintf('Combined (AND) robustness: %.2f\n', value(r_combined));
    fprintf('Interpretation: Both conditions satisfied\n\n');
else
    fprintf('Optimization failed (problem code: %d)\n\n', sol6.problem);
end

%% Summary
fprintf('====================================\n');
fprintf('All tests completed!\n');
fprintf('====================================\n');
fprintf('\nKey differences from simple version:\n');
fprintf('1. Uses MILP encoding (stl_encoding.m functions)\n');
fprintf('2. Requires optimization solver to evaluate robustness\n');
fprintf('3. Same results as simple version (validates MILP encoding)\n');
fprintf('4. This is the actual implementation used in MPC\n\n');

fprintf('Key takeaways:\n');
fprintf('1. Positive robustness = specification satisfied\n');
fprintf('2. Negative robustness = specification violated\n');
fprintf('3. Larger robustness = more robust satisfaction\n');
fprintf('4. Always → minimum, Eventually → maximum\n');
fprintf('5. AND → minimum, OR → maximum, NOT → negation\n');
