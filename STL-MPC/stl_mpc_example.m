%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       STL-MPC 구현 예제 (한국어 주석)         %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% STL 제약 조건을 사용한 MPC의 완전한 예제
% 참고문헌: Raman et al., "Model Predictive Control with Signal
% Temporal Logic Specifications", CDC 2014
%
% 시나리오:
%   이중 적분기 시스템이 원점에서 출발하여 목표 위치에 도달하되,
%   복잡한 STL 제약 조건을 만족해야 합니다.
%
% STL 제약:
%   φ = □[0,10](|x1| ≤ 8) ∧ ◇[5,15](x1 ≥ 7) ∧ □[0,20](|x2| ≤ 6)
%   1. □[0,10](|x1| ≤ 8): 처음 10스텝 동안 위치가 항상 [-8, 8] 범위 내
%   2. ◇[5,15](x1 ≥ 7): 5~15스텝 사이에 언젠가 위치가 7 이상
%   3. □[0,20](|x2| ≤ 6): 전체 구간 동안 속도가 항상 [-6, 6] 범위 내
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all

%% 경로 설정 (필요시 주석 해제 후 수정)
% addpath(genpath('C:/path/to/YALMIP-master'))
% addpath(genpath('C:/path/to/solver'))  % 예: Gurobi, CPLEX, MOSEK
% savepath

%% 1. 시스템 동역학
fprintf('========================================\n');
fprintf('STL-MPC 구현 예제\n');
fprintf('========================================\n\n');
fprintf('1. 시스템 동역학 정의 중...\n');

% 이중 적분기 모델: x_{k+1} = A*x_k + B*u_k
% 상태: [위치; 속도]
% 입력: 가속도

dt = 0.1;  % 샘플링 시간
A = [1, dt; 0, 1];
B = [0.5*dt^2; dt];

n = size(A, 1);  % 상태 차원
m = size(B, 2);  % 입력 차원

%% 2. MPC 파라미터
fprintf('2. MPC 파라미터 설정 중...\n');

N = 20;  % 예측 구간

% 상태 및 입력 제약
x_min = [-10; -6];
x_max = [10; 6];
u_min = -10;
u_max = 10;

%% 3. 결정 변수
fprintf('3. 최적화 변수 정의 중...\n');

% 상태 궤적
x = sdpvar(n, N+1);  % x(:,1)은 초기 상태, x(:,N+1)은 최종 상태

% 입력 궤적
u = sdpvar(m, N);

%% 4. MPC 기본 제약 조건
fprintf('4. MPC 기본 제약 조건 설정 중...\n');

F = [];

% 초기 조건 (각 MPC 반복마다 업데이트됨)
x0 = [0; 0];  % 원점에서 시작
F = [F, x(:,1) == x0];

% 시스템 동역학
for k = 1:N
    F = [F, x(:,k+1) == A*x(:,k) + B*u(:,k)];
end

% 입력 제약
for k = 1:N
    F = [F, u_min <= u(:,k) <= u_max];
end

% 상태 제약 (하드 제약)
for k = 1:N+1
    F = [F, x_min <= x(:,k) <= x_max];
end


%% 5. STL 제약 조건 인코딩
fprintf('5. STL 제약 조건 인코딩 중...\n');

% Atomic predicate 헬퍼 함수 (c - h'*x 형태)
atomic_pred = @(x_k, h, c) deal(c - h'*x_k, []);

% 제약 1: □[0,10](|x1| ≤ 8)
% 즉, □[0,10](x1 ≤ 8 ∧ x1 ≥ -8)
spec1_upper = @(k) atomic_pred(x(:,k+1), [1;0], 8);   % 8 - x1 ≥ 0
spec1_lower = @(k) atomic_pred(x(:,k+1), [-1;0], 8);  % 8 - (-x1) ≥ 0
spec1_and = @(k) robust_and(@() spec1_upper(k), @() spec1_lower(k));
[r_spec1, F_spec1] = robust_always(0, [0, min(10,N)], spec1_and);

% 제약 2: ◇[5,15](x1 ≥ 7)
% 즉, ◇[5,15](-7 - (-x1) ≥ 0) = ◇[5,15](x1 - 7 ≥ 0)
spec2 = @(k) atomic_pred(x(:,k+1), [-1;0], -7);  % -7 - (-x1) = x1 - 7 ≥ 0
[r_spec2, F_spec2] = robust_eventually(0, [5, min(15,N)], spec2);

% 제약 3: □[0,20](|x2| ≤ 6)
% 즉, □[0,20](x2 ≤ 6 ∧ x2 ≥ -6)
spec3_upper = @(k) atomic_pred(x(:,k+1), [0;1], 6);   % 6 - x2 ≥ 0
spec3_lower = @(k) atomic_pred(x(:,k+1), [0;-1], 6);  % 6 - (-x2) ≥ 0
spec3_and = @(k) robust_and(@() spec3_upper(k), @() spec3_lower(k));
[r_spec3, F_spec3] = robust_always(0, [0, N], spec3_and);

% 모든 제약을 AND로 결합
[r_total, F_total] = robust_and(...
    @() robust_and(@() deal(r_spec1, F_spec1), @() deal(r_spec2, F_spec2)), ...
    @() deal(r_spec3, F_spec3));

% STL 만족 제약 (양의 robustness margin)
F = [F, F_total, r_total >= 0.01];

%% 6. 비용 함수
fprintf('6. 비용 함수 정의 중...\n');

% 제어 입력과 목표 도달 최소화
Q = eye(n);
R = 0.1 * eye(m);
x_ref = [8; 0];  % 목표 상태 (목표 위치)

cost = 0;
for k = 1:N
    cost = cost + (x(:,k) - x_ref)' * Q * (x(:,k) - x_ref) + u(:,k)' * R * u(:,k);
end
% 종료 비용
cost = cost + (x(:,N+1) - x_ref)' * Q * (x(:,N+1) - x_ref);

%% 7. 최적화 문제 해결
fprintf('7. 최적화 문제 해결 중...\n');

options = sdpsettings('verbose', 1, 'solver', 'bmibnb');  % 'gurobi', 'cplex', 'mosek' 사용

fprintf('\nSTL-MPC 최적화 문제 풀이 시작...\n');
fprintf('결정 변수 개수: %d\n', length([x(:); u(:)]));
fprintf('제약 조건 개수: %d\n', length(F));

sol = optimize(F, cost, options);

%% 8. 결과 확인
if sol.problem == 0
    fprintf('\n최적화 성공!\n');
    fprintf('최적 비용: %.4f\n', value(cost));
    fprintf('STL robustness: %.4f\n', value(r_total));

    % 최적해 추출
    x_opt = value(x);
    u_opt = value(u);

    % 결과 시각화
    figure('Position', [100, 100, 1200, 800]);

    % 위치 궤적
    subplot(2,2,1);
    time = 0:N;
    plot(time, x_opt(1,:), 'b-', 'LineWidth', 2);
    hold on;
    plot(time, 8*ones(size(time)), 'r--', 'LineWidth', 1.5);
    plot(time, -8*ones(size(time)), 'r--', 'LineWidth', 1.5);
    plot(time, 7*ones(size(time)), 'g--', 'LineWidth', 1.5);
    xlabel('시간 (스텝)');
    ylabel('위치 (x_1)');
    title('위치 궤적');
    legend('위치', '상한 (8)', '하한 (-8)', '목표 (7)');
    grid on;

    % 속도 궤적
    subplot(2,2,2);
    plot(time, x_opt(2,:), 'b-', 'LineWidth', 2);
    hold on;
    plot(time, 6*ones(size(time)), 'r--', 'LineWidth', 1.5);
    plot(time, -6*ones(size(time)), 'r--', 'LineWidth', 1.5);
    xlabel('시간 (스텝)');
    ylabel('속도 (x_2)');
    title('속도 궤적');
    legend('속도', '상한 (6)', '하한 (-6)');
    grid on;

    % 제어 입력
    subplot(2,2,3);
    stairs(0:N-1, u_opt, 'b-', 'LineWidth', 2);
    hold on;
    plot(0:N-1, u_max*ones(1,N), 'r--', 'LineWidth', 1.5);
    plot(0:N-1, u_min*ones(1,N), 'r--', 'LineWidth', 1.5);
    xlabel('시간 (스텝)');
    ylabel('제어 입력 (u)');
    title('제어 입력');
    legend('입력', '상한', '하한');
    grid on;

    % 위상 평면
    subplot(2,2,4);
    plot(x_opt(1,:), x_opt(2,:), 'b-o', 'LineWidth', 2, 'MarkerSize', 4);
    hold on;
    plot(x_opt(1,1), x_opt(2,1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(x_opt(1,end), x_opt(2,end), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    xlabel('위치 (x_1)');
    ylabel('속도 (x_2)');
    title('위상 평면');
    legend('궤적', '시작', '종료');
    grid on;
    axis equal;

    fprintf('\n그래프가 성공적으로 생성되었습니다!\n');
else
    fprintf('\n최적화 실패!\n');
    fprintf('문제 코드: %d\n', sol.problem);
    fprintf('솔버 출력: %s\n', sol.info);
end