%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%     간단한 STL-MPC 데모 (한국어 주석)         %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% 이 파일은 STL-MPC의 기본 개념을 이해하기 위한 간단한 예제입니다.
%
% 시나리오:
%   로봇이 원점(0,0)에서 출발하여 목표(5,5)에 도달하되,
%   장애물 영역 [2,3]×[2,3]을 피해야 합니다.
%
% STL 제약:
%   1. □[0,20](상태 범위 제약)
%   2. ◇[10,20](목표 도달)
%   3. □[0,20](장애물 회피)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all

addpath(genpath('C:\gurobi1300\win64'))  % 예: Gurobi, CPLEX, MOSEKC:\gurobi1300

fprintf('========================================\n');
fprintf('간단한 STL-MPC 데모\n');
fprintf('========================================\n\n');

%% 1. 시스템 모델 정의
fprintf('1. 시스템 모델 정의 중...\n');

% 2차원 점 질량 (point mass) 모델
% 상태: [x; y; vx; vy] (위치와 속도)
% 입력: [ax; ay] (가속도)

dt = 0.5;  % 샘플링 시간

% 연속 시간 모델: ẋ = Ac*x + Bc*u
Ac = [0, 0, 1, 0;
      0, 0, 0, 1;
      0, 0, 0, 0;
      0, 0, 0, 0];

Bc = [0, 0;
      0, 0;
      1, 0;
      0, 1];

% 이산 시간 모델로 변환
sysc = ss(Ac, Bc, [], []);
sysd = c2d(sysc, dt); % zoh
A = sysd.A;
B = sysd.B;

n = 4;  % 상태 차원
m = 2;  % 입력 차원

%% 2. MPC 파라미터 설정
fprintf('2. MPC 파라미터 설정 중...\n');

N = 20;  % 예측 구간

% 상태 제약
x_max = [10; 10; 2; 2];    % [pos_x; pos_y; vel_x; vel_y]
x_min = -x_max;

% 입력 제약
u_max = [1; 1];  % 최대 가속도
u_min = -u_max;

%% 3. STL 제약 조건 정의
fprintf('3. STL 제약 조건 정의 중...\n');

% 결정 변수
x = sdpvar(n, N+1);  % 상태 궤적
u = sdpvar(m, N);    % 입력 궤적

% 초기 조건
x0 = [0; 0; 0; 0];  % 원점에서 정지 상태로 시작

% MPC 제약
F = [];
F = [F, x(:,1) == x0];  % 초기 조건

% 시스템 동역학
for k = 1:N
    F = [F, x(:,k+1) == A*x(:,k) + B*u(:,k)];
end

% 입력 제약
for k = 1:N
    F = [F, u_min <= u(:,k) <= u_max];
end

% 상태 제약 (hard constraints)
for k = 1:N+1
    F = [F, x_min <= x(:,k) <= x_max];
end


%% 4. STL 제약 인코딩

% --- STL 제약 1: 목표 도달 ---
% ◇[10,20]((x-5)^2 + (y-5)^2 ≤ 0.5)
% 간단히 하기 위해: ◇[10,20](x ≥ 4.5 ∧ y ≥ 4.5)

fprintf('   - 목표 도달 제약 인코딩...\n');

goal_x = @(k) deal(x(1,k+1) - 4.5, []);  % x ≥ 4.5
goal_y = @(k) deal(x(2,k+1) - 4.5, []);  % y ≥ 4.5
goal = @(k) robust_and(@() goal_x(k), @() goal_y(k));

[r_goal, F_goal] = robust_eventually(0, [10, N], goal);

% --- STL 제약 2: 장애물 회피 ---
% □[0,20](¬(x∈[2,3] ∧ y∈[2,3]))

fprintf('   - 장애물 회피 제약 인코딩...\n');

% 장애물 내부 조건
obs_x_in = @(k) robust_and(...
    @() deal(x(1,k+1) - 2, []), ...      % x ≥ 2
    @() deal(3 - x(1,k+1), []));         % x ≤ 3

obs_y_in = @(k) robust_and(...
    @() deal(x(2,k+1) - 2, []), ...      % y ≥ 2
    @() deal(3 - x(2,k+1), []));         % y ≤ 3

obs_in = @(k) robust_and(@() obs_x_in(k), @() obs_y_in(k));

% NOT 적용: 장애물 밖에 있어야 함
avoid = @(k) robust_not(@() obs_in(k));

[r_avoid, F_avoid] = robust_always(0, [0, N], avoid);

% --- 모든 STL 제약 결합 ---
fprintf('   - 제약 조건 결합...\n');

[r_stl, F_stl] = robust_and(...
    @() deal(r_goal, F_goal), ...
    @() deal(r_avoid, F_avoid));

% STL 만족 조건: robustness > 0
F = [F, F_stl, r_stl >= 0.3];

%% 5. 비용 함수 정의
fprintf('4. 비용 함수 정의 중...\n');

% 목표 상태
x_target = [5; 5; 0; 0];

% 가중치
Q = diag([10, 10, 1, 1]);  % 상태 가중치 (위치가 더 중요)
R = 0.1 * eye(m);          % 입력 가중치

% 2차 비용 함수
cost = 0;
for k = 1:N
    cost = cost + (x(:,k) - x_target)' * Q * (x(:,k) - x_target);
    cost = cost + u(:,k)' * R * u(:,k);
end

% 종단 비용
P = 10 * Q;
cost = cost + (x(:,N+1) - x_target)' * P * (x(:,N+1) - x_target);

%% 6. 최적화 문제 해결
fprintf('5. 최적화 문제 해결 중...\n');

% Solver 옵션
options = sdpsettings('verbose', 1, 'solver', 'gurobi');
% options = sdpsettings('verbose', 1, 'solver', 'bmibnb');

fprintf('\n--- 최적화 시작 ---\n');
fprintf('결정 변수 개수: %d\n', length([x(:); u(:)]));
fprintf('제약 조건 개수: %d\n', length(F));

tic;
sol = optimize(F, cost, options);
solve_time = toc;

%% 7. 결과 확인 및 시각화
fprintf('\n--- 최적화 결과 ---\n');

if sol.problem == 0
    fprintf('✓ 최적화 성공!\n');
    fprintf('  소요 시간: %.2f 초\n', solve_time);
    fprintf('  최적 비용: %.4f\n', value(cost));
    fprintf('  STL robustness: %.4f (>0 이면 만족)\n', value(r_stl));
    fprintf('  목표 도달 robustness: %.4f\n', value(r_goal));
    fprintf('  장애물 회피 robustness: %.4f\n', value(r_avoid));

    % 해 추출
    x_opt = value(x);
    u_opt = value(u);

    % 각 스텝별 robustness 계산 및 출력

    fprintf('\n--- 스텝별 Robustness 분석 ---\n');

    for k = 1:N+1
        % 현재 스텝의 위치
        x_k = x_opt(:, k);
        % 목표 도달 robustness (x >= 4.5 ∧ y >= 4.5)
        r_goal_x = x_k(1) - 4.5;
        r_goal_y = x_k(2) - 4.5;
        r_goal_k = min(r_goal_x, r_goal_y);

        % 장애물 회피 robustness
        % obs_in: (x >= 2 ∧ x <= 3 ∧ y >= 2 ∧ y <= 3)
        r_obs_x_in = min(x_k(1) - 2, 3 - x_k(1));
        r_obs_y_in = min(x_k(2) - 2, 3 - x_k(2));
        r_obs_in = min(r_obs_x_in, r_obs_y_in);
        r_avoid_k = -r_obs_in;  % NOT

 

        fprintf('스텝 %2d: 위치=(%.2f, %.2f), 목표=%.4f, 회피=%.4f\n', k-1, x_k(1), x_k(2), r_goal_k, r_avoid_k);
    end
 
    % 시각화
    visualize_results(x_opt, u_opt, N, dt);

else
    fprintf('✗ 최적화 실패\n');
    fprintf('  문제 코드: %d\n', sol.problem);
    fprintf('  정보: %s\n', sol.info);

    if sol.problem == 1
        fprintf('\n해결 방법:\n');
        fprintf('  1. STL 제약이 너무 엄격할 수 있습니다.\n');
        fprintf('  2. 예측 구간 N을 늘려보세요.\n');
        fprintf('  3. robustness margin을 줄여보세요 (0.1 → 0.01).\n');
    end
end

%% 시각화 함수
function visualize_results(x_opt, u_opt, N, dt)
    figure('Position', [100, 100, 1400, 900]);

    % 시간 축
    time = 0:N;
    time_u = 0:N-1;

    % 1. 2D 궤적
    subplot(2,3,1);
    plot(x_opt(1,:), x_opt(2,:), 'b-o', 'LineWidth', 2, 'MarkerSize', 5);
    hold on;

    % 시작점
    plot(x_opt(1,1), x_opt(2,1), 'go', 'MarkerSize', 15, 'LineWidth', 3);

    % 목표 영역
    rectangle('Position', [4.5, 4.5, 1, 1], 'EdgeColor', 'g', ...
              'LineWidth', 2, 'LineStyle', '--');
    text(5, 5.8, '목표 영역', 'FontSize', 10, 'Color', 'g', ...
         'HorizontalAlignment', 'center');

    % 장애물
    rectangle('Position', [2, 2, 1, 1], 'FaceColor', [1, 0.7, 0.7], ...
              'EdgeColor', 'r', 'LineWidth', 2);
    text(2.5, 2.5, '장애물', 'FontSize', 10, 'Color', 'r', ...
         'HorizontalAlignment', 'center');

    % 끝점
    plot(x_opt(1,end), x_opt(2,end), 'ro', 'MarkerSize', 15, 'LineWidth', 3);

    xlabel('X 위치 [m]');
    ylabel('Y 위치 [m]');
    title('2D 궤적');
    legend('궤적', '시작', '', '', '종료', 'Location', 'best');
    grid on;
    axis equal;
    xlim([-1, 6]);
    ylim([-1, 6]);

    % 2. X 위치
    subplot(2,3,2);
    plot(time, x_opt(1,:), 'b-o', 'LineWidth', 2);
    hold on;
    plot([0, N], [4.5, 4.5], 'g--', 'LineWidth', 1.5);
    xlabel('시간 스텝');
    ylabel('X 위치 [m]');
    title('X 위치 변화');
    legend('X', '목표 (≥4.5)', 'Location', 'best');
    grid on;

    % 3. Y 위치
    subplot(2,3,3);
    plot(time, x_opt(2,:), 'r-o', 'LineWidth', 2);
    hold on;
    plot([0, N], [4.5, 4.5], 'g--', 'LineWidth', 1.5);
    xlabel('시간 스텝');
    ylabel('Y 위치 [m]');
    title('Y 위치 변화');
    legend('Y', '목표 (≥4.5)', 'Location', 'best');
    grid on;

    % 4. 속도
    subplot(2,3,4);
    plot(time, x_opt(3,:), 'b-o', 'LineWidth', 2);
    hold on;
    plot(time, x_opt(4,:), 'r-o', 'LineWidth', 2);
    plot([0, N], [2, 2], 'k--', 'LineWidth', 1);
    plot([0, N], [-2, -2], 'k--', 'LineWidth', 1);
    xlabel('시간 스텝');
    ylabel('속도 [m/s]');
    title('속도 변화');
    legend('V_x', 'V_y', '상한', '하한', 'Location', 'best');
    grid on;

    % 5. 제어 입력
    subplot(2,3,5);
    stairs(time_u, u_opt(1,:), 'b-', 'LineWidth', 2);
    hold on;
    stairs(time_u, u_opt(2,:), 'r-', 'LineWidth', 2);
    plot([0, N], [1, 1], 'k--', 'LineWidth', 1);
    plot([0, N], [-1, -1], 'k--', 'LineWidth', 1);
    xlabel('시간 스텝');
    ylabel('가속도 [m/s²]');
    title('제어 입력');
    legend('a_x', 'a_y', '상한', '하한', 'Location', 'best');
    grid on;

    % 6. 속도 크기
    subplot(2,3,6);
    vel_mag = sqrt(x_opt(3,:).^2 + x_opt(4,:).^2);
    plot(time, vel_mag, 'g-o', 'LineWidth', 2);
    hold on;
    plot([0, N], [2, 2], 'r--', 'LineWidth', 1.5);
    xlabel('시간 스텝');
    ylabel('속도 크기 [m/s]');
    title('속도 크기');
    legend('|V|', '최대값', 'Location', 'best');
    grid on;

    sgtitle('STL-MPC 시뮬레이션 결과', 'FontSize', 14, 'FontWeight', 'bold');

    % 3D 시공간 궤적
    figure('Position', [150, 150, 800, 600]);
    plot3(x_opt(1,:), x_opt(2,:), time, 'b-o', 'LineWidth', 2, 'MarkerSize', 5);
    hold on;

    % 장애물 (시간에 따라 확장)
    [obs_x, obs_y] = meshgrid(2:0.1:3, 2:0.1:3);
    for t = [0, 10, 20]
        obs_z = t * ones(size(obs_x));
        surf(obs_x, obs_y, obs_z, 'FaceColor', 'r', 'FaceAlpha', 0.3, ...
             'EdgeColor', 'none');
    end

    xlabel('X 위치 [m]');
    ylabel('Y 위치 [m]');
    zlabel('시간 스텝');
    title('시공간 궤적 (3D)');
    grid on;
    view(45, 30);
end