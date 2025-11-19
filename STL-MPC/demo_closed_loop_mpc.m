%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%           Closed-Loop STL-MPC 데모           %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% 이 파일은 Receding Horizon 방식의 STL-MPC를 구현합니다.
%
% 차이점:
%   - Open-loop (demo_simple_mpc.m): 한 번만 최적화, 전체 궤적 계획
%   - Closed-loop (이 파일): 매 스텝마다 최적화, 피드백 적용
%
% 시나리오:
%   로봇이 원점(0,0)에서 출발하여 목표(5,5)에 도달하되,
%   장애물 영역 [2,3]×[2,3]을 피해야 합니다.
%
% STL 제약:
%   1. □[0,N](상태 범위 제약)
%   2. ◇[10,N](목표 도달)
%   3. □[0,N](장애물 회피)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all

addpath(genpath('C:\gurobi1300\win64'))  % 예: Gurobi, CPLEX, MOSEKC:\gurobi1300

fprintf('========================================\n');
fprintf('Closed-Loop STL-MPC 데모\n');
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

N = 20;       % 예측 구간 (horizon)
T_sim = 40;   % 전체 시뮬레이션 스텝 수

% 상태 제약
x_max = [10; 10; 2; 2];    % [pos_x; pos_y; vel_x; vel_y]
x_min = -x_max;

% 입력 제약
u_max = [1; 1];  % 최대 가속도
u_min = -u_max;

% 목표 상태
x_target = [5; 5; 0; 0];

% 가중치
Q = diag([10, 10, 1, 1]);  % 상태 가중치 (위치가 더 중요)
R = 0.1 * eye(m);          % 입력 가중치
P = 10 * Q;                % 종단 비용

%% 3. 시뮬레이션 초기화
fprintf('3. 시뮬레이션 초기화 중...\n');

% 초기 상태
x_current = [0; 0; 0; 0];  % 원점에서 정지 상태로 시작

% 실제 궤적 저장
x_history = zeros(n, T_sim+1);
u_history = zeros(m, T_sim);
x_history(:,1) = x_current;

% 계획된 궤적 저장 (마지막 MPC 예측)
x_planned = [];

% 최적화 통계
solve_times = zeros(T_sim, 1);
robustness_history = zeros(T_sim, 3);  % [r_stl, r_goal, r_avoid]

% Solver 옵션
options = sdpsettings('verbose', 0, 'solver', 'gurobi');

%% 4. Receding Horizon MPC 루프
fprintf('4. Receding Horizon MPC 시뮬레이션 시작...\n\n');

for t = 1:T_sim
    fprintf('--- 스텝 %d/%d ---\n', t, T_sim);

    %% 4.1 MPC 최적화 문제 정의

    % 결정 변수
    x = sdpvar(n, N+1);  % 상태 궤적
    u = sdpvar(m, N);    % 입력 궤적

    % MPC 제약
    F = [];
    F = [F, x(:,1) == x_current];  % 초기 조건 (현재 상태)

    % 시스템 동역학
    for k = 1:N
        F = [F, x(:,k+1) == A*x(:,k) + B*u(:,k)];
    end

    % 입력 제약
    for k = 1:N
        F = [F, u_min <= u(:,k) <= u_max];
    end

    % 상태 제약 (hard constraints)
    for k = 1:N
        F = [F, x_min <= x(:,k) <= x_max];
    end

    %% 4.2 STL 제약 인코딩

    % --- STL 제약 1: 목표 도달 ---
    % ◇[10,N](x ≥ 4.5 ∧ y ≥ 4.5)

    goal_x = @(k) deal(x(1,k+1) - 4.5, []);  % x ≥ 4.5
    goal_y = @(k) deal(x(2,k+1) - 4.5, []);  % y ≥ 4.5
    goal = @(k) robust_and_simple(@() goal_x(k), @() goal_y(k));

    % 목표 도달 시간 조정: 남은 시간에 따라
    goal_time_start = min(10, N-5);  % 최소 5 스텝 여유
    goal_time_start = max(0, goal_time_start);

    [r_goal, F_goal] = robust_eventually_simple(0, [goal_time_start, N], goal);

    % --- STL 제약 2: 장애물 회피 ---
    % □[0,N](¬(x∈[2,3] ∧ y∈[2,3]))

    % 장애물 내부 조건
    obs_x_in = @(k) robust_and(...
        @() deal(x(1,k+1) - 2, []), ...      % x ≥ 2
        @() deal(3 - x(1,k+1), []));         % x ≤ 3

    obs_y_in = @(k) robust_and(...
        @() deal(x(2,k+1) - 2, []), ...      % y ≥ 2
        @() deal(3 - x(2,k+1), []));         % y ≤ 3

    obs_in = @(k) robust_and(@() obs_x_in(k), @() obs_y_in(k));

    % NOT 적용: 장애물 밖에 있어야 함
    avoid = @(k) deal(-obs_in(k), []);  % robustness 부호 반전

    [r_avoid, F_avoid] = robust_always_simple(0, [0, N], avoid);

    % --- 모든 STL 제약 결합 ---
    [r_stl, F_stl] = robust_and_simple(...
        @() deal(r_goal, F_goal), ...
        @() deal(r_avoid, F_avoid));

    % STL 만족 조건: robustness > 0
    F = [F, F_stl, r_stl >= 0.3];

    %% 4.3 비용 함수 정의

    cost = 0;
    for k = 1:N
        cost = cost + (x(:,k) - x_target)' * Q * (x(:,k) - x_target);
        cost = cost + u(:,k)' * R * u(:,k);
    end

    % 종단 비용
    cost = cost + (x(:,N+1) - x_target)' * P * (x(:,N+1) - x_target);

    %% 4.4 최적화 문제 해결

    tic;
    sol = optimize(F, cost, options);
    solve_times(t) = toc;

    %% 4.5 결과 처리

    if sol.problem == 0
        % 최적 해 추출
        x_opt = value(x);
        u_opt = value(u);

        % 첫 번째 입력만 적용 (MPC의 핵심!)
        u_applied = u_opt(:,1);

        % robustness 저장
        robustness_history(t,:) = [value(r_stl), value(r_goal), value(r_avoid)];

        fprintf('  ✓ 최적화 성공 (%.3f초)\n', solve_times(t));
        fprintf('    현재 위치: (%.2f, %.2f)\n', x_current(1), x_current(2));
        fprintf('    적용 입력: (%.3f, %.3f)\n', u_applied(1), u_applied(2));
        fprintf('  장애물 회피 robustness: %.4f\n', value(r_avoid));
        fprintf('  목표 도달 robustness: %.4f\n', value(r_goal));
        fprintf('    STL robustness: %.4f\n', value(r_stl));

        % 마지막 스텝의 계획 궤적 저장 (시각화용)
        if t == T_sim
            x_planned = x_opt;
        end

    else
        fprintf('  ✗ 최적화 실패 (문제 코드: %d)\n', sol.problem);
        fprintf('    현재 위치: (%.2f, %.2f)\n', x_current(1), x_current(2));

        % Fallback: 안전한 입력 (정지)
        u_applied = zeros(m, 1);
        robustness_history(t,:) = [NaN, NaN, NaN];
    end

    %% 4.6 시스템 상태 업데이트 (실제 로봇 동작)

    % 입력 저장
    u_history(:,t) = u_applied;

    % 상태 업데이트 (시스템 동역학)
    x_current = A*x_current + B*u_applied;

    % 상태 저장
    x_history(:,t+1) = x_current;

    % 목표 도달 확인
    dist_to_goal = norm(x_current(1:2) - x_target(1:2));
    if dist_to_goal < 0.5
        fprintf('  🎯 목표 도달! (거리: %.3f)\n', dist_to_goal);
        % 나머지 스텝은 목표에서 유지
        x_history(:,t+2:end) = repmat(x_current, 1, T_sim-t);
        u_history(:,t+1:end) = zeros(m, T_sim-t);
        T_sim = t;  % 시뮬레이션 종료
        break;
    end

    fprintf('\n');
end

%% 5. 결과 시각화
fprintf('========================================\n');
fprintf('5. 시뮬레이션 완료! 결과 시각화 중...\n');
fprintf('========================================\n\n');

fprintf('통계:\n');
fprintf('  총 스텝 수: %d\n', T_sim);
fprintf('  평균 최적화 시간: %.3f초\n', mean(solve_times(1:T_sim)));
fprintf('  최종 위치: (%.2f, %.2f)\n', x_history(1,T_sim+1), x_history(2,T_sim+1));
fprintf('  목표까지 거리: %.3f\n', norm(x_history(1:2,T_sim+1) - x_target(1:2)));
fprintf('  평균 STL robustness: %.4f\n', mean(robustness_history(1:T_sim,1), 'omitnan'));

% 시각화
visualize_closed_loop_results(x_history, u_history, x_planned, T_sim, dt, ...
                              solve_times, robustness_history);

%% 시각화 함수
function visualize_closed_loop_results(x_history, u_history, x_planned, T_sim, dt, ...
                                       solve_times, robustness_history)

    figure('Position', [100, 100, 1600, 1000]);

    % 시간 축
    % time = 0:T_sim;
    % time_u = 0:T_sim-1;
    time = 0:size(x_history, 2)-1;
    time_u = 0:size(u_history, 2)-1;

    %% 1. 2D 궤적 (실제 vs 계획)
    subplot(3,3,1);
    plot(x_history(1,:), x_history(2,:), 'b-o', 'LineWidth', 2.5, 'MarkerSize', 6);
    hold on;

    % 마지막 계획 궤적 (있으면 표시)
    if ~isempty(x_planned)
        plot(x_planned(1,:), x_planned(2,:), 'c--', 'LineWidth', 1.5, 'MarkerSize', 4);
    end

    % 시작점
    plot(x_history(1,1), x_history(2,1), 'go', 'MarkerSize', 18, 'LineWidth', 3);

    % 목표 영역
    rectangle('Position', [4.5, 4.5, 1, 1], 'EdgeColor', 'g', ...
              'LineWidth', 2, 'LineStyle', '--');
    text(5, 5.8, '목표 영역', 'FontSize', 11, 'Color', 'g', ...
         'HorizontalAlignment', 'center', 'FontWeight', 'bold');

    % 장애물
    rectangle('Position', [2, 2, 1, 1], 'FaceColor', [1, 0.7, 0.7], ...
              'EdgeColor', 'r', 'LineWidth', 2.5);
    text(2.5, 2.5, '장애물', 'FontSize', 11, 'Color', 'r', ...
         'HorizontalAlignment', 'center', 'FontWeight', 'bold');

    % 끝점
    plot(x_history(1,end), x_history(2,end), 'ro', 'MarkerSize', 18, 'LineWidth', 3);

    xlabel('X 위치 [m]', 'FontSize', 11);
    ylabel('Y 위치 [m]', 'FontSize', 11);
    title('2D 궤적 (Closed-Loop)', 'FontSize', 12, 'FontWeight', 'bold');
    legend('실제 궤적', '마지막 계획', '시작', '', '', '종료', 'Location', 'best');
    grid on;
    axis equal;
    xlim([-1, 6]);
    ylim([-1, 6]);

    %% 2. X 위치
    subplot(3,3,2);
    plot(time, x_history(1,:), 'b-o', 'LineWidth', 2);
    hold on;
    plot([0, T_sim], [4.5, 4.5], 'g--', 'LineWidth', 1.5);
    xlabel('시간 스텝', 'FontSize', 11);
    ylabel('X 위치 [m]', 'FontSize', 11);
    title('X 위치 변화', 'FontSize', 12, 'FontWeight', 'bold');
    legend('X', '목표 (≥4.5)', 'Location', 'best');
    grid on;

    %% 3. Y 위치
    subplot(3,3,3);
    plot(time, x_history(2,:), 'r-o', 'LineWidth', 2);
    hold on;
    plot([0, T_sim], [4.5, 4.5], 'g--', 'LineWidth', 1.5);
    xlabel('시간 스텝', 'FontSize', 11);
    ylabel('Y 위치 [m]', 'FontSize', 11);
    title('Y 위치 변화', 'FontSize', 12, 'FontWeight', 'bold');
    legend('Y', '목표 (≥4.5)', 'Location', 'best');
    grid on;

    %% 4. 속도
    subplot(3,3,4);
    plot(time, x_history(3,:), 'b-o', 'LineWidth', 2);
    hold on;
    plot(time, x_history(4,:), 'r-o', 'LineWidth', 2);
    plot([0, T_sim], [2, 2], 'k--', 'LineWidth', 1);
    plot([0, T_sim], [-2, -2], 'k--', 'LineWidth', 1);
    xlabel('시간 스텝', 'FontSize', 11);
    ylabel('속도 [m/s]', 'FontSize', 11);
    title('속도 변화', 'FontSize', 12, 'FontWeight', 'bold');
    legend('V_x', 'V_y', '상한', '하한', 'Location', 'best');
    grid on;

    %% 5. 제어 입력
    subplot(3,3,5);
    stairs(time_u, u_history(1,:), 'b-', 'LineWidth', 2);
    hold on;
    stairs(time_u, u_history(2,:), 'r-', 'LineWidth', 2);
    plot([0, T_sim], [1, 1], 'k--', 'LineWidth', 1);
    plot([0, T_sim], [-1, -1], 'k--', 'LineWidth', 1);
    xlabel('시간 스텝', 'FontSize', 11);
    ylabel('가속도 [m/s²]', 'FontSize', 11);
    title('제어 입력', 'FontSize', 12, 'FontWeight', 'bold');
    legend('a_x', 'a_y', '상한', '하한', 'Location', 'best');
    grid on;

    %% 6. 속도 크기
    subplot(3,3,6);
    vel_mag = sqrt(x_history(3,:).^2 + x_history(4,:).^2);
    plot(time, vel_mag, 'g-o', 'LineWidth', 2);
    hold on;
    plot([0, T_sim], [2, 2], 'r--', 'LineWidth', 1.5);
    xlabel('시간 스텝', 'FontSize', 11);
    ylabel('속도 크기 [m/s]', 'FontSize', 11);
    title('속도 크기', 'FontSize', 12, 'FontWeight', 'bold');
    legend('|V|', '최대값', 'Location', 'best');
    grid on;

    %% 7. STL Robustness
    subplot(3,3,7);
    plot(time_u, robustness_history(:,1), 'k-o', 'LineWidth', 2);
    hold on;
    plot(time_u, robustness_history(:,2), 'g--', 'LineWidth', 1.5);
    plot(time_u, robustness_history(:,3), 'r--', 'LineWidth', 1.5);
    plot([0, T_sim], [0, 0], 'k:', 'LineWidth', 1);
    xlabel('시간 스텝', 'FontSize', 11);
    ylabel('Robustness', 'FontSize', 11);
    title('STL Robustness', 'FontSize', 12, 'FontWeight', 'bold');
    legend('전체 STL', '목표 도달', '장애물 회피', 'Location', 'best');
    grid on;

    %% 8. 최적화 시간
    subplot(3,3,8);
    bar(1:T_sim, solve_times(1:T_sim), 'FaceColor', [0.3, 0.6, 0.9]);
    hold on;
    plot([0, T_sim+1], [mean(solve_times(1:T_sim)), mean(solve_times(1:T_sim))], ...
         'r--', 'LineWidth', 2);
    xlabel('시간 스텝', 'FontSize', 11);
    ylabel('계산 시간 [초]', 'FontSize', 11);
    title('MPC 최적화 시간', 'FontSize', 12, 'FontWeight', 'bold');
    legend('스텝별 시간', sprintf('평균: %.3fs', mean(solve_times(1:T_sim))), ...
           'Location', 'best');
    grid on;

    %% 9. 목표까지 거리
    subplot(3,3,9);
    dist_to_goal = sqrt((x_history(1,:) - 5).^2 + (x_history(2,:) - 5).^2);
    plot(time, dist_to_goal, 'm-o', 'LineWidth', 2);
    hold on;
    plot([0, T_sim], [0.5, 0.5], 'g--', 'LineWidth', 1.5);
    xlabel('시간 스텝', 'FontSize', 11);
    ylabel('거리 [m]', 'FontSize', 11);
    title('목표까지 거리', 'FontSize', 12, 'FontWeight', 'bold');
    legend('거리', '목표 임계값', 'Location', 'best');
    grid on;

    sgtitle('Closed-Loop STL-MPC 시뮬레이션 결과', 'FontSize', 16, 'FontWeight', 'bold');

    %% 3D 시공간 궤적
    figure('Position', [150, 150, 900, 700]);
    plot3(x_history(1,:), x_history(2,:), time, 'b-o', 'LineWidth', 2.5, 'MarkerSize', 6);
    hold on;

    % 시작점과 끝점 강조
    plot3(x_history(1,1), x_history(2,1), 0, 'go', 'MarkerSize', 20, 'LineWidth', 3);
    plot3(x_history(1,end), x_history(2,end), time(end), 'ro', 'MarkerSize', 20, 'LineWidth', 3);

    % 장애물 (시간에 따라 확장)
    [obs_x, obs_y] = meshgrid(2:0.1:3, 2:0.1:3);
    for t_viz = [0, T_sim/2, T_sim]
        obs_z = t_viz * ones(size(obs_x));
        surf(obs_x, obs_y, obs_z, 'FaceColor', 'r', 'FaceAlpha', 0.3, ...
             'EdgeColor', 'none');
    end

    % 목표 영역
    goal_x = [4.5, 5.5, 5.5, 4.5, 4.5];
    goal_y = [4.5, 4.5, 5.5, 5.5, 4.5];
    for t_viz = [0, T_sim/2, T_sim]
        goal_z = t_viz * ones(size(goal_x));
        plot3(goal_x, goal_y, goal_z, 'g--', 'LineWidth', 2);
    end

    xlabel('X 위치 [m]', 'FontSize', 12);
    ylabel('Y 위치 [m]', 'FontSize', 12);
    zlabel('시간 스텝', 'FontSize', 12);
    title('시공간 궤적 (3D) - Closed-Loop', 'FontSize', 14, 'FontWeight', 'bold');
    grid on;
    view(45, 30);
    legend('실제 궤적', '시작', '종료', 'Location', 'best');
end

%% Helper 함수들

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
