%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Adaptive Cruise Control - MPC (CasADi)      %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% CasADi를 사용한 ACC MPC 구현
%
% 상태변수:
%   x = [v_ego; d_rel; v_pre]
%   - v_ego: 자차 속도 (m/s)
%   - d_rel: 상대 거리 (m)
%   - v_pre: 선행차량 속도 (m/s)
%
% 제어변수:
%   u = a (가속도, m/s^2)
%
% 제약 조건:
%   φ := G(15 < d_rel < 50)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all

% CasADi import
addpath('C:\CasADi\casadi-3.7.2-windows64-matlab2018b')
import casadi.*

fprintf('========================================\n');
fprintf('Adaptive Cruise Control - MPC (CasADi)\n');
fprintf('========================================\n\n');

%% 1. 시스템 파라미터 설정
fprintf('1. 시스템 파라미터 설정 중...\n');

dt = 0.1;       % 샘플링 시간 (s)
N = 30;         % 예측 구간 (horizon)
T_sim = 30;     % 시뮬레이션 시간 (s)
N_sim = T_sim / dt;  % 시뮬레이션 스텝 수

% 상태 및 입력 차원
n_x = 3;  % 상태 차원 [v_ego; d_rel; v_pre]
n_u = 1;  % 입력 차원 [a]

%% 2. 시스템 동역학 (이산 시간 모델)
fprintf('2. 시스템 동역학 정의 중...\n');

% 상태: x = [v_ego; d_rel; v_pre]
A = [1,   0,   0;      % v_ego(k+1) = v_ego(k)
     -dt, 1,   dt;     % d_rel(k+1) = d_rel(k) + dt*(v_pre - v_ego)
     0,   0,   1];     % v_pre(k+1) = v_pre(k)

B = [dt;               % v_ego에 가속도 영향
     0;                % d_rel에 직접 영향 없음
     0];               % v_pre에 영향 없음

Bd = [0;               % 선행차량 가속도가 v_ego에 영향 없음
      0;               % d_rel에 직접 영향 없음
      dt];             % v_pre에 선행차량 가속도 영향

%% 3. 제약 조건 설정
fprintf('3. 제약 조건 설정 중...\n');

% 상태 제약
v_ego_min = 0;      v_ego_max = 40;
d_rel_min = 15;     d_rel_max = 50;     % STL 제약: G(15 < d_rel < 50)
v_pre_min = 0;      v_pre_max = 40;

x_min = [v_ego_min; d_rel_min; v_pre_min];
x_max = [v_ego_max; d_rel_max; v_pre_max];

% 입력 제약
a_min = -3;         % 최대 감속 (m/s^2)
a_max = 2;          % 최대 가속 (m/s^2)

%% 4. MPC 비용 함수 가중치
fprintf('4. MPC 비용 함수 설정 중...\n');

d_ref = 30;         % 목표 상대거리 (m)
v_ref = 25;         % 목표 속도 (m/s)

Q = diag([10, 50, 0]);   % 상태 가중치
R = 10;                   % 입력 가중치
Q_terminal = Q * 2;       % 종료 비용 가중치

%% 5. CasADi MPC 문제 설정
fprintf('5. CasADi MPC 문제 설정 중...\n');

% CasADi symbolic 변수
x = MX.sym('x', n_x);           % 상태
u = MX.sym('u', n_u);           % 입력
a_pre = MX.sym('a_pre', 1);     % 선행차량 가속도 (파라미터)

% 시스템 동역학 함수
x_next = A * x + B * u + Bd * a_pre;
f_dynamics = Function('f', {x, u, a_pre}, {x_next});

% MPC 최적화 문제 구성
opti = Opti();  % CasADi Opti Stack

% 결정 변수
X = opti.variable(n_x, N+1);    % 상태 궤적
U = opti.variable(n_u, N);      % 입력 궤적

% 파라미터 (초기 상태, 선행차량 가속도 예측)
P_x0 = opti.parameter(n_x, 1);          % 초기 상태
P_a_pre = opti.parameter(1, N);         % 선행차량 가속도 예측
P_v_pre_current = opti.parameter(1, 1); % 현재 선행차량 속도

% 비용 함수
cost = 0;
for k = 1:N
    x_ref_k = [v_ref; d_ref; P_v_pre_current];
    cost = cost + (X(:,k) - x_ref_k)' * Q * (X(:,k) - x_ref_k);
    cost = cost + U(:,k)' * R * U(:,k);
end
% 종료 비용
x_ref_N = [v_ref; d_ref; P_v_pre_current];
cost = cost + (X(:,N+1) - x_ref_N)' * Q_terminal * (X(:,N+1) - x_ref_N);

opti.minimize(cost);

% 제약 조건
% 초기 조건
opti.subject_to(X(:,1) == P_x0);

% 동역학 및 상태/입력 제약
for k = 1:N
    % 시스템 동역학
    opti.subject_to(X(:,k+1) == A * X(:,k) + B * U(:,k) + Bd * P_a_pre(k));

    % 상태 제약: 15 < d_rel < 50 (STL 핵심 제약)
    opti.subject_to(x_min <= X(:,k+1) <= x_max);

    % 입력 제약
    opti.subject_to(a_min <= U(:,k) <= a_max);
end

% 솔버 설정 (IPOPT)
opts = struct;
opts.ipopt.print_level = 0;
opts.print_time = 0;
opts.ipopt.warm_start_init_point = 'yes';
opti.solver('ipopt', opts);

%% 6. 초기 조건 및 시뮬레이션 설정
fprintf('6. 시뮬레이션 설정 중...\n');

% 초기 상태
v_ego_0 = 20;       % 자차 초기 속도 (m/s)
d_rel_0 = 35;       % 초기 상대거리 (m)
v_pre_0 = 22;       % 선행차량 초기 속도 (m/s)

x0 = [v_ego_0; d_rel_0; v_pre_0];

% 선행차량 가속도 시나리오
a_pre_scenario = zeros(1, N_sim);
a_pre_scenario(50:100) = -1.5;   % 5~10초: 감속
a_pre_scenario(150:200) = 1.0;   % 15~20초: 가속

% 결과 저장
x_history = zeros(n_x, N_sim + 1);
u_history = zeros(n_u, N_sim);
solve_time = zeros(1, N_sim);
x_history(:, 1) = x0;

% Warm start용 초기 추측
X_init = repmat(x0, 1, N+1);
U_init = zeros(n_u, N);

%% 7. MPC 시뮬레이션 루프
fprintf('7. MPC 시뮬레이션 시작...\n');

for k = 1:N_sim
    tic;

    % 현재 상태
    x_current = x_history(:, k);

    % 선행차량 가속도 예측 (현재값으로 일정하다고 가정)
    a_pre_pred = a_pre_scenario(k) * ones(1, N);

    % 파라미터 설정
    opti.set_value(P_x0, x_current);
    opti.set_value(P_a_pre, a_pre_pred);
    opti.set_value(P_v_pre_current, x_current(3));

    % Warm start
    opti.set_initial(X, X_init);
    opti.set_initial(U, U_init);

    % 최적화 문제 해결
    try
        sol = opti.solve();

        % 최적 해 추출
        X_opt = sol.value(X);
        U_opt = sol.value(U);
        u_opt = U_opt(:, 1);

        % 다음 스텝 warm start 업데이트
        X_init = [X_opt(:, 2:end), X_opt(:, end)];
        U_init = [U_opt(:, 2:end), U_opt(:, end)];

    catch ME
        warning('MPC 최적화 실패 at step %d: %s', k, ME.message);
        if k > 1
            u_opt = u_history(:, k-1);
        else
            u_opt = 0;
        end
    end

    solve_time(k) = toc;

    % 입력 적용 및 상태 업데이트
    u_history(:, k) = u_opt;
    x_history(:, k+1) = A * x_current + B * u_opt + Bd * a_pre_scenario(k);

    % 진행 상황 출력
    if mod(k, 50) == 0
        fprintf('  시뮬레이션 진행: %d/%d (%.1f%%), 평균 solve time: %.3f ms\n', ...
            k, N_sim, 100*k/N_sim, mean(solve_time(1:k))*1000);
    end
end

fprintf('\nMPC 시뮬레이션 완료!\n');
fprintf('평균 solve time: %.3f ms\n', mean(solve_time)*1000);
fprintf('최대 solve time: %.3f ms\n', max(solve_time)*1000);

%% 8. 결과 시각화
fprintf('8. 결과 시각화 중...\n');

time = (0:N_sim) * dt;
time_u = (0:N_sim-1) * dt;

figure('Position', [100, 100, 1400, 900]);

% 속도 비교
subplot(2,2,1);
plot(time, x_history(1,:), 'b-', 'LineWidth', 2, 'DisplayName', 'v_{ego} (자차)');
hold on;
plot(time, x_history(3,:), 'r--', 'LineWidth', 2, 'DisplayName', 'v_{pre} (선행차)');
plot(time, v_ref * ones(size(time)), 'g:', 'LineWidth', 1.5, 'DisplayName', '목표 속도');
xlabel('시간 (s)');
ylabel('속도 (m/s)');
title('속도 프로파일');
legend('Location', 'best');
grid on;

% 상대 거리
subplot(2,2,2);
plot(time, x_history(2,:), 'b-', 'LineWidth', 2, 'DisplayName', 'd_{rel}');
hold on;
plot(time, d_ref * ones(size(time)), 'g:', 'LineWidth', 1.5, 'DisplayName', '목표 거리');
plot(time, d_rel_min * ones(size(time)), 'r--', 'LineWidth', 1.5, 'DisplayName', '최소 거리 (15m)');
plot(time, d_rel_max * ones(size(time)), 'r--', 'LineWidth', 1.5, 'DisplayName', '최대 거리 (50m)');
xlabel('시간 (s)');
ylabel('상대 거리 (m)');
title('상대 거리 (STL 제약: 15 < d_{rel} < 50)');
legend('Location', 'best');
grid on;
ylim([0, 60]);

% 제어 입력
subplot(2,2,3);
stairs(time_u, u_history, 'b-', 'LineWidth', 2, 'DisplayName', 'a (자차 가속도)');
hold on;
stairs(time_u, a_pre_scenario, 'm--', 'LineWidth', 1.5, 'DisplayName', 'a_{pre} (선행차 가속도)');
plot(time_u, a_max * ones(size(time_u)), 'r:', 'LineWidth', 1.5, 'DisplayName', '최대 가속');
plot(time_u, a_min * ones(size(time_u)), 'r:', 'LineWidth', 1.5, 'DisplayName', '최대 감속');
xlabel('시간 (s)');
ylabel('가속도 (m/s^2)');
title('제어 입력 (가속도)');
legend('Location', 'best');
grid on;

% Solve time
subplot(2,2,4);
plot(time_u, solve_time * 1000, 'b-', 'LineWidth', 1.5);
xlabel('시간 (s)');
ylabel('Solve time (ms)');
title('MPC Solve Time');
grid on;

sgtitle('Adaptive Cruise Control - MPC (CasADi) 시뮬레이션 결과');

%% 9. STL 제약 만족 여부 확인
fprintf('\n9. 제약 조건 만족 여부 확인...\n');

d_rel_history = x_history(2,:);
stl_satisfied = all(d_rel_history > d_rel_min) && all(d_rel_history < d_rel_max);

fprintf('   상대거리 범위: [%.2f, %.2f] m\n', min(d_rel_history), max(d_rel_history));
fprintf('   STL 제약 φ := G(15 < d_rel < 50): ');
if stl_satisfied
    fprintf('만족 ✓\n');
else
    fprintf('위반 ✗\n');
    violations = sum(d_rel_history <= d_rel_min | d_rel_history >= d_rel_max);
    fprintf('   위반 횟수: %d / %d\n', violations, length(d_rel_history));
end

fprintf('\n========================================\n');
fprintf('시뮬레이션 완료!\n');
fprintf('========================================\n');
