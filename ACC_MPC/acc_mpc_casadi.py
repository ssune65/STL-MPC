"""
=========================================================
    Adaptive Cruise Control - MPC (CasADi, Python)
=========================================================

CasADi를 사용한 ACC MPC 구현 (Python 버전)

상태변수:
    x = [v_ego, d_rel, v_pre]
    - v_ego: 자차 속도 (m/s)
    - d_rel: 상대 거리 (m)
    - v_pre: 선행차량 속도 (m/s)

제어변수:
    u = a (가속도, m/s^2)

제약 조건:
    φ := G(15 < d_rel < 50)

설치:
    pip install casadi numpy matplotlib

=========================================================
"""

import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
import time
import platform

# 한글 폰트 설정
def set_korean_font():
    system = platform.system()
    font_name = 'Malgun Gothic'
    plt.rcParams['font.family'] = font_name
    plt.rcParams['axes.unicode_minus'] = False
set_korean_font()

def main():
    print("=" * 50)
    print("Adaptive Cruise Control - MPC (CasADi, Python)")
    print("=" * 50 + "\n")

    # ========================================
    # 1. 시스템 파라미터 설정
    # ========================================
    print("1. 시스템 파라미터 설정 중...")

    dt = 0.1        # 샘플링 시간 (s)
    N = 30          # 예측 구간 (horizon)
    T_sim = 30      # 시뮬레이션 시간 (s)
    N_sim = int(T_sim / dt)  # 시뮬레이션 스텝 수

    n_x = 3  # 상태 차원 [v_ego, d_rel, v_pre]
    n_u = 1  # 입력 차원 [a]

    # ========================================
    # 2. 시스템 동역학 (이산 시간 모델)
    # ========================================
    print("2. 시스템 동역학 정의 중...")

    # x(k+1) = A * x(k) + B * u(k) + Bd * a_pre(k)
    A = np.array([
        [1,   0,   0],      # v_ego(k+1) = v_ego(k)
        [-dt, 1,   dt],     # d_rel(k+1) = d_rel(k) + dt*(v_pre - v_ego)
        [0,   0,   1]       # v_pre(k+1) = v_pre(k)
    ])

    B = np.array([
        [dt],   # v_ego에 가속도 영향
        [0],    # d_rel에 직접 영향 없음
        [0]     # v_pre에 영향 없음
    ])

    Bd = np.array([
        [0],    # 선행차량 가속도가 v_ego에 영향 없음
        [0],    # d_rel에 직접 영향 없음
        [dt]    # v_pre에 선행차량 가속도 영향
    ])

    # ========================================
    # 3. 제약 조건 설정
    # ========================================
    print("3. 제약 조건 설정 중...")

    # 상태 제약
    v_ego_min, v_ego_max = 0, 40
    d_rel_min, d_rel_max = 15, 50   # STL 제약: G(15 < d_rel < 50)
    v_pre_min, v_pre_max = 0, 40

    x_min = np.array([v_ego_min, d_rel_min, v_pre_min])
    x_max = np.array([v_ego_max, d_rel_max, v_pre_max])

    # 입력 제약
    a_min = -3  # 최대 감속 (m/s^2)
    a_max = 2   # 최대 가속 (m/s^2)

    # ========================================
    # 4. MPC 비용 함수 가중치
    # ========================================
    print("4. MPC 비용 함수 설정 중...")

    d_ref = 30      # 목표 상대거리 (m)
    v_ref = 25      # 목표 속도 (m/s)

    Q = np.diag([10, 50, 0])    # 상태 가중치
    R = np.array([[10]])        # 입력 가중치
    Q_terminal = Q * 2          # 종료 비용 가중치

    # ========================================
    # 5. CasADi MPC 문제 설정
    # ========================================
    print("5. CasADi MPC 문제 설정 중...")

    opti = ca.Opti()

    # 결정 변수
    X = opti.variable(n_x, N + 1)   # 상태 궤적
    U = opti.variable(n_u, N)       # 입력 궤적

    # 파라미터
    P_x0 = opti.parameter(n_x, 1)           # 초기 상태
    P_a_pre = opti.parameter(1, N)          # 선행차량 가속도 예측
    P_v_pre_current = opti.parameter(1, 1)  # 현재 선행차량 속도

    # 비용 함수
    cost = 0
    for k in range(N):
        x_ref_k = ca.vertcat(v_ref, d_ref, P_v_pre_current)
        cost += ca.mtimes([(X[:, k] - x_ref_k).T, Q, (X[:, k] - x_ref_k)])
        cost += ca.mtimes([U[:, k].T, R, U[:, k]])

    # 종료 비용
    x_ref_N = ca.vertcat(v_ref, d_ref, P_v_pre_current)
    cost += ca.mtimes([(X[:, N] - x_ref_N).T, Q_terminal, (X[:, N] - x_ref_N)])

    opti.minimize(cost)

    # 제약 조건
    # 초기 조건
    opti.subject_to(X[:, 0] == P_x0)

    # 동역학 및 상태/입력 제약
    for k in range(N):
        # 시스템 동역학
        x_next = ca.mtimes(A, X[:, k]) + ca.mtimes(B, U[:, k]) + Bd.flatten() * P_a_pre[k]
        opti.subject_to(X[:, k + 1] == x_next)

        # 상태 제약: 15 < d_rel < 50 (STL 핵심 제약)
        opti.subject_to(opti.bounded(x_min, X[:, k + 1], x_max))

        # 입력 제약
        opti.subject_to(opti.bounded(a_min, U[:, k], a_max))

    # 솔버 설정 (IPOPT)
    opts = {
        'ipopt.print_level': 0,
        'print_time': 0,
        'ipopt.warm_start_init_point': 'yes'
    }
    opti.solver('ipopt', opts)

    # ========================================
    # 6. 초기 조건 및 시뮬레이션 설정
    # ========================================
    print("6. 시뮬레이션 설정 중...")

    # 초기 상태
    v_ego_0 = 20    # 자차 초기 속도 (m/s)
    d_rel_0 = 35    # 초기 상대거리 (m)
    v_pre_0 = 22    # 선행차량 초기 속도 (m/s)

    x0 = np.array([v_ego_0, d_rel_0, v_pre_0])

    # 선행차량 가속도 시나리오
    a_pre_scenario = np.zeros(N_sim)
    a_pre_scenario[50:100] = -1.5   # 5~10초: 감속
    a_pre_scenario[150:200] = 1.0   # 15~20초: 가속

    # 결과 저장
    x_history = np.zeros((n_x, N_sim + 1))
    u_history = np.zeros((n_u, N_sim))
    solve_times = np.zeros(N_sim)
    x_history[:, 0] = x0

    # Warm start용 초기 추측
    X_init = np.tile(x0.reshape(-1, 1), (1, N + 1))
    U_init = np.zeros((n_u, N))

    # ========================================
    # 7. MPC 시뮬레이션 루프
    # ========================================
    print("7. MPC 시뮬레이션 시작...")

    for k in range(N_sim):
        start_time = time.time()

        # 현재 상태
        x_current = x_history[:, k]

        # 선행차량 가속도 예측
        a_pre_pred = np.ones(N) * a_pre_scenario[k]

        # 파라미터 설정
        opti.set_value(P_x0, x_current)
        opti.set_value(P_a_pre, a_pre_pred.reshape(1, -1))
        opti.set_value(P_v_pre_current, x_current[2])

        # Warm start
        opti.set_initial(X, X_init)
        opti.set_initial(U, U_init)

        try:
            sol = opti.solve()
            # 최적 해 추출 (차원 보정)
            X_opt = np.atleast_2d(sol.value(X))
            U_opt = np.atleast_2d(sol.value(U))

            # U가 (N,) 형태로 반환될 수 있으므로 (1, N) 형태로 보정
            if U_opt.shape[0] == N:
                U_opt = U_opt.T

            u_opt = U_opt[0, 0]  # 스칼라 값 추출

            # 다음 스텝 warm start 업데이트
            X_init = np.hstack([X_opt[:, 1:], X_opt[:, -1:]])
            U_init = np.hstack([U_opt[:, 1:], U_opt[:, -1:]])

        except Exception as e:
            print(f"MPC 최적화 실패 at step {k}: {e}")
            u_opt = u_history[0, k - 1] if k > 0 else 0.0

        solve_times[k] = time.time() - start_time

        # 입력 적용 및 상태 업데이트
        u_history[:, k] = u_opt
        x_history[:, k + 1] = (A @ x_current + B.flatten() * u_opt +
                               Bd.flatten() * a_pre_scenario[k])

        # 진행 상황 출력
        if (k + 1) % 50 == 0:
            avg_time = np.mean(solve_times[:k + 1]) * 1000
            print(f"  시뮬레이션 진행: {k + 1}/{N_sim} ({100 * (k + 1) / N_sim:.1f}%), "
                  f"평균 solve time: {avg_time:.3f} ms")

    print(f"\nMPC 시뮬레이션 완료!")
    print(f"평균 solve time: {np.mean(solve_times) * 1000:.3f} ms")
    print(f"최대 solve time: {np.max(solve_times) * 1000:.3f} ms")

    # ========================================
    # 8. 결과 시각화
    # ========================================
    print("8. 결과 시각화 중...")

    time_vec = np.arange(N_sim + 1) * dt
    time_u = np.arange(N_sim) * dt

    fig, axes = plt.subplots(2, 2, figsize=(14, 9))

    # 속도 비교
    ax1 = axes[0, 0]
    ax1.plot(time_vec, x_history[0, :], 'b-', linewidth=2, label='v_ego (자차)')
    ax1.plot(time_vec, x_history[2, :], 'r--', linewidth=2, label='v_pre (선행차)')
    ax1.axhline(y=v_ref, color='g', linestyle=':', linewidth=1.5, label='목표 속도')
    ax1.set_xlabel('시간 (s)')
    ax1.set_ylabel('속도 (m/s)')
    ax1.set_title('속도 프로파일')
    ax1.legend(loc='best')
    ax1.grid(True)

    # 상대 거리
    ax2 = axes[0, 1]
    ax2.plot(time_vec, x_history[1, :], 'b-', linewidth=2, label='d_rel')
    ax2.axhline(y=d_ref, color='g', linestyle=':', linewidth=1.5, label='목표 거리')
    ax2.axhline(y=d_rel_min, color='r', linestyle='--', linewidth=1.5, label='최소 거리 (15m)')
    ax2.axhline(y=d_rel_max, color='r', linestyle='--', linewidth=1.5, label='최대 거리 (50m)')
    ax2.set_xlabel('시간 (s)')
    ax2.set_ylabel('상대 거리 (m)')
    ax2.set_title('상대 거리 (STL 제약: 15 < d_rel < 50)')
    ax2.legend(loc='best')
    ax2.grid(True)
    ax2.set_ylim([0, 60])

    # 제어 입력
    ax3 = axes[1, 0]
    ax3.step(time_u, u_history.flatten(), 'b-', linewidth=2, where='post', label='a (자차 가속도)')
    ax3.step(time_u, a_pre_scenario, 'm--', linewidth=1.5, where='post', label='a_pre (선행차 가속도)')
    ax3.axhline(y=a_max, color='r', linestyle=':', linewidth=1.5, label='최대 가속')
    ax3.axhline(y=a_min, color='r', linestyle=':', linewidth=1.5, label='최대 감속')
    ax3.set_xlabel('시간 (s)')
    ax3.set_ylabel('가속도 (m/s²)')
    ax3.set_title('제어 입력 (가속도)')
    ax3.legend(loc='best')
    ax3.grid(True)

    # Solve time
    ax4 = axes[1, 1]
    ax4.plot(time_u, solve_times * 1000, 'b-', linewidth=1.5)
    ax4.set_xlabel('시간 (s)')
    ax4.set_ylabel('Solve time (ms)')
    ax4.set_title('MPC Solve Time')
    ax4.grid(True)

    fig.suptitle('Adaptive Cruise Control - MPC (CasADi, Python) 시뮬레이션 결과', fontsize=14)
    plt.tight_layout()

    # ========================================
    # 9. STL 제약 만족 여부 확인
    # ========================================
    print("\n9. 제약 조건 만족 여부 확인...")

    d_rel_history = x_history[1, :]
    stl_satisfied = np.all(d_rel_history > d_rel_min) and np.all(d_rel_history < d_rel_max)

    print(f"   상대거리 범위: [{np.min(d_rel_history):.2f}, {np.max(d_rel_history):.2f}] m")
    print(f"   STL 제약 φ := G(15 < d_rel < 50): ", end="")
    if stl_satisfied:
        print("만족 ✓")
    else:
        print("위반 ✗")
        violations = np.sum((d_rel_history <= d_rel_min) | (d_rel_history >= d_rel_max))
        print(f"   위반 횟수: {violations} / {len(d_rel_history)}")

    print("\n" + "=" * 50)
    print("시뮬레이션 완료!")
    print("=" * 50)

    plt.show()


if __name__ == "__main__":
    main()
