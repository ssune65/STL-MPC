"""
=========================================================
    ACC MPC with HighwayEnv Simulation
=========================================================

CasADi MPC를 HighwayEnv 시뮬레이터와 연동한 ACC 구현

설치:
    pip install casadi numpy matplotlib highway-env gymnasium

HighwayEnv 문서:
    https://highway-env.farama.org/

=========================================================
"""

import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
import platform
import time

# HighwayEnv import
import gymnasium as gym
import highway_env

# 한글 폰트 설정
def set_korean_font():
    system = platform.system()
    if system == 'Windows':
        font_name = 'Malgun Gothic'
    elif system == 'Darwin':
        font_name = 'AppleGothic'
    else:
        font_name = 'NanumGothic'
    plt.rcParams['font.family'] = font_name
    plt.rcParams['axes.unicode_minus'] = False

set_korean_font()


class ACC_MPC_Controller:
    """CasADi 기반 ACC MPC 컨트롤러"""

    def __init__(self, dt=0.1, N=20):
        """
        Args:
            dt: 샘플링 시간 (s)
            N: 예측 구간
        """
        self.dt = dt
        self.N = N
        self.n_x = 3  # [v_ego, d_rel, v_pre]
        self.n_u = 1  # [a]

        # 시스템 행렬
        self.A = np.array([
            [1,   0,   0],
            [-dt, 1,   dt],
            [0,   0,   1]
        ])
        self.B = np.array([[dt], [0], [0]])
        self.Bd = np.array([[0], [0], [dt]])

        # 제약 조건
        self.v_ego_min, self.v_ego_max = 0, 40
        self.d_rel_min, self.d_rel_max = 15, 50  # STL: G(15 < d_rel < 50)
        self.v_pre_min, self.v_pre_max = 0, 40
        self.a_min, self.a_max = -3, 2

        # 비용 함수 파라미터
        self.d_ref = 30
        self.v_ref = 25
        self.Q = np.diag([10, 50, 0])
        self.R = np.array([[10]])
        self.Q_terminal = self.Q * 2

        # MPC 문제 설정
        self._setup_mpc()

        # Warm start 변수
        self.X_init = None
        self.U_init = None

    def _setup_mpc(self):
        """CasADi MPC 문제 설정"""
        opti = ca.Opti()

        # 결정 변수
        X = opti.variable(self.n_x, self.N + 1)
        U = opti.variable(self.n_u, self.N)

        # 파라미터
        P_x0 = opti.parameter(self.n_x, 1)
        P_a_pre = opti.parameter(1, self.N)
        P_v_pre_current = opti.parameter(1, 1)

        # 비용 함수
        cost = 0
        for k in range(self.N):
            x_ref_k = ca.vertcat(self.v_ref, self.d_ref, P_v_pre_current)
            cost += ca.mtimes([(X[:, k] - x_ref_k).T, self.Q, (X[:, k] - x_ref_k)])
            cost += ca.mtimes([U[:, k].T, self.R, U[:, k]])

        x_ref_N = ca.vertcat(self.v_ref, self.d_ref, P_v_pre_current)
        cost += ca.mtimes([(X[:, self.N] - x_ref_N).T, self.Q_terminal, (X[:, self.N] - x_ref_N)])

        opti.minimize(cost)

        # 제약 조건
        opti.subject_to(X[:, 0] == P_x0)

        x_min = np.array([self.v_ego_min, self.d_rel_min, self.v_pre_min])
        x_max = np.array([self.v_ego_max, self.d_rel_max, self.v_pre_max])

        for k in range(self.N):
            x_next = ca.mtimes(self.A, X[:, k]) + ca.mtimes(self.B, U[:, k]) + self.Bd.flatten() * P_a_pre[k]
            opti.subject_to(X[:, k + 1] == x_next)
            opti.subject_to(opti.bounded(x_min, X[:, k + 1], x_max))
            opti.subject_to(opti.bounded(self.a_min, U[:, k], self.a_max))

        # 솔버 설정
        opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.warm_start_init_point': 'yes'}
        opti.solver('ipopt', opts)

        # 저장
        self.opti = opti
        self.X = X
        self.U = U
        self.P_x0 = P_x0
        self.P_a_pre = P_a_pre
        self.P_v_pre_current = P_v_pre_current

    def compute_control(self, v_ego, d_rel, v_pre, a_pre_pred=0.0):
        """
        MPC 제어 입력 계산

        Args:
            v_ego: 자차 속도 (m/s)
            d_rel: 상대 거리 (m)
            v_pre: 선행차량 속도 (m/s)
            a_pre_pred: 선행차량 가속도 예측 (m/s^2)

        Returns:
            a_opt: 최적 가속도 (m/s^2)
        """
        x_current = np.array([v_ego, d_rel, v_pre])
        a_pre_array = np.ones(self.N) * a_pre_pred

        # 파라미터 설정
        self.opti.set_value(self.P_x0, x_current)
        self.opti.set_value(self.P_a_pre, a_pre_array.reshape(1, -1))
        self.opti.set_value(self.P_v_pre_current, v_pre)

        # Warm start
        if self.X_init is not None:
            self.opti.set_initial(self.X, self.X_init)
            self.opti.set_initial(self.U, self.U_init)
        else:
            self.opti.set_initial(self.X, np.tile(x_current.reshape(-1, 1), (1, self.N + 1)))
            self.opti.set_initial(self.U, np.zeros((self.n_u, self.N)))

        try:
            sol = self.opti.solve()

            X_opt = np.atleast_2d(sol.value(self.X))
            U_opt = np.atleast_2d(sol.value(self.U))

            if U_opt.shape[0] == self.N:
                U_opt = U_opt.T

            a_opt = float(U_opt[0, 0])

            # Warm start 업데이트
            self.X_init = np.hstack([X_opt[:, 1:], X_opt[:, -1:]])
            self.U_init = np.hstack([U_opt[:, 1:], U_opt[:, -1:]])

            return a_opt

        except Exception as e:
            print(f"MPC solve failed: {e}")
            return 0.0


def extract_state_from_obs(obs, ego_speed=None, ego_vehicle_idx=0):
    """
    HighwayEnv 관측값에서 ACC 상태 추출

    HighwayEnv Kinematics observation (absolute=False):
        각 차량: [presence, x, y, vx, vy]
        - presence: 차량 존재 여부 (1 or 0)
        - x, y: 자차 기준 상대 위치 (정규화됨)
        - vx, vy: 자차 기준 상대 속도 (정규화됨)

        자차(ego)는 항상 [1, 0, 0, 0, 0]으로 표시됨 (상대좌표 기준점)

    Args:
        obs: HighwayEnv observation (N_vehicles x 5)
        ego_speed: 자차 실제 속도 (m/s) - env.vehicle.speed에서 가져옴
        ego_vehicle_idx: 자차 인덱스 (보통 0)

    Returns:
        v_ego, d_rel, v_pre, has_lead_vehicle
    """
    # 정규화 스케일 (HighwayEnv 기본값)
    # features_range: x=[-100,100], y=[-10,10], vx/vy=[-20,20]
    X_SCALE = 100.0  # x 정규화 스케일
    Y_SCALE = 10.0   # y 정규화 스케일 (차선 폭 고려)
    V_SCALE = 20.0   # 속도 정규화 스케일

    # 자차 속도 (환경에서 직접 가져옴)
    if ego_speed is not None:
        v_ego = ego_speed
    else:
        v_ego = 25.0  # 기본값 fallback

    # 선행 차량 찾기 (자차 앞에 있는 가장 가까운 차량)
    lead_vehicle = None
    min_distance = float('inf')
    lead_rel_vx = 0.0

    for i, vehicle in enumerate(obs):
        if i == ego_vehicle_idx:
            continue
        if vehicle[0] < 0.5:  # presence check
            continue

        # 상대 위치 (이미 자차 기준 상대좌표)
        rel_x = vehicle[1] * X_SCALE  # x 역정규화 (앞쪽이 양수)
        rel_y = vehicle[2] * Y_SCALE  # y 역정규화 (옆 차선)
        rel_vx = vehicle[3] * V_SCALE  # 상대 속도 x

        # 같은 차선 (y 차이가 작음) & 앞에 있음 (x > 0)
        # 차선 폭 약 4m이므로 ±2m 이내는 같은 차선으로 판단
        LANE_THRESHOLD = 2.5
        if abs(rel_y) < LANE_THRESHOLD and rel_x > 1.0 and rel_x < min_distance:
            min_distance = rel_x
            lead_vehicle = vehicle
            lead_rel_vx = rel_vx

    if lead_vehicle is not None:
        d_rel = min_distance
        # 선행차 속도 = 자차 속도 + 상대속도
        v_pre = max(0.0, v_ego + lead_rel_vx)  # 음수 속도 방지
        # 거리 안전 범위 클램핑
        d_rel = max(1.0, min(d_rel, 150.0))
        return v_ego, d_rel, v_pre, True
    else:
        # 선행 차량 없음 - 기본값 사용
        return v_ego, 100.0, v_ego, False


def run_simulation(render_mode='human', max_steps=500, num_episodes=10):
    """
    HighwayEnv에서 ACC MPC 시뮬레이션 실행 (다중 에피소드)

    Args:
        render_mode: 'human' (시각화) 또는 'rgb_array'
        max_steps: 에피소드당 최대 시뮬레이션 스텝
        num_episodes: 반복할 에피소드 수
    """
    print("=" * 50)
    print("ACC MPC + HighwayEnv Simulation")
    print(f"총 {num_episodes}개 에피소드 실행")
    print("=" * 50 + "\n")

    # HighwayEnv 설정
    env_config = {
        "observation": {
            "type": "Kinematics",
            "vehicles_count": 5,
            "features": ["presence", "x", "y", "vx", "vy"],
            "absolute": False,
            "normalize": True,
        },
        "action": {
            "type": "ContinuousAction",
            "acceleration_range": [-3.0, 3.0],
            "steering_range": [-0.3, 0.3],
        },
        "lanes_count": 3,
        "vehicles_count": 10,
        "duration": 200,  # 에피소드 최대 시간 (초)
        "initial_spacing": 2,
        "collision_reward": -1,
        "simulation_frequency": 10,
        "policy_frequency": 10,
    }

    env = gym.make('highway-v0', render_mode=render_mode, config=env_config)

    # MPC 컨트롤러 초기화
    mpc = ACC_MPC_Controller(dt=0.1, N=20)

    # 에피소드별 통계 저장
    episode_stats = {
        'rewards': [],
        'steps': [],
        'collisions': [],
        'min_d_rel': [],
        'avg_d_rel': []
    }

    # 마지막 에피소드 기록 (시각화용)
    last_history = None

    print("시뮬레이션 시작...")
    print("(ESC 또는 창 닫기로 종료)\n")

    try:
        for episode in range(num_episodes):
            print(f"\n{'='*30}")
            print(f"에피소드 {episode + 1}/{num_episodes}")
            print(f"{'='*30}")

            # 에피소드 결과 저장
            history = {
                'v_ego': [], 'd_rel': [], 'v_pre': [],
                'acceleration': [], 'reward': [], 'time': []
            }

            # 에피소드 시작
            obs, info = env.reset()
            total_reward = 0
            step = 0
            collision = False

            # MPC warm start 리셋
            mpc.X_init = None
            mpc.U_init = None

            while step < max_steps:
                # 자차 실제 속도 가져오기
                ego_speed = env.unwrapped.vehicle.speed if hasattr(env.unwrapped, 'vehicle') else 25.0

                # 상태 추출
                v_ego, d_rel, v_pre, has_lead = extract_state_from_obs(obs, ego_speed=ego_speed)

                # MPC 제어 계산
                if has_lead and d_rel < 80:
                    # 긴급 제동: 거리가 너무 가까우면 (MPC 제약 위반 가능)
                    if d_rel < 10:
                        acceleration = -3.0  # 최대 제동
                    elif d_rel < 15:
                        # 부드러운 긴급 제동
                        acceleration = -2.5
                    else:
                        # 선행 차량이 있고 안전거리 내일 때 MPC 사용
                        # d_rel을 MPC 제약 범위 내로 클램핑
                        d_rel_clamped = np.clip(d_rel, 16, 49)  # 약간의 여유 두고 클램핑
                        acceleration = mpc.compute_control(v_ego, d_rel_clamped, v_pre)
                else:
                    # 선행 차량 없거나 멀 때 - 목표 속도로 가속/감속
                    target_speed = 25
                    acceleration = np.clip(0.5 * (target_speed - v_ego), -3, 2)

                # 환경에 액션 적용 [acceleration, steering]
                # steering은 0으로 고정 (차선 유지)
                action = np.array([acceleration / 3.0, 0.0])  # 정규화

                obs, reward, terminated, truncated, info = env.step(action)
                total_reward += reward
                step += 1

                # 기록 저장
                history['v_ego'].append(v_ego)
                history['d_rel'].append(d_rel if has_lead else np.nan)
                history['v_pre'].append(v_pre if has_lead else np.nan)
                history['acceleration'].append(acceleration)
                history['reward'].append(reward)
                history['time'].append(step * 0.1)

                # 진행 상황 출력
                if step % 100 == 0:
                    print(f"  Step {step}: v_ego={v_ego:.1f} m/s, d_rel={d_rel:.1f} m, a={acceleration:.2f} m/s²")

                if terminated or truncated:
                    if terminated:
                        collision = True
                        print(f"  충돌 발생! (step {step})")
                    break

            # 에피소드 통계 저장
            d_rel_values = [d for d in history['d_rel'] if not np.isnan(d)]
            episode_stats['rewards'].append(total_reward)
            episode_stats['steps'].append(step)
            episode_stats['collisions'].append(collision)
            episode_stats['min_d_rel'].append(min(d_rel_values) if d_rel_values else np.nan)
            episode_stats['avg_d_rel'].append(np.mean(d_rel_values) if d_rel_values else np.nan)

            print(f"  결과: 리워드={total_reward:.2f}, 스텝={step}, 충돌={'예' if collision else '아니오'}")
            if d_rel_values:
                print(f"  상대거리: 최소={min(d_rel_values):.1f}m, 평균={np.mean(d_rel_values):.1f}m")

            last_history = history

    except KeyboardInterrupt:
        print("\n\n사용자에 의해 중단됨")

    env.close()

    # 전체 통계 출력
    print("\n" + "=" * 50)
    print("전체 시뮬레이션 결과 요약")
    print("=" * 50)
    print(f"총 에피소드: {len(episode_stats['rewards'])}")
    print(f"평균 리워드: {np.mean(episode_stats['rewards']):.2f} ± {np.std(episode_stats['rewards']):.2f}")
    print(f"평균 스텝: {np.mean(episode_stats['steps']):.1f}")
    print(f"충돌 횟수: {sum(episode_stats['collisions'])} / {len(episode_stats['collisions'])}")
    print(f"충돌률: {100 * sum(episode_stats['collisions']) / len(episode_stats['collisions']):.1f}%")

    valid_min_d = [d for d in episode_stats['min_d_rel'] if not np.isnan(d)]
    if valid_min_d:
        print(f"최소 상대거리 (전체): {min(valid_min_d):.1f}m")
        print(f"평균 최소거리: {np.mean(valid_min_d):.1f}m")

    # STL 제약 만족 여부
    stl_violations = sum(1 for d in valid_min_d if d < 15)
    print(f"\nSTL 제약 위반 (d_rel < 15m): {stl_violations} / {len(valid_min_d)} 에피소드")

    # 마지막 에피소드 결과 시각화
    if last_history:
        plot_results(last_history, episode_stats)

    return episode_stats


def plot_results(history, episode_stats=None):
    """시뮬레이션 결과 시각화"""
    if episode_stats is not None:
        fig, axes = plt.subplots(2, 3, figsize=(18, 9))
    else:
        fig, axes = plt.subplots(2, 2, figsize=(14, 9))
        axes = np.array([[axes[0, 0], axes[0, 1], None],
                         [axes[1, 0], axes[1, 1], None]])

    time = history['time']

    # 속도
    ax1 = axes[0, 0]
    ax1.plot(time, history['v_ego'], 'b-', linewidth=2, label='v_ego (자차)')
    ax1.plot(time, history['v_pre'], 'r--', linewidth=2, label='v_pre (선행차)')
    ax1.axhline(y=25, color='g', linestyle=':', linewidth=1.5, label='목표 속도')
    ax1.set_xlabel('시간 (s)')
    ax1.set_ylabel('속도 (m/s)')
    ax1.set_title('속도 프로파일 (마지막 에피소드)')
    ax1.legend(loc='best')
    ax1.grid(True)

    # 상대 거리
    ax2 = axes[0, 1]
    ax2.plot(time, history['d_rel'], 'b-', linewidth=2, label='d_rel')
    ax2.axhline(y=30, color='g', linestyle=':', linewidth=1.5, label='목표 거리')
    ax2.axhline(y=15, color='r', linestyle='--', linewidth=1.5, label='최소 거리 (15m)')
    ax2.axhline(y=50, color='r', linestyle='--', linewidth=1.5, label='최대 거리 (50m)')
    ax2.set_xlabel('시간 (s)')
    ax2.set_ylabel('상대 거리 (m)')
    ax2.set_title('상대 거리 (STL: 15 < d_rel < 50)')
    ax2.legend(loc='best')
    ax2.grid(True)
    ax2.set_ylim([0, 80])

    # 가속도
    ax3 = axes[1, 0]
    ax3.plot(time, history['acceleration'], 'b-', linewidth=2, label='가속도')
    ax3.axhline(y=2, color='r', linestyle=':', linewidth=1.5, label='최대 가속')
    ax3.axhline(y=-3, color='r', linestyle=':', linewidth=1.5, label='최대 감속')
    ax3.set_xlabel('시간 (s)')
    ax3.set_ylabel('가속도 (m/s²)')
    ax3.set_title('제어 입력')
    ax3.legend(loc='best')
    ax3.grid(True)

    # 리워드
    ax4 = axes[1, 1]
    ax4.plot(time, np.cumsum(history['reward']), 'b-', linewidth=2)
    ax4.set_xlabel('시간 (s)')
    ax4.set_ylabel('누적 리워드')
    ax4.set_title('누적 리워드 (마지막 에피소드)')
    ax4.grid(True)

    # 에피소드별 통계 (있는 경우)
    if episode_stats is not None:
        num_episodes = len(episode_stats['rewards'])
        episodes = range(1, num_episodes + 1)

        # 에피소드별 리워드
        ax5 = axes[0, 2]
        colors = ['r' if c else 'b' for c in episode_stats['collisions']]
        ax5.bar(episodes, episode_stats['rewards'], color=colors, alpha=0.7)
        ax5.axhline(y=np.mean(episode_stats['rewards']), color='g', linestyle='--',
                    linewidth=2, label=f'평균: {np.mean(episode_stats["rewards"]):.1f}')
        ax5.set_xlabel('에피소드')
        ax5.set_ylabel('리워드')
        ax5.set_title('에피소드별 리워드 (빨간색=충돌)')
        ax5.legend()
        ax5.grid(True, axis='y')

        # 에피소드별 최소 거리
        ax6 = axes[1, 2]
        ax6.bar(episodes, episode_stats['min_d_rel'], color='steelblue', alpha=0.7)
        ax6.axhline(y=15, color='r', linestyle='--', linewidth=2, label='STL 제약 (15m)')
        ax6.axhline(y=np.mean([d for d in episode_stats['min_d_rel'] if not np.isnan(d)]),
                    color='g', linestyle='--', linewidth=2, label='평균')
        ax6.set_xlabel('에피소드')
        ax6.set_ylabel('최소 상대거리 (m)')
        ax6.set_title('에피소드별 최소 상대거리')
        ax6.legend()
        ax6.grid(True, axis='y')

    title = 'ACC MPC + HighwayEnv 시뮬레이션 결과'
    if episode_stats is not None:
        collision_rate = 100 * sum(episode_stats['collisions']) / len(episode_stats['collisions'])
        title += f' (총 {len(episode_stats["rewards"])}개 에피소드, 충돌률: {collision_rate:.0f}%)'
    fig.suptitle(title, fontsize=14)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    # 시뮬레이션 실행
    # render_mode='human': 실시간 시각화
    # render_mode='rgb_array': 시각화 없이 실행 (더 빠름)
    # num_episodes: 반복할 에피소드 수
    episode_stats = run_simulation(
        render_mode='human',
        max_steps=1000,  # 1000 스텝 = 100초 (duration=200까지 가능)
        num_episodes=10
    )
