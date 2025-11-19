# STL-MPC 빠른 시작 가이드

## 5분 안에 시작하기

### 1단계: 필수 도구 설치

#### YALMIP 설치
```matlab
% 1. https://yalmip.github.io/download/ 에서 YALMIP 다운로드
% 2. 압축 해제 후 MATLAB 경로에 추가
addpath(genpath('C:/path/to/YALMIP-master'))
savepath
```

#### Solver 설치 (Gurobi 권장)
```matlab
% 1. https://www.gurobi.com/downloads/ 에서 Gurobi 다운로드
% 2. 학술 라이선스: https://www.gurobi.com/academia/
% 3. MATLAB 경로에 추가
addpath('C:/gurobi1100/win64/matlab')
savepath
```

#### 설치 확인
```matlab
yalmiptest  % YALMIP과 solver가 제대로 설치되었는지 확인
```

### 2단계: 간단한 예제 실행

#### 예제 A: 기본 STL 연산자 테스트
```matlab
% test_stl_operators.m 실행
test_stl_operators
```

**무엇을 배울 수 있나요?**
- Atomic predicates 사용법
- Always, Eventually 연산자
- AND, OR, NOT 연산자
- Robustness 값 해석

#### 예제 B: 완전한 STL-MPC 문제
```matlab
% stl_mpc_example.m 실행
% 주의: solver 설정을 먼저 확인하세요
stl_mpc_example
```

**무엇을 배울 수 있나요?**
- 시스템 동역학 정의
- STL 제약조건 인코딩
- MPC 최적화 문제 해결
- 결과 시각화

### 3단계: 나만의 STL 제약조건 만들기

#### 기본 템플릿

```matlab
%% 1. 시스템 정의
A = [1, 0.1; 0, 1];  % 상태 전이 행렬
B = [0; 0.1];         % 입력 행렬
N = 20;               % 예측 구간

%% 2. 변수 정의
x = sdpvar(2, N+1);   % 상태 궤적
u = sdpvar(1, N);     % 입력 궤적

%% 3. STL 제약조건 정의
% 예: □[0,10](x1 ≤ 5) - "x1이 항상 5 이하"

% Predicate 정의
pred = @(k) deal(5 - x(1, k+1), []);

% Always 연산자 적용
[r_stl, F_stl] = robust_always_simple(0, [0, 10], pred);

% STL 만족 조건
F = [F_stl, r_stl >= 0];

%% 4. 최적화 및 해결
% (시스템 동역학, 비용함수 등 추가)
% optimize(F, cost, options)
```

## 자주 사용하는 STL 패턴

### 패턴 1: 안전 제약 (Safety)
"항상 안전 영역 내에 머물러야 함"

```matlab
% □[0,N](x_min ≤ x ≤ x_max)

% Lower bound: x >= x_min → -x + x_min <= 0
pred_lower = @(k) deal(x_min - x(:,k+1), []);

% Upper bound: x <= x_max → x - x_max <= 0
pred_upper = @(k) deal(x_max - x(:,k+1), []);

% AND 조합
safety = @(k) robust_and_simple(@() pred_lower(k), @() pred_upper(k));

% ALWAYS 적용
[r_safety, F_safety] = robust_always_simple(0, [0, N], safety);
```

### 패턴 2: 목표 도달 (Reachability)
"특정 시간 내에 목표에 도달해야 함"

```matlab
% ◇[t1,t2](‖x - x_goal‖ ≤ ε)

% 목표 근처 조건 (간단히 하기 위해 각 차원 독립적으로)
goal_x1 = @(k) deal(eps - abs(x(1,k+1) - x_goal(1)), []);
goal_x2 = @(k) deal(eps - abs(x(2,k+1) - x_goal(2)), []);

goal = @(k) robust_and_simple(@() goal_x1(k), @() goal_x2(k));

% EVENTUALLY 적용
[r_reach, F_reach] = robust_eventually_simple(0, [t1, t2], goal);
```

### 패턴 3: 장애물 회피 (Obstacle Avoidance)
"항상 장애물 영역을 피해야 함"

```matlab
% □[0,N](¬obstacle)
% obstacle: x ∈ [obs_min, obs_max]

% 장애물 내부 조건
obs_x1_in = @(k) robust_and_simple(...
    @() deal(x(1,k+1) - obs_min(1), []), ...  % x1 >= obs_min(1)
    @() deal(obs_max(1) - x(1,k+1), []));     % x1 <= obs_max(1)

obs_x2_in = @(k) robust_and_simple(...
    @() deal(x(2,k+1) - obs_min(2), []), ...
    @() deal(obs_max(2) - x(2,k+1), []));

obstacle_in = @(k) robust_and_simple(@() obs_x1_in(k), @() obs_x2_in(k));

% NOT 적용 (장애물 밖)
avoid = @(k) deal(-obstacle_in(k));  % 간단히 부호 반전

% ALWAYS 적용
[r_avoid, F_avoid] = robust_always_simple(0, [0, N], avoid);
```

### 패턴 4: 순차 목표 (Sequential Goals)
"목표 A를 먼저 방문한 후 목표 B를 방문"

```matlab
% (◇[0,t1](goal_A)) ∧ (◇[t1,t2](goal_B))

% 목표 A
[r_A, F_A] = robust_eventually_simple(0, [0, t1], goal_A_handle);

% 목표 B (목표 A 이후)
[r_B, F_B] = robust_eventually_simple(0, [t1, t2], goal_B_handle);

% AND 조합
[r_seq, F_seq] = robust_and_simple(@() deal(r_A, F_A), @() deal(r_B, F_B));
```

### 패턴 5: 조건부 행동 (If-Then)
"조건 A가 만족되면 B를 해야 함"

```matlab
% A → B (implication)
% Equivalent to: ¬A ∨ B

not_A = @(k) deal(-A_handle(k));  % NOT A
impl = @(k) robust_or_simple(@() not_A(k), @() B_handle(k));

[r_impl, F_impl] = robust_always_simple(0, [0, N], impl);
```

## 문제 해결 체크리스트

### ✓ 최적화가 실패할 때 (Infeasible)

1. **STL 제약이 너무 엄격한가?**
   ```matlab
   % robustness margin을 줄여보기
   F = [F_stl, r_stl >= 0.01];  % 0.1 대신 0.01
   ```

2. **시간 구간이 적절한가?**
   ```matlab
   % 시간 구간을 넓히거나 좁히기
   [r, F] = robust_eventually(0, [0, 20], ...);  % [0,10] 대신 [0,20]
   ```

3. **시스템 제약이 STL과 충돌하는가?**
   ```matlab
   % 입력/상태 제약을 확인하고 조정
   ```

### ✓ 계산이 너무 느릴 때

1. **예측 구간 줄이기**
   ```matlab
   N = 10;  % 20 대신
   ```

2. **STL 구간 줄이기**
   ```matlab
   [r, F] = robust_always(0, [0, 5], ...);  % [0,10] 대신 [0,5]
   ```

3. **더 나은 solver 사용**
   ```matlab
   options = sdpsettings('solver', 'gurobi');  % 'mosek' 대신
   ```

### ✓ Robustness 값이 음수일 때

1. **어느 제약이 위반되었는지 확인**
   ```matlab
   fprintf('Safety robustness: %.4f\n', value(r_safety));
   fprintf('Reachability robustness: %.4f\n', value(r_reach));
   ```

2. **제약 완화 또는 수정**
   ```matlab
   % 목표값 조정
   % 시간 구간 조정
   ```

## 다음 단계

1. **논문 읽기**: `README.md`에 나열된 참고 논문들
2. **고급 예제**: `stl_mpc_example.m` 코드 분석
3. **자신의 문제에 적용**: 위의 패턴들을 조합하여 사용

## 도움말

- **YALMIP 문서**: https://yalmip.github.io/
- **Gurobi 문서**: https://www.gurobi.com/documentation/
- **STL 튜토리얼**: `test_stl_operators.m` 실행

좋은 연구 되세요! 🚀
