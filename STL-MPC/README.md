# STL-MPC 인코딩 구현

Signal Temporal Logic (STL)을 사용한 Model Predictive Control 구현입니다.

## 참고 논문

**Raman, V., Donzé, A., Maasoumy, M., Murray, R. M., Sangiovanni-Vincentelli, A., & Seshia, S. A. (2014)**
*"Model Predictive Control with Signal Temporal Logic Specifications"*
IEEE Conference on Decision and Control (CDC), 2014

## 주요 파일

- `stl_encoding.m`: STL 연산자들의 robust semantics 인코딩
- `stl_mpc_example.m`: STL-MPC를 사용한 완전한 예제

## 필요 도구

1. **MATLAB** (R2018b 이상 권장)
2. **YALMIP** - 최적화 모델링 툴박스
   - 다운로드: https://yalmip.github.io/download/
3. **MILP Solver** (다음 중 하나):
   - Gurobi (권장, 학술 라이선스 무료)
   - CPLEX (IBM, 학술 라이선스 무료)
   - MOSEK (학술 라이선스 무료)

## 설치 방법

### 1. YALMIP 설치

```matlab
% YALMIP을 다운로드한 후
addpath(genpath('C:/path/to/YALMIP-master'))
savepath
```

### 2. Solver 설치

#### Gurobi (권장)
1. https://www.gurobi.com/academia/academic-program-and-licenses/ 에서 학술 라이선스 신청
2. Gurobi 설치 후 MATLAB 경로에 추가:
```matlab
addpath('C:/gurobi1100/win64/matlab')
savepath
```

## STL Robust Semantics

STL formula의 robustness degree (ρ)는 formula의 만족도를 정량적으로 측정합니다:
- **ρ > 0**: Formula 만족
- **ρ ≤ 0**: Formula 불만족
- **ρ 값이 클수록**: 더 robust하게 만족

### 구현된 연산자

#### 1. Always (□[a,b]φ)
시간 구간 [a,b] 동안 항상 φ가 만족되어야 함

**Robust semantics**:
```
ρ(□[a,b]φ, t) = min_{t'∈[t+a, t+b]} ρ(φ, t')
```

**사용 예**:
```matlab
% □[0,10](x ≤ 5): 0부터 10까지 x가 5 이하를 유지
predicate = @(k) atomic_predicate(x(:,k+1), [1;0], 5);
[r, F] = robust_always(0, [0, 10], predicate);
```

#### 2. Eventually (◇[a,b]φ)
시간 구간 [a,b] 동안 적어도 한 번 φ가 만족되어야 함

**Robust semantics**:
```
ρ(◇[a,b]φ, t) = max_{t'∈[t+a, t+b]} ρ(φ, t')
```

**사용 예**:
```matlab
% ◇[5,15](x ≥ 8): 5부터 15 사이에 x가 8 이상이 되어야 함
predicate = @(k) atomic_predicate(x(:,k+1), [-1;0], -8);
[r, F] = robust_eventually(0, [5, 15], predicate);
```

#### 3. And (φ1 ∧ φ2)
두 formula가 모두 만족되어야 함

**Robust semantics**:
```
ρ(φ1 ∧ φ2) = min(ρ(φ1), ρ(φ2))
```

**사용 예**:
```matlab
% (x1 ≤ 5) ∧ (x2 ≤ 5): 두 조건 모두 만족
[r, F] = robust_and(handle_A, handle_B);
```

#### 4. Or (φ1 ∨ φ2)
적어도 하나의 formula가 만족되어야 함

**Robust semantics**:
```
ρ(φ1 ∨ φ2) = max(ρ(φ1), ρ(φ2))
```

#### 5. Not (¬φ)
Formula가 만족되지 않아야 함

**Robust semantics**:
```
ρ(¬φ) = -ρ(φ)
```

#### 6. Until (φ1 U[a,b] φ2)
φ2가 [a,b] 내에 만족될 때까지 φ1이 유지되어야 함

**Robust semantics**:
```
ρ(φ1 U[a,b] φ2) = max_{t∈[a,b]} min(ρ(φ2,t), min_{s∈[0,t)} ρ(φ1,s))
```

## 사용 예제

### 예제 1: 안전 제약 (Safety)

```matlab
% □[0,T](x1 ≤ 5 ∧ x2 ≤ 5)
% "항상 경계 내에 머물러야 함"

T = 10;
x = sdpvar(2, T+1);

predicate_x1 = @(k) atomic_predicate(x(:,k+1), [1;0], 5);
predicate_x2 = @(k) atomic_predicate(x(:,k+1), [0;1], 5);
and_handle = @(k) robust_and(@() predicate_x1(k), @() predicate_x2(k));

[r_stl, F_stl] = robust_always(0, [0, T], and_handle);
F_stl = [F_stl, r_stl >= 0];  % STL 만족 조건
```

### 예제 2: 도달-회피 (Reach-Avoid)

```matlab
% ◇[0,T1](goal) ∧ □[0,T2](avoid)
% "목표에 도달하면서 장애물을 회피"

T1 = 10;
T2 = 10;
x = sdpvar(2, T2+1);

% 목표: x1 ≥ 8 ∧ x2 ≥ 8
goal_x1 = @(k) atomic_predicate(x(:,k+1), [-1;0], -8);
goal_x2 = @(k) atomic_predicate(x(:,k+1), [0;-1], -8);
goal_handle = @(k) robust_and(@() goal_x1(k), @() goal_x2(k));

[r_reach, F_reach] = robust_eventually(0, [0, T1], goal_handle);

% 장애물 회피: ¬(obstacle)
% obstacle: x1 ∈ [4,6] ∧ x2 ∈ [4,6]
obs_handle = @(k) ...; % 장애물 정의
avoid_handle = @(k) robust_not(@() obs_handle(k));

[r_avoid, F_avoid] = robust_always(0, [0, T2], avoid_handle);

% 결합
[r_stl, F_stl] = robust_and(@() deal(r_reach, F_reach),
                            @() deal(r_avoid, F_avoid));
```

### 예제 3: 완전한 MPC 문제

`stl_mpc_example.m` 파일을 참조하세요. 이 파일은:
- 이중 적분기 시스템
- 복합 STL 제약조건
- YALMIP을 사용한 MILP 인코딩
- Gurobi solver를 사용한 최적화
- 결과 시각화

를 포함한 완전한 예제입니다.

## 실행 방법

### 1. 경로 설정

```matlab
% YALMIP 경로 추가
addpath(genpath('C:/path/to/YALMIP-master'))

% Solver 경로 추가 (Gurobi 예시)
addpath('C:/gurobi1100/win64/matlab')

savepath
```

### 2. 예제 실행

```matlab
% STL-MPC 예제 실행
stl_mpc_example
```

## MILP 인코딩

STL formula를 MILP 제약조건으로 변환하는 핵심 아이디어:

1. **min/max 연산**: YALMIP이 자동으로 binary variables와 big-M constraints로 변환
2. **시간 전개**: 각 시간 단계에서 robustness 변수 생성
3. **재귀적 인코딩**: 복잡한 formula를 하위 formula들의 조합으로 분해

### YALMIP의 min/max 인코딩

```matlab
% YALMIP에서 자동 변환
r = min([r1, r2, r3]);

% 내부적으로 다음과 같이 변환됨:
% r <= r1, r <= r2, r <= r3
% binary variables를 사용하여 실제 최소값 선택
```

## 성능 최적화 팁

1. **시간 구간 최소화**: STL formula의 시간 구간을 필요한 만큼만 설정
2. **적절한 solver 선택**: Gurobi > CPLEX > MOSEK 순으로 권장
3. **Warm start**: 이전 MPC 해를 초기값으로 사용
4. **Robustness margin**: 너무 큰 margin(ε)은 feasibility 문제 유발 가능

## 문제 해결

### "No solver found" 에러
```matlab
% Solver가 제대로 설치되었는지 확인
yalmiptest
```

### Infeasible solution
- STL 제약조건이 너무 엄격한지 확인
- Robustness margin (ε)을 줄여보기
- 시간 구간을 조정해보기

### 느린 계산 속도
- 예측 구간(N)을 줄이기
- STL formula를 단순화하기
- 더 강력한 solver 사용 (Gurobi 권장)

## 추가 자료

1. **YALMIP Tutorial**: https://yalmip.github.io/tutorials/
2. **STL 소개**: Donzé, A., & Maler, O. (2010). "Robust satisfaction of temporal logic over real-valued signals"
3. **MPC 기초**: Rawlings, J. B., & Mayne, D. Q. (2009). "Model Predictive Control: Theory and Design"

## 라이선스

MIT License

## 참고사항

이 코드는 교육 및 연구 목적으로 작성되었습니다. 실제 시스템에 적용하기 전에 충분한 검증이 필요합니다.
