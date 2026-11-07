# "이동 설계" 페이지 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 대시보드에 사이드바 "이동 설계" 페이지를 더해, 브라우저가 `odom` · IMU 로 계산하며 직선 왕복과 원을 그리게 한다.

**Architecture:** 순수 로직 모듈(`web/js/motion/*.js`, DOM 없음)을 `node --test` 로 시험하고, 얇은 페이지(`web/js/pages/motion.js`)가 `status` 를 받아 컨트롤러를 돌려 0.1 s 마다 `teleop` 을 보낸다. 로봇 쪽 변경은 없다.

**Tech Stack:** 브라우저 ES 모듈 (빌드 없음), Node 22 `node:test`, 기존 대시보드 WS 프로토콜.

**Spec:** `docs/superpowers/specs/2026-10-25-motion-planner-page-design.md`

## Global Constraints

- 들여쓰기는 탭, 주석 · 문구는 한국어, 이름은 camelCase (기존 `web/js` 규칙). 라이브러리 · 빌드 도구를 추가하지 않는다.
- `web/` 안의 새 파일은 브라우저가 서빙한다: 시험 파일은 `tests/js/` 에 둔다 (`web/` 밖).
- 기본 속도 원 0.12 m/s · 직선 0.10 m/s. 입력 속도는 바퀴 상한(4.601942363656923 rad/s)의 80 % 안으로 막는다.
- 상태 수신은 약 0.27 s 마다(4 Hz), 서버는 `teleop` 이 0.3 s 끊기면 0, 2 s 끊기면 권한 회수 · 토크 끔. `teleop` 은 0.1 s 마다 보낸다.
- IMU 방향은 `status.imu.gyro[2]` 를 적분한다 (`imu.yaw` 는 쓰지 않는다: 자세 추정이 기울어 있다).
- 이 페이지가 열려 있는 동안 `createTeleop` 을 올리지 않는다.
- 커밋은 사용자가 요청할 때만 한다. 파일은 작업 트리에 남긴다. 로봇에 올리기는 Task 5 에서만, 정적 파일 복사만 한다.

## Review Focus

1. `status` 가 3 s 끊긴다 (WS 끊김): 계속 움직이지 않고 정지한다 → Task 3 `Supervisor` 시험 + Task 4 페이지가 0 을 보냄.
2. 창이 가려진다 (`document.hidden`): 정지한다 → Task 4.
3. `status.imu` 가 없거나 `null`: 시작하지 않는다 → Task 4 시작 조건.
4. 요청 속도가 상한 밖(반지름 0.2 m 에서 0.2 m/s 등): 막거나 거부한다, 상한을 넘겨 보내지 않는다 → Task 3 `maxCircleSpeed`, `toTeleop` null.
5. 자이로 부호가 반대(장착 방향 다른 로봇): 5° 돌았을 때 정지한다 → Task 3.
6. 주행 중 `odom` 이 초기화되어 위치가 튄다: 한 샘플 0.3 m 넘는 점프에 정지한다 → Task 3.

---

### Task 1: 추정기 (방향 · 위치)

**Files:**
- Create: `web/js/motion/geometry.js`, `web/js/motion/estimator.js`
- Test: `tests/js/estimator.test.js`

**Interfaces:**
- Produces:
  - `geometry.js`: `wrapAngle(angle: number): number` (−π, π], `rotate(x, y, angle): [number, number]`, `TAU`.
  - `estimator.js`: `class PoseEstimator { constructor({ useImuYaw = true, tau = 15 } = {}); reset(sample): void; update(sample): Pose; get pose(): Pose; get odomYaw(): number; get gyroYaw(): number }`
    with `Sample = { t: number /*초*/, odom: { x, y, yaw /*rad*/ }, gyroZ: number /*rad/s*/ }`, `Pose = { x, y, yaw }`.
    `reset` 이 기준이다: 그 뒤 `pose` 는 (0, 0, 0) 에서 시작하고 `odomYaw`, `gyroYaw` 도 0 에서 시작한다.

- [ ] **Step 1: 실패하는 시험 작성** (`tests/js/estimator.test.js`)
  - `wrapAngle` : `wrapAngle(3*Math.PI)` ≈ π (또는 −π 와 같은 각), `wrapAngle(-0.5)` = −0.5.
  - `odom 만 쓰면 odom 자세를 출발점 기준으로 돌려 준다`: `useImuYaw:false`, reset 에 odom (1, 2, 0.5 rad), 이어서 odom (1.1, 2.0, 0.5) → `pose.x` ≈ 0.1·cos(0.5)·… 가 아니라 출발 yaw 의 반대로 돌린 값: 기대 `[Δx cos(−0.5) − Δy sin(−0.5), …]` 를 테스트 안에서 계산해 비교, `pose.yaw` = 0.
  - `제자리 회전: 자이로 적분이 odom yaw 와 같으면 융합 yaw 도 같다`: 0.25 s 간격 40 샘플, `gyroZ = 0.4`, odom yaw 가 `0.4·t` → `pose.yaw` ≈ 4.0 rad (±0.02), `gyroYaw` ≈ `odomYaw`.
  - `자이로 바이어스는 odom 쪽으로 천천히 끌려간다`: 정지(odom 그대로), `gyroZ = 0.01` 로 60 s (샘플 0.25 s) → `gyroYaw` ≥ 0.5(바이어스 적분만큼 커짐), `pose.yaw` < 0.2 (보완 필터가 눌러줌), `pose.yaw` > 0.
  - `원 위를 돌면 위치가 원 위에 있다`: 반지름 0.5, 속도 0.12 의 odom(자세가 원 위)과 gyro = 0.24 를 0.27 s 간격으로 한 바퀴(약 26 s) 먹임 → 마지막 `pose` 와 시작점 사이 거리 < 0.03.
  - `샘플 간격이 1 s 를 넘으면 그 구간은 적분하지 않는다`: 5 s 건너뛴 샘플 → `gyroYaw` 가 `gyroZ·5` 만큼 늘지 않는다.
- [ ] **Step 2: 실행해 실패 확인** — `cd /home/khw/workspace/lekiwi-repo/lekiwi_web_operator && node --test tests/js/estimator.test.js` → FAIL (모듈 없음). ESM 감지가 안 되면 `tests/js/package.json` 에 `{"type":"module"}` 을 넣는다.
- [ ] **Step 3: 구현**
  - 한 번의 경로: 매 `update` 에서 `dt = t − tPrev` (0 < dt ≤ 1 일 때만 적분), `bodyStep = rotate(Δodom.x, Δodom.y, −prevOdomYaw)`,
    `yaw ← yaw + 0.5·(gzPrev + gz)·dt + (dt/tau)·wrapAngle(odomYawRel − yaw)`, `useImuYaw=false` 면 `yaw = odomYawRel`,
    `pos += rotate(bodyStep, (yawPrev + yaw)/2)`. `gyroYaw` 는 `0.5·(gzPrev+gz)·dt` 를 보완 없이 따로 쌓은 값.
- [ ] **Step 4: 실행해 통과 확인** — 같은 명령, 전부 PASS.

---

### Task 2: 컨트롤러 · 시뮬레이션 · 지표

**Files:**
- Create: `web/js/motion/controllers.js`, `web/js/motion/metrics.js`, `tests/js/sim.js`
- Test: `tests/js/controllers.test.js`, `tests/js/metrics.test.js`

**Interfaces:**
- Consumes: Task 1 `PoseEstimator`, `wrapAngle`, `rotate`, `TAU`.
- Produces:
  - `controllers.js`:
    `export const GAINS = { heading, cross, crossMax, lateral, approach, minSpeed, arrive, rampAccel }` (초기값: 1.5, 2.0, 0.5, 1.0, 1.5, 0.02, 0.01, 0.3. 시뮬레이션으로 맞춘다).
    `class CircleController { constructor({ radius, speed, direction = 1 /*1 왼쪽, −1 오른쪽*/, laps = 1 /*Infinity 가능*/ }); start(pose): void; update(pose, dt): Command; get center(): {x, y} }`
    `class ShuttleController { constructor({ distance, speed, rounds = 1 /*Infinity 가능*/ }); start(pose): void; update(pose, dt): Command }`
    `Command = { vx, vy, wz, done: boolean, info }`; 원 `info = { radiusError, headingError, lap /*끝낸 바퀴 수*/, angle }`,
    직선 `info = { alongTrack, lateralError, headingError, leg: 'forward'|'dwell'|'back', round }`.
  - `metrics.js`: `class LapRecorder { constructor(center, radius); add(x, y): void; finishLap(pose, startPose): { meanRadius, stdRadius, closure } }`, `toCsv(rows: object[], columns: string[]): string`.
  - `tests/js/sim.js` (시험 전용): `simulate({ plant, controller, estimatorOptions, seconds }): { truth: {t,x,y,yaw}[], done: boolean, stopped: string|null }`,
    `plant = { speedScale, odomScale, yawDrift /*rad/s, odom 이 못 보는 실제 회전*/, gyroBias /*rad/s*/, latency /*s, 기본 0.3*/, seed }`.
    서버 모델: `teleop` 0.1 s, 명령 0.3 s 끊기면 0, 램프 선 0.4 m/s² · 회전 2.4 rad/s², 상태 0.27 s 주기 · `latency` 지연,
    `odom` = 참 이동 × `odomScale`(+ 잡음 σ 1 mm), `gyro` = 참 wz + `gyroBias`.

- [ ] **Step 1: 실패하는 시험 작성**
  - `controllers.test.js`:
    - `원: 이상적인 로봇이 한 바퀴 돌고 done`: speedScale 1, 나머지 0, laps 1 → `done` true, 참 경로의 평균 반지름 0.5 ± 0.01, 복귀 오차 < 0.03.
    - `원: 로봇 4대(plant 변형)가 같은 원을 그린다`: 변형 = `{speedScale, odomScale, yawDrift}` = `{0.85, 0.97, +0.0026}`, `{1.0, 1.0, 0}`, `{1.15, 1.03, −0.0026}`, `{0.95, 0.986, +0.001}`, `gyroBias 0.002`, 지연 0.3, laps 4, 2~4바퀴째 참 경로: 각 평균 반지름 0.5 ± 0.03, 네 로봇의 평균 반지름 최대−최소 ≤ 0.03, 복귀 오차 ≤ 0.05.
    - `원: 오른쪽(direction −1)도 같다`: 위 이상적인 로봇으로 반지름 0.5 ± 0.01.
    - `원: 중심은 출발 자세 옆 r 지점이다`: pose (1, 2, π/2) 에서 start → `center` ≈ (1 − 0.5, 2) (왼쪽).
    - `원: 속도는 램프로 올라간다`: 첫 update 의 `vx` ≤ `GAINS.rampAccel·dt + 1e-9`, 그리고 `wz/vx` = 1/r.
    - `직선: 1 m 왕복 2회`: 이상적인 로봇, rounds 2 → done, 끝점이 앞쪽 1.00 ± 0.03, 돌아온 점이 시작 ± 0.03, 가로 오차 최대 ≤ 0.05.
    - `직선: 옆으로 밀려도 옆걸음으로 돌아온다`: 매 초 참 자세를 가로로 +0.01 m 씩 밀면 마지막 가로 오차 ≤ 0.05.
    - `직선: 구간 끝에서 쉰다`: `leg` 가 'forward' → 'dwell'(약 1 s) → 'back' 순서로 바뀐다.
    - `플랜트 변형 4대에서 직선 거리 오차`: `odomScale 0.97/1.03` 에서 참 거리 1 ± 0.04.
  - `metrics.test.js`: 반지름 0.5 원 위의 점 100개 → `meanRadius` 0.5 ± 1e-9, `stdRadius` ≈ 0, `closure` = 두 자세 사이 거리; `toCsv` 가 열 순서대로 머리줄 + 행을 만들고 소수 6자리로 쓴다.
- [ ] **Step 2: 실행해 실패 확인** — `node --test tests/js/controllers.test.js tests/js/metrics.test.js` → FAIL.
- [ ] **Step 3: 구현** (`controllers.js`, `metrics.js`, `sim.js`)
  - 원: 중심 `C = p0 + dir·r·(−sin θ0, cos θ0)`. 매 update 에서 `ρ = |p − C|`, `radiusError = ρ − r`(+바깥), `φ = atan2(p − C)`,
    `angle += dir·wrapAngle(φ − φPrev)`, 접선 목표 `θd = φ + dir·π/2`, 안쪽 기울기 `tilt = dir·clamp(atan(GAINS.cross·radiusError), ±GAINS.crossMax)`,
    `headingError = wrapAngle(θd + tilt − θ)`. `v` 는 `GAINS.rampAccel·dt` 씩 `speed` 까지 올린다. `vx = v`, `vy = 0`, `wz = dir·v/r + GAINS.heading·headingError`.
    `laps` 바퀴(= `angle ≥ 2π·laps`) 에 이르면 0 과 `done`.
  - 직선: 기준 방향 `θref` = start 의 yaw, 단위 `u = (cos θref, sin θref)`, 법선 `n = (−sin θref, cos θref)`.
    `alongTrack = (p − p0)·u`, `lateralError = (p − p0)·n`, `headingError = wrapAngle(θref − θ)`.
    앞 구간: 목표 `D`, 뒤 구간: 목표 0. `remaining = 목표 − alongTrack`; `v = sign·clamp(GAINS.approach·|remaining|, GAINS.minSpeed, speed)`; `|remaining| < GAINS.arrive` 이면 구간 종료 → 1 s 'dwell' (모두 0) → 반대 구간.
    몸체 기준 `vx = v`, `vy = clamp(−GAINS.lateral·lateralError, ±0.05)` (θ ≈ θref 이므로 몸체 vy ≈ 법선 방향), `wz = GAINS.heading·headingError`. 한 왕복(앞 + 뒤)이 `round` 1.
  - `sim.js`: 위 서버 모델을 0.01 s 스텝으로 돌린다. 컨트롤러는 새 상태 샘플이 도착할 때만 `update` 하고 그 명령을 0.1 s 마다 `teleop` 처럼 유지한다. 난수는 `seed` 로 고정(예: mulberry32).
  - 시뮬레이션이 목표를 못 맞추면 `GAINS` 를 조정한다. 그래도 안 되면 목표를 말없이 느슨하게 하지 말고 수치와 함께 보고한다.
- [ ] **Step 4: 실행해 통과 확인** — 같은 명령, 전부 PASS. 통과한 이득 값을 `GAINS` 와 설계 문서의 "빌더에게 맡기는 것" 아래에 숫자로 적는다.

---

### Task 3: 속도 변환 · 감시기

**Files:**
- Create: `web/js/motion/command.js`, `web/js/motion/safety.js`
- Test: `tests/js/command.test.js`, `tests/js/safety.test.js`

**Interfaces:**
- Consumes: Task 1 `wrapAngle`.
- Produces:
  - `command.js`: `MAX_WHEEL_SPEED = 4.601942363656923`, `WHEEL_MARGIN = 0.8`,
    `wheelPeak(vx, vy, wz): number` (가장 빠른 바퀴 속도 rad/s, `omni_kinematics.cpp` 와 같은 식),
    `maxCircleSpeed(radius): number`, `maxLineSpeed(): number`,
    `levelFor(levels: number[][], speed, turn): number | null`,
    `toTeleop({ vx, vy, wz }, levels): { x, y, yaw, level } | null`.
  - `safety.js`: `LIMITS = { staleSeconds: 1.0, radiusError: 0.30, lateralError: 0.20, headingError: Math.PI/3, runawaySeconds: 1.0, rateMismatch: 0.15, rateWindow: 2.0, signCheckAngle: 5*Math.PI/180, jump: 0.3 }`,
    `class Supervisor { constructor(limits = LIMITS); observe({ t, pose, primaryError, headingError, odomYaw, gyroYaw }): string | null; checkStale(t): string | null }` — 반환값은 정지 이유(한국어) 또는 `null`.
    `primaryError` 는 원이면 `|radiusError|`, 직선이면 `|lateralError|`. `limits.radiusError` 와 `lateralError` 중 어느 쪽을 쓸지는 `observe` 의 `kind: 'circle'|'line'` 인자로 정한다 (이 인자를 `observe` 에 추가한다).

- [ ] **Step 1: 실패하는 시험 작성**
  - `command.test.js`: `wheelPeak(1, 0, 2)` ≈ 23.3045 (±0.01, 반지름 0.5 의 단위 속도); `maxCircleSpeed(0.5)` ≈ 0.158 (±0.002) 이고 `0.8·MAX_WHEEL_SPEED / wheelPeak(1,0,2)` 와 같다;
    `maxCircleSpeed(0.2)` < `maxCircleSpeed(0.5)`; `maxLineSpeed()` ≤ 0.23;
    레벨표 `[[0.01,0.06],[0.05,0.29],[0.12,0.70],[0.23,1.53]]` 에서 `levelFor(levels, 0.12, 0.24)` = 2, `levelFor(levels, 0.3, 0)` = null;
    `toTeleop({vx:0.12, vy:0, wz:0.24}, levels)` = `{ level: 2, x: 1, y: 0, yaw: 0.24/0.70 }` (±1e-9); `toTeleop({vx:0,vy:0,wz:0}, levels)` = `{x:0,y:0,yaw:0,level:0}`; 상한 밖이면 `null`.
  - `safety.test.js`: 정상 샘플 → `null`; 샘플 1.2 s 없이 `checkStale` → 이유 문자열, 0.5 s 면 `null`;
    반지름 오차 0.35 가 1.2 s 이어지면 정지, 0.5 s 만 이어지면 `null`; 방향 오차 70° 가 1.2 s 이어지면 정지;
    한 샘플에 위치가 0.4 m 튀면 즉시 정지; `odomYaw` 가 +6° 쌓였는데 `gyroYaw` 가 −5° 면 '부호' 이유로 정지, +5.5° 면 `null`;
    2 s 창에서 회전 속도 차이가 0.2 rad/s 면 정지, 0.05 rad/s 면 `null`.
- [ ] **Step 2: 실행해 실패 확인** — `node --test tests/js/command.test.js tests/js/safety.test.js` → FAIL.
- [ ] **Step 3: 구현.** `wheelPeak`: 상수 `WHEEL_RADIUS 0.05`, `BASE_RADIUS 0.13647`, `KINEMATICS_X 0.007587843843`, `KINEMATICS_Y −0.010776931094`, `KINEMATICS_YAW 2.094379865161`
  (`lekiwi_base_controller` 설정 값을 복사했다는 주석), 식은 `omni_kinematics.cpp` 의 `ToKinematicsFrame` + `ToRawWheelSpeeds`. `maxCircleSpeed` 는
  `min(0.23, 1.53·radius, WHEEL_MARGIN·MAX_WHEEL_SPEED / wheelPeak(1, 0, 1/radius))`. `Supervisor` 는 지속 시간 · 창 · 부호 검사 상태를 안에 두고, 첫 `observe` 의 `odomYaw`/`gyroYaw` 를 기준으로 삼는다.
- [ ] **Step 4: 실행해 통과 확인** — 같은 명령, 전부 PASS.

---

### Task 4: 페이지 · 연결

**Files:**
- Create: `web/js/pages/motion.js`
- Modify: `web/js/app.js` (import · `PAGES` 에 `motion` 추가), `web/index.html` (사이드바 링크 `#/motion` "이동 설계", 휴대폰 탭바 줄임말), `web/css/app.css` (필요한 만큼)
- Test: 브라우저 점검 (Task 5), 그리고 `node --check` 문법 검사.

**Interfaces:**
- Consumes: Task 1~3 의 모든 export, 대시보드 페이지 계약 (`title`, `channels = ['odom','imu']`, `mount(el, ctx)`, `unmount()`, `ctx.on('update')`, `ctx.status`, `ctx.isMine()`, `ctx.blockReason()`, `ctx.acquire()`, `ctx.release()`, `ctx.confirm()`, `ctx.conn.send()`, `ctx.conn.post()`).
- Produces: 라우트 `#/motion`.

- [ ] **Step 1: 페이지 작성** (`motion.js`)
  - 화면: 동작 선택(직선 왕복 / 원), 매개변수(거리 m 1.0 · 반지름 m 0.5 · 속도 m/s · 방향 왼쪽/오른쪽 · 횟수 '1회'/'무한'), 'IMU 방향 사용' 체크(기본 켜짐),
    조작 권한 받기/놓기 · 조작 켜기/끄기 버튼(`teleop.js` 와 같은 확인 창 문구, `POST /api/motor/power`), 시작 · 정지, 진행 · 오차 표시, 바퀴/왕복 결과 표, 'CSV 저장'.
    속도 입력은 `maxCircleSpeed`/`maxLineSpeed` 로 막고 상한을 화면에 보인다. 동작을 바꾸면 기본 속도(원 0.12, 직선 0.10)로 되돌린다.
  - 시작 조건(하나라도 아니면 이유를 글로 보이고 시작하지 않는다): `ctx.isMine()`, `status.motor.ready === true`, `status.estop.is_latched !== true`, `status.odom` 과 `status.imu` 가 있음,
    `toTeleop` 이 `null` 이 아님. 통과하면 3 s 카운트다운(정지 버튼으로 취소), 그다음 추정기 `reset` + 컨트롤러 `start`.
  - 실행 중: `ctx.on('update', …)` 에서 새 샘플(`status.time` 이 달라졌을 때)로 추정기 `update` → 컨트롤러 `update` → `Supervisor.observe`. 0.1 s 타이머가 마지막 명령을 `{type:'teleop', x, y, yaw, level, pad:'drive'}` 로 보내고 같은 타이머에서 `Supervisor.checkStale`.
    정지 이유가 생기면: 이유를 표시, 0 을 1 s 동안 0.1 s 마다 보내고 멈춘다.
  - 정지 조건: `Supervisor` 이유, `document.hidden`, `unmount`, 권한 잃음, `motor.ready` 가 false, 비상 정지, 정지 버튼.
  - CSV 열: `t, odomX, odomY, odomYaw, gyroZ, gyroYaw, fusedX, fusedY, fusedYaw, vx, vy, wz, primaryError, headingError`. 메모리 20000 행까지만 보관한다.
- [ ] **Step 2: 연결.** `app.js` 에 `import * as motion from './pages/motion.js'` 와 `PAGES` 항목. `index.html` 사이드바에 `<a href="#/motion" data-route="motion">` 링크(아이콘 SVG, `이동 설계`, 탭바용 `nav-short` 줄임말 `이동`).
  휴대폰 탭바 CSS 가 8칸 고정이면 9칸에 맞게 고친다.
- [ ] **Step 3: 문법 검사** — 모든 `web/js/**/*.js` 에 `node --check`, `node --test "tests/js/*.test.js"` 전체 PASS (글롭으로 준다).

---

### Task 5: 04 에 올려 점검

**Files:**
- Copy (읽기 전용 확인 뒤): 새 · 수정 `web/` 파일 → 04 `~/lekiwi_web_operator/web/`

- [ ] **Step 1: 백업과 복사.** `ssh roboseasy@10.42.0.117 'cp -a ~/lekiwi_web_operator/web ~/lekiwi_web_operator/web.bak-20261025'` 후, 바뀐 파일만 `scp`/`rsync`. 서비스 재시작과 bringup 은 건드리지 않는다.
- [ ] **Step 2: 브라우저 점검 (움직이지 않음).** `http://10.42.0.117:8080/#/motion` 을 열어 콘솔 오류 없음, 사이드바 링크, 화면이 그려짐, 조작 권한이 없으면 시작이 거절되고 이유가 보임을 확인한다.
- [ ] **Step 3: 사용자 입회 실물 시험.** 사용자에게 알린 뒤에만: 직선 0.10 m/s 1 m 1왕복 → 원 0.12 m/s 반지름 0.5 m 2바퀴. CSV 와 화면 결과를 읽고 바닥 표시로 비교한다.
