// 이동 한 번의 실행: 센서 샘플 → 추정 → 컨트롤러 → 감시기 → 바퀴 결과 (DOM 없음: 시뮬레이션에서 그대로 돌려 시험한다).
// 페이지(pages/motion.js)는 화면 · 전송 · 권한 같은 바깥 조건만 맡는다.

import { CircleController, ShuttleController } from './controllers.js';
import { PoseEstimator } from './estimator.js';
import { LapRecorder } from './metrics.js';
import { Supervisor } from './safety.js';

const MAX_STEP = 1.0;        // [s] 이웃한 샘플의 시간 간격이 이보다 길거나 0 이하면 시계가 건너뛴 것
const LINE_OVERRUN = 0.3;    // [m] 직선: 구간 양끝 밖으로 이만큼 넘으면 정지
const MAX_ROWS = 20000;      // CSV 로 보관하는 최대 행 수 (넘으면 오래된 것부터 버린다)
// 진행 감시: 명령은 나가는데 위치가 안 변하면 (odom 이 값만 새로 오며 얼었거나, 바퀴가 막혔거나, 사람을 밀고 있다) 정지.
// 엔코더 정지는 컨트롤러가 0.5 s 만에 토크를 끄지만, 값이 새로 오면서 얼어 있는 경우는 여기서만 잡힌다.
const PROGRESS_WINDOW = 3.0;        // [s]
const PROGRESS_MIN_SPEED = 0.03;    // [m/s] 이보다 느리게 명령한 구간은 판단하지 않는다 (구간 끝 감속 · 쉬는 구간)
const PROGRESS_RATIO = 0.3;         // 명령한 이동 거리의 이 비율보다 적게 움직이면 정지
const ZERO = { vx: 0, vy: 0, wz: 0 };

export class MotionRun {
	/**
	 * @param {{kind: 'circle'|'line', speed: number, useImu: boolean, tau: number, radius?: number, direction?: number,
	 *     laps?: number, distance?: number, rounds?: number}} plan
	 * @param {{t: number, odom: object, gyroZ: number}} first 시작 시점의 센서 샘플 (이 자세가 출발점).
	 */
	constructor(plan, first) {
		this.plan = plan;
		this.estimator = new PoseEstimator({ useImuYaw: plan.useImu, tau: plan.tau });
		this.estimator.reset(first);
		this.previousT = first.t;
		this.controller = plan.kind === 'circle'
			? new CircleController({ radius: plan.radius, speed: plan.speed, direction: plan.direction, laps: plan.laps })
			: new ShuttleController({ distance: plan.distance, speed: plan.speed, rounds: plan.rounds });
		this.controller.start(this.estimator.pose);
		this.recorder = plan.kind === 'circle' ? new LapRecorder(this.controller.center, plan.radius) : null;
		this.supervisor = new Supervisor();
		this.supervisor.arm(first.t);
		this.info = null;
		this.lapsDone = 0;
		this.rows = [];
		this.results = [];
		this.roundStats = { maxAlong: 0, maxLateral: 0 };
		this.isDone = false;
		this.lastPose = this.estimator.pose;
		this.lastCommand = ZERO;
		this.progress = [];
	}

	/**
	 * 새 센서 샘플 하나를 처리한다.
	 * @returns {{command: {vx, vy, wz}, stop: null | {reason: string, tone: string}}} stop 이 있으면 command 는 0.
	 */
	step(sample) {
		const dt = sample.t - this.previousT;
		if (!(dt > 0 && dt <= MAX_STEP)) return this.halt('센서 시각이 건너뛰거나 거꾸로 가서 멈췄습니다 (서버 시계가 바뀌었을 수 있습니다)');
		this.previousT = sample.t;
		const previousPose = this.lastPose;
		const pose = this.estimator.update(sample);
		this.lastPose = pose;
		const stall = this.watchProgress(sample.t, Math.hypot(this.lastCommand.vx, this.lastCommand.vy) * dt, Math.hypot(pose.x - previousPose.x, pose.y - previousPose.y));
		const result = this.controller.update(pose, dt);
		this.info = result.info;
		const isCircle = this.plan.kind === 'circle';
		const primaryError = isCircle ? this.info.radiusError : this.info.lateralError;
		const reason = this.supervisor.observe({
			kind: this.plan.kind, t: sample.t, pose, primaryError, headingError: this.info.headingError,
			odomYaw: this.estimator.odomYaw, gyroYaw: this.estimator.gyroYaw, radius: this.plan.radius,
		});
		if (this.rows.length >= MAX_ROWS) this.rows.splice(0, this.rows.length - MAX_ROWS + 1);
		this.rows.push({
			t: sample.t, odomX: sample.odom.x, odomY: sample.odom.y, odomYaw: sample.odom.yaw, gyroZ: sample.gyroZ,
			gyroYaw: this.estimator.gyroYaw, fusedX: pose.x, fusedY: pose.y, fusedYaw: pose.yaw,
			vx: result.vx, vy: result.vy, wz: result.wz, primaryError, headingError: this.info.headingError,
		});
		if (reason || stall) return this.halt(reason ?? stall);
		if (!isCircle && (this.info.alongTrack > this.plan.distance + LINE_OVERRUN || this.info.alongTrack < -LINE_OVERRUN)) {
			return this.halt(`왕복 범위를 ${LINE_OVERRUN} m 넘게 벗어나 멈췄습니다`);
		}
		this.record(pose);
		if (result.done) {
			this.isDone = true;
			const text = isCircle ? `${this.plan.laps} 바퀴를 돌았습니다` : `${this.plan.rounds} 회 왕복했습니다`;
			return { command: ZERO, stop: { reason: text, tone: 'ok' } };
		}
		this.lastCommand = { vx: result.vx, vy: result.vy, wz: result.wz };
		return { command: this.lastCommand, stop: null };
	}

	/** 센서 샘플이 오지 않은 채 시간이 흐른 경우 (타이머에서 부른다). 정지 이유 또는 null. */
	checkStale(serverNow) {
		return this.supervisor.checkStale(serverNow);
	}

	/** 멈추는 동안 받은 샘플로 자세만 갱신한다 (마지막 바퀴의 복귀 오차를 멈춘 자리에서 재려고). */
	settle(sample) {
		const dt = sample.t - this.previousT;
		if (!(dt > 0 && dt <= MAX_STEP)) return;
		this.previousT = sample.t;
		this.lastPose = this.estimator.update(sample);
		if (this.recorder && this.isDone) this.recorder.add(this.lastPose.x, this.lastPose.y);
	}

	/** 다 돌고 끝난 원이면 마지막 바퀴 결과를 더한다. 멈춘 뒤 한 번 부른다. */
	finish() {
		if (!this.isDone || this.plan.kind !== 'circle' || this.lapsDone >= this.plan.laps) return;
		this.lapsDone = this.plan.laps;
		this.results.push(this.lapLine(this.lapsDone, this.recorder.finishLap(this.lastPose, { x: 0, y: 0 })));
	}

	/** 최근 PROGRESS_WINDOW 동안 명령한 이동 거리와 실제(추정) 이동 거리를 비교한다. 정지 이유 또는 null. */
	watchProgress(t, commanded, travelled) {
		this.progress.push({ t, commanded, travelled });
		while (this.progress.length > 1 && t - this.progress[1].t >= PROGRESS_WINDOW) this.progress.shift();
		if (t - this.progress[0].t < PROGRESS_WINDOW) return null;
		const commandedSum = this.progress.reduce((sum, item) => sum + item.commanded, 0);
		const travelledSum = this.progress.reduce((sum, item) => sum + item.travelled, 0);
		if (commandedSum >= 0.8 * PROGRESS_MIN_SPEED * PROGRESS_WINDOW && travelledSum < PROGRESS_RATIO * commandedSum) {
			return '로봇이 명령대로 움직이지 않습니다 (바퀴가 막혔거나 odom 이 멈췄을 수 있습니다)';
		}
		return null;
	}

	halt(reason) {
		return { command: ZERO, stop: { reason, tone: 'danger' } };
	}

	lapLine(number, lap) {
		return `바퀴 ${number}: 반지름 평균 ${lap.meanRadius.toFixed(3)} m · 편차 ${lap.stdRadius.toFixed(3)} m · 복귀 오차 ${lap.closure.toFixed(3)} m`;
	}

	record(pose) {
		if (this.plan.kind === 'circle') {
			this.recorder.add(pose.x, pose.y);
			if (this.info.lap > this.lapsDone) {
				this.lapsDone = this.info.lap;
				this.results.push(this.lapLine(this.lapsDone, this.recorder.finishLap(pose, { x: 0, y: 0 })));
			}
			return;
		}
		this.roundStats.maxAlong = Math.max(this.roundStats.maxAlong, this.info.alongTrack);
		this.roundStats.maxLateral = Math.max(this.roundStats.maxLateral, Math.abs(this.info.lateralError));
		if (this.info.round > this.lapsDone) {
			this.lapsDone = this.info.round;
			this.results.push(
				`왕복 ${this.lapsDone}: 최대 거리 ${this.roundStats.maxAlong.toFixed(3)} m · 가로 최대 ${this.roundStats.maxLateral.toFixed(3)} m · 끝 위치 ${this.info.alongTrack.toFixed(3)} m`,
			);
			this.roundStats = { maxAlong: 0, maxLateral: 0 };
		}
	}
}
