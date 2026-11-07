// 이동 설계 컨트롤러: 원과 직선 왕복. 추정기 자세(출발점 기준 x, y, yaw)를 받아 (vx, vy, wz) 를 낸다.
// DOM 없음. 이득 값은 tests/js/controllers.test.js 의 시뮬레이션(지연 0.3 s, 4 Hz 샘플링)으로 맞췄다.

import { TAU, clamp, wrapAngle } from './geometry.js';

export const GAINS = {
	heading: 1.0,      // [1/s] 방향 오차 → wz
	cross: 2.0,        // [1/m] 반지름 오차 → 접선 기울기 atan(cross·오차)
	crossMax: 0.5,     // [rad] 기울기 상한
	lateral: 1.0,      // [1/s] 가로 오차 → vy (직선)
	approach: 1.0,     // [1/s] 남은 거리 → 속도 (직선 끝에서 줄인다)
	lookahead: 0.6,    // [s] 센서 · 명령 지연만큼 앞을 내다본다 (끝 맞추기)
	minSpeed: 0.02,    // [m/s]
	arrive: 0.01,      // [m] 이 안이면 닿았다
	rampAccel: 0.3,    // [m/s²] 컨트롤러가 속도를 올리는 기울기
};
const WZ_LIMIT = 1.0;     // [rad/s]
const VY_LIMIT = 0.05;    // [m/s]
const DWELL = 1.0;        // [s] 직선 구간 끝에서 쉬는 시간

const ZERO = { vx: 0, vy: 0, wz: 0 };

export class CircleController {
	/**
	 * @param {{radius: number, speed: number, direction?: number, laps?: number}} options
	 *     direction 1 = 왼쪽(반시계), −1 = 오른쪽. laps 는 Infinity 가능.
	 */
	constructor({ radius, speed, direction = 1, laps = 1 }) {
		this.radius = radius;
		this.speed = speed;
		this.direction = direction;
		this.laps = laps;
		this.center = { x: 0, y: 0 };
	}

	start(pose) {
		this.center = {
			x: pose.x - this.direction * this.radius * Math.sin(pose.yaw),
			y: pose.y + this.direction * this.radius * Math.cos(pose.yaw),
		};
		this.previousPhi = Math.atan2(pose.y - this.center.y, pose.x - this.center.x);
		this.angle = 0;
		this.v = 0;
	}

	update(pose, dt) {
		const dx = pose.x - this.center.x;
		const dy = pose.y - this.center.y;
		const phi = Math.atan2(dy, dx);
		this.angle += this.direction * wrapAngle(phi - this.previousPhi);
		this.previousPhi = phi;
		const radiusError = Math.hypot(dx, dy) - this.radius;   // + 바깥
		const lap = Math.min(Math.max(Math.floor(this.angle / TAU), 0), this.laps);
		const tilt = this.direction * clamp(Math.atan(GAINS.cross * radiusError), GAINS.crossMax);
		const headingError = wrapAngle(phi + (this.direction * Math.PI) / 2 + tilt - pose.yaw);
		const info = { radiusError, headingError, lap, angle: this.angle };
		let wanted = this.speed;
		if (Number.isFinite(this.laps)) {
			// 끝 맞추기: 센서 · 명령 지연 동안 더 가는 거리를 미리 빼고(서버는 0 명령에서 램프 없이 바로 선다),
			// 마지막 구간에서는 직선처럼 속도를 줄인다 (4 Hz 판단 단위에서 생기는 정지 위치 오차를 속도만큼 줄인다)
			const remaining = (TAU * this.laps - this.angle) * this.radius - this.v * GAINS.lookahead;
			if (remaining < GAINS.arrive) return { ...ZERO, done: true, info };
			wanted = Math.max(Math.min(this.speed, GAINS.approach * remaining), GAINS.minSpeed);
		}
		this.v = Math.min(wanted, this.v + GAINS.rampAccel * dt);
		const wz = clamp(this.direction * (this.v / this.radius) + GAINS.heading * headingError, WZ_LIMIT);
		return { vx: this.v, vy: 0, wz, done: false, info };
	}
}

export class ShuttleController {
	/** @param {{distance: number, speed: number, rounds?: number}} options rounds 1 = 앞으로 갔다가 돌아오기. Infinity 가능. */
	constructor({ distance, speed, rounds = 1 }) {
		this.distance = distance;
		this.speed = speed;
		this.rounds = rounds;
	}

	start(pose) {
		this.origin = { x: pose.x, y: pose.y };
		this.reference = pose.yaw;
		this.leg = 'forward';
		this.nextLeg = 'forward';
		this.dwell = 0;
		this.round = 0;
		this.v = 0;
		this.finished = false;
	}

	update(pose, dt) {
		const cos = Math.cos(this.reference);
		const sin = Math.sin(this.reference);
		const px = pose.x - this.origin.x;
		const py = pose.y - this.origin.y;
		const alongTrack = px * cos + py * sin;
		const lateralError = -px * sin + py * cos;
		const headingError = wrapAngle(this.reference - pose.yaw);
		const info = () => ({ alongTrack, lateralError, headingError, leg: this.leg, round: this.round });

		if (this.finished) return { ...ZERO, done: true, info: info() };   // 끝난 뒤에는 다시 움직이지 않는다
		if (this.leg === 'dwell') {
			this.dwell -= dt;
			if (this.dwell <= 0) this.leg = this.nextLeg;
			return { ...ZERO, done: false, info: info() };
		}
		const target = this.leg === 'forward' ? this.distance : 0;
		const direction = this.leg === 'forward' ? 1 : -1;
		// 센서 · 명령 지연 동안 더 간 만큼을 미리 빼서 남은 거리를 본다
		const remaining = target - alongTrack - direction * this.v * GAINS.lookahead;
		if (remaining * direction < GAINS.arrive) {
			if (this.leg === 'back') this.round += 1;
			this.finished = this.round >= this.rounds;
			this.nextLeg = this.leg === 'forward' ? 'back' : 'forward';
			this.leg = this.finished ? 'done' : 'dwell';
			this.dwell = DWELL;
			this.v = 0;
			return { ...ZERO, done: this.finished, info: info() };
		}
		const wanted = clamp(GAINS.approach * Math.abs(remaining), this.speed);
		this.v = Math.min(Math.max(wanted, GAINS.minSpeed), this.v + GAINS.rampAccel * dt, this.speed);
		return {
			vx: direction * this.v,
			vy: clamp(-GAINS.lateral * lateralError, VY_LIMIT),
			wz: clamp(GAINS.heading * headingError, WZ_LIMIT),
			done: false,
			info: info(),
		};
	}
}
