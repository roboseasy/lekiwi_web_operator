// 시험 전용 시뮬레이터: 대시보드 서버(램프 · 명령 끊김) + 로봇 + 센서 지연 + 브라우저 제어 주기를 흉내 낸다.
// 컨트롤러는 새 상태 샘플이 도착할 때만 갱신하고, 그 명령을 0.1 s 마다 teleop 처럼 서버로 보낸다.

import { PoseEstimator } from '../../web/js/motion/estimator.js';
import { MotionRun } from '../../web/js/motion/run.js';
import { rotate } from '../../web/js/motion/geometry.js';

const LINEAR_ACCEL = 0.4;     // [m/s²] 서버 램프-업 (LINEAR_STEP 0.02 / 50 ms). 내릴 때는 램프 없이 0
const ANGULAR_ACCEL = 2.4;    // [rad/s²] (ANGULAR_STEP 0.12 / 50 ms)
const COMMAND_TIMEOUT = 0.3;  // [s] teleop 이 끊기면 목표 0
const TELEOP_PERIOD = 0.1;    // [s]
const STATUS_PERIOD = 0.27;   // [s] 실측 수신 간격
const COMMAND_LATENCY = 0.1;  // [s] 브라우저 → 서버
const STEP = 0.01;            // [s]
const SETTLE = 3;             // [s] 끝난 뒤 더 돌려서 멈추는 것을 본다

export function mulberry32(seed) {
	let state = seed >>> 0;
	return () => {
		state = (state + 0x6d2b79f5) >>> 0;
		let t = state;
		t = Math.imul(t ^ (t >>> 15), t | 1);
		t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
		return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
	};
}

function approach(value, target, step) {
	if (target > value) return Math.min(target, value + step);
	return Math.max(target, value - step);
}

/**
 * @param {object} options
 * @param {object} options.plant 로봇 모델: speedScale(모터 속도 배율, 0 이면 바퀴가 막힌 것), odomScale(odom 거리 배율), yawDrift(odom 이 못 보는 실제 회전 rad/s),
 *     gyroBias(rad/s), strafeBias(바퀴가 만드는 옆 쏠림 m/s, odom 도 본다), latency(센서 → 브라우저 s, 기본 0.3), seed, noise(기본 true),
 *     frozenOdom(odom 이 값은 새로 오는데 움직이지 않는다).
 * @param {object} [options.controller] start(pose) / update(pose, dt) 를 가진 컨트롤러 (감시기 없이 제어만 시험).
 * @param {object} [options.plan] 주면 MotionRun(추정 · 컨트롤러 · 감시기 · 바퀴 결과) 전체를 돌린다.
 * @param {object} [options.estimatorOptions] PoseEstimator 옵션.
 * @param {number} [options.seconds] 최대 시뮬레이션 시간.
 * @returns {{truth: {t, x, y, yaw, info}[], done: boolean, stopReason: string|null, run: MotionRun|null}} 참 자세(출발점이 (0, 0, 0))와 컨트롤러 info.
 */
export function simulate({ plant = {}, controller, plan, estimatorOptions = {}, seconds = 200 }) {
	const {
		speedScale = 1, odomScale = 1, yawDrift = 0, gyroBias = 0, strafeBias = 0,
		latency = 0.3, seed = 1, noise = true, frozenOdom = false,
	} = plant;
	const random = mulberry32(seed);
	const gauss = () => Math.sqrt(-2 * Math.log(random() + 1e-12)) * Math.cos(2 * Math.PI * random());
	const jitter = (sigma) => (noise ? gauss() * sigma : 0);

	const truth = { x: 0, y: 0, yaw: 0 };
	const odom = { x: 0, y: 0, yaw: 0 };
	let out = { vx: 0, vy: 0, wz: 0 };
	let received = { cmd: { vx: 0, vy: 0, wz: 0 }, at: -Infinity };
	let cmd = { vx: 0, vy: 0, wz: 0 };
	let info = null;
	let doneAt = null;
	let run = null;
	let stopReason = null;
	let started = false;
	let lastSampleT = 0;
	let nextStatus = 0;
	let nextTeleop = 0;
	let nextRow = 0;
	const commands = [];
	const samples = [];
	const estimator = new PoseEstimator(estimatorOptions);
	const rows = [];

	for (let step = 0; step * STEP < seconds; step += 1) {
		const now = step * STEP;
		if (now >= nextTeleop) {
			commands.push({ arrive: now + COMMAND_LATENCY, cmd: { ...cmd } });
			nextTeleop += TELEOP_PERIOD;
		}
		while (commands.length && commands[0].arrive <= now) received = { cmd: commands.shift().cmd, at: now };
		const target = now - received.at <= COMMAND_TIMEOUT ? received.cmd : { vx: 0, vy: 0, wz: 0 };
		// teleop.py step(): 목표가 모두 0 이면 출력도 램프 없이 바로 0 (정지는 즉시), 아니면 램프
		const isZero = target.vx === 0 && target.vy === 0 && target.wz === 0;
		out = isZero ? { vx: 0, vy: 0, wz: 0 } : {
			vx: approach(out.vx, target.vx, LINEAR_ACCEL * STEP),
			vy: approach(out.vy, target.vy, LINEAR_ACCEL * STEP),
			wz: approach(out.wz, target.wz, ANGULAR_ACCEL * STEP),
		};

		const wheelX = out.vx * speedScale;
		const wheelY = out.vy * speedScale + strafeBias;
		const wheelW = out.wz * speedScale;
		const trueW = wheelW + yawDrift;
		const [dx, dy] = rotate(wheelX * STEP, wheelY * STEP, truth.yaw + (trueW * STEP) / 2);
		truth.x += dx;
		truth.y += dy;
		truth.yaw += trueW * STEP;
		const [ox, oy] = rotate(wheelX * odomScale * STEP, wheelY * odomScale * STEP, odom.yaw + (wheelW * STEP) / 2);
		if (!frozenOdom) {
			odom.x += ox;
			odom.y += oy;
			odom.yaw += wheelW * STEP;
		}

		if (now >= nextStatus) {
			nextStatus += STATUS_PERIOD + (random() - 0.5) * 0.04;
			samples.push({
				arrive: now + latency,
				sample: {
					t: now,
					odom: { x: odom.x + jitter(0.001), y: odom.y + jitter(0.001), yaw: odom.yaw + jitter(0.0005) },
					gyroZ: trueW + gyroBias + jitter(0.001),
				},
			});
		}
		while (samples.length && samples[0].arrive <= now) {
			const { sample } = samples.shift();
			if (!started) {
				estimator.reset(sample);
				if (plan) run = new MotionRun(plan, sample);
				else controller.start(estimator.pose);
				started = true;
				lastSampleT = sample.t;
				continue;
			}
			if (plan) {
				if (doneAt !== null) {
					run.settle(sample);   // 멈춘 뒤에는 자세만 갱신
				} else {
					const result = run.step(sample);
					cmd = result.command;
					info = run.info;
					if (result.stop) {
						doneAt = now;
						if (result.stop.tone !== 'ok') stopReason = result.stop.reason;
					}
				}
				continue;
			}
			const pose = estimator.update(sample);
			const result = controller.update(pose, sample.t - lastSampleT);
			lastSampleT = sample.t;
			cmd = { vx: result.vx, vy: result.vy, wz: result.wz };
			info = result.info;
			if (result.done && doneAt === null) doneAt = now;
		}
		if (now >= nextRow) {
			rows.push({ t: now, x: truth.x, y: truth.y, yaw: truth.yaw, info });
			nextRow += 0.05;
		}
		if (doneAt !== null && now - doneAt > SETTLE) break;
	}
	if (run) run.finish();
	return { truth: rows, done: plan ? Boolean(run?.isDone) : doneAt !== null, stopReason, stopAt: doneAt, run };
}
