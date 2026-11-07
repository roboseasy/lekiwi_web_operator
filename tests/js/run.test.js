import assert from 'node:assert/strict';
import test from 'node:test';

import { maxCircleSpeed } from '../../web/js/motion/command.js';
import { MotionRun } from '../../web/js/motion/run.js';
import { simulate } from './sim.js';

const PLANTS = [
	{ noise: true, seed: 11 },
	{ noise: true, seed: 12, speedScale: 0.85, odomScale: 0.97, gyroBias: 0.002 },   // H1
	{ noise: true, seed: 13, speedScale: 1.15, odomScale: 1.03, yawDrift: -0.0026 },   // H2
];

function circlePlan(radius, direction, useImu, laps = 2) {
	return { kind: 'circle', radius, direction, laps, speed: Math.min(0.12, 0.9 * maxCircleSpeed(radius)), useImu, tau: 100 };
}

test('감시기를 켠 닫힌 루프: 정상 원은 거짓 정지가 없다 (반지름 × 방향 × IMU 켬/끔 × 로봇 모델)', () => {
	for (const radius of [0.2, 0.5, 2]) {
		for (const direction of [1, -1]) {
			for (const useImu of [true, false]) {
				for (const plant of PLANTS) {
					const { stopReason, done } = simulate({ plant, plan: circlePlan(radius, direction, useImu), seconds: 600 });
					const label = JSON.stringify({ radius, direction, useImu, plant });
					assert.equal(stopReason, null, `거짓 정지 ${stopReason} ${label}`);
					assert.ok(done, `끝나지 않았습니다 ${label}`);
				}
			}
		}
	}
});

test('감시기를 켠 닫힌 루프: 정상 직선 왕복은 거짓 정지가 없다 (IMU 켬/끔)', () => {
	for (const useImu of [true, false]) {
		for (const plant of PLANTS) {
			const plan = { kind: 'line', distance: 1, rounds: 2, speed: 0.1, useImu, tau: 100 };
			const { stopReason, done } = simulate({ plant, plan, seconds: 400 });
			assert.equal(stopReason, null, `거짓 정지 ${stopReason} ${JSON.stringify({ useImu, plant })}`);
			assert.ok(done);
		}
	}
});

test('유한한 원은 바퀴마다 결과가 나온다 (마지막 바퀴 포함)', () => {
	for (const laps of [1, 3]) {
		const { run } = simulate({ plant: { noise: false }, plan: circlePlan(0.5, 1, true, laps), seconds: 400 });
		assert.equal(run.results.length, laps, run.results.join('\n'));
		assert.match(run.results[laps - 1], new RegExp(`^바퀴 ${laps}:`));
	}
});

test('직선 왕복은 왕복마다 결과가 나온다', () => {
	const plan = { kind: 'line', distance: 1, rounds: 2, speed: 0.1, useImu: true, tau: 100 };
	const { run } = simulate({ plant: { noise: false }, plan, seconds: 400 });
	assert.equal(run.results.length, 2);
});

function straightSamples(run, { start, count, dt, dx }) {
	const outputs = [];
	for (let i = 1; i <= count; i += 1) {
		outputs.push(run.step({ t: start + i * dt, odom: { x: i * dx, y: 0, yaw: 0 }, gyroZ: 0 }));
	}
	return outputs;
}

test('센서 시각이 거꾸로 가면 멈춘다 (서버 시계가 조정된 경우)', () => {
	const plan = { kind: 'line', distance: 1, rounds: 1, speed: 0.1, useImu: true, tau: 100 };
	const run = new MotionRun(plan, { t: 1000, odom: { x: 0, y: 0, yaw: 0 }, gyroZ: 0 });
	assert.equal(run.step({ t: 1000.27, odom: { x: 0, y: 0, yaw: 0 }, gyroZ: 0 }).stop, null);
	const backwards = run.step({ t: 400, odom: { x: 0, y: 0, yaw: 0 }, gyroZ: 0 });
	assert.match(backwards.stop.reason, /시각/);
	assert.deepEqual(backwards.command, { vx: 0, vy: 0, wz: 0 });
});

test('센서 시각이 앞으로 크게 건너뛰어도 멈춘다', () => {
	const plan = { kind: 'circle', radius: 0.5, direction: 1, laps: 1, speed: 0.12, useImu: true, tau: 100 };
	const run = new MotionRun(plan, { t: 1000, odom: { x: 0, y: 0, yaw: 0 }, gyroZ: 0 });
	assert.match(run.step({ t: 1005, odom: { x: 0, y: 0, yaw: 0 }, gyroZ: 0 }).stop.reason, /시각/);
});

test('직선: 구간 밖(거리 + 0.3 m 넘게)으로 가면 멈춘다', () => {
	const plan = { kind: 'line', distance: 1, rounds: 1, speed: 0.1, useImu: true, tau: 100 };
	const run = new MotionRun(plan, { t: 0, odom: { x: 0, y: 0, yaw: 0 }, gyroZ: 0 });
	// 컨트롤러가 멈추라고 해도 로봇이 계속 앞으로 간다고 가정하고 위치만 먹인다 (한 샘플 0.05 m, 점프 한도 안)
	const outputs = straightSamples(run, { start: 0, count: 30, dt: 0.25, dx: 0.05 });
	const stopped = outputs.find((output) => output.stop);
	assert.ok(stopped, '멈추지 않았습니다');
	assert.match(stopped.stop.reason, /범위|벗어/);
});

test('센서가 멈춘 채 시간이 흐르면 checkStale 이 멈춘다', () => {
	const plan = { kind: 'line', distance: 1, rounds: 1, speed: 0.1, useImu: true, tau: 100 };
	const run = new MotionRun(plan, { t: 100, odom: { x: 0, y: 0, yaw: 0 }, gyroZ: 0 });
	assert.equal(run.checkStale(100.5), null);
	assert.match(run.checkStale(101.5), /센서/);
});

test('진행 감시: odom 이 값은 새로 오는데 움직이지 않으면 (센서 나이는 작다) 멈춘다 — 원 · 직선, IMU 켬/끔', () => {
	const plans = [
		{ kind: 'line', distance: 1, rounds: 5, speed: 0.1, useImu: true, tau: 100 },
		{ kind: 'line', distance: 3, rounds: 5, speed: 0.2, useImu: false, tau: 100 },
		{ kind: 'circle', radius: 0.5, direction: 1, laps: 5, speed: 0.12, useImu: true, tau: 100 },
		{ kind: 'circle', radius: 2, direction: -1, laps: 5, speed: 0.12, useImu: false, tau: 100 },
	];
	for (const plan of plans) {
		for (const plant of [{ noise: true, frozenOdom: true }, { noise: true, speedScale: 0 }]) {
			const { stopReason, stopAt } = simulate({ plant, plan, seconds: 60 });
			const label = JSON.stringify({ plan, plant });
			assert.ok(stopReason, `멈추지 않았습니다 ${label}`);
			assert.ok(stopAt <= 12, `너무 늦게 멈췄습니다 ${stopAt}s ${label}`);
		}
	}
});

test('진행 감시: 로봇이 정상으로 움직이면 (아주 느린 속도 · 구간 끝 감속 포함) 거짓 정지가 없다', () => {
	for (const speed of [0.03, 0.05]) {
		const plan = { kind: 'line', distance: 1, rounds: 2, speed, useImu: true, tau: 100 };
		const { stopReason, done } = simulate({ plant: { noise: true, seed: 5 }, plan, seconds: 400 });
		assert.equal(stopReason, null, `거짓 정지 ${stopReason} 속도 ${speed}`);
		assert.ok(done);
	}
});

test('CSV 용 행은 상한 이후 오래된 것부터 버린다 (끝까지 최신이 남는다)', () => {
	const plan = { kind: 'circle', radius: 0.5, direction: 1, laps: Infinity, speed: 0.12, useImu: true, tau: 100 };
	const run = new MotionRun(plan, { t: 0, odom: { x: 0, y: 0, yaw: 0 }, gyroZ: 0 });
	for (let i = 1; i <= 20050; i += 1) run.rows.push({ t: i });   // 상한까지 채운다
	run.step({ t: 0.25, odom: { x: 0, y: 0, yaw: 0 }, gyroZ: 0 });
	assert.equal(run.rows.length, 20000);
	assert.equal(run.rows.at(-1).t, 0.25);
});
