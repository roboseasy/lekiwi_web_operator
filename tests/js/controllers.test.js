import assert from 'node:assert/strict';
import test from 'node:test';

import { CircleController, GAINS, ShuttleController } from '../../web/js/motion/controllers.js';
import { simulate } from './sim.js';

const IDEAL = { noise: false };
// 로봇 4대씩 두 가설 묶음 (설계 문서의 열린 점: 직진 중 자이로와 odom yaw 가 어긋나는 원인을 모른다)
//  H1: odom 이 맞고 자이로 바이어스가 거짓말 (gyroBias ±)   H2: 자이로가 맞고 odom 이 못 보는 실제 회전이 있다 (yawDrift ±)
// 공통: 모터 속도 ±15 %, odom 거리 배율 ±3 % (실측 -1.4 %), 센서 지연 0.3 s, 4 Hz 샘플링, 잡음
const ROBOTS = {
	'H1 자이로 바이어스': [
		{ speedScale: 0.85, odomScale: 0.97, gyroBias: 0.002, seed: 1 },
		{ speedScale: 1.0, odomScale: 1.0, gyroBias: 0, seed: 2 },
		{ speedScale: 1.15, odomScale: 1.03, gyroBias: -0.002, seed: 3 },
		{ speedScale: 0.95, odomScale: 0.986, gyroBias: 0.001, seed: 4 },
	],
	'H2 실제 요 미끄러짐': [
		{ speedScale: 0.85, odomScale: 0.97, yawDrift: 0.0026, seed: 1 },
		{ speedScale: 1.0, odomScale: 1.0, yawDrift: 0, seed: 2 },
		{ speedScale: 1.15, odomScale: 1.03, yawDrift: -0.0026, seed: 3 },
		{ speedScale: 0.95, odomScale: 0.986, yawDrift: 0.001, seed: 4 },
	],
};

function near(actual, expected, tolerance, label = '') {
	assert.ok(Math.abs(actual - expected) <= tolerance, `${label} ${actual} 가 ${expected} ± ${tolerance} 밖입니다`);
}

/** 참 경로에서 lapFrom..lapTo(0부터, 끝 포함)바퀴째의 평균 반지름 · 마지막 점의 복귀 오차. */
function circleStats(truth, { radius, direction, lapFrom, lapTo }) {
	const first = truth[0];
	const center = {
		x: first.x - direction * radius * Math.sin(first.yaw),
		y: first.y + direction * radius * Math.cos(first.yaw),
	};
	const radii = truth
		.filter((row) => row.info && row.info.lap >= lapFrom && row.info.lap <= lapTo)
		.map((row) => Math.hypot(row.x - center.x, row.y - center.y));
	const last = truth[truth.length - 1];
	const mean = radii.reduce((sum, value) => sum + value, 0) / radii.length;
	return { mean, count: radii.length, closure: Math.hypot(last.x - first.x, last.y - first.y) };
}

function runCircle(plant, { laps = 4, direction = 1, radius = 0.5, speed = 0.12, estimatorOptions } = {}) {
	const controller = new CircleController({ radius, speed, direction, laps });
	const result = simulate({ plant, controller, estimatorOptions, seconds: 400 });
	return { ...result, stats: circleStats(result.truth, { radius, direction, lapFrom: 1, lapTo: laps - 1 }) };
}

test('원: 이상적인 로봇이 한 바퀴 돌고 done', () => {
	const { done, stats } = runCircle(IDEAL, { laps: 1 });
	assert.ok(done, '끝나지 않았습니다');
	const single = runCircle(IDEAL, { laps: 2 });
	near(single.stats.mean, 0.5, 0.01, '평균 반지름');
	near(stats.closure, 0, 0.03, '복귀 오차');
});

// 복귀 오차는 방향각을 어느 센서가 맞게 보는지에 달려 있다 (한 숫자로 두 가설을 다 맞출 수 없다: 추정기 DEFAULT_TAU 주석의 표).
// 그래서 가설마다 맞는 설정을 따로 시험한다. 반지름과 로봇 사이 차이는 어느 설정이든 같은 기준이다.
const CASES = [
	{ name: 'H1 자이로 바이어스, 기본 설정 (알려진 한계: 복귀 오차 0.10 이내)', robots: ROBOTS['H1 자이로 바이어스'], estimatorOptions: {}, closure: 0.10 },
	{ name: 'H1 자이로 바이어스, IMU 방향 끔 (odom 만)', robots: ROBOTS['H1 자이로 바이어스'], estimatorOptions: { useImuYaw: false }, closure: 0.05 },
	{ name: 'H2 실제 요 미끄러짐, 기본 설정 (알려진 한계: 복귀 오차 0.07 이내)', robots: ROBOTS['H2 실제 요 미끄러짐'], estimatorOptions: {}, closure: 0.07 },
	{ name: 'H2 실제 요 미끄러짐, τ=200', robots: ROBOTS['H2 실제 요 미끄러짐'], estimatorOptions: { tau: 200 }, closure: 0.05 },
];
for (const { name, robots, estimatorOptions, closure } of CASES) {
	test(`원: 로봇 4대가 같은 원을 그린다 (${name})`, () => {
		const means = [];
		for (const robot of robots) {
			const { done, stats } = runCircle({ latency: 0.3, ...robot }, { laps: 4, estimatorOptions });
			assert.ok(done, `끝나지 않았습니다 ${JSON.stringify(robot)}`);
			assert.ok(stats.count > 100, '2~4바퀴 표본이 부족합니다');
			near(stats.mean, 0.5, 0.03, `평균 반지름 ${JSON.stringify(robot)}`);
			assert.ok(stats.closure <= closure, `복귀 오차 ${stats.closure} > ${closure} ${JSON.stringify(robot)}`);
			means.push(stats.mean);
		}
		assert.ok(Math.max(...means) - Math.min(...means) <= 0.03, `로봇 사이 차이 ${means}`);
	});
}

test('원: 오른쪽(direction −1)도 같다', () => {
	const { stats } = runCircle(IDEAL, { laps: 2, direction: -1 });
	near(stats.mean, 0.5, 0.01);
});

test('원: 중심은 출발 자세 옆 r 지점이다', () => {
	const controller = new CircleController({ radius: 0.5, speed: 0.12 });
	controller.start({ x: 1, y: 2, yaw: Math.PI / 2 });
	near(controller.center.x, 0.5, 1e-9);
	near(controller.center.y, 2, 1e-9);
});

test('원: 속도는 램프로 올라간다', () => {
	const controller = new CircleController({ radius: 0.5, speed: 0.12 });
	controller.start({ x: 0, y: 0, yaw: 0 });
	const first = controller.update({ x: 0, y: 0, yaw: 0 }, 0.27);
	assert.ok(first.vx <= GAINS.rampAccel * 0.27 + 1e-9, `vx ${first.vx}`);
	near(first.wz / first.vx, 1 / 0.5, 1e-9);
});

test('직선: 1 m 왕복 2회', () => {
	const controller = new ShuttleController({ distance: 1, speed: 0.1, rounds: 2 });
	const { truth, done } = simulate({ plant: IDEAL, controller, seconds: 300 });
	assert.ok(done, '끝나지 않았습니다');
	const maxAlong = Math.max(...truth.map((row) => row.x));
	const last = truth[truth.length - 1];
	near(maxAlong, 1.0, 0.03, '앞 끝점');
	near(Math.hypot(last.x, last.y), 0, 0.03, '돌아온 점');
	assert.ok(Math.max(...truth.map((row) => Math.abs(row.y))) <= 0.05, '가로 오차');
});

test('직선: 옆으로 쏠려도 옆걸음으로 돌아온다', () => {
	const controller = new ShuttleController({ distance: 1, speed: 0.1, rounds: 1 });
	const { truth, done } = simulate({ plant: { noise: false, strafeBias: 0.01 }, controller, seconds: 300 });
	assert.ok(done);
	assert.ok(Math.max(...truth.map((row) => Math.abs(row.y))) <= 0.05, '가로 오차');
});

test('직선: 구간 끝에서 쉰다', () => {
	const controller = new ShuttleController({ distance: 1, speed: 0.1, rounds: 1 });
	const { truth } = simulate({ plant: IDEAL, controller, seconds: 300 });
	const legs = [];
	for (const row of truth) {
		if (row.info && legs[legs.length - 1] !== row.info.leg) legs.push(row.info.leg);
	}
	assert.deepEqual(legs, ['forward', 'dwell', 'back', 'done']);
});

test('직선: odom 거리 배율 ±3 % 에서도 참 거리 1 ± 0.04', () => {
	for (const odomScale of [0.97, 1.03]) {
		const controller = new ShuttleController({ distance: 1, speed: 0.1, rounds: 1 });
		const { truth } = simulate({ plant: { noise: false, odomScale }, controller, seconds: 300 });
		near(Math.max(...truth.map((row) => row.x)), 1.0, 0.04, `odomScale ${odomScale}`);
	}
});
