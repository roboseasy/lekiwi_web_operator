import assert from 'node:assert/strict';
import test from 'node:test';

import { LIMITS, Supervisor } from '../../web/js/motion/safety.js';

const DEG = Math.PI / 180;

/** 정상 샘플 하나. 필요한 값만 덮어쓴다. */
function sample(overrides = {}) {
	return {
		kind: 'circle', t: 0, pose: { x: 0, y: 0, yaw: 0 },
		primaryError: 0, headingError: 0, odomYaw: 0, gyroYaw: 0, ...overrides,
	};
}

/** 0.25 s 간격으로 seconds 동안 같은 오차를 먹이고, 처음으로 나온 정지 이유를 돌려 준다. */
function feed(supervisor, seconds, overrides) {
	for (let t = 0; t <= seconds + 1e-9; t += 0.25) {
		const reason = supervisor.observe(sample({ t, ...overrides }));
		if (reason) return { reason, t };
	}
	return { reason: null, t: seconds };
}

test('정상 샘플은 null', () => {
	assert.equal(new Supervisor().observe(sample()), null);
});

test('샘플이 1.2 s 없으면 정지, 0.5 s 면 null', () => {
	const supervisor = new Supervisor();
	supervisor.arm(0);
	assert.equal(supervisor.checkStale(0.5), null);
	assert.match(supervisor.checkStale(1.2), /센서/);
});

test('반지름 오차 0.35 m 가 1.2 s 이어지면 정지, 0.5 s 만 이어지면 null', () => {
	assert.match(feed(new Supervisor(), 1.25, { primaryError: 0.35 }).reason, /벗어/);
	assert.equal(feed(new Supervisor(), 0.5, { primaryError: 0.35 }).reason, null);
});

test('직선은 가로 오차 0.25 m 가 이어지면 정지 (원의 한도 0.30 보다 엄격)', () => {
	assert.match(feed(new Supervisor(), 1.25, { kind: 'line', primaryError: 0.25 }).reason, /벗어/);
	assert.equal(feed(new Supervisor(), 1.25, { kind: 'circle', primaryError: 0.25 }).reason, null);
});

test('방향 오차 70° 가 1.2 s 이어지면 정지', () => {
	assert.match(feed(new Supervisor(), 1.25, { headingError: 70 * DEG }).reason, /방향/);
});

test('오차가 중간에 돌아오면 지속 시간을 다시 센다', () => {
	const supervisor = new Supervisor();
	assert.equal(feed(supervisor, 0.75, { primaryError: 0.35 }).reason, null);
	assert.equal(supervisor.observe(sample({ t: 1.0, primaryError: 0.0 })), null);
	assert.equal(supervisor.observe(sample({ t: 1.25, primaryError: 0.35 })), null);
	assert.equal(supervisor.observe(sample({ t: 1.75, primaryError: 0.35 })), null);
});

test('한 샘플에 위치가 0.4 m 튀면 즉시 정지 (odom 초기화)', () => {
	const supervisor = new Supervisor();
	assert.equal(supervisor.observe(sample({ t: 0, pose: { x: 0, y: 0, yaw: 0 } })), null);
	assert.match(supervisor.observe(sample({ t: 0.25, pose: { x: 0.4, y: 0, yaw: 0 } })), /튀/);
});

test('2 s 창에서 odom yaw 가 +6° 변했는데 자이로가 −5° 면 부호 이유로 정지, +5.5° 면 null', () => {
	const run = (gyroDegrees) => {
		const supervisor = new Supervisor();
		for (let t = 0; t <= 2.25; t += 0.25) {
			const reason = supervisor.observe(sample({ t, odomYaw: 6 * DEG * (t / 2), gyroYaw: gyroDegrees * DEG * (t / 2) }));
			if (reason) return reason;
		}
		return null;
	};
	assert.match(run(-5), /부호/);
	assert.equal(run(5.5), null);
});

test('직선 중 아주 느린 방향 보정(0.003 rad/s)이 100 s 쌓여도 부호 거짓 정지는 없다', () => {
	const supervisor = new Supervisor();
	for (let t = 0; t <= 100; t += 0.25) {
		const reason = supervisor.observe(sample({ t, odomYaw: 0.003 * t, gyroYaw: 0 }));
		assert.equal(reason, null, `t=${t}`);
	}
});

test('2 s 창에서 회전 속도 차이가 0.2 rad/s 면 정지, 0.05 rad/s 면 null', () => {
	const run = (gyroRate) => {
		const supervisor = new Supervisor();
		for (let t = 0; t <= 2.5; t += 0.25) {
			const reason = supervisor.observe(sample({ t, odomYaw: 0, gyroYaw: gyroRate * t }));
			if (reason) return reason;
		}
		return null;
	};
	assert.match(run(0.2), /회전/);
	assert.equal(run(0.05), null);
});

test('한도 상수가 설계 문서와 같다', () => {
	assert.equal(LIMITS.radiusError, 0.30);
	assert.equal(LIMITS.lateralError, 0.20);
	assert.equal(LIMITS.staleSeconds, 1.0);
	assert.equal(LIMITS.rateMismatch, 0.15);
});

test('작은 원은 반지름의 0.8 배를 한도로 쓴다 (중심으로 파고들어도 잡는다)', () => {
	assert.match(feed(new Supervisor(), 1.25, { radius: 0.2, primaryError: 0.18 }).reason, /벗어/);
	assert.equal(feed(new Supervisor(), 1.25, { radius: 0.5, primaryError: 0.18 }).reason, null);
	assert.match(feed(new Supervisor(), 1.25, { radius: 0.5, primaryError: 0.35 }).reason, /벗어/);
});
