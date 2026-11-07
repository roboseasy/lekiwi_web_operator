import assert from 'node:assert/strict';
import test from 'node:test';

import {
	MAX_WHEEL_SPEED, WHEEL_MARGIN, capCommand, levelFor, maxCircleSpeed, maxLineSpeed, toTeleop, wheelPeak,
} from '../../web/js/motion/command.js';

const LEVELS = [[0.01, 0.06], [0.05, 0.29], [0.12, 0.70], [0.23, 1.53]];

function near(actual, expected, tolerance, label = '') {
	assert.ok(Math.abs(actual - expected) <= tolerance, `${label} ${actual} 가 ${expected} ± ${tolerance} 밖입니다`);
}

test('wheelPeak: 반지름 0.5 원의 단위 속도 (vx 1, wz 2) 는 약 23.30 rad/s', () => {
	near(wheelPeak(1, 0, 2), 23.3045, 0.01);
});

test('maxCircleSpeed(0.5) 는 바퀴 상한의 80 % 에서 나온다', () => {
	near(maxCircleSpeed(0.5), (WHEEL_MARGIN * MAX_WHEEL_SPEED) / wheelPeak(1, 0, 2), 1e-3);
	near(maxCircleSpeed(0.5), 0.158, 0.002);
});

test('반지름이 작을수록 최대 속도가 낮다', () => {
	assert.ok(maxCircleSpeed(0.2) < maxCircleSpeed(0.5));
});

test('maxLineSpeed 는 베이스 직선 상한 0.23 m/s 이하', () => {
	assert.ok(maxLineSpeed() <= 0.23);
	assert.ok(maxLineSpeed() > 0.1);
});

test('levelFor: 선속도와 회전속도가 모두 들어가는 가장 낮은 단계', () => {
	assert.equal(levelFor(LEVELS, 0.12, 0.24), 2);
	assert.equal(levelFor(LEVELS, 0.3, 0), null);
	assert.equal(levelFor(LEVELS, 0, 0), 0);
});

test('toTeleop: (vx, wz) 를 단계와 -1..1 입력으로 바꾼다', () => {
	const message = toTeleop({ vx: 0.12, vy: 0, wz: 0.24 }, LEVELS);
	assert.equal(message.level, 2);
	near(message.x, 1, 1e-9);
	near(message.y, 0, 1e-9);
	near(message.yaw, 0.24 / 0.7, 1e-9);
});

test('toTeleop: 후진과 오른쪽 회전은 음수', () => {
	const message = toTeleop({ vx: -0.05, vy: 0.02, wz: -0.1 }, LEVELS);
	assert.ok(message.x < 0 && message.y > 0 && message.yaw < 0);
});

test('toTeleop: 0 명령은 0 입력', () => {
	assert.deepEqual(toTeleop({ vx: 0, vy: 0, wz: 0 }, LEVELS), { x: 0, y: 0, yaw: 0, level: 0 });
});

test('toTeleop: 상한 밖이면 null', () => {
	assert.equal(toTeleop({ vx: 0.3, vy: 0, wz: 0 }, LEVELS), null);
});

test('로봇이 보고한 속도 상한(status.teleop.limits)이 더 낮으면 그것을 따른다', () => {
	const old = { max_linear_x: 0.11, max_linear_y: 0.11, max_linear_speed: 0.11, max_angular_z: 0.75 };
	assert.ok(maxCircleSpeed(0.5, old) <= 0.11);
	assert.ok(maxCircleSpeed(0.2, old) <= 0.75 * 0.2 + 1e-9);
	assert.ok(maxLineSpeed(old) <= 0.11);
	assert.equal(maxCircleSpeed(0.5, null), maxCircleSpeed(0.5));
});

test('capCommand: 보정이 단계 상한을 넘으면 멈추지 않고 상한으로 자른다 (방향 유지)', () => {
	assert.equal(capCommand({ vx: 0, vy: 0, wz: 2 }, LEVELS).wz, 1.53);
	assert.equal(capCommand({ vx: 0, vy: 0, wz: -3 }, LEVELS).wz, -1.53);
	const capped = capCommand({ vx: 0.2, vy: 0.2, wz: 0 }, LEVELS);
	assert.ok(Math.hypot(capped.vx, capped.vy) <= 0.23 + 1e-9);
	near(capped.vy / capped.vx, 1, 1e-9);
	assert.ok(toTeleop(capped, LEVELS) !== null);
	assert.deepEqual(capCommand({ vx: 0.05, vy: 0, wz: -0.1 }, LEVELS), { vx: 0.05, vy: 0, wz: -0.1 });
});

test('capCommand: 바퀴 한 개가 상한을 넘는 조합은 방향을 유지한 채 비례해서 줄인다', () => {
	const request = { vx: 0.23, vy: 0.05, wz: 0.9 };
	assert.ok(wheelPeak(request.vx, request.vy, request.wz) > MAX_WHEEL_SPEED);   // 줄이기 전에는 바퀴가 포화된다
	const capped = capCommand(request, LEVELS);
	assert.ok(wheelPeak(capped.vx, capped.vy, capped.wz) <= MAX_WHEEL_SPEED + 1e-9);
	near(capped.vy / capped.vx, request.vy / request.vx, 1e-9);
	near(capped.wz / capped.vx, request.wz / request.vx, 1e-9);
});
