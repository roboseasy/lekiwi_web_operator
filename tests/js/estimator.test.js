import assert from 'node:assert/strict';
import test from 'node:test';

import { rotate, wrapAngle } from '../../web/js/motion/geometry.js';
import { PoseEstimator } from '../../web/js/motion/estimator.js';

function near(actual, expected, tolerance, label = '') {
	assert.ok(Math.abs(actual - expected) <= tolerance, `${label} ${actual} 가 ${expected} ± ${tolerance} 밖입니다`);
}

test('wrapAngle 은 (−π, π] 로 접는다', () => {
	near(Math.abs(wrapAngle(3 * Math.PI)), Math.PI, 1e-9);
	assert.equal(wrapAngle(-0.5), -0.5);
});

test('odom 만 쓰면 odom 자세를 출발점 기준으로 돌려 준다', () => {
	const estimator = new PoseEstimator({ useImuYaw: false });
	estimator.reset({ t: 0, odom: { x: 1, y: 2, yaw: 0.5 }, gyroZ: 0 });
	const pose = estimator.update({ t: 0.27, odom: { x: 1.1, y: 2.0, yaw: 0.5 }, gyroZ: 0 });
	const [expectedX, expectedY] = rotate(0.1, 0, -0.5);
	near(pose.x, expectedX, 1e-9);
	near(pose.y, expectedY, 1e-9);
	near(pose.yaw, 0, 1e-9);
});

test('제자리 회전: 자이로 적분이 odom yaw 와 같으면 융합 yaw 도 같다', () => {
	const estimator = new PoseEstimator();
	estimator.reset({ t: 0, odom: { x: 0, y: 0, yaw: 0 }, gyroZ: 0.4 });
	let t = 0;
	for (let i = 0; i < 40; i += 1) {
		t += 0.27;
		estimator.update({ t, odom: { x: 0, y: 0, yaw: wrapAngle(0.4 * t) }, gyroZ: 0.4 });
	}
	near(estimator.pose.yaw, 0.4 * t, 0.02);
	near(estimator.odomYaw, 0.4 * t, 1e-6);
	near(estimator.gyroYaw, estimator.odomYaw, 0.02);
});

test('자이로 바이어스는 odom 쪽으로 천천히 끌려간다', () => {
	const estimator = new PoseEstimator({ tau: 15 });
	estimator.reset({ t: 0, odom: { x: 0, y: 0, yaw: 0 }, gyroZ: 0.01 });
	for (let i = 1; i <= 240; i += 1) {
		estimator.update({ t: i * 0.25, odom: { x: 0, y: 0, yaw: 0 }, gyroZ: 0.01 });
	}
	assert.ok(estimator.gyroYaw >= 0.5, `자이로 적분 ${estimator.gyroYaw}`);
	assert.ok(estimator.pose.yaw < 0.2, `융합 yaw ${estimator.pose.yaw}`);
	assert.ok(estimator.pose.yaw > 0, `융합 yaw ${estimator.pose.yaw}`);
});

test('원 위를 돌면 위치가 원 위에 있다 (한 바퀴 뒤 출발점 근처)', () => {
	const radius = 0.5;
	const speed = 0.12;
	const omega = speed / radius;
	const lapTime = (2 * Math.PI) / omega;
	const step = lapTime / 100;
	const start = { x: 2, y: 3, yaw: 0.3 };
	const odomAt = (t) => {
		const [dx, dy] = rotate(radius * Math.sin(omega * t), radius * (1 - Math.cos(omega * t)), start.yaw);
		return { x: start.x + dx, y: start.y + dy, yaw: wrapAngle(start.yaw + omega * t) };
	};
	const estimator = new PoseEstimator();
	estimator.reset({ t: 0, odom: odomAt(0), gyroZ: omega });
	for (let i = 1; i <= 100; i += 1) {
		estimator.update({ t: i * step, odom: odomAt(i * step), gyroZ: omega });
	}
	assert.ok(Math.hypot(estimator.pose.x, estimator.pose.y) < 0.03, `복귀 ${Math.hypot(estimator.pose.x, estimator.pose.y)}`);
});

test('샘플 간격이 1 s 를 넘으면 그 구간은 자이로를 적분하지 않는다', () => {
	const estimator = new PoseEstimator();
	estimator.reset({ t: 0, odom: { x: 0, y: 0, yaw: 0 }, gyroZ: 0.4 });
	estimator.update({ t: 5, odom: { x: 0, y: 0, yaw: 0 }, gyroZ: 0.4 });
	assert.ok(estimator.gyroYaw < 0.1, `자이로 적분 ${estimator.gyroYaw}`);
});

test('IMU 방향을 꺼도 자이로 적분은 계속 쌓인다 (감시기가 비교하려면 필요하다)', () => {
	const estimator = new PoseEstimator({ useImuYaw: false });
	estimator.reset({ t: 0, odom: { x: 0, y: 0, yaw: 0 }, gyroZ: 0.4 });
	let t = 0;
	for (let i = 0; i < 20; i += 1) {
		t += 0.27;
		estimator.update({ t, odom: { x: 0, y: 0, yaw: wrapAngle(0.4 * t) }, gyroZ: 0.4 });
	}
	near(estimator.gyroYaw, 0.4 * t, 1e-6);
	near(estimator.pose.yaw, estimator.odomYaw, 1e-9);   // 융합 방향은 odom 만 따른다
});
