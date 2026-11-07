import assert from 'node:assert/strict';
import test from 'node:test';

import { MAX_SENSOR_AGE, readSample } from '../../web/js/motion/sample.js';

function status(overrides = {}) {
	return {
		time: 1000.5,
		odom: { x: 1, y: 2, yaw: 0.3, vx: 0, vy: 0, wz: 0 },
		imu: { gyro: [0.001, 0.002, 0.03], accel: [0, 0, 9.8], yaw: 10 },
		ages: { odom: 0.02, imu: 0.0 },
		...overrides,
	};
}

test('정상 status 는 샘플로 바뀐다 (자이로는 gyro[2])', () => {
	assert.deepEqual(readSample(status()), { t: 1000.5, odom: { x: 1, y: 2, yaw: 0.3 }, gyroZ: 0.03 });
});

test('odom 이 멈춰 나이가 오래되면 (게이트웨이는 마지막 값을 계속 준다) 샘플이 아니다', () => {
	assert.equal(readSample(status({ ages: { odom: MAX_SENSOR_AGE + 0.1, imu: 0 } })), null);
	assert.equal(readSample(status({ ages: { odom: 0.1, imu: 1.5 } })), null);
});

test('나이가 없거나 null 이면 샘플이 아니다', () => {
	assert.equal(readSample(status({ ages: undefined })), null);
	assert.equal(readSample(status({ ages: { odom: null, imu: 0 } })), null);
	assert.equal(readSample(status({ ages: { imu: 0 } })), null);
});

test('값이 없거나 숫자가 아니면 샘플이 아니다', () => {
	assert.equal(readSample(status({ odom: null })), null);
	assert.equal(readSample(status({ imu: null })), null);
	assert.equal(readSample(status({ imu: { gyro: [0, 0] } })), null);
	assert.equal(readSample(status({ odom: { x: NaN, y: 0, yaw: 0 } })), null);
	assert.equal(readSample(status({ time: undefined })), null);
	assert.equal(readSample(null), null);
});
