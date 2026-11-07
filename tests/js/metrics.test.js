import assert from 'node:assert/strict';
import test from 'node:test';

import { LapRecorder, toCsv } from '../../web/js/motion/metrics.js';

test('반지름 0.5 원 위의 점 100개: 평균 0.5, 편차 0, 복귀 오차는 두 자세 사이 거리', () => {
	const recorder = new LapRecorder({ x: 1, y: 2 }, 0.5);
	for (let i = 0; i < 100; i += 1) {
		const angle = (i / 100) * 2 * Math.PI;
		recorder.add(1 + 0.5 * Math.cos(angle), 2 + 0.5 * Math.sin(angle));
	}
	const lap = recorder.finishLap({ x: 3, y: 4 }, { x: 0, y: 0 });
	assert.ok(Math.abs(lap.meanRadius - 0.5) < 1e-9);
	assert.ok(lap.stdRadius < 1e-9);
	assert.ok(Math.abs(lap.closure - 5) < 1e-9);
});

test('바퀴를 끝내면 표본이 비워진다', () => {
	const recorder = new LapRecorder({ x: 0, y: 0 }, 1);
	recorder.add(1, 0);
	recorder.finishLap({ x: 0, y: 0 }, { x: 0, y: 0 });
	recorder.add(2, 0);
	assert.equal(recorder.finishLap({ x: 0, y: 0 }, { x: 0, y: 0 }).meanRadius, 2);
});

test('toCsv 는 열 순서대로 머리줄과 행을 만들고 숫자는 소수 6자리로 쓴다', () => {
	const csv = toCsv([{ t: 1, a: 0.123456789, b: 'x' }, { t: 2, a: 2, b: 'y' }], ['t', 'a', 'b']);
	assert.equal(csv, 't,a,b\n1.000000,0.123457,x\n2.000000,2.000000,y\n');
});
