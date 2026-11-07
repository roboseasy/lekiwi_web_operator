// 바퀴별 결과와 CSV (DOM 없음)

export class LapRecorder {
	/** @param {{x: number, y: number}} center 원 중심. */
	constructor(center, radius) {
		this.center = center;
		this.radius = radius;
		this.radii = [];
	}

	add(x, y) {
		this.radii.push(Math.hypot(x - this.center.x, y - this.center.y));
	}

	/** 지금까지의 표본으로 한 바퀴 결과를 내고 표본을 비운다. closure = 끝 자세와 출발 자세 사이 거리 [m]. */
	finishLap(pose, startPose) {
		const count = this.radii.length || 1;
		const meanRadius = this.radii.reduce((sum, value) => sum + value, 0) / count;
		const variance = this.radii.reduce((sum, value) => sum + (value - meanRadius) ** 2, 0) / count;
		this.radii = [];
		return {
			meanRadius,
			stdRadius: Math.sqrt(variance),
			closure: Math.hypot(pose.x - startPose.x, pose.y - startPose.y),
		};
	}
}

/** rows 를 columns 순서의 CSV 로. 숫자는 소수 6자리, 나머지는 그대로. */
export function toCsv(rows, columns) {
	const cell = (value) => (typeof value === 'number' ? value.toFixed(6) : String(value ?? ''));
	const lines = rows.map((row) => columns.map((column) => cell(row[column])).join(','));
	return `${[columns.join(','), ...lines].join('\n')}\n`;
}
