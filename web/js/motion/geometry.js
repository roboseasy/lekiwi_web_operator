// 이동 설계 공용 기하 함수 (DOM 없음: Node 에서도 시험한다)

export const TAU = 2 * Math.PI;

/** 각도를 (−π, π] 로 접는다. */
export function wrapAngle(angle) {
	return Math.atan2(Math.sin(angle), Math.cos(angle));
}

/** (x, y) 를 angle [rad] 만큼 반시계로 돌린다. */
export function rotate(x, y, angle) {
	const cos = Math.cos(angle);
	const sin = Math.sin(angle);
	return [x * cos - y * sin, x * sin + y * cos];
}

export function clamp(value, limit) {
	return Math.max(-limit, Math.min(limit, value));
}
