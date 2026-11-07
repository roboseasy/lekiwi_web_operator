// (vx, vy, wz) → 대시보드 teleop 입력(-1..1 + 속도 단계)과 속도 상한 계산 (DOM 없음)

// lekiwi_base_controller 설정 값을 복사했다. 컨트롤러 설정을 바꾸면 여기도 고친다.
const WHEEL_RADIUS = 0.05;
const BASE_RADIUS = 0.13647;
const KINEMATICS_X = 0.007587843843;
const KINEMATICS_Y = -0.010776931094;
const KINEMATICS_YAW = 2.094379865161;
const MAX_LINEAR = 0.23;     // [m/s]
const MAX_ANGULAR = 1.53;    // [rad/s]
export const MAX_WHEEL_SPEED = 4.601942363656923;   // [rad/s]
export const WHEEL_MARGIN = 0.8;   // 입력 속도는 바퀴 상한의 이만큼까지만 (피드백이 보정할 여유)

const WHEEL_ANGLES = [(2 * Math.PI) / 3, 0, (4 * Math.PI) / 3];

/** 가장 빠른 바퀴의 속도 [rad/s]. lekiwi_base_controller omni_kinematics.cpp 의 ToKinematicsFrame + ToRawWheelSpeeds 와 같은 식. */
export function wheelPeak(vx, vy, wz) {
	const x = vx - wz * KINEMATICS_Y;
	const y = vy + wz * KINEMATICS_X;
	const cos = Math.cos(KINEMATICS_YAW);
	const sin = Math.sin(KINEMATICS_YAW);
	const motorX = cos * x + sin * y;
	const lateral = -(-sin * x + cos * y);
	return Math.max(...WHEEL_ANGLES.map((angle) => Math.abs(
		(-Math.sin(angle) * motorX + Math.cos(angle) * lateral + BASE_RADIUS * wz) / WHEEL_RADIUS,
	)));
}

/** limits = status.teleop.limits ({max_linear_speed, max_linear_x, max_angular_z}). 없으면 이 파일의 상수 (lekiwi_ws 기준). */
function linearCap(limits) {
	return limits ? Math.min(MAX_LINEAR, limits.max_linear_speed, limits.max_linear_x) : MAX_LINEAR;
}

function angularCap(limits) {
	return limits ? Math.min(MAX_ANGULAR, limits.max_angular_z) : MAX_ANGULAR;
}

/** 반지름 radius 원을 앞을 보고 도는 최대 선속도 [m/s]: 바퀴 상한의 WHEEL_MARGIN 과 로봇이 보고한 베이스 상한 안. */
export function maxCircleSpeed(radius, limits = null) {
	return Math.min(linearCap(limits), angularCap(limits) * radius, (WHEEL_MARGIN * MAX_WHEEL_SPEED) / wheelPeak(1, 0, 1 / radius));
}

/** 직선 최대 속도 [m/s]. */
export function maxLineSpeed(limits = null) {
	return Math.min(linearCap(limits), (WHEEL_MARGIN * MAX_WHEEL_SPEED) / wheelPeak(1, 0, 0));
}

/**
 * 보정이 더해진 명령을 멈추지 않고 한도 안으로 줄인다 (방향은 유지):
 * 가장 높은 단계 상한 → 바퀴 한 개라도 MAX_WHEEL_SPEED 를 넘으면 전체를 같은 비율로.
 * (베이스 컨트롤러가 바퀴마다 따로 자르면 움직임의 방향이 바뀐다.)
 */
export function capCommand({ vx, vy, wz }, levels) {
	const top = levels[levels.length - 1];
	let command = { vx, vy, wz };
	if (top) {
		// 세 성분을 같은 비율로 줄인다 (선속도만 줄이면 곡률이 바뀐다)
		const speed = Math.hypot(vx, vy);
		const scale = Math.min(1, speed > 0 ? top[0] / speed : Infinity, wz !== 0 ? top[1] / Math.abs(wz) : Infinity);
		command = { vx: vx * scale, vy: vy * scale, wz: wz * scale };
	}
	const peak = wheelPeak(command.vx, command.vy, command.wz);
	if (peak > MAX_WHEEL_SPEED) {
		const scale = MAX_WHEEL_SPEED / peak;
		command = { vx: command.vx * scale, vy: command.vy * scale, wz: command.wz * scale };
	}
	return command;
}

/** levels = [[선속도, 회전속도], …] (status.teleop.pads.drive.levels). 둘 다 들어가는 가장 낮은 단계, 없으면 null. */
export function levelFor(levels, speed, turn) {
	const index = levels.findIndex(([linear, angular]) => linear >= speed - 1e-9 && angular >= turn - 1e-9);
	return index < 0 ? null : index;
}

/** {x, y, yaw, level} (teleop 메시지 값). 단계 상한 밖이면 null. 서버가 x · y · yaw 에 그 단계의 속도를 곱한다. */
export function toTeleop({ vx, vy, wz }, levels) {
	const level = levelFor(levels, Math.hypot(vx, vy), Math.abs(wz));
	if (level === null) return null;
	const [linear, angular] = levels[level];
	return { x: vx / linear, y: vy / linear, yaw: wz / angular, level };
}
