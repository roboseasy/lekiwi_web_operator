// 이동 설계 감시기: 센서가 끊기거나 거짓이거나 폭주하면 정지 이유를 돌려준다 (DOM 없음).
// 반환값은 한국어 정지 이유, 문제가 없으면 null.

export const LIMITS = {
	staleSeconds: 1.0,                      // [s] 새 샘플이 이보다 오래 안 오면 정지
	radiusError: 0.30,                      // [m] 원: 반지름 오차
	lateralError: 0.20,                     // [m] 직선: 가로 오차
	headingError: Math.PI / 3,              // [rad] 방향 오차
	runawaySeconds: 1.0,                    // [s] 위 오차가 이만큼 이어지면 정지
	rateMismatch: 0.15,                     // [rad/s] odom 과 자이로의 회전 속도 차이
	rateWindow: 2.0,                        // [s] 회전 속도를 비교하는 창
	signCheckAngle: (5 * Math.PI) / 180,    // [rad] rateWindow 안에서 odom yaw 가 이만큼 변했으면 자이로 부호를 본다
	jump: 0.3,                              // [m] 한 샘플에 위치가 이만큼 튀면 (odom 초기화) 정지
};

export class Supervisor {
	constructor(limits = LIMITS) {
		this.limits = limits;
		this.lastSampleTime = null;
		this.badSince = null;
		this.previousPose = null;
		this.window = [];
	}

	/** 시작 시각을 기록한다 (첫 샘플이 한 번도 안 오는 경우를 잡으려고). */
	arm(t) {
		this.lastSampleTime = t;
	}

	/** 새 샘플이 오지 않은 채 시간이 흐른 경우를 검사한다 (타이머에서 부른다). */
	checkStale(t) {
		if (this.lastSampleTime !== null && t - this.lastSampleTime > this.limits.staleSeconds) {
			return `센서 값이 ${this.limits.staleSeconds} 초 넘게 오지 않아 멈췄습니다`;
		}
		return null;
	}

	/**
	 * 새 샘플마다 부른다.
	 * @param {{kind: 'circle'|'line', t: number, pose: {x, y}, primaryError: number, headingError: number,
	 *     odomYaw: number, gyroYaw: number, radius?: number}} sample primaryError 는 원이면 반지름 오차, 직선이면 가로 오차 [m].
	 */
	observe({ kind, t, pose, primaryError, headingError, odomYaw, gyroYaw, radius = Infinity }) {
		const { limits } = this;
		this.lastSampleTime = t;

		if (this.previousPose) {
			const jump = Math.hypot(pose.x - this.previousPose.x, pose.y - this.previousPose.y);
			if (jump > limits.jump) return `위치가 한 번에 ${jump.toFixed(2)} m 튀어 멈췄습니다 (odom 이 초기화됐을 수 있습니다)`;
		}
		this.previousPose = { x: pose.x, y: pose.y };

		// 작은 원은 반지름의 0.8 배까지만 (0.30 m 한도로는 중심으로 파고들어도 안 잡힌다)
		const limit = kind === 'line' ? limits.lateralError : Math.min(limits.radiusError, 0.8 * radius);
		const reasons = [];
		if (Math.abs(primaryError) > limit) reasons.push(`${kind === 'line' ? '가로' : '반지름'} 오차 ${Math.abs(primaryError).toFixed(2)} m 가 한도 ${limit} m 를 벗어났습니다`);
		if (Math.abs(headingError) > limits.headingError) reasons.push(`방향 오차 ${Math.abs((headingError * 180) / Math.PI).toFixed(0)}° 가 한도를 벗어났습니다`);
		if (reasons.length) {
			if (this.badSince === null) this.badSince = t;
			if (t - this.badSince >= limits.runawaySeconds) return `${reasons[0]} (${limits.runawaySeconds} 초 이상)`;
		} else {
			this.badSince = null;
		}

		this.window.push({ t, odomYaw, gyroYaw });
		while (this.window.length > 1 && t - this.window[1].t >= limits.rateWindow) this.window.shift();
		const oldest = this.window[0];
		if (t - oldest.t >= limits.rateWindow) {
			const span = t - oldest.t;
			const odomTurned = odomYaw - oldest.odomYaw;
			const gyroTurned = gyroYaw - oldest.gyroYaw;
			// 창 안에서 odom 이 의미 있게 돌았는데 자이로가 반대로(또는 안) 돌았으면 IMU 장착이 반대다.
			// 출발 뒤 누적값으로 보면 안 된다: 직선 중 아주 느린 방향 보정(0.003 rad/s)도 100 s 에 15° 가 쌓인다
			if (Math.abs(odomTurned) >= limits.signCheckAngle && odomTurned * gyroTurned <= 0) {
				return '자이로 회전 부호가 odom 과 반대입니다 (IMU 장착 방향을 확인하세요)';
			}
			const difference = Math.abs(odomTurned / span - gyroTurned / span);
			if (difference > limits.rateMismatch) {
				return `odom 과 자이로의 회전 속도가 ${difference.toFixed(2)} rad/s 어긋나 멈췄습니다`;
			}
		}
		return null;
	}
}
