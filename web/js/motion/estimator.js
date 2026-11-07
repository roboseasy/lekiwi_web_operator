// 위치 · 방향 추정: odom 자세 증분을 융합 방향으로 쌓는다.
// 방향 = 자이로 z 적분 + odom yaw 쪽으로 느리게 끄는 보완 필터 (자이로 바이어스를 눌러 준다).
// IMU 의 imu.yaw 는 쓰지 않는다: IMU 노드의 자세 추정이 기울어 있다 (설계 문서).

import { rotate, wrapAngle } from './geometry.js';

// 보완 필터 시간 상수 [s]. 작으면 odom 을, 크면 자이로를 믿는다. 어느 쪽이 맞는지는 로봇에서 비교해 정한다 (설계 문서의 열린 점).
// 4바퀴 시뮬레이션(속도 0.12 m/s, 센서 지연 0.3 s)의 최악 복귀 오차 [m], H1 자이로 바이어스 / H2 실제 요 미끄러짐:
//   odom 만: 0.012 / 0.154   τ=15: 0.028 / 0.136   τ=40: 0.053 / 0.105   τ=100: 0.091 / 0.058   τ=200: 0.113 / 0.030   자이로만: 0.147 / 0.017
//   (반지름 오차는 모든 설정에서 ≤ 0.016 m)
// 실측(lekiwi04)에서 자이로-odom 차이의 부호가 이동 방향을 따라 바뀌어서(전진 +, 후진 -) H2 쪽에 가깝다고 보고 100 으로 둔다.
export const DEFAULT_TAU = 100;
const MAX_GAP = 1.0;   // [s] 샘플 간격이 이보다 길면 그 구간은 적분하지 않고 odom 방향에 다시 맞춘다

export class PoseEstimator {
	/**
	 * @param {{useImuYaw?: boolean, tau?: number}} options useImuYaw=false 면 odom yaw 만 쓴다. tau: 보완 필터 시간 상수 [s].
	 */
	constructor({ useImuYaw = true, tau = DEFAULT_TAU } = {}) {
		this.useImuYaw = useImuYaw;
		this.tau = tau;
		this.reset({ t: 0, odom: { x: 0, y: 0, yaw: 0 }, gyroZ: 0 });
	}

	/** 이 샘플을 기준(0, 0, 0)으로 삼는다. sample = { t [s], odom: {x, y, yaw}, gyroZ [rad/s] }. */
	reset(sample) {
		this.prev = { t: sample.t, odom: { ...sample.odom }, gyroZ: sample.gyroZ };
		this.x = 0;
		this.y = 0;
		this.yaw = 0;
		this.relativeOdomYaw = 0;   // odom yaw 를 풀어서(unwrap) 쌓은 값
		this.integratedGyroYaw = 0;   // 보완 없이 자이로만 적분한 값
	}

	/** 새 샘플로 갱신하고 출발점 기준 자세를 돌려 준다. */
	update(sample) {
		const { prev } = this;
		const dt = sample.t - prev.t;
		const [bodyX, bodyY] = rotate(sample.odom.x - prev.odom.x, sample.odom.y - prev.odom.y, -prev.odom.yaw);
		this.relativeOdomYaw += wrapAngle(sample.odom.yaw - prev.odom.yaw);
		const yawBefore = this.yaw;
		const isContinuous = dt > 0 && dt <= MAX_GAP;
		if (isContinuous) this.integratedGyroYaw += 0.5 * (prev.gyroZ + sample.gyroZ) * dt;   // IMU 방향을 꺼도 쌓는다 (감시기가 비교한다)
		if (this.useImuYaw && isContinuous) {
			const predicted = this.yaw + 0.5 * (prev.gyroZ + sample.gyroZ) * dt;   // 새 odom yaw 와는 갱신된 값끼리 비교한다
			this.yaw = predicted + (dt / this.tau) * wrapAngle(this.relativeOdomYaw - predicted);
		} else {
			this.yaw = this.relativeOdomYaw;
		}
		const [worldX, worldY] = rotate(bodyX, bodyY, yawBefore);
		this.x += worldX;
		this.y += worldY;
		this.prev = { t: sample.t, odom: { ...sample.odom }, gyroZ: sample.gyroZ };
		return this.pose;
	}

	get pose() {
		return { x: this.x, y: this.y, yaw: this.yaw };
	}

	get odomYaw() {
		return this.relativeOdomYaw;
	}

	get gyroYaw() {
		return this.integratedGyroYaw;
	}
}
