// 대시보드 status 에서 센서 샘플 하나를 읽는다 (DOM 없음).

// status.time 은 게이트웨이가 status 를 만든 시각이라 항상 새로워 보인다. 센서 자신의 나이(status.ages)를 본다:
// 센서가 멈추면 값은 그대로 두고 나이만 커지다가 STALE_DATA_AGE(2 s) 뒤에 null 이 되고, 한 번도 안 온 센서의 나이도 null 이다.
export const MAX_SENSOR_AGE = 0.5;   // [s]

/** @returns {{t: number, odom: {x, y, yaw}, gyroZ: number} | null} 센서가 신선하고 값이 모두 유한한 숫자일 때만. */
export function readSample(status) {
	const ages = status?.ages;
	// null <= 0.5 는 참이다: 서버는 센서가 끊기면 나이를 null 로 준다. 숫자인지 먼저 본다
	const isFresh = (age) => Number.isFinite(age) && age <= MAX_SENSOR_AGE;
	if (!isFresh(ages?.odom) || !isFresh(ages?.imu)) return null;
	const odom = status.odom;
	const gyro = status.imu?.gyro;
	const values = [status.time, odom?.x, odom?.y, odom?.yaw, gyro?.[2]];
	if (!values.every(Number.isFinite)) return null;
	return { t: status.time, odom: { x: odom.x, y: odom.y, yaw: odom.yaw }, gyroZ: gyro[2] };
}
