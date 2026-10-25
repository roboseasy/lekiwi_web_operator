// 센서: IMU · odom(+ 원점으로) · 모터 표 (SPEC §11-5)

import { DASH, badge, degrees, esc, fixed, isNum, put, putHtml, readout, refs } from '../format.js';

export const title = '센서';
// IMU (100 Hz) · odom · 모터 상태는 보는 화면이 있을 때만 서버가 받는다
export const channels = ['imu', 'odom', 'motors'];

const MODES = { 0: '위치', 1: '속도', 2: 'PWM', 3: '스텝', 255: '모름' };
const AXIS_NAMES = ['x', 'y', 'z'];
const MOTOR_COLUMNS = [   // [이름, 숫자 칸인지]
	['ID', true], ['이름', false], ['연결', false], ['모드', false], ['토크', false],
	['위치 (rad)', true], ['속도 (rad/s)', true], ['부하 (%)', true], ['전압 (V)', true],
	['온도 (℃)', true], ['전류 (A)', true], ['상태', false],
];
const MOTOR_HEADER = MOTOR_COLUMNS
	.map(([label, isNumber]) => `<th scope="col" class="${isNumber ? 'num' : ''}">${label}</th>`)
	.join('');

function axes(prefix, label) {
	return AXIS_NAMES.map((axis) => readout(`${prefix}${axis}`, `${label} ${axis}`)).join('');
}

function motorRow(item) {
	const isOnline = item.online === true;
	const labels = item.status_labels ?? [];
	const state = labels.length ? badge('danger', labels.join(', ')) : badge('ok', '정상');
	const num = (value, digits) => `<td class="num">${isOnline ? fixed(value, digits) : DASH}</td>`;
	return `<tr>
		<td class="num">${esc(item.id)}</td>
		<td class="mono">${esc(item.name ?? DASH)}</td>
		<td>${isOnline ? badge('ok', '연결') : badge('danger', '끊김')}</td>
		<td>${esc(MODES[item.mode] ?? item.mode ?? DASH)}</td>
		<td>${item.torque_enabled ? badge('warn', '켜짐') : badge('neutral', '꺼짐')}</td>
		${num(item.position, 2)}${num(item.velocity, 2)}${num(item.load, 0)}
		${num(item.voltage, 1)}${num(item.temperature, 0)}${num(item.current, 2)}
		<td>${isOnline ? state : DASH}</td>
	</tr>`;
}

export function mount(el, ctx) {
	el.innerHTML = `
		<header class="page-head"><h1>센서</h1></header>
		<div class="sensors-grid">
			<section class="panel" aria-labelledby="imu-title">
				<h2 id="imu-title">IMU</h2>
				<dl class="readouts readouts-3">
					${readout('roll', 'roll', '°')}
					${readout('pitch', 'pitch', '°')}
					${readout('yaw', 'yaw', '°')}
				</dl>
				<h3 class="sub">자이로 (rad/s)</h3>
				<dl class="readouts readouts-3 readouts-small">${axes('gyro', '자이로')}</dl>
				<h3 class="sub">가속도 (m/s²)</h3>
				<dl class="readouts readouts-3 readouts-small">${axes('accel', '가속도')}</dl>
			</section>
			<section class="panel" aria-labelledby="odom-title">
				<div class="panel-head">
					<h2 id="odom-title">odom</h2>
					<button type="button" class="btn btn-small" data-k="reset">odom 원점으로</button>
				</div>
				<dl class="readouts readouts-3">
					${readout('x', '위치 x', 'm')}
					${readout('y', '위치 y', 'm')}
					${readout('odomYaw', '방향 yaw', '°')}
				</dl>
				<h3 class="sub">속도</h3>
				<dl class="readouts readouts-3 readouts-small">
					${readout('vx', 'vx', 'm/s')}
					${readout('vy', 'vy', 'm/s')}
					${readout('wz', 'wz', 'rad/s')}
				</dl>
				<p class="reason" data-k="resetReason"></p>
			</section>
			<section class="panel sensors-motors" aria-labelledby="motors-title">
				<h2 id="motors-title">모터</h2>
				<p class="feature-off" data-k="motorOff" hidden>
					모터 메시지(lekiwi_msgs)를 불러오지 못해 모터 상태를 볼 수 없습니다.
				</p>
				<div class="table-wrap">
					<table class="table">
						<thead><tr>${MOTOR_HEADER}</tr></thead>
						<tbody data-k="motors"></tbody>
					</table>
				</div>
			</section>
		</div>`;
	const k = refs(el);
	k.reset.addEventListener('click', () => ctx.conn.post('/api/odom/reset', {}));

	function render() {
		const status = ctx.status;
		const imu = status?.imu;
		put(k.roll, fixed(imu?.roll, 1));
		put(k.pitch, fixed(imu?.pitch, 1));
		put(k.yaw, fixed(imu?.yaw, 1));
		AXIS_NAMES.forEach((axis, index) => {
			put(k[`gyro${axis}`], fixed(imu?.gyro?.[index], 3));
			put(k[`accel${axis}`], fixed(imu?.accel?.[index], 2));
		});
		const odom = status?.odom;
		put(k.x, fixed(odom?.x, 3));
		put(k.y, fixed(odom?.y, 3));
		put(k.odomYaw, degrees(odom?.yaw, 1));
		put(k.vx, fixed(odom?.vx, 3));
		put(k.vy, fixed(odom?.vy, 3));
		put(k.wz, fixed(odom?.wz, 3));
		const reason = ctx.blockReason();
		k.reset.disabled = Boolean(reason);
		put(k.resetReason, reason ? `odom 원점으로: ${reason}` : '');
		k.motorOff.hidden = ctx.info?.features?.has_motor_msgs !== false;
		const motors = Array.isArray(status?.motors) ? status.motors : [];
		// rates null: 서버가 모터 상태를 받지 않는다 ('motors' 를 막 구독한 직후 등)
		const empty = isNum(status?.rates?.motor_states) ? '모터 상태가 들어오지 않습니다' : DASH;
		putHtml(k.motors, motors.length
			? motors.map(motorRow).join('')
			: `<tr><td colspan="${MOTOR_COLUMNS.length}">${empty}</td></tr>`);
	}

	ctx.on('update', render);
	render();
}

export function unmount() {}
