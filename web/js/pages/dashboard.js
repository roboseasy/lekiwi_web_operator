// 대시보드: 경고 · 기능 켜고 끄기 · 모터 · 센서 수신 · 시스템 · 로봇 · 노드를 한눈에 (SPEC §11-1)

import {
	DASH, badge, duration, esc, fixed, isNum, isOn, processBadge, put, putHtml, readout, refs,
} from '../format.js';
import { createCameraView } from '../widgets/camera_view.js';

export const title = '대시보드';
// 스캔 · odom 은 센서 수신 빈도 · 끊김 경고에만, 모터 상태는 모터 · 전원 칸에 쓴다.
// IMU (100 Hz) 는 구독하지 않는다: Pi 에서 대시보드만 열어도 코어를 크게 쓴다. 살아 있는지는 노드 표,
// 값은 센서 화면에서 본다 (여기서는 '보지 않음')
export const channels = ['camera/front', 'scan', 'odom', 'motors'];

const FEATURES = [
	{
		name: 'bringup',
		title: '로봇',
		about: '센서 · 모터 · 카메라 노드를 켭니다 (bringup)',
		start: '로봇 켜기',
		stop: '로봇 끄기',
	},
	{
		name: 'mapping',
		title: '지도 작성',
		about: '로봇을 천천히 움직여 지도를 만듭니다 (Cartographer)',
		start: '지도 작성 시작',
		stop: '지도 작성 끄기',
		href: '#/mapping',
		link: '지도 작성 화면',
	},
	{
		name: 'navigation',
		title: '자율 주행',
		about: '저장한 지도에서 목표까지 스스로 갑니다 (Nav2)',
		stop: '자율 주행 끄기',
		href: '#/navigation',
		link: '지도 고르고 시작',
	},
];
// [rates 키, 이름, '정상' 최소 Hz, ages 키 (카메라는 없음)]
// shortcut: '늦음' 기준은 실물 주기(스캔 10 · 모터 20 · 카메라 30 Hz)의 절반쯤으로 고정, 센서 설정이 바뀌면 고친다
const SENSORS = [
	['scan', '스캔', 5, 'scan'],
	['odom', 'odom', 10, 'odom'],
	['imu', 'IMU', 10, 'imu'],
	['motor_states', '모터 상태', 5, 'motor_states'],
	['front_camera', '전방 카메라', 5, null],
	['wrist_camera', '손목 카메라', 5, null],
];
// 서버가 빈도를 재는 구간 (ros_node.RATE_WINDOW): 막 받기 시작하면 (구독 · 로봇 켜기 직후) 덜 차서
// 낮게 나온다
const RATE_WINDOW_MS = 2000;

let camera = null;

function featureRow(feature) {
	const { name } = feature;
	const cameraBox = name === 'bringup'
		? '<label class="check" data-k="useCameraBox">'
			+ '<input type="checkbox" data-k="useCamera" checked> 카메라 포함</label>'
		: '';
	const start = feature.start
		? `<button type="button" class="btn btn-primary" data-k="${name}Start">${feature.start}`
			+ '</button>'
		: '';
	const link = feature.href
		? `<a class="btn btn-ghost" href="${feature.href}">${feature.link}</a>`
		: '';
	return `
		<li class="feature">
			<div class="feature-text">
				<div class="feature-head">
					<h3>${feature.title}</h3><span data-k="${name}Badge"></span>
				</div>
				<p class="feature-about">${feature.about}</p>
				<p class="feature-meta" data-k="${name}Meta"></p>
			</div>
			<div class="feature-side">
				<div class="actions">
					${cameraBox}${start}
					<button type="button" class="btn btn-danger" data-k="${name}Stop" hidden>
						${feature.stop}
					</button>
					${link}
				</div>
				<p class="reason" data-k="${name}Reason"></p>
			</div>
		</li>`;
}

function template() {
	return `
		<header class="page-head"><h1>대시보드</h1></header>
		<div class="dash">
			<section class="panel dash-warnings" data-k="warningsPanel"
				aria-labelledby="warnings-title">
				<h2 id="warnings-title" data-k="warningsTitle">경고</h2>
				<ul class="warning-list" data-k="warnings"></ul>
			</section>
			<section class="panel dash-features" aria-labelledby="features-title">
				<h2 id="features-title">기능</h2>
				<ul class="features">${FEATURES.map(featureRow).join('')}</ul>
			</section>
			<section class="panel dash-camera" aria-labelledby="camera-title">
				<h2 id="camera-title">전방 카메라</h2>
				<div class="well camera-well" data-k="camera"></div>
			</section>
			<section class="panel dash-motor" aria-labelledby="motor-title">
				<h2 id="motor-title">모터 · 전원</h2>
				<p class="feature-off" data-k="motorOff" hidden>
					모터 메시지(lekiwi_msgs)를 불러오지 못해 모터 상태를 볼 수 없습니다.
				</p>
				<dl class="readouts">
					${readout('torque', '바퀴 토크')}
					${readout('voltage', '전압 (최저 – 최고)', 'V')}
					${readout('temperature', '최고 온도', '℃')}
					${readout('online', '연결된 모터')}
				</dl>
				<ul class="fault-list" data-k="faults"></ul>
			</section>
			<section class="panel dash-sensors" aria-labelledby="sensors-title">
				<h2 id="sensors-title">센서 수신</h2>
				<table class="table">
					<thead><tr>
						<th scope="col">센서</th><th scope="col" class="num">빈도</th>
						<th scope="col">상태</th>
					</tr></thead>
					<tbody data-k="sensors"></tbody>
				</table>
			</section>
			<section class="panel dash-system" aria-labelledby="system-title">
				<h2 id="system-title">시스템</h2>
				<dl class="readouts">
					${readout('cpu', 'CPU', '%')}
					${readout('cpuTemp', 'CPU 온도', '℃')}
					${readout('memory', '메모리', '%')}
					${readout('disk', '디스크', '%')}
					${readout('load', '부하 (1 · 5 · 15분)')}
				</dl>
				<p class="power-line" data-k="power"></p>
			</section>
			<section class="panel dash-robot" aria-labelledby="robot-title">
				<h2 id="robot-title">로봇</h2>
				<dl class="readouts readouts-text">
					${readout('robotName', '이름')}
					${readout('host', '호스트')}
					${readout('ip', 'IP')}
					${readout('uptime', '가동 시간')}
					${readout('version', '버전')}
					${readout('ros', 'ROS')}
				</dl>
				<ul class="url-list" data-k="urls"></ul>
			</section>
			<section class="panel dash-nodes" aria-labelledby="nodes-title">
				<h2 id="nodes-title">노드</h2>
				<table class="table">
					<thead><tr><th scope="col">노드</th><th scope="col">상태</th></tr></thead>
					<tbody data-k="nodes"></tbody>
				</table>
			</section>
		</div>`;
}

function processMeta(process) {
	if (!process) return '';
	const parts = [];
	if (process.detail) parts.push(process.detail);
	if (isNum(process.pid)) parts.push(`PID ${process.pid}`);
	if (process.state === 'failed' && isNum(process.exit_code)) {
		parts.push(`종료 코드 ${process.exit_code}`);
	}
	if (isOn(process) && process.options?.use_camera === false) parts.push('카메라 없이 켬');
	if (isOn(process) && process.map) parts.push(`지도 ${process.map}`);
	return parts.join(' · ');
}

function renderFeatures(k, ctx) {
	const processes = ctx.status?.processes;
	for (const { name } of FEATURES) {
		const process = processes?.[name];
		const isActive = isOn(process);
		const startButton = k[`${name}Start`];
		const stopButton = k[`${name}Stop`];
		putHtml(k[`${name}Badge`], processBadge(process));
		put(k[`${name}Meta`], processMeta(process));
		stopButton.hidden = !isActive;
		if (startButton) startButton.hidden = isActive;
		const button = isActive ? stopButton : startButton;
		const reason = button ? ctx.processBlock(name, isActive ? 'stop' : 'start') : null;
		if (button) button.disabled = Boolean(reason);
		// 끌 수 없는 외부 실행은 이유가 detail 그대로라 위 줄(meta)에 이미 있다
		put(k[`${name}Reason`], reason && reason !== process?.detail ? reason : '');
	}
	k.useCameraBox.hidden = isOn(processes?.bringup);
}

function renderWarnings(k, warnings) {
	if (!Array.isArray(warnings)) {
		k.warningsPanel.dataset.tone = 'neutral';
		put(k.warningsTitle, '경고');
		putHtml(k.warnings, '<li class="is-calm">연결되면 로봇 경고를 보여 줍니다</li>');
		return;
	}
	k.warningsPanel.dataset.tone = warnings.length ? 'warn' : 'ok';
	put(k.warningsTitle, warnings.length ? `경고 ${warnings.length}개` : '경고 없음');
	putHtml(k.warnings, warnings.map((text) => `<li>${esc(text)}</li>`).join(''));
}

function renderMotor(k, ctx) {
	const status = ctx.status;
	const motor = status?.motor;
	const motors = Array.isArray(status?.motors) ? status.motors : [];
	const online = motors.filter((item) => item.online);
	const temperatures = online.map((item) => item.temperature).filter(isNum);
	k.motorOff.hidden = ctx.info?.features?.has_motor_msgs !== false;
	const torque = { true: '켜짐', false: '꺼짐' }[motor?.ready] ?? DASH;
	put(k.torque, torque !== DASH && motor?.is_powered_by_us ? `${torque} · 웹에서 켬` : torque);
	const power = status?.power;
	const range = `${fixed(power?.voltage_min, 1)} – ${fixed(power?.voltage_max, 1)}`;
	put(k.voltage, power ? range : DASH);
	put(k.temperature, temperatures.length ? fixed(Math.max(...temperatures), 0) : DASH);
	// rates null: 서버가 모터 상태를 받지 않는다 ('motors' 를 막 구독한 직후 등). 0 / 0 대신 —
	put(k.online, isNum(status?.rates?.motor_states)
		? `${online.length} / ${motors.length}` : DASH);
	const faults = motors.flatMap((item) => {
		const name = `${item.id} ${item.name ?? ''}`;
		if (!item.online) return [badge('danger', `${name}: 연결 끊김`)];
		if (item.status_labels?.length) {
			return [badge('danger', `${name}: ${item.status_labels.join(', ')}`)];
		}
		return [];
	});
	if (status && motors.length && !faults.length) faults.push(badge('ok', '모터 상태 비트 이상 없음'));
	putHtml(k.faults, faults.map((html) => `<li>${html}</li>`).join(''));
}

function sensorState(rate, age, minHz, isCamera, isRobotOn, isMeasuring) {
	if (!isNum(rate)) return ['neutral', '보지 않음'];   // 서버가 구독하지 않는 토픽은 rates 가 null
	if (isCamera) {
		if (rate === 0) return ['neutral', '없음'];
	} else {
		if (!isNum(age)) return ['neutral', '없음'];
		if (age > 1 || rate === 0) {
			return isRobotOn ? ['danger', '끊김'] : ['neutral', '꺼짐'];   // 일부러 끈 로봇은 경고가 아니다
		}
	}
	if (rate >= minHz) return ['ok', '정상'];
	return isMeasuring ? ['neutral', '측정 중'] : ['warn', '늦음'];
}

/** @param {object} rateSince rates 키 → 받기 시작한 것을 본 시각 (performance.now). 이 함수가 고친다. */
function renderSensors(k, status, rateSince) {
	const isRobotOn = isOn(status?.processes?.bringup);
	const now = performance.now();
	const rows = SENSORS.map(([key, label, minHz, ageKey]) => {
		const rate = status?.rates?.[key];
		if (!isNum(rate) || rate === 0) delete rateSince[key];   // 안 받는다: 다시 오면 처음부터 잰다
		else rateSince[key] ??= now;
		const isMeasuring = now - rateSince[key] < RATE_WINDOW_MS;
		const age = ageKey ? status?.ages?.[ageKey] : null;
		const [tone, text] = status
			? sensorState(rate, age, minHz, !ageKey, isRobotOn, isMeasuring)
			: ['neutral', DASH];
		const hz = isNum(rate) ? `${fixed(rate, 1)} Hz` : DASH;
		return `<tr><td>${label}</td><td class="num">${hz}</td><td>${badge(tone, text)}</td></tr>`;
	});
	putHtml(k.sensors, rows.join(''));
}

function powerLine(system) {
	const throttled = system?.throttled;
	if (!system) return '';
	if (!throttled) return badge('neutral', '전원 상태를 읽을 수 없습니다 (vcgencmd 없음)');
	if (throttled.is_undervoltage_now) {
		return badge('danger', '지금 전원 전압이 낮습니다. 배터리 · 어댑터를 확인하세요');
	}
	if (throttled.is_throttled_now) return badge('warn', 'CPU 속도가 제한되고 있습니다');
	if (throttled.has_undervoltage_occurred) return badge('warn', '부팅 뒤 전원 부족이 있었습니다');
	return badge('ok', '전원 정상');
}

function renderSystem(k, system) {
	put(k.cpu, fixed(system?.cpu_percent, 0));
	put(k.cpuTemp, fixed(system?.cpu_temp, 1));
	put(k.memory, fixed(system?.memory_percent, 0));
	put(k.disk, fixed(system?.disk_percent, 0));
	const load = system?.load;
	put(k.load, Array.isArray(load) ? load.map((value) => fixed(value, 2)).join(' · ') : DASH);
	putHtml(k.power, powerLine(system));
}

function renderRobot(k, ctx) {
	const { info, status } = ctx;
	const ros = status?.ros;
	const addresses = (status?.network?.interfaces ?? [])
		.filter((item) => item.is_up && item.name !== 'lo')
		.flatMap((item) => item.ipv4 ?? []);
	put(k.robotName, info?.robot_name ?? DASH);
	put(k.host, info?.host ?? status?.system?.hostname ?? DASH);
	put(k.ip, addresses.length ? addresses.join(', ') : DASH);
	put(k.uptime, duration(status?.system?.uptime));
	put(k.version, info?.version ?? DASH);
	const rosText = ros ? `도메인 ${ros.domain_id ?? DASH} · 노드 ${ros.node_count ?? DASH}개` : DASH;
	put(k.ros, ros && ros.is_ok === false ? `${rosText} · 오류` : rosText);
	const urls = info?.urls ?? [];
	putHtml(k.urls, urls.map((url) => `<li><a href="${esc(url)}">${esc(url)}</a></li>`).join(''));
}

function renderNodes(k, health) {
	const entries = Object.entries(health ?? {});
	const rows = entries.map(([name, isAlive]) => `<tr><td class="mono">${esc(name)}</td>`
		+ `<td>${isAlive ? badge('ok', '있음') : badge('neutral', '없음')}</td></tr>`);
	putHtml(k.nodes, rows.length ? rows.join('') : `<tr><td colspan="2">${DASH}</td></tr>`);
}

export function mount(el, ctx) {
	el.innerHTML = template();
	const k = refs(el);
	camera = createCameraView(k.camera, '전방 카메라');
	const rateSince = {};   // 이 화면을 연 동안만 (다시 열면 처음부터 잰다)
	for (const { name } of FEATURES) {
		k[`${name}Start`]?.addEventListener('click', () => {
			const options = name === 'bringup' ? { use_camera: k.useCamera.checked } : {};
			ctx.startProcess(name, options);
		});
		k[`${name}Stop`].addEventListener('click', () => ctx.stopProcess(name));
	}

	function render() {
		renderWarnings(k, ctx.status?.warnings);
		renderFeatures(k, ctx);
		renderMotor(k, ctx);
		renderSensors(k, ctx.status, rateSince);
		renderSystem(k, ctx.status?.system);
		renderRobot(k, ctx);
		renderNodes(k, ctx.status?.health);
	}

	ctx.on('update', render);
	ctx.on('camera', ({ camera: name, blob }) => {
		if (name === 'front') camera.setFrame(blob);
	});
	render();
}

export function unmount() {
	camera?.destroy();
	camera = null;
}
