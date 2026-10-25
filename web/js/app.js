// 웹 오퍼레이터 화면의 뼈대: 상단 바 · 화면 전환 · 토스트 · 확인 창 · 여러 화면이 쓰는 명령 (SPEC §11)

import { Connection } from './connection.js';
import { badge, isExploring, isOn, putHtml } from './format.js';
import * as calibration from './pages/calibration.js';
import * as dashboard from './pages/dashboard.js';
import * as drive from './pages/drive.js';
import * as logs from './pages/logs.js';
import * as mapping from './pages/mapping.js';
import * as navigation from './pages/navigation.js';
import * as network from './pages/network.js';
import * as sensors from './pages/sensors.js';

const PAGES = { dashboard, calibration, drive, mapping, navigation, sensors, network, logs };
const THEME_KEY = 'lekiwi-web-operator.theme';   // index.html 첫 스크립트와 같은 이름
const TOAST_LIMIT = 4;
// 같은 글이 이 안에 또 오면 (실패 결과와 같은 글의 오류 이벤트, WS · POST 두 길로 보낸 비상 정지의
// 같은 알림 등) 사이에 다른 토스트가 끼어도 한 번만 띄운다
const TOAST_REPEAT_MS = 2000;
const ESTOP_CLICK_MS = 1000;    // 누를 때 보낸 비상 정지의 뒤따르는 click 은 다시 보내지 않는다
// POST 가 실패해도 이 안에 WS 로 보낸 비상 정지가 걸린 것이 status 로 보이면 실패를 띄우지 않는다
// (비상 정지 직후 1 s 는 토크 켜기를 거절하므로 그 전에 본다)
const ESTOP_CONFIRM_MS = 750;
const LINK_CHIPS = {
	connecting: ['warn', '연결 중'],
	open: ['ok', '연결됨'],
	closed: ['danger', '연결 끊김 · 다시 연결 중'],
};
const MODE_CHIPS = {
	idle: ['neutral', '대기 중'],
	mapping: ['info', '지도 작성 중'],
	navigation: ['info', '자율 주행 중'],
	exploring: ['warn', '자동 주행 중'],   // status.mode 가 아니다: 지도 작성 중 자동 주행 맵 생성
};
const MOTOR_CHIPS = {   // 앞에 '바퀴' 는 index.html 에 있다 (휴대폰에서는 줄인다)
	true: ['warn', '토크 켜짐'],
	false: ['neutral', '토크 꺼짐'],
};
const EVENT_TONES = { info: 'info', warn: 'warn', error: 'error' };
const STOP_QUESTIONS = {
	bringup: {
		title: '로봇을 끌까요?',
		message: '센서 · 모터 · 카메라 노드가 모두 꺼집니다. 다시 쓰려면 ‘로봇 켜기’를 누르세요.',
		confirmLabel: '로봇 끄기',
		isDanger: true,
	},
	mapping: {
		title: '지도 작성을 끌까요?',
		message: '저장하지 않은 지도는 사라집니다. 지도가 필요하면 먼저 ‘지도 저장’을 누르세요.',
		confirmLabel: '지도 작성 끄기',
		isDanger: true,
	},
};
// 로봇 끄기는 이 게이트웨이가 켠 팔 토크가 남아 있으면 팔을 먼저 접는다. 못 접으면 409 와 이 글로 시작하는 이유
// shortcut: 서버 글(robot.py stop_process)의 앞부분으로 가른다, 서버가 따로 code 를 주면 그것으로 바꾼다
const ARM_FOLD_REFUSED = '로봇을 끄지 않았습니다';
const FORCE_STOP_QUESTION = {
	title: '팔을 접지 못했습니다. 그래도 끌까요?',
	confirmLabel: '그래도 끄기',
	isDanger: true,
};

const conn = new Connection(toast);
// map: 서버는 화면을 바꿔도 계속 구독 중인 채널은 다시 보내지 않으므로 마지막 지도를 여기 둔다
const state = { info: null, status: null, map: null };
const darkQuery = matchMedia('(prefers-color-scheme: dark)');
const main = document.getElementById('main');
const robotName = document.getElementById('robot-name');
const robotMode = document.getElementById('robot-mode');   // 휴대폰에서 모드 칩 대신 이름 아래에
const linkChip = document.getElementById('chip-link');
const modeChip = document.getElementById('chip-mode');
const motorChip = document.getElementById('chip-motor');
const controlBox = document.getElementById('control');
const themeButton = document.getElementById('theme-toggle');
// 상단 바 + 확인 창 안 (모달 확인 창이 열리면 상단 바는 눌리지 않는다)
const estopButtons = document.querySelectorAll('[data-estop]');
const estopBanner = document.getElementById('estop-banner');
const toastBox = document.getElementById('toasts');
const dialog = document.getElementById('confirm');
const navLinks = document.querySelectorAll('.sidenav a');
const recentToasts = new Map();   // 글 → 띄운 시각 (TOAST_REPEAT_MS 동안만 둔다)
let current = null;   // { page, offs }
const stopping = new Set();   // 끄기 응답을 기다리는 기능 (로봇 끄기는 팔을 접느라 최대 약 12 s)
let estopPressedAt = -Infinity;

// ── 공통 상태 판단 ───────────────────────────────────────

function isMine() {
	const holder = state.status?.control?.holder;
	return Boolean(conn.isOpen && holder && holder.client_id === conn.clientId);
}

/** 조작 권한이 필요한 명령을 지금 못 보내는 이유. 보낼 수 있으면 null. */
function blockReason() {
	if (!conn.isOpen) return '서버에 연결되지 않았습니다';
	if (state.info?.is_read_only) return '보기 전용 화면이라 명령을 보낼 수 없습니다';
	if (isMine()) return null;
	const holder = state.status?.control?.holder;
	if (holder) return `다른 화면(${holder.label ?? '알 수 없음'})이 조작 중입니다`;
	return '먼저 조작 권한을 받으세요';
}

/**
 * 기능(launch) 시작 · 끄기를 지금 못 하는 이유. 할 수 있으면 null.
 * 서버가 최종 판단하고, 여기서는 누르기 전에 이유를 보여 준다 (SPEC §8 의존 규칙).
 */
function processBlock(name, action) {
	const reason = blockReason();
	if (reason) return reason;
	if (state.info?.features?.can_control_processes === false) {
		return '이 로봇에서는 기능을 켜고 끌 수 없습니다';
	}
	const processes = state.status?.processes;
	if (!processes) return '로봇 상태를 기다리는 중입니다';
	const processState = processes[name]?.state;
	if (stopping.has(name) && action === 'stop') {
		return state.status?.teleop_arm?.rest?.state === 'resting'
			? '팔을 접은 뒤 로봇을 끕니다 (최대 약 12초)' : '끄는 중입니다';
	}
	if (processState === 'stopping') return '끄는 중입니다';
	if (action === 'stop') {
		if (!isOn(processes[name])) return '켜져 있지 않습니다';
		// 핵심 노드만 보이고 이 로봇에 프로세스가 없다: 서버가 끌 수 없다 (이유는 detail)
		if (processState === 'external' && processes[name].pid == null) {
			return processes[name].detail || '다른 컴퓨터에서 실행 중이라 끌 수 없습니다';
		}
		if (name === 'bringup' && (isOn(processes.mapping) || isOn(processes.navigation))) {
			return '지도 작성 · 자율 주행을 먼저 끄세요';
		}
		if (name === 'mapping' && processes.save_map?.state === 'running') {
			return '지도를 저장하는 중입니다. 저장이 끝난 뒤 끄세요';
		}
		return null;
	}
	if (isOn(processes[name])) return '이미 켜져 있습니다';
	if (name === 'bringup') return null;
	if (!['running', 'external'].includes(processes.bringup?.state)) return '먼저 로봇을 켜세요';
	if (name === 'mapping' && isOn(processes.navigation)) return '자율 주행을 먼저 끄세요';
	if (name === 'navigation' && isOn(processes.mapping)) return '지도 작성을 먼저 끄세요';
	return null;
}

// ── 공통 명령 ────────────────────────────────────────────

function acquire() {
	return conn.post('/api/control/acquire', {});
}

function release() {
	return conn.post('/api/control/release', {});
}

async function startProcess(name, options = {}) {
	if (name === 'navigation') {
		const isConfirmed = await confirmAction({
			title: '자율 주행을 시작할까요?',
			message: `${options.map} 지도로 Nav2 를 켭니다. 켜지면 먼저 ‘초기 위치’로 `
				+ '로봇이 실제로 있는 곳을 정하세요. 목표를 보내면 로봇이 스스로 움직입니다.',
			confirmLabel: '자율 주행 시작',
		});
		if (!isConfirmed) return null;
	}
	return conn.post('/api/process/start', { name, options });
}

async function stopProcess(name) {
	const question = STOP_QUESTIONS[name];
	if (question && !(await confirmAction(question))) return null;
	const send = async (body) => {
		stopping.add(name);
		refresh();
		try {
			return await conn.post('/api/process/stop', body);
		} finally {
			stopping.delete(name);
			refresh();
		}
	};
	const result = await send({ name });
	const isFoldRefused = name === 'bringup' && result.code === 'conflict'
		&& String(result.message).startsWith(ARM_FOLD_REFUSED);
	if (!isFoldRefused) return result;
	const message = `${result.message}\n\n그래도 끄면 팔을 접지 않고 팔 토크를 끈 뒤 로봇을 끕니다. `
		+ '편 팔과 잡은 물건이 떨어집니다. 팔을 손으로 받치세요.';
	if (!(await confirmAction({ ...FORCE_STOP_QUESTION, message }))) return result;
	return send({ name, force: true });
}

// ── 토스트 · 확인 창 ─────────────────────────────────────

function toast(message, tone = 'info') {
	if (!message) return;
	const now = Date.now();
	for (const [text, at] of recentToasts) {
		if (now - at >= TOAST_REPEAT_MS) recentToasts.delete(text);
	}
	if (recentToasts.has(message)) return;
	recentToasts.set(message, now);
	const item = document.createElement('div');
	item.className = 'toast';
	item.dataset.tone = tone;
	if (tone === 'error') item.setAttribute('role', 'alert');
	item.textContent = message;
	item.addEventListener('click', () => item.remove());
	toastBox.append(item);
	while (toastBox.children.length > TOAST_LIMIT) toastBox.firstElementChild.remove();
	setTimeout(() => item.remove(), tone === 'error' ? 8000 : 4500);
}

/**
 * 위험한 명령 전에 묻는다. window.confirm 은 쓰지 않는다:
 * 열려 있는 동안 타이머가 멈춰 heartbeat 가 끊기고 서버가 조작 권한을 회수한다.
 */
function confirmAction({ title, message, confirmLabel, isDanger = false }) {
	if (dialog.open) return Promise.resolve(false);
	document.getElementById('confirm-title').textContent = title;
	document.getElementById('confirm-message').textContent = message;
	const okButton = document.getElementById('confirm-ok');
	okButton.textContent = confirmLabel;
	okButton.className = isDanger ? 'btn btn-danger-fill' : 'btn btn-primary';
	dialog.returnValue = '';
	dialog.showModal();
	return new Promise((resolve) => {
		const onClose = () => resolve(dialog.returnValue === 'ok');
		dialog.addEventListener('close', onClose, { once: true });
	});
}

// ── 상단 바 ──────────────────────────────────────────────

function setChip(chip, [tone, text]) {
	chip.dataset.tone = tone;
	const label = chip.querySelector('.chip-text');
	if (label.textContent !== text) label.textContent = text;
}

function controlHtml() {
	if (!conn.isOpen) return '';
	if (state.info?.is_read_only) return badge('info', '보기 전용', 'chip');
	if (isMine()) {
		// 자동 주행은 지도 작성 화면을 떠나도 계속된다: 어느 화면에서든 토크를 끄지 않고 멈출 수 있게
		const stop = isExploring(state.status?.exploration)
			? '<button type="button" class="btn btn-small btn-danger" data-action="explore-stop">'
				+ '자동 주행 멈춤</button>'
			: '';
		return badge('accent', '내가 조작 중', 'chip') + stop
			+ '<button type="button" class="btn btn-small" data-action="release">권한 놓기</button>';
	}
	const holder = state.status?.control?.holder;
	if (holder) return badge('neutral', `${holder.label ?? '다른 화면'} 조작 중`, 'chip');
	return '<button type="button" class="btn btn-small btn-primary" data-action="acquire">'
		+ '조작 권한 받기</button>';
}

function renderShell() {
	const status = state.status;
	const name = state.info?.robot_name ?? 'LeKiwi';
	if (robotName.textContent !== name) robotName.textContent = name;
	const modeName = isExploring(status?.exploration) ? 'exploring' : status?.mode;
	const mode = MODE_CHIPS[modeName] ?? ['neutral', '모드 —'];
	setChip(linkChip, LINK_CHIPS[conn.linkState]);
	setChip(modeChip, mode);
	if (robotMode.textContent !== mode[1]) robotMode.textContent = mode[1];
	setChip(motorChip, MOTOR_CHIPS[status?.motor?.ready] ?? ['neutral', '토크 —']);
	putHtml(controlBox, controlHtml());
	// 보기 전용 게이트웨이는 서비스 클라이언트가 없어 비상 정지도 보낼 수 없다: 눌러지는 척하지 않는다
	const isReadOnly = Boolean(state.info?.is_read_only);
	for (const button of estopButtons) {
		if (button.disabled === isReadOnly) continue;
		button.disabled = isReadOnly;
		button.title = isReadOnly ? '보기 전용 화면이라 비상 정지를 보낼 수 없습니다' : '';
	}
	// 비상 정지는 풀릴 때까지 걸려 있다 (서버 status.estop): 어느 화면에서 눌렀든 모두에게 보인다
	const estop = status?.estop;
	estopBanner.hidden = estop?.is_latched !== true;
	if (!estopBanner.hidden) {
		const who = estop.label ? ` (${estop.label})` : '';
		const next = status.mode === 'navigation'   // 감독 도구가 끝났으니 자율 주행을 새로 켜야 한다
			? '자율 주행을 껐다가 다시 켜세요' : '‘조작 켜기’를 누르세요';
		const text = `비상 정지됨${who}. 다시 움직이려면 ${next}`;
		if (estopBanner.textContent !== text) estopBanner.textContent = text;
	}
}

function refresh() {
	renderShell();
	conn.emit('update');
}

// ── 밝기 ─────────────────────────────────────────────────

// 밝기는 늘 data-theme 로 정한다 (index.html 첫 스크립트가 저장값 · 시스템 설정으로 먼저 정한다)
function isDarkTheme() {
	return document.documentElement.dataset.theme === 'dark';
}

/** 사용자가 고른 밝기. 고른 적이 없거나 저장소를 못 쓰면 null (시스템 설정을 따른다). */
function storedTheme() {
	try {
		const theme = localStorage.getItem(THEME_KEY);
		return theme === 'light' || theme === 'dark' ? theme : null;
	} catch {
		return null;   // 사생활 보호 모드 등
	}
}

function themeChanged() {
	themeButton.setAttribute('aria-label', isDarkTheme() ? '밝은 화면으로 바꾸기' : '어두운 화면으로 바꾸기');
	document.dispatchEvent(new Event('themechange'));
}

// ── 화면 전환 ────────────────────────────────────────────

function makeContext(offs) {
	return {
		conn,
		get info() {
			return state.info;
		},
		get status() {
			return state.status;
		},
		/** 마지막으로 받은 WS map 메시지 (없으면 null). */
		get map() {
			return state.map;
		},
		get isConnected() {
			return conn.isOpen;
		},
		/** 이 화면이 열려 있는 동안만 받는다 (화면을 바꾸면 자동으로 끊는다). */
		on(type, callback) {
			offs.push(conn.on(type, callback));
		},
		setChannels: (channels) => conn.subscribe(channels),
		confirm: confirmAction,
		isMine,
		blockReason,
		processBlock,
		acquire,
		release,
		startProcess,
		stopProcess,
	};
}

function route() {
	const name = location.hash.replace(/^#\/?/, '') || 'dashboard';
	const page = PAGES[name];
	if (!page) {
		location.replace('#/dashboard');
		return;
	}
	if (current) {
		try {
			current.page.unmount();
		} catch (error) {
			console.error(error);
		}
		for (const off of current.offs) off();
	}
	for (const link of navLinks) {
		if (link.dataset.route === name) link.setAttribute('aria-current', 'page');
		else link.removeAttribute('aria-current');
	}
	current = { page, offs: [] };
	main.replaceChildren();
	main.scrollTop = 0;
	conn.subscribe(page.channels);
	page.mount(main, makeContext(current.offs));
	document.title = `${page.title} · LeKiwi`;
}

// ── 시작 ─────────────────────────────────────────────────

conn.on('welcome', (message) => {
	state.info = message.info ?? null;
});
conn.on('link', (linkState) => {
	if (linkState !== 'open') {   // 끊긴 동안 값은 믿을 수 없어 비운다
		state.status = null;
		state.map = null;
	}
	refresh();
});
conn.on('status', (message) => {
	// 지도 작성 · 자율 주행을 막 켰으면 들고 있던 지도는 지난번 것이다: 새 지도가 올 때까지 비운다.
	// 다시 연결한 직후(state.status 없음)는 뺀다: 이미 비웠고, 먼저 온 새 지도를 지우면 다시 오지 않는다.
	const hasStarted = (name) => message.processes?.[name]?.state === 'starting'
		&& state.status?.processes?.[name]?.state !== 'starting';
	if (state.status && (hasStarted('mapping') || hasStarted('navigation'))) state.map = null;
	state.status = message;
	refresh();
});
conn.on('map', (message) => {
	state.map = message;
});
conn.on('event', (message) => toast(message.message, EVENT_TONES[message.level] ?? 'info'));

controlBox.addEventListener('click', (event) => {
	const action = event.target.closest('[data-action]')?.dataset.action;
	if (action === 'acquire') acquire();
	if (action === 'release') release();
	if (action === 'explore-stop') conn.post('/api/explore/stop', {});
});

async function emergencyStop() {
	conn.emit('estop');   // 조작 화면이 누르고 있던 키를 바로 놓는다
	const since = state.status?.estop?.since;
	// 열린 WS 로 먼저 보낸다: 전체 연결 수가 꽉 차 새 HTTP 연결이 막혀도 간다.
	// POST 도 늘 보낸다: WS 가 죽었거나 다시 연결 중일 때 대신 간다. 알림은 서버 이벤트 하나
	const isSentByWs = conn.send({ type: 'estop' });
	const posted = conn.post('/api/estop', {}, isSentByWs);
	if (dialog.open) dialog.close();   // returnValue '' 그대로: 묻던 명령은 취소된다
	const result = await posted;
	if (result.ok || !isSentByWs) return;   // 실패는 WS 로 못 보냈으면 post 가 이미 띄웠다
	await new Promise((resolve) => setTimeout(resolve, ESTOP_CONFIRM_MS));
	const estop = state.status?.estop;
	if (estop?.is_latched !== true || estop.since === since) toast(result.message, 'error');
}

for (const button of estopButtons) {
	// 누르는 순간 보낸다. click 은 손가락이 미끄러지거나 다른 손가락이 화면에 있으면 오지 않는다
	button.addEventListener('pointerdown', (event) => {
		if (event.button !== 0 || button.disabled) return;   // disabled 버튼에도 pointerdown 은 온다
		estopPressedAt = performance.now();
		emergencyStop();
	});
	// 키보드 · 보조 기기는 click 만 보낸다
	button.addEventListener('click', () => {
		if (performance.now() - estopPressedAt > ESTOP_CLICK_MS) emergencyStop();
	});
}
themeButton.addEventListener('click', () => {
	const theme = isDarkTheme() ? 'light' : 'dark';
	document.documentElement.dataset.theme = theme;
	try {
		localStorage.setItem(THEME_KEY, theme);
	} catch {
		// 저장하지 못해도 지금 화면은 바뀐다
	}
	themeChanged();
});
darkQuery.addEventListener('change', () => {
	if (!storedTheme()) {   // 사용자가 고른 밝기가 없을 때만 시스템을 따른다
		document.documentElement.dataset.theme = darkQuery.matches ? 'dark' : 'light';
	}
	themeChanged();
});
window.addEventListener('hashchange', route);

themeChanged();
renderShell();
route();
conn.connect();
