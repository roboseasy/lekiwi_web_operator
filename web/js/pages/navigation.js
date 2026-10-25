// 자율 주행: 지도 고르기 · 시작 · 끄기 · 지도 위 초기 위치 · 목표 · 취소 · 진행 (SPEC §11-4)
// + 오른쪽 칸의 작은 카메라 두 대 · 속도 · CPU · RAM

import {
	DASH, badge, duration, esc, fixed, isNum, isOn, processBadge, put, putHtml, readout, refs,
} from '../format.js';
import { createCameraView } from '../widgets/camera_view.js';
import { createMapView } from '../widgets/map_view.js';

export const title = '자율 주행';
// 카메라는 작게 보여도 조작 화면과 같은 프레임을 받는다 (Pi CPU, README 9장)
export const channels = ['map', 'scan', 'plan', 'camera/front', 'camera/wrist'];

const GIB = 2 ** 30;   // free -h · htop 과 같은 GB

const NAV_STATES = {
	unavailable: ['neutral', 'Nav2 없음'],
	idle: ['neutral', '목표 없음'],
	active: ['info', '목표로 가는 중'],
	succeeded: ['ok', '도착'],
	aborted: ['danger', '실패'],
	canceled: ['neutral', '취소됨'],
	rejected: ['danger', '거절됨'],
};

const DATE_FORMAT = { dateStyle: 'short', timeStyle: 'short' };

let view = null;
let cameras = [];
let selected = null;   // 고른 지도 파일 이름 (화면을 다시 열어도 기억)
let lastSaved = null;  // 지난번 목록의 마지막 저장 지도 '이름@수정 시각' (새로 저장하면 그것을 고른다)

function mapItem(item) {
	const date = isNum(item.modified)
		? new Date(item.modified * 1000).toLocaleString('ko-KR', DATE_FORMAT)
		: DASH;
	const size = isNum(item.width) && isNum(item.height) && isNum(item.resolution)
		? `${fixed(item.width * item.resolution, 1)} × ${fixed(item.height * item.resolution, 1)} m`
		: DASH;
	return `
		<li><button type="button" class="pick" data-name="${esc(item.name)}"
			aria-pressed="${item.name === selected}">
			<span class="pick-name">${esc(item.name)}</span>
			${item.is_last ? badge('accent', '마지막 저장') : '<span></span>'}
			<span class="pick-meta">${date} · ${size}</span>
		</button></li>`;
}

export function mount(el, ctx) {
	el.innerHTML = `
		<header class="page-head"><h1>자율 주행</h1><span data-k="badge"></span></header>
		<p class="feature-off" data-k="featureOff" hidden>
			Nav2 메시지(nav2_msgs)를 불러오지 못해 이 로봇에서는 자율 주행을 쓸 수 없습니다.
		</p>
		<div class="split">
			<div class="split-main">
				<div class="well map-well" data-k="map"></div>
				<p class="reason" data-k="toolReason"></p>
				<p class="guide">
					시작하면 먼저 ‘초기 위치’로 로봇이 실제로 있는 곳을 누른 채 보는 방향으로 끄세요.
					그다음 ‘목표’로 갈 곳을 같은 방법으로 정합니다.
				</p>
			</div>
			<div class="split-side">
				<!-- 지도 칸이 먼저: 꺼져 있을 때 지도가 많아도 '시작'이 한 화면에, 켜면 이 칸은 한 줄로 준다 -->
				<section class="panel" aria-labelledby="maps-title">
					<div class="panel-head">
						<h2 id="maps-title">지도</h2>
						<button type="button" class="btn btn-small btn-ghost" data-k="reload">
							목록 새로 고침
						</button>
					</div>
					<p class="guide" data-k="inUse"></p>
					<ul class="pick-list" data-k="maps"></ul>
					<img class="map-preview" data-k="preview" alt="고른 지도 미리보기" hidden>
					<div class="actions">
						<button type="button" class="btn btn-primary" data-k="start">
							자율 주행 시작
						</button>
						<button type="button" class="btn btn-danger" data-k="stop" hidden>
							자율 주행 끄기
						</button>
					</div>
					<p class="reason" data-k="reason"></p>
				</section>
				<section class="panel" aria-labelledby="cameras-title">
					<h2 id="cameras-title">카메라</h2>
					<div class="nav-cameras">
						<div class="well camera-well" data-k="front"></div>
						<div class="well camera-well" data-k="wrist"></div>
					</div>
				</section>
				<section class="panel" aria-labelledby="progress-title">
					<div class="panel-head">
						<h2 id="progress-title">진행</h2><span data-k="navBadge"></span>
					</div>
					<dl class="readouts readouts-3">
						${readout('distance', '남은 거리', 'm')}
						${readout('eta', '예상 시간 (어림)')}
						${readout('recoveries', '복구 횟수')}
						${readout('localized', '위치 추정')}
						${readout('torque', '바퀴 토크')}
					</dl>
					<p class="guide nav-message" data-k="navMessage"></p>
					<div class="actions panel-actions">
						<button type="button" class="btn btn-danger" data-k="cancel" hidden>
							목표 취소
						</button>
					</div>
					<p class="reason" data-k="cancelReason"></p>
					<!-- 서버가 status.navigation.speed_percent 를 주지 않으면 (그 기능이 없는 게이트웨이) 숨긴다 -->
					<div data-k="speed" hidden>
						<h3 class="sub" id="speed-title">속도 <b data-k="speedNow"></b></h3>
						<div class="seg" role="group" aria-labelledby="speed-title" data-k="speeds">
							<button type="button" data-percent="25">25 %</button>
							<button type="button" data-percent="50">50 %</button>
							<button type="button" data-percent="75">75 %</button>
							<button type="button" data-percent="100">100 %</button>
						</div>
						<p class="reason" data-k="speedReason"></p>
						<p class="hint">Nav2 최고 속도의 비율입니다. 누르면 가는 중인 목표에도 바로 적용됩니다.</p>
					</div>
					<p class="hint">
						비상 정지를 누르거나 조작 권한을 놓거나 이 브라우저 창을 닫거나 연결이 끊기면
						감독 도구가 바퀴 토크를 끕니다. 다시 움직이려면 자율 주행을 껐다가 다시 켜세요.
					</p>
				</section>
				<section class="panel" aria-labelledby="system-title">
					<h2 id="system-title">CPU · RAM</h2>
					<dl class="readouts nav-system">
						${readout('cpu', 'CPU', '%')}
						${readout('ram', 'RAM', '%')}
						${readout('ramBytes', 'RAM 사용 / 전체', 'GB')}
					</dl>
				</section>
			</div>
		</div>`;
	const k = refs(el);
	let maps = null;       // 아직 못 읽었으면 null
	let lastPlan = null;
	let isPreviewBroken = false;   // 미리보기를 못 읽었으면 깨진 그림 대신 숨긴다
	view = createMapView(k.map, (tool, pose) => {
		ctx.conn.post(tool === 'initial' ? '/api/nav/initial_pose' : '/api/nav/goal', pose);
	});
	const front = createCameraView(k.front, '전방 카메라');
	const wrist = createCameraView(k.wrist, '손목 카메라');
	cameras = [front, wrist];

	async function loadMaps() {
		const data = await ctx.conn.get('/api/maps');
		if (!data) return;
		maps = Array.isArray(data.maps) ? data.maps : [];
		k.maps.dataset.dir = data.map_dir ?? '';
		const last = maps.find((item) => item.is_last);
		const saved = last ? `${last.name}@${last.modified}` : null;
		if (!maps.some((item) => item.name === selected) || (saved && saved !== lastSaved)) {
			selected = (last ?? maps[0])?.name ?? null;
		}
		lastSaved = saved;
		render();
	}

	function renderMaps(isLocked) {
		// 실행 중에는 지도를 바꿀 수 없으므로 목록 · 미리보기를 접고 '지금 쓰는 지도' 만 둔다
		k.maps.hidden = isLocked;
		k.reload.hidden = isLocked;
		if (maps === null) {
			putHtml(k.maps, '<li class="is-calm">연결되면 저장한 지도 목록을 읽습니다</li>');
		} else if (!maps.length) {
			putHtml(k.maps, '<li class="is-calm">저장한 지도가 없습니다. ‘지도 작성’ 화면에서 먼저 지도를 만드세요.'
				+ ` (${esc(k.maps.dataset.dir)})</li>`);
		} else {
			putHtml(k.maps, maps.map(mapItem).join(''));
		}
		const src = selected ? `/api/maps/preview?name=${encodeURIComponent(selected)}` : '';
		if (k.preview.dataset.src !== src) {
			k.preview.dataset.src = src;
			isPreviewBroken = false;
			if (src) k.preview.src = src;
		}
		k.preview.hidden = isLocked || !src || isPreviewBroken;
	}

	function render() {
		const status = ctx.status;
		const nav = status?.navigation;
		const process = status?.processes?.navigation;
		const isActive = isOn(process);
		const isRunning = ['running', 'external'].includes(process?.state);
		const isGoalActive = nav?.state === 'active';
		const hasNavigation = ctx.info?.features?.has_navigation !== false;
		k.featureOff.hidden = hasNavigation;
		putHtml(k.badge, processBadge(process));
		put(k.inUse, isActive && process?.map ? `지금 쓰는 지도: ${process.map}` : '');
		renderMaps(isActive);
		k.reload.disabled = !ctx.isConnected;

		k.start.hidden = isActive;
		k.stop.hidden = !isActive;
		let reason = ctx.processBlock('navigation', isActive ? 'stop' : 'start');
		if (!reason && !isActive && !hasNavigation) reason = '이 로봇에서는 자율 주행을 쓸 수 없습니다';
		if (!reason && !isActive && !selected) reason = '먼저 지도를 고르세요';
		(isActive ? k.stop : k.start).disabled = Boolean(reason);
		put(k.reason, reason ?? '');

		const toolReason = ctx.blockReason()
			?? (isRunning ? null : '초기 위치 · 목표는 자율 주행이 실행 중일 때 정할 수 있습니다');
		view.setToolsEnabled(!toolReason);
		put(k.toolReason, toolReason ?? '');

		const [tone, text] = NAV_STATES[nav?.state] ?? ['neutral', DASH];
		putHtml(k.navBadge, badge(tone, text));
		put(k.distance, fixed(nav?.distance_remaining, 2));
		put(k.eta, duration(nav?.eta));
		put(k.recoveries, isNum(nav?.recoveries) ? String(nav.recoveries) : DASH);
		// 끈 뒤에도 TF 가 잠시 남아 '잡힘' 으로 보이므로 실행 중일 때만
		put(k.localized, nav && isRunning ? (nav.is_localized ? '잡힘' : '아직') : DASH);
		put(k.torque, { true: '켜짐', false: '꺼짐' }[status?.motor?.ready] ?? DASH);
		put(k.navMessage, nav?.message ?? '');
		k.cancel.hidden = !isGoalActive;
		k.cancel.disabled = !ctx.isConnected;
		put(k.cancelReason, isGoalActive && !ctx.isConnected ? '서버에 연결되지 않았습니다' : '');
		const speed = nav?.speed_percent;
		const speedReason = ctx.blockReason();
		k.speed.hidden = !isNum(speed);
		// 다른 화면 · API 가 25 · 50 · 75 · 100 밖의 값(10..100)을 정했어도 지금 값이 보이게
		put(k.speedNow, isNum(speed) ? `${speed} %` : '');
		for (const button of k.speeds.children) {
			button.setAttribute('aria-pressed', String(Number(button.dataset.percent) === speed));
			button.disabled = Boolean(speedReason);
		}
		put(k.speedReason, speedReason ?? '');

		const system = status?.system;
		const used = system?.memory_used;
		const total = system?.memory_total;
		put(k.cpu, fixed(system?.cpu_percent, 0));
		put(k.ram, fixed(system?.memory_percent, 0));
		put(k.ramBytes, isNum(used) && isNum(total)
			? `${fixed(used / GIB, 1)} / ${fixed(total / GIB, 1)}` : DASH);

		// 자율 주행이 꺼져 있으면 서버에 남은 지난 지도 · 자세를 보이지 않는다.
		// ctx.map 은 app.js 가 연결이 끊기거나 새로 켤 때 비운다 (게이트웨이를 다시 켠 뒤의 지난 지도 방지).
		view.setMap(isActive ? ctx.map : null);
		view.setRobot(isActive ? status?.pose?.map : null);
		view.setGoal(isGoalActive ? nav.goal : null);
		view.setPlan(isGoalActive ? lastPlan : null);
		view.setEmptyText(isActive ? '지도를 기다리는 중입니다' : '자율 주행을 시작하면 여기에 지도가 보입니다');
	}

	k.maps.addEventListener('click', (event) => {
		const button = event.target.closest('[data-name]');
		if (!button) return;
		selected = button.dataset.name;
		render();
	});
	k.preview.addEventListener('error', () => {
		isPreviewBroken = true;
		k.preview.hidden = true;
	});
	k.reload.addEventListener('click', loadMaps);
	k.start.addEventListener('click', () => ctx.startProcess('navigation', { map: selected }));
	k.stop.addEventListener('click', () => ctx.stopProcess('navigation'));
	k.cancel.addEventListener('click', () => ctx.conn.post('/api/nav/cancel', {}));
	k.speeds.addEventListener('click', (event) => {
		const button = event.target.closest('[data-percent]');
		if (button) ctx.conn.post('/api/nav/speed', { percent: Number(button.dataset.percent) });
	});
	ctx.on('camera', ({ camera, blob }) => (camera === 'front' ? front : wrist).setFrame(blob));
	ctx.on('map', render);
	ctx.on('scan', (message) => view.setScan(message));
	ctx.on('plan', (message) => {
		lastPlan = message;
		render();
	});
	ctx.on('welcome', loadMaps);   // 다시 연결되면 목록도 다시 읽는다
	ctx.on('update', render);
	render();
	if (ctx.isConnected) loadMaps();
}

export function unmount() {
	view?.destroy();
	view = null;
	for (const camera of cameras) camera.destroy();
	cameras = [];
}
