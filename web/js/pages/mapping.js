// 지도 작성: 시작 · 끄기 · 실시간 지도(지도 + 스캔 + 로봇) · 자동 주행 맵 생성 · 조작 패드 축소판 · 지도 저장
// (SPEC §11-3, 자동 주행 맵 생성 명세 §4)

import {
	DASH, badge, duration, esc, fixed, isExploring, isNum, isOn, processBadge, put, putHtml,
	readout, refs,
} from '../format.js';
import { createTeleop } from '../teleop.js';
import { createMapView } from '../widgets/map_view.js';

export const title = '지도 작성';
export const channels = ['map', 'scan'];

const EXPLORE_STATES = {   // status.exploration.state
	idle: ['neutral', '대기'],
	starting: ['warn', '시작하는 중'],
	exploring: ['info', '탐색 중'],
	waiting: ['warn', '기다리는 중'],
	saving: ['info', '지도 저장 중'],
	done: ['ok', '끝남'],
	stopped: ['warn', '멈춤'],
};
const EXPLORE_QUESTION = {
	title: '자동 주행 맵 생성을 시작할까요?',
	message: [   // 문장마다 줄을 바꿔 보인다 (dialog p 는 pre-line)
		'로봇이 스스로 움직이며 방을 돌아다닙니다.',
		'라이다는 바닥 가까이(약 2.5 cm)만 보고 바퀴 쪽 세 방향은 보지 못합니다.',
		'현관 · 베란다 · 계단처럼 바닥이 낮아지는 곳과 유리 · 거울 · 검은 가구는 막아 두세요.',
		'소파 · 침대 · 책상 아래 틈, 앞으로 약 10 cm 튀어나온 팔(바닥에서 약 16~30 cm 높이, 라이다가 못 본다)에 '
			+ '걸릴 의자 · 책상 가장자리, 바닥의 케이블을 치우세요.',
		'팔은 접은 자세로 두고, 화면을 켠 채(꺼지면 멈춥니다) 비상 정지를 누를 수 있게 지켜보세요.',
	].join('\n'),
	confirmLabel: '자동 주행 시작',
};

let parts = [];

function saveBlock(ctx, mapping) {
	const reason = ctx.blockReason();
	if (reason) return reason;
	if (!['running', 'external'].includes(mapping?.state)) return '지도 작성이 실행 중일 때 저장할 수 있습니다';
	return null;
}

/** 저장 중에는 버튼 글자가 알려 주므로 끝난 결과만. */
function saveResult(save) {
	if (save?.state === 'failed') {
		return badge('danger', save.detail ? `저장하지 못했습니다: ${save.detail}` : '저장하지 못했습니다');
	}
	if (save?.state !== 'succeeded') return '';
	return `${badge('ok', '저장했습니다')} <span class="mono">${esc(save.map ?? '')}</span>`
		+ '<br>‘자율 주행’ 화면에서 이 지도를 고를 수 있습니다.';
}

/** 자동 주행 맵 생성을 지금 시작하지 못하는 이유. 시작할 수 있으면 null (서버가 최종 판단한다). */
function exploreBlock(ctx, mapping, run) {
	const reason = ctx.blockReason();
	if (reason) return reason;
	if (!['running', 'external'].includes(mapping?.state)) return '지도 작성이 실행 중일 때 시작할 수 있습니다';
	if (run?.state === 'saving') return '지도를 저장하는 중입니다';
	const teleop = ctx.status?.teleop;
	if (!teleop?.is_allowed) return teleop?.blocked_reason ?? '지금은 조작할 수 없습니다';
	// 이 권한으로 켠 토크여야 권한이 끝날 때 서버가 토크를 끈다
	const motor = ctx.status?.motor;
	if (motor?.ready !== true || motor.is_powered_by_us !== true) return '먼저 ‘조작 켜기’를 누르세요';
	if (!ctx.map) return '지도를 기다리는 중입니다';
	return null;
}

/** 끝난 (또는 기다리는) 이유와 자동 저장한 지도. */
function exploreResult(run) {
	const lines = [];
	if (run?.reason) lines.push(esc(run.reason));
	if (run?.saved_map) lines.push(`저장한 지도 <span class="mono">${esc(run.saved_map)}</span>`);
	return lines.join('<br>');
}

export function mount(el, ctx) {
	el.innerHTML = `
		<header class="page-head"><h1>지도 작성</h1><span data-k="badge"></span></header>
		<div class="split">
			<div class="well map-well" data-k="map"></div>
			<div class="split-side">
				<section class="panel" aria-labelledby="mapping-title">
					<h2 id="mapping-title">지도 작성</h2>
					<p class="guide">
						천천히, 특히 회전을 천천히 움직이세요. 한 바퀴 돌아 처음 자리로 돌아오면 지도가 맞춰집니다.
					</p>
					<div class="actions">
						<button type="button" class="btn btn-primary" data-k="start">
							지도 작성 시작
						</button>
						<button type="button" class="btn btn-danger" data-k="stop" hidden>
							지도 작성 끄기
						</button>
						<button type="button" class="btn" data-k="save">지도 저장</button>
					</div>
					<p class="reason" data-k="reason"></p>
					<p class="reason" data-k="saveReason"></p>
					<p class="save-result" data-k="saveResult" role="status"></p>
				</section>
				<section class="panel" aria-labelledby="explore-title">
					<div class="panel-head">
						<h2 id="explore-title">자동 주행 맵 생성</h2><span data-k="exploreBadge"></span>
					</div>
					<p class="guide">
						로봇이 라이다로 장애물을 피하며 방을 돌아다니고, 지도가 거의 다 되면 멈춰서 저장합니다.
					</p>
					<div class="actions">
						<button type="button" class="btn btn-primary" data-k="exploreStart">
							자동 주행 맵 생성
						</button>
						<button type="button" class="btn btn-danger" data-k="exploreStop" hidden>
							자동 주행 멈춤
						</button>
					</div>
					<p class="reason" data-k="exploreReason"></p>
					<dl class="readouts readouts-3 explore-stats" data-k="exploreStats" hidden>
						${readout('exploreTime', '걸린 시간')}
						${readout('exploreArea', '알아낸 넓이', 'm²')}
						${readout('exploreFrontiers', '남은 프런티어')}
					</dl>
					<p class="save-result" data-k="exploreResult" role="status"></p>
				</section>
				<section class="panel" data-k="teleop" aria-label="조작"></section>
			</div>
		</div>`;
	const k = refs(el);
	const map = createMapView(k.map);
	parts = [map, createTeleop(k.teleop, ctx, true)];
	k.start.addEventListener('click', () => ctx.startProcess('mapping'));
	k.stop.addEventListener('click', () => ctx.stopProcess('mapping'));
	k.save.addEventListener('click', () => ctx.conn.post('/api/map/save', {}));
	// window.confirm 은 쓰지 않는다: 열려 있는 동안 heartbeat 가 멈춰 권한이 회수된다 (app.js)
	k.exploreStart.addEventListener('click', async () => {
		if (!(await ctx.confirm(EXPLORE_QUESTION))) return;
		const result = await ctx.conn.post('/api/explore/start', {});
		// 다음 status(0.25 s 마다)를 기다리지 않고 '시작하는 중'으로 본다: 그사이 누른 Space · 정지도
		// 멈춤을 보낸다 (teleop.js). 다음 status 가 이 값을 통째로 바꾼다
		if (result.ok && ctx.status) ctx.status.exploration = { state: 'starting' };
	});
	k.exploreStop.addEventListener('click', () => ctx.conn.post('/api/explore/stop', {}));
	ctx.on('map', render);
	ctx.on('scan', (message) => map.setScan(message));

	function render() {
		const processes = ctx.status?.processes;
		const mapping = processes?.mapping;
		const isActive = isOn(mapping);
		const save = processes?.save_map;
		const isSaving = save?.state === 'running';
		// 지도 작성 중에는 이번에 저장한 결과만 (지난번 '저장했습니다' 를 지금 지도로 오해하지 않게)
		const isOldSave = isActive && isNum(save?.since) && isNum(mapping?.since)
			&& save.since < mapping.since;
		putHtml(k.badge, processBadge(mapping));
		k.start.hidden = isActive;
		k.stop.hidden = !isActive;
		const reason = ctx.processBlock('mapping', isActive ? 'stop' : 'start');
		(isActive ? k.stop : k.start).disabled = Boolean(reason);
		put(k.reason, reason ?? '');
		const saveReason = isActive && !isSaving ? saveBlock(ctx, mapping) : null;
		k.save.hidden = !isActive;
		k.save.disabled = isSaving || Boolean(saveReason);
		put(k.save, isSaving ? '저장하는 중…' : '지도 저장');
		put(k.saveReason, saveReason && saveReason !== reason ? saveReason : '');
		putHtml(k.saveResult, isOldSave ? '' : saveResult(save));

		// 자동 주행 맵 생성: 끝난 결과도 같은 규칙 (움직이는 중이면 늘 보인다: 멈춤 버튼이 숨지 않게)
		const exploration = ctx.status?.exploration;
		const isOldRun = isActive && !isExploring(exploration) && isNum(exploration?.started_at)
			&& isNum(mapping?.since) && exploration.started_at < mapping.since;
		const run = isOldRun ? null : exploration;
		const isRunning = isExploring(run);
		const [tone, text] = EXPLORE_STATES[run?.state ?? 'idle'] ?? ['neutral', DASH];
		const isUnsaved = run?.state === 'done' && !run.saved_map;   // 끝났지만 자동 저장을 못 했다
		putHtml(k.exploreBadge, badge(isUnsaved ? 'warn' : tone, text));
		k.exploreStart.hidden = isRunning;
		k.exploreStop.hidden = !isRunning;
		const exploreReason = isRunning ? ctx.blockReason() : exploreBlock(ctx, mapping, run);
		(isRunning ? k.exploreStop : k.exploreStart).disabled = Boolean(exploreReason);
		put(k.exploreReason, exploreReason ?? '');
		k.exploreStats.hidden = !run || run.state === 'idle';
		put(k.exploreTime, duration(run?.elapsed));
		put(k.exploreArea, fixed(run?.explored_area, 1));
		put(k.exploreFrontiers, isNum(run?.frontiers) ? String(run.frontiers) : DASH);
		putHtml(k.exploreResult, exploreResult(run));

		// 지도 작성이 꺼지면 그 지도는 사라진다: 서버에 남은 마지막 지도 · 자세를 살아 있는 것처럼 보이지 않는다
		map.setMap(isActive ? ctx.map : null);
		map.setRobot(isActive ? ctx.status?.pose?.map : null);
		map.setGoal(isActive && isRunning ? run.target : null);
		map.setPlan(isActive && isRunning ? { points: run.path } : null);
		map.setEmptyText(isActive ? '지도를 기다리는 중입니다' : '지도 작성을 시작하면 여기에 지도가 그려집니다');
	}

	ctx.on('update', render);
	render();
}

export function unmount() {
	for (const part of parts) part.destroy();
	parts = [];
}
