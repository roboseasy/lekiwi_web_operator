// 키보드 · 화면 패드로 베이스를 움직이는 조작 묶음. 조작 화면과 지도 작성 화면이 함께 쓴다 (SPEC §11).
// 안전: 키를 모두 떼면 바로 0, 창이 포커스를 잃거나 숨거나 떠나면 모든 키를 놓고 0 을 보낸다.
// 키는 event.code 로 읽는다 (한글 입력 상태에서도 KeyW 는 KeyW 다).
// 자동 주행 맵 생성 중: 0 입력은 서버가 무시하므로 Space · 패드 '정지' 는 자동 주행을 멈추는 요청을 보낸다.
// 패드 방향 버튼은 막고, 키보드 방향 키는 그대로 보낸다 (서버가 자동 주행을 끝내고 그 키는 버린다).

import { DASH, degrees, fixed, isExploring, put, refs } from './format.js';

const SEND_MS = 100;
const AXES = {   // event.code → [x 앞 +, y 왼쪽 +, yaw 반시계 +]
	KeyW: [1, 0, 0],
	KeyS: [-1, 0, 0],
	KeyA: [0, 1, 0],
	KeyD: [0, -1, 0],
	KeyQ: [0, 0, 1],
	KeyE: [0, 0, -1],
};
const PAD = [   // [event.code, 글쇠, 짧은 뜻, 읽어 주는 뜻] 키보드 Q W E / A S D 배치 그대로
	['KeyQ', 'Q', '↺ 왼쪽 회전', '왼쪽으로 돌기'],
	['KeyW', 'W', '↑ 앞', '앞으로'],
	['KeyE', 'E', '↻ 오른쪽 회전', '오른쪽으로 돌기'],
	['KeyA', 'A', '← 왼쪽', '왼쪽으로'],
	['KeyS', 'S', '↓ 뒤', '뒤로'],
	['KeyD', 'D', '→ 오른쪽', '오른쪽으로'],
];
// 글자를 치는 칸에 포커스가 있으면 키를 조작으로 쓰지 않는다 (슬라이더는 괜찮다)
const TYPING = 'textarea, select, [contenteditable], '
	+ 'input:not([type="range"], [type="checkbox"], [type="radio"])';
// '조작 켜기' 는 바퀴 토크만 켠다 (팔 토크는 텔레오퍼레이션 묶음의 '팔 시작')
const POWER_ON_QUESTION = {
	title: '조작을 켤까요?',
	message: '바퀴 토크를 켭니다. 바퀴가 움직입니다. 주변에 사람 · 케이블이 없고, 처음에는 바퀴를 띄웠는지 확인하세요.',
	confirmLabel: '조작 켜기',
};
const EXPLORING_NOTICE = '자동 주행 중입니다. ‘정지’(Space)를 누르면 멈춥니다. '
	+ '키보드 이동 키를 누르면 자동 주행이 끝나고 직접 조작합니다';
const KEY_GUIDE = `
	<dl class="keys">
		<div><dt><kbd>W</kbd><kbd>S</kbd></dt><dd>앞 · 뒤</dd></div>
		<div><dt><kbd>A</kbd><kbd>D</kbd></dt><dd>왼쪽 · 오른쪽</dd></div>
		<div><dt><kbd>Q</kbd><kbd>E</kbd></dt><dd>제자리 회전</dd></div>
		<div><dt><kbd>Space</kbd></dt><dd>정지</dd></div>
		<div><dt><kbd>R</kbd><kbd>F</kbd></dt><dd>속도 단계 ±1</dd></div>
		<div><dt><kbd>Esc</kbd></dt><dd>조작 끄기 (바퀴 토크)</dd></div>
	</dl>`;
const KEY_GUIDE_SHORT = '<p class="keys-short">'
	+ '<kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> 이동 · <kbd>Q</kbd><kbd>E</kbd> 회전 · '
	+ '<kbd>Space</kbd> 정지 · <kbd>Esc</kbd> 조작 끄기</p>';

function template(isCompact) {
	const keys = PAD.map(([code, key, hint, label]) => `
		<button type="button" tabindex="-1" data-code="${code}" aria-label="${label} (${key})">
			<span class="pad-key">${key}</span><span class="pad-hint">${hint}</span>
		</button>`).join('');
	return `
		<div class="actions">
			<button type="button" class="btn" data-k="control"></button>
			<button type="button" class="btn" data-k="power"></button>
		</div>
		<p class="notice" data-k="reason" role="status"></p>
		<div class="pad" data-k="pad" role="group" aria-label="조작 패드. 누르고 있는 동안 움직입니다">
			${keys}
			<button type="button" tabindex="-1" class="pad-stop" data-code="Space"
				aria-label="정지 (Space)">
				<span class="pad-key">정지</span><span class="pad-hint">Space</span>
			</button>
		</div>
		<div class="level">
			<label for="teleop-level">속도 단계 <b data-k="levelNo"></b></label>
			<output for="teleop-level" data-k="levelText"></output>
			<input id="teleop-level" type="range" min="0" max="0" step="1" data-k="level">
		</div>
		${isCompact ? KEY_GUIDE_SHORT : KEY_GUIDE}`;
}

/**
 * root 안에 조작 묶음을 그리고 키보드 · 패드 입력을 teleop 메시지로 보낸다.
 *
 * @param {HTMLElement} root 그릴 곳.
 * @param {object} ctx app.js 의 화면 문맥.
 * @param {boolean} isCompact 지도 작성 화면용 축소판.
 * @param {string|null} pad teleop 메시지의 pad ('drive' = 조작 화면 상한). null 이면 보내지 않는다 (서버 기본
 *     'mapping', 낮은 상한).
 * @returns {{destroy: () => void}}
 */
export function createTeleop(root, ctx, isCompact = false, pad = null) {
	root.classList.add('teleop');
	root.classList.toggle('is-compact', isCompact);
	root.innerHTML = template(isCompact);
	const k = refs(root);
	const keys = new Set();        // 누르고 있는 event.code
	const pointers = new Map();    // pointerId → event.code (여러 손가락 동시)
	let level = 0;
	let isLevelSet = false;        // info.start_level 로 한 번 맞췄는지
	let isPowerBusy = false;       // 토크 켜기 응답(최대 약 10 s)을 기다리는 중
	let wasExploring = false;

	// 확인 창이 열리면 화면이 막혀 Space · 패드를 못 쓰므로 누르고 있던 키도 놓는다 (render 에서).
	// 비상 정지가 걸려 있어도 (어느 화면에서 눌렀든) 놓는다: '조작 켜기'로 풀 때까지 다시 누르지 못한다.
	const canDrive = () => ctx.isMine() && ctx.status?.teleop?.is_allowed === true
		&& ctx.status?.estop?.is_latched !== true && !document.querySelector('dialog[open]');
	const isRunExploring = () => isExploring(ctx.status?.exploration);
	// 자동 주행 중 패드 방향 버튼은 막는다: 화면을 잘못 건드려 자동 주행이 끝나지 않게
	const canPad = () => canDrive() && !isRunExploring();

	/** Space · 패드 '정지': 자동 주행 중이면 멈춘다 (권한을 가진 화면만, 서버가 다시 확인한다). */
	function stopExploration() {
		if (ctx.isMine() && isRunExploring()) ctx.conn.post('/api/explore/stop', {});
	}

	function held() {
		return new Set([...keys, ...pointers.values()]);
	}

	function send() {
		if (!ctx.isMine()) return;
		let x = 0;
		let y = 0;
		let yaw = 0;
		for (const code of held()) {
			x += AXES[code][0];
			y += AXES[code][1];
			yaw += AXES[code][2];
		}
		ctx.conn.send({ type: 'teleop', x, y, yaw, level, ...(pad ? { pad } : {}) });
	}

	function changed() {
		const codes = held();
		for (const button of k.pad.querySelectorAll('[data-code]')) {
			button.classList.toggle('is-held', codes.has(button.dataset.code));
		}
		send();   // 다음 주기를 기다리지 않는다. 특히 모두 떼면 바로 0
	}

	function releaseAll() {
		keys.clear();
		pointers.clear();
		changed();
	}

	/** 이 패드의 {max_level, levels} (status.teleop.pads[pad]). pad 가 없으면 기본 패드 값 (status.teleop). */
	const caps = () => (pad ? ctx.status?.teleop?.pads?.[pad] : ctx.status?.teleop);

	function maxLevel() {
		const value = caps()?.max_level;
		if (Number.isInteger(value)) return value;
		return Math.max((ctx.info?.speed_levels?.length ?? 1) - 1, 0);
	}

	function paintLevel() {
		const pair = (caps()?.levels ?? ctx.info?.speed_levels)?.[level];
		put(k.levelNo, String(level));
		put(k.levelText, pair ? `${fixed(pair[0], 2)} m/s · ${degrees(pair[1], 0)}°/s` : DASH);
	}

	function setLevel(next) {
		level = Math.min(Math.max(next, 0), maxLevel());
		k.level.value = String(level);
		paintLevel();
		send();
	}

	async function setPower(isEnabled) {
		if (isEnabled) {
			if (isPowerBusy || !(await ctx.confirm(POWER_ON_QUESTION))) return;
			isPowerBusy = true;
			render();
		}
		// 끄기는 켜기 응답을 기다리는 중에도 바로 보낸다
		await ctx.conn.post('/api/motor/power', { enabled: isEnabled });
		if (isEnabled) {
			isPowerBusy = false;
			render();
		}
	}

	function reasonText(blocked, teleop, isReady) {
		if (blocked) return blocked;
		if (!teleop) return '로봇 상태를 기다리는 중입니다';
		if (!teleop.is_allowed) return teleop.blocked_reason ?? '지금은 조작할 수 없습니다';
		if (!isReady) return '‘조작 켜기’를 눌러야 바퀴가 움직입니다';
		if (isRunExploring()) return EXPLORING_NOTICE;
		return '';
	}

	function render() {
		const teleop = ctx.status?.teleop;
		const isMine = ctx.isMine();
		const isReady = ctx.status?.motor?.ready === true;
		const blocked = ctx.blockReason();
		const isExploringNow = isRunExploring();
		if (!canDrive() && (keys.size || pointers.size)) releaseAll();   // 권한 · 허용이 사라지면 누름을 버린다
		// 키를 눌러 자동 주행을 멈췄으면 그 키도 놓는다: 서버가 0 을 받을 때까지 버리므로 다시 눌러야 움직인다
		if (wasExploring && !isExploringNow && (keys.size || pointers.size)) releaseAll();
		wasExploring = isExploringNow;
		if (!isLevelSet && ctx.info) {
			level = ctx.info.start_level ?? 0;
			isLevelSet = true;
		}
		level = Math.min(level, maxLevel());
		k.level.max = String(maxLevel());
		k.level.value = String(level);
		paintLevel();
		k.control.textContent = isMine ? '조작 권한 놓기' : '조작 권한 받기';
		k.control.classList.toggle('btn-primary', !isMine);
		k.control.disabled = !ctx.isConnected || Boolean(ctx.info?.is_read_only)
			|| (!isMine && Boolean(ctx.status?.control?.holder));
		k.power.textContent = isPowerBusy ? '조작 켜는 중…' : (isReady ? '조작 끄기' : '조작 켜기');
		k.power.classList.toggle('btn-primary', !isReady);
		const canPowerOn = teleop?.is_allowed === true;
		k.power.disabled = isPowerBusy || Boolean(blocked) || (!isReady && !canPowerOn);
		put(k.reason, reasonText(blocked, teleop, isReady));
		// 방향 버튼에만 둔다: 그룹에 두면 '정지'까지 꺼진 것으로 읽힌다 (자동 주행 중에는 멈춤 버튼이다)
		for (const button of k.pad.querySelectorAll('button:not(.pad-stop)')) {
			button.setAttribute('aria-disabled', String(!canPad()));
		}
	}

	function isIgnored(event) {
		if (event.ctrlKey || event.metaKey || event.altKey) return true;
		if (event.target instanceof Element && event.target.matches(TYPING)) return true;
		return Boolean(document.querySelector('dialog[open]'));
	}

	/** Space 로 포커스된 버튼이 눌리지 않게 막는다. 비상 정지 버튼만은 눌려야 한다. */
	function blockSpace(event) {
		const isEstop = event.target instanceof Element && event.target.closest('[data-estop]');
		if (!isEstop) event.preventDefault();
	}

	function onKeyDown(event) {
		// macOS 는 Cmd 를 누른 동안 뗀 키의 keyup 을 보내지 않는다: 누르고 있던 키를 미리 놓는다
		if (event.key === 'Meta') {
			releaseAll();
			return;
		}
		if (isIgnored(event)) return;
		const code = event.code;
		if (code === 'Escape') {
			if (ctx.isMine()) setPower(false);
			return;
		}
		if (code === 'KeyR' || code === 'KeyF') {
			if (!event.repeat) setLevel(level + (code === 'KeyR' ? 1 : -1));
			return;
		}
		if (code === 'Space') {
			blockSpace(event);   // 포커스된 버튼이 눌리거나 화면이 내려가지 않게
			if (!event.repeat) stopExploration();
			releaseAll();
			return;
		}
		if (!(code in AXES)) return;
		event.preventDefault();
		if (event.repeat || !canDrive()) return;
		keys.add(code);
		changed();
	}

	function onKeyUp(event) {
		if (event.code === 'Space' && !isIgnored(event)) blockSpace(event);   // 버튼은 keyup 에 눌린다
		if (keys.delete(event.code)) changed();   // 입력 칸으로 포커스가 옮겨 가도 뗌은 받는다
	}

	function onPointerDown(event) {
		const button = event.target.closest('[data-code]');
		if (!button) return;
		event.preventDefault();
		if (button.dataset.code === 'Space') {
			stopExploration();
			releaseAll();
			return;
		}
		if (!canPad()) return;
		button.setPointerCapture(event.pointerId);
		pointers.set(event.pointerId, button.dataset.code);
		changed();
	}

	function onPointerUp(event) {
		if (pointers.delete(event.pointerId)) changed();
	}

	function onVisibility() {
		if (document.hidden) releaseAll();
	}

	k.pad.addEventListener('pointerdown', onPointerDown);
	for (const type of ['pointerup', 'pointercancel', 'lostpointercapture']) {
		k.pad.addEventListener(type, onPointerUp);
	}
	k.pad.addEventListener('contextmenu', (event) => event.preventDefault());
	k.level.addEventListener('input', () => setLevel(Number(k.level.value)));
	k.control.addEventListener('click', () => (ctx.isMine() ? ctx.release() : ctx.acquire()));
	k.power.addEventListener('click', () => setPower(ctx.status?.motor?.ready !== true));
	window.addEventListener('keydown', onKeyDown);
	window.addEventListener('keyup', onKeyUp);
	window.addEventListener('blur', releaseAll);
	window.addEventListener('pagehide', releaseAll);
	document.addEventListener('visibilitychange', onVisibility);
	const timer = setInterval(send, SEND_MS);   // 권한이 있으면 0 이어도 0.1 s 마다
	ctx.on('update', render);
	ctx.on('estop', releaseAll);
	render();

	return {
		destroy() {
			releaseAll();
			clearInterval(timer);
			window.removeEventListener('keydown', onKeyDown);
			window.removeEventListener('keyup', onKeyUp);
			window.removeEventListener('blur', releaseAll);
			window.removeEventListener('pagehide', releaseAll);
			document.removeEventListener('visibilitychange', onVisibility);
		},
	};
}
