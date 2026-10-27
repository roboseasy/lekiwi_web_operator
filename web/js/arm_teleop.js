// 텔레오퍼레이션 묶음 (조작 화면): PC 에이전트 허락 · 리더암 연결 · 팔 시작 · 멈춤 · 힘 빼기 · 관절 표.
// 바퀴는 조작 묶음(teleop.js)의 키보드 · 패드가 그대로 맡는다: 바퀴는 키, 팔은 리더암으로 함께 쓴다.
// 상태: status.agent (docs/agent_protocol.md) · status.teleop_arm ('arm' 채널을 구독하는 동안 follower · joints).
// 고르기 칸(select)에 포커스가 있으면 조작 키를 쓰지 않는다 (teleop.js TYPING): 고른 뒤 포커스를 놓아
// 바로 다시 W A S D 로 몰 수 있게 한다.

import { DASH, badge, esc, fixed, isNum, put, putHtml, refs } from './format.js';

const DIFF_WARN = 8;   // [도] 리더 − 팔로워가 이보다 크면 표에서 노랑 (그리퍼는 0~100 단위로 같은 값)
const JOINTS = [   // [lekiwi/arm/state 이름, 화면 이름, 각도인지]
	['arm_shoulder_pan', '어깨 회전', true],
	['arm_shoulder_lift', '어깨 들기', true],
	['arm_elbow_flex', '팔꿈치', true],
	['arm_wrist_flex', '손목 굽힘', true],
	['arm_wrist_roll', '손목 회전', true],
	['arm_gripper', '그리퍼', false],
];
const LEADER_STATES = {
	disconnected: ['neutral', '연결 안 됨'],
	connecting: ['warn', '연결 중'],
	connected: ['ok', '연결됨'],
	error: ['danger', '오류'],
};
const FOLLOWER_STATES = {
	idle: ['neutral', '대기'],
	ramping: ['warn', '맞추는 중'],
	following: ['info', '따라가는 중'],
	error: ['danger', '오류'],
	resting: ['warn', '접는 중'],
	rested: ['neutral', '접음'],
};
const START_QUESTION = {
	title: '팔을 시작할까요?',
	message: '팔 토크를 켜고 리더암을 따라 움직입니다. 자세 차이가 크면 2초 동안 천천히 맞춥니다. '
		+ '팔 주변을 비우세요.\n비상 정지는 팔 토크도 꺼서 편 팔과 잡은 물건이 떨어집니다. '
		+ '팔을 놓지 않고 멈추려면 ‘멈춤’을 누르세요.',
	confirmLabel: '팔 시작',
};
const RELAX_QUESTION = {
	title: '팔 힘을 뺄까요?',
	message: '팔 원격 조작을 멈추고 팔 토크를 끕니다. 편 팔과 잡은 물건이 떨어집니다. '
		+ '팔 아래를 비우거나 팔을 손으로 받치세요.',
	confirmLabel: '팔 힘 빼기',
	isDanger: true,
};
const NO_AGENT = 'PC 에이전트가 연결되지 않았습니다 (PC에서 실행 명령을 실행하세요)';   // 서버 agent.py 와 같은 글
// 캘리브레이션 화면의 리더 '이 보정 쓰기': 이 브라우저의 보정 선택 기본값 (서버에는 없다)
const LEADER_CALIBRATION_KEY = 'lekiwi-web-operator.leader-calibration';

// PC 에이전트 저장소 (공개). 누구의 노트북이든 홈에 받아 쓴다
const AGENT_REPO = 'https://github.com/roboseasy/lekiwi_pc_agent';

/** PC 에이전트 실행 명령. 에이전트는 제 폴더에서만 돈다 (python -m). --no-capture-output: conda run 이 출력을 붙잡지 않게 */
export function agentCommand() {
	return 'cd ~/lekiwi_pc_agent && conda run '
		+ `--no-capture-output -n lerobot060 python -m lekiwi_pc_agent --robot ${location.hostname}`;
}

/** PC 에이전트 받기 · 실행 안내 (조작 · 캘리브레이션 화면). 채우기와 복사 단추는 bindAgentRunBox */
export function agentRunBoxHtml(note) {
	return `
		<div class="run-box" data-k="runBox" hidden>
			<p class="arm-note">${esc(note)} · 저장소
				<a href="${AGENT_REPO}" target="_blank" rel="noopener">github.com/roboseasy/lekiwi_pc_agent</a></p>
			<p class="arm-note">처음 한 번: 저장소 받기 (인터넷이 되는 곳에서. 로봇 AP 에 붙어 있으면 안 된다)</p>
			<div class="run-cmd">
				<code class="mono" data-k="cloneCommand"></code>
				<button type="button" class="btn btn-small" data-k="copyClone">복사</button>
			</div>
			<p class="arm-note">실행 (conda 환경 lerobot060 · 끌 때 Ctrl+C)</p>
			<div class="run-cmd">
				<code class="mono" data-k="command"></code>
				<button type="button" class="btn btn-small" data-k="copy">복사</button>
			</div>
		</div>`;
}

/** agentRunBoxHtml 의 두 명령을 채우고 복사 단추를 잇는다. k = refs(root) */
export function bindAgentRunBox(k, conn) {
	put(k.cloneCommand, `cd ~ && git clone ${AGENT_REPO}.git`);
	put(k.command, agentCommand());
	for (const [button, code] of [[k.copyClone, k.cloneCommand], [k.copy, k.command]]) {
		button.addEventListener('click', async () => {
			const isCopied = await copyText(code);
			conn.toast(isCopied ? '명령을 복사했습니다' : '복사하지 못했습니다. 골라 둔 명령을 Ctrl+C 로 복사하세요',
				isCopied ? 'ok' : 'error');
		});
	}
}

/** 리더 보정 기본값 (고른 적이 없거나 저장소를 못 쓰면 null). */
export function getLeaderCalibration() {
	try {
		return localStorage.getItem(LEADER_CALIBRATION_KEY);
	} catch {
		return null;   // 사생활 보호 모드 등
	}
}

export function setLeaderCalibration(id) {
	try {
		localStorage.setItem(LEADER_CALIBRATION_KEY, id);
	} catch {
		// 저장하지 못하면 기본값은 'leader' 그대로다
	}
}

function template() {
	const rows = JOINTS.map(([name, label]) => `
		<tr data-joint="${name}">
			<th scope="row">${label}</th>
			<td class="num" data-c="leader">${DASH}</td>
			<td class="num" data-c="follower">${DASH}</td>
			<td class="num" data-c="diff">${DASH}</td>
		</tr>`).join('');
	return `
		<div class="panel-head">
			<h2 id="arm-title">텔레오퍼레이션</h2>
			<span data-k="torque"></span>
		</div>
		<div class="arm-part" role="group" aria-labelledby="agent-title">
			<h3 class="arm-label" id="agent-title">PC 에이전트</h3>
			<p class="arm-line"><span data-k="agentBadge"></span><span data-k="agentWho"></span></p>
			<div class="actions" data-k="agentActions">
				<button type="button" class="btn" data-k="approve"></button>
			</div>
			<p class="reason" data-k="agentReason"></p>
			${agentRunBoxHtml('노트북 터미널에서 실행합니다')}
		</div>
		<div class="arm-part" role="group" aria-labelledby="leader-title">
			<h3 class="arm-label" id="leader-title">리더암</h3>
			<div class="leader-fields">
				<div>
					<label class="field-label" for="leader-port">포트</label>
					<select class="input" id="leader-port" data-k="port"></select>
				</div>
				<div>
					<label class="field-label" for="leader-calibration">보정</label>
					<select class="input" id="leader-calibration" data-k="calibration"></select>
				</div>
			</div>
			<div class="actions">
				<button type="button" class="btn btn-primary" data-k="connect">리더암 연결</button>
				<button type="button" class="btn" data-k="disconnect" hidden>리더암 끊기</button>
			</div>
			<p class="reason" data-k="leaderReason"></p>
			<p class="arm-line"><span data-k="leaderBadge"></span><span data-k="leaderMeta"></span></p>
			<p class="arm-error" data-k="leaderError"></p>
		</div>
		<div class="arm-part" role="group" aria-labelledby="follower-title">
			<h3 class="arm-label" id="follower-title">팔</h3>
			<div class="actions">
				<button type="button" class="btn btn-primary" data-k="start">팔 시작</button>
				<button type="button" class="btn" data-k="stop" hidden>멈춤</button>
				<button type="button" class="btn btn-danger" data-k="relax">팔 힘 빼기</button>
			</div>
			<p class="reason" data-k="armReason"></p>
			<p class="arm-line"><span data-k="followerBadge"></span><span data-k="followerReason"></span></p>
			<ul class="warning-list arm-warnings" data-k="warnings"></ul>
			<div class="table-wrap">
				<table class="table arm-joints">
					<caption class="arm-caption">관절 (도, 그리퍼 0~100). 차이가 ${DIFF_WARN} 넘으면 노랑</caption>
					<thead><tr>
						<th scope="col">관절</th><th scope="col" class="num">리더</th>
						<th scope="col" class="num">팔로워</th><th scope="col" class="num">차이</th>
					</tr></thead>
					<tbody data-k="joints">${rows}</tbody>
				</table>
			</div>
			<p class="hint">‘멈춤’은 팔을 토크를 켠 채 그 자리에 둡니다. ‘팔 힘 빼기’와 비상 정지는 팔 토크를 꺼서
				편 팔과 잡은 물건이 떨어집니다.</p>
		</div>`;
}

/** 리더 관절 키 'arm_shoulder_pan.pos' (에이전트 그대로) · 'arm_shoulder_pan' 둘 다 받는다. */
function leaderJoints(raw) {
	const joints = {};
	for (const [key, value] of Object.entries(raw ?? {})) joints[key.replace(/\.pos$/, '')] = value;
	return joints;
}

function jointText(value, isAngle, isSigned = false) {
	if (!isNum(value)) return DASH;
	const sign = isSigned && value > 0 ? '+' : '';
	return `${sign}${fixed(value, 1)}${isAngle ? '°' : ''}`;
}

/** select 의 option HTML. 고른 값에 selected 를 붙여 putHtml 로 다시 써도 고른 것이 남는다. */
function optionsHtml(items, chosen, empty) {
	if (!items.length) return `<option value="">${esc(empty)}</option>`;
	return items.map(([value, label]) => `<option value="${esc(value)}"`
		+ `${value === chosen ? ' selected' : ''}>${esc(label)}</option>`).join('');
}

/**
 * 보안 문맥(https · localhost)이 아니면 navigator.clipboard 가 없다 (로봇 주소는 http): 그때는
 * 글을 골라 두고 execCommand('copy') 를 쓴다. 실패해도 글은 골라진 채라 Ctrl+C 로 복사할 수 있다.
 */
export async function copyText(el) {
	try {
		await navigator.clipboard.writeText(el.textContent);
		return true;
	} catch {
		const range = document.createRange();
		range.selectNodeContents(el);
		getSelection().removeAllRanges();
		getSelection().addRange(range);
		let isCopied = false;
		try {
			isCopied = document.execCommand('copy');
		} catch {
			// 못 하면 골라 둔 채로 둔다 (Ctrl+C)
		}
		if (isCopied) getSelection().removeAllRanges();
		return isCopied;
	}
}

/**
 * root 안에 텔레오퍼레이션 묶음을 그린다.
 *
 * @param {HTMLElement} root 그릴 곳 (조작 화면의 판).
 * @param {object} ctx app.js 의 화면 문맥.
 * @returns {{destroy: () => void}}
 */
export function createArmTeleop(root, ctx) {
	root.classList.add('arm-box');
	root.innerHTML = template();
	const k = refs(root);
	const cells = new Map([...k.joints.querySelectorAll('[data-joint]')].map((row) => [
		row.dataset.joint,
		Object.fromEntries([...row.querySelectorAll('[data-c]')].map((cell) => [cell.dataset.c, cell])),
	]));
	let port = null;          // 고른 포트 (사용자가 고르기 전에는 리더 포트 · 첫 포트)
	let calibration = null;   // 고른 보정 id
	let isStarting = false;   // 팔 시작 응답(최대 약 10 s)을 기다리는 중
	bindAgentRunBox(k, ctx.conn);

	const agentOf = () => ctx.status?.agent;
	const armOf = () => ctx.status?.teleop_arm;
	const isLeaderOn = (leader) => leader?.state === 'connected' || leader?.state === 'connecting';

	function agentBlock(agent) {
		return ctx.blockReason() ?? (!ctx.status ? '로봇 상태를 기다리는 중입니다' : null)
			?? (agent?.is_connected !== true ? NO_AGENT : null);
	}

	function leaderBlock(agent, ports, ids) {
		const reason = agentBlock(agent);
		if (reason) return reason;
		if (isLeaderOn(agent.leader)) return null;   // 끊기
		if (!ports.length) return '리더암 포트가 없습니다. PC에 리더암 USB 를 꽂으세요';
		if (!ids.length) return '리더암 보정 파일이 없습니다. PC에서 lerobot 으로 리더암을 보정하세요';
		return null;
	}

	/** 팔 시작을 막는 이유: [권한 이유] → 서버 이유 (비상 정지 · 자율 주행 · 팔 노드 …) → 에이전트 · 리더. */
	function startBlock(agent, arm) {
		const reason = ctx.blockReason();
		if (reason) return reason;
		if (!arm) return '로봇 상태를 기다리는 중입니다';
		if (arm.blocked_reason) return arm.blocked_reason;
		if (agent?.is_connected !== true) return NO_AGENT;
		if (agent.is_approved !== true) return '먼저 ‘이 PC 쓰기’를 누르세요';
		if (agent.leader?.state !== 'connected') return '먼저 ‘리더암 연결’을 누르세요';
		return null;
	}

	function renderAgent(agent) {
		const isConnected = agent?.is_connected === true;
		const who = [agent?.host, agent?.address].filter(Boolean).map(esc).join(' · ');
		putHtml(k.agentBadge, isConnected ? badge('ok', '연결됨') : badge('neutral', '연결 안 됨'));
		putHtml(k.agentWho, isConnected
			? `${who ? `<span class="mono">${who}</span>` : ''}`
				+ (agent.is_approved === true ? badge('accent', '이 PC 씀') : badge('neutral', '허락 전'))
			: '');
		k.agentActions.hidden = !isConnected;
		k.runBox.hidden = isConnected;
		const isApproved = agent?.is_approved === true;
		put(k.approve, isApproved ? '허락 취소' : '이 PC 쓰기');
		k.approve.classList.toggle('btn-primary', !isApproved);
		const reason = agentBlock(agent);
		k.approve.disabled = Boolean(reason);
		// 연결이 없을 때는 아래 실행 명령이 할 일이다: 같은 말을 두 번 쓰지 않는다
		put(k.agentReason, reason === NO_AGENT ? '' : (reason ?? ''));
	}

	function renderLeader(agent) {
		const leader = agent?.is_connected === true ? agent.leader : null;
		const ports = Array.isArray(agent?.ports) ? agent.ports.filter((item) => item?.path) : [];
		const ids = Array.isArray(agent?.calibration_ids) ? agent.calibration_ids : [];
		const isOn = isLeaderOn(leader);
		if (isOn && leader.port) port = leader.port;
		if (!ports.some((item) => item.path === port)) port = ports[0]?.path ?? null;
		if (isOn && leader.calibration_id) calibration = leader.calibration_id;
		if (!ids.includes(calibration)) {
			const preferred = getLeaderCalibration();
			calibration = [preferred, 'leader', ids[0]].find((id) => ids.includes(id)) ?? null;
		}
		// by_id 는 /dev/serial/by-id/… 전체 경로 또는 null: 이름만 붙인다. 보내는 port 는 늘 path
		const portItems = ports.map((item) => [
			item.path, item.by_id ? `${item.path} · ${item.by_id.split('/').pop()}` : item.path]);
		putHtml(k.port, optionsHtml(portItems, port, agent?.is_connected ? '포트 없음' : DASH));
		putHtml(k.calibration, optionsHtml(ids.map((id) => [id, id]), calibration,
			agent?.is_connected ? '보정 없음' : DASH));
		const reason = leaderBlock(agent, ports, ids);
		// 연결한 뒤에는 포트 · 보정을 바꿔도 소용없다: 끊은 뒤에 고른다
		k.port.disabled = isOn || !ports.length || Boolean(ctx.blockReason());
		k.calibration.disabled = isOn || !ids.length || Boolean(ctx.blockReason());
		k.connect.hidden = isOn;
		k.disconnect.hidden = !isOn;
		k.connect.disabled = Boolean(reason);
		k.disconnect.disabled = Boolean(reason);
		put(k.leaderReason, reason === NO_AGENT ? '' : (reason ?? ''));
		const [tone, text] = LEADER_STATES[leader?.state] ?? ['neutral', leader ? DASH : '연결 안 됨'];
		putHtml(k.leaderBadge, badge(tone, text));
		const meta = [];
		if (leader?.calibration_id && isOn) meta.push(`보정 ${esc(leader.calibration_id)}`);
		if (isNum(leader?.rate_hz)) meta.push(`${fixed(leader.rate_hz, 1)} Hz`);
		if (armOf()?.leader?.is_relaying === true) meta.push('팔로 보내는 중');
		putHtml(k.leaderMeta, meta.join(' · '));
		put(k.leaderError, leader?.state === 'error' ? (leader.reason || '리더암 오류') : '');
	}

	function renderFollower(agent, arm) {
		const isActive = arm?.is_session_active === true;
		const armReady = arm?.arm_ready;
		const torque = armReady === true || arm?.is_torque_maybe_on === true
			? ['warn', '팔 토크 켜짐']
			: armReady === false ? ['neutral', '팔 토크 꺼짐'] : ['neutral', '팔 토크 —'];
		putHtml(k.torque, badge(...torque));

		k.start.hidden = isActive;
		k.stop.hidden = !isActive && !isStarting;   // 켜는 사이 '멈춤'도 시작을 멈춘다 (서버 409)
		k.start.textContent = isStarting ? '팔 켜는 중…' : '팔 시작';
		const block = ctx.blockReason();
		const reason = isActive || isStarting ? block : startBlock(agent, arm);
		k.start.disabled = isStarting || Boolean(reason);
		k.stop.disabled = Boolean(block);
		k.relax.disabled = Boolean(block);   // 비상 정지 · 자율 주행 중에도 된다 (서버)
		// 연결이 없으면 실행 명령은 위 'PC 에이전트' 칸에 있다
		put(k.armReason, reason === NO_AGENT ? `${NO_AGENT} — 위 ‘PC 에이전트’ 칸의 명령` : (reason ?? ''));

		const follower = arm?.follower;
		let state = FOLLOWER_STATES[follower?.state];
		if (!follower) state = arm?.is_follower_present === false ? ['neutral', '팔 노드 없음'] : ['neutral', '상태 없음'];
		const [tone, text] = state ?? ['neutral', DASH];
		const progress = follower?.state === 'ramping' && isNum(follower.ramp_progress)
			? ` ${Math.round(follower.ramp_progress * 100)}%` : '';
		putHtml(k.followerBadge, badge(tone, text + progress));
		put(k.followerReason, follower?.reason || '');
		const warnings = Array.isArray(follower?.warnings) ? follower.warnings : [];
		putHtml(k.warnings, warnings.map((line) => `<li>${esc(line)}</li>`).join(''));
	}

	function renderJoints(arm) {
		const leader = leaderJoints(agentOf()?.leader?.joints ?? arm?.leader?.joints);
		const follower = arm?.joints ?? {};
		const diffs = arm?.follower?.diffs ?? {};
		for (const [name, , isAngle] of JOINTS) {
			const cell = cells.get(name);
			const lead = leader[name];
			const follow = follower[name];
			// 리더를 모르면 팔로워가 낸 차이 (자른 목표 − 팔로워, 명령하는 동안만)
			const diff = isNum(lead) && isNum(follow) ? lead - follow : diffs[name];
			put(cell.leader, jointText(lead, isAngle));
			put(cell.follower, jointText(follow, isAngle));
			const text = jointText(diff, isAngle, true);
			putHtml(cell.diff, isNum(diff) && Math.abs(diff) > DIFF_WARN ? badge('warn', text) : esc(text));
		}
	}

	function render() {
		const agent = agentOf();
		const arm = armOf();
		renderAgent(agent);
		renderLeader(agent);
		renderFollower(agent, arm);
		renderJoints(arm);
	}

	async function start() {
		if (isStarting || !(await ctx.confirm(START_QUESTION))) return;
		isStarting = true;
		render();
		await ctx.conn.post('/api/arm/start', {});
		isStarting = false;
		render();
	}

	async function relax() {
		if (await ctx.confirm(RELAX_QUESTION)) ctx.conn.post('/api/arm/relax', {});
	}

	function choose(event) {
		if (event.target === k.port) port = k.port.value;
		if (event.target === k.calibration) calibration = k.calibration.value;
		event.target.blur();   // 포커스를 놓아 조작 키가 다시 바로 먹게 (고르기 칸 안에서는 쓰지 않는다)
	}

	k.approve.addEventListener('click', () => ctx.conn.post(
		agentOf()?.is_approved === true ? '/api/agent/unapprove' : '/api/agent/approve',
		// 허락은 지금 보이는 연결에만: 그사이 다른 에이전트로 바뀌었으면 서버가 409로 거절한다
		{ agent_session: agentOf()?.agent_session }));
	k.connect.addEventListener('click', () => ctx.conn.post('/api/agent/leader_connect',
		{ port, calibration_id: calibration }));
	k.disconnect.addEventListener('click', () => ctx.conn.post('/api/agent/leader_disconnect', {}));
	k.port.addEventListener('change', choose);
	k.calibration.addEventListener('change', choose);
	k.start.addEventListener('click', start);
	k.stop.addEventListener('click', () => ctx.conn.post('/api/arm/stop', {}));
	k.relax.addEventListener('click', relax);
	ctx.on('update', render);
	render();

	return { destroy() {} };   // 리스너는 이 묶음 안 요소와 ctx.on 뿐이다 (화면을 바꾸면 함께 사라진다)
}
