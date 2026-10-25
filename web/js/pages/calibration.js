// 캘리브레이션: 팔로워(르키위 팔, 로봇) · 리더(SO-101, 노트북)의 lerobot 보정 확인 · 고르기 · 새로 만들기
// (docs/calibration_plan.md). 상태는 status.calibration (follower · leader), 리더는 PC 에이전트를 거친다.
// 새로 보정은 lerobot 순서 그대로: 준비 → 제로-포지션 위치 시키기 → 범위 재기 → 끝내기. 어느 단계든 취소하면 쓰지 않는다.

import { agentCommand, copyText, getLeaderCalibration, setLeaderCalibration } from '../arm_teleop.js';
import { DASH, badge, esc, isNum, isOn, put, putHtml, refs } from '../format.js';

export const title = '캘리브레이션';
// 'arm': 팔로워가 쓰는 보정 이름 (teleop_arm.follower.robot_id) · 팔 토크 (arm_ready)
export const channels = ['arm', 'calibration'];   // calibration: 모터 보정 레지스터 (arm_registers)

const TICKS = 4095;
const NAME_RE = /^[A-Za-z0-9_-]{1,40}$/;
const RUNNING = new Set(['starting', 'waiting_middle', 'recording', 'saving']);
const STEPS = ['준비', '가운데', '범위', '끝내기'];
const STEP_OF = { starting: 1, waiting_middle: 2, recording: 3, saving: 4 };
const JOINT_NAMES = {   // lerobot 이름 (팔로워는 앞에 arm_) → 화면 이름
	shoulder_pan: '어깨 회전',
	shoulder_lift: '어깨 들기',
	elbow_flex: '팔꿈치',
	wrist_flex: '손목 굽힘',
	wrist_roll: '손목 회전',
	gripper: '그리퍼',
	base_left_wheel: '왼쪽 바퀴',
	base_back_wheel: '뒤 바퀴',
	base_right_wheel: '오른쪽 바퀴',
};
// lerobot 과 같게 한 바퀴 관절(손목 회전 · 바퀴)은 범위를 재지 않고 0~4095 로 둔다
const isFullTurn = (name) => /wrist_roll|wheel/.test(name);
const MIDDLE_GUIDE = `
	<ul class="calib-middle">
		<li><b>어깨 회전</b> 왼쪽 끝과 오른쪽 끝의 가운데</li>
		<li><b>어깨 들기</b> 앞쪽 끝과 뒤쪽 끝의 가운데</li>
		<li><b>팔꿈치</b> 다 편 것과 다 접은 것의 가운데</li>
		<li><b>손목 굽힘</b> 위쪽 끝과 아래쪽 끝의 가운데</li>
		<li><b>손목 회전</b> 돌아가는 범위의 가운데</li>
		<li><b>그리퍼</b> 반쯤 연 자세</li>
	</ul>`;
const DATASET_WARNING = '새로 보정하면 같은 자세라도 값이 달라질 수 있습니다. 예전 데이터셋과 섞어 학습할 때 주의하세요';
const NO_AGENT = 'PC 에이전트가 연결되지 않았습니다 (PC에서 실행 명령을 실행하세요)';   // 서버 agent.py 와 같은 글
const DATE_FORMAT = { dateStyle: 'short', timeStyle: 'short' };
const RESTORE_FIRST = '지난 새로 보정이 끝나지 않았습니다. 먼저 ‘복원’을 누르세요';
const ARM_SESSION = '팔 원격 조작 중입니다. 조작 화면에서 먼저 ‘멈춤’을 누르세요 (리더 가운데가 바뀌면 팔이 튑니다)';
const NO_REST_POSE = '접은 자세가 저장돼 있지 않습니다. 없으면 로봇을 끌 때 팔을 접지 못합니다. '
	+ '팔을 접은 자세로 두고 ‘접은 자세 저장’을 누르세요';
const FILE_KEYS = new Set(['select', 'fromMotors', 'restPose', 'check']);   // 파일 칸 버튼 (거절 글을 그 아래에)

const SIDES = {
	follower: {
		title: '팔로워 (로봇)',
		about: '르키위 팔. 보정 파일은 로봇의 ~/.cache/huggingface/lerobot/calibration/robots/lekiwi/ 에 있습니다.',
		api: '/api/calibration/follower',
		idKey: 'robot_id',
		placeholder: 'lekiwi05',
		isStartNewOnly: true,   // 서버: 있는 이름은 409 (리더 에이전트는 .bak 으로 남기고 새로 쓴다)
		startNote: '보정하는 동안 로봇 기능(카메라 · 바퀴 · 라이다)이 꺼집니다. 끝내면 다시 켭니다.',
		startQuestion: '팔로워를 새로 보정할까요?',
		selectQuestion: (id) => ({
			title: '팔로워 보정을 바꿀까요?',
			message: `${id} 보정을 씁니다. 로봇 기능(카메라 · 바퀴 · 라이다)을 다시 켜는 동안 잠깐 꺼집니다.`,
			confirmLabel: '이 보정 쓰기',
		}),
	},
	leader: {
		title: '리더 (노트북)',
		about: '노트북의 SO-101 리더암. 보정 파일은 노트북의 …/calibration/teleoperators/so_leader/ 에 있습니다.',
		api: '/api/calibration/leader',
		idKey: 'calibration_id',
		placeholder: 'leader',
		startNote: '보정을 시작하면 리더암 연결을 끊습니다. 보정하는 동안 리더 값을 팔로 보내지 않습니다.',
		startQuestion: '리더를 새로 보정할까요?',
	},
};

const selected = { follower: null, leader: null };   // 고른 파일 (화면을 다시 열어도 기억)

function template(side) {
	const config = SIDES[side];
	const steps = STEPS.map((name, index) => `
		<li data-step="${index + 1}"><span class="step-no">${index + 1}</span><span>${name}</span></li>`).join('');
	const followerOnly = side === 'follower';
	return `
		<section class="panel calib" aria-labelledby="${side}-title">
			<div class="panel-head">
				<h2 id="${side}-title">${config.title}</h2><span data-k="headBadge"></span>
			</div>
			<p class="calib-about">${config.about}</p>
			<dl class="calib-now" data-k="now"></dl>
			<p class="arm-error" data-k="nowReason"></p>
			${followerOnly ? '' : `
			<div class="run-box" data-k="runBox" hidden>
				<p class="arm-note">PC 에이전트가 연결되지 않았습니다. 노트북 터미널에서 실행하세요 (끌 때 Ctrl+C)</p>
				<div class="run-cmd">
					<code class="mono" data-k="command"></code>
					<button type="button" class="btn btn-small" data-k="copy">복사</button>
				</div>
			</div>`}
			${followerOnly ? `
			<div class="calib-restore" data-k="restoreBox" role="alert" hidden>
				<p class="calib-restore-title">지난 새로 보정이 끝나지 않았습니다</p>
				<p>모터 보정 값이 반쯤 쓰였을 수 있습니다 (전원이 꺼졌거나 보정 프로그램이 멈췄습니다).
					‘복원’을 누르면 새로 보정을 시작하기 전의 모터 값을 다시 씁니다. 복원하는 동안 로봇 기능(카메라 · 바퀴 ·
					라이다)이 꺼집니다. 복원하기 전에는 다른 보정 작업을 할 수 없습니다.</p>
				<div class="actions">
					<button type="button" class="btn btn-primary" data-k="restore">복원</button>
				</div>
				<div data-k="restoreReasons"></div>
				<p class="arm-error" data-k="restoreRefusal"></p>
			</div>` : ''}

			<h3 class="sub">보정 파일</h3>
			<ul class="pick-list" data-k="files"></ul>
			<div class="actions">
				${followerOnly ? '' : '<button type="button" class="btn" data-k="check">모터와 맞춰 보기</button>'}
				<button type="button" class="btn btn-primary" data-k="select">이 보정 쓰기</button>
				<button type="button" class="btn" data-k="fromMotors">모터 값으로 파일 만들기</button>
				${followerOnly ? '<button type="button" class="btn" data-k="restPose">접은 자세 저장</button>' : ''}
			</div>
			<div data-k="fileReasons"></div>
			<p class="arm-error" data-k="fileRefusal"></p>
			<form class="name-form" data-k="nameForm" hidden>
				<label class="field-label" for="${side}-file-name">새 파일 이름 (영문 · 숫자 · _ · -, 40자까지)</label>
				<div class="name-row">
					<input class="input mono" id="${side}-file-name" data-k="fileName" maxlength="40"
						autocomplete="off" spellcheck="false" placeholder="${config.placeholder}">
					<button type="submit" class="btn btn-primary" data-k="fileCreate">파일 만들기</button>
					<button type="button" class="btn" data-k="fileCancel">취소</button>
				</div>
				<p class="reason" data-k="fileNameReason"></p>
			</form>
			${followerOnly ? '<p class="hint mono-line" data-k="restPoseLine"></p>'
				+ '<p class="notice calib-rest-notice" data-k="restPoseNotice"></p>' : ''}

			<h3 class="sub">새로 보정</h3>
			<ol class="stepper" data-k="steps" aria-label="새로 보정 단계">${steps}</ol>
			<div data-k="prepare">
				<p class="notice">${DATASET_WARNING}</p>
				<p class="calib-note">${config.startNote}</p>
				<label class="field-label" for="${side}-new-name">보정 이름</label>
				<div class="name-row">
					<input class="input mono" id="${side}-new-name" data-k="newName" maxlength="40"
						autocomplete="off" spellcheck="false" placeholder="${config.placeholder}">
					<button type="button" class="btn btn-primary" data-k="start">새로 보정 시작</button>
				</div>
				<div data-k="startReasons"></div>
				<p class="calib-result" data-k="result"></p>
			</div>
			<div data-k="run" hidden>
				<p class="calib-step" data-k="instruction" role="status"></p>
				<div data-k="middleGuide" hidden>${MIDDLE_GUIDE}</div>
				<ul class="range-list" data-k="ranges"></ul>
				<div class="actions">
					<button type="button" class="btn btn-primary" data-k="middle" hidden>제로-포지션 위치 시키기</button>
					<button type="button" class="btn btn-primary" data-k="finish" hidden>끝내기</button>
					<button type="button" class="btn" data-k="cancel">취소</button>
				</div>
				<div data-k="runReasons"></div>
			</div>
			<p class="arm-error" data-k="stepRefusal"></p>
		</section>`;
}

/** [버튼 이름, 이유] 들 → 이유 줄. 모든 버튼이 같은 이유로 막혔으면 한 줄, 아니면 '버튼: 이유'. */
function reasonsHtml(pairs) {
	const blocked = pairs.filter(([, reason]) => reason);
	const texts = new Set(blocked.map(([, reason]) => reason));
	if (texts.size <= 1 && (blocked.length === pairs.length || pairs.length === 1)) {
		return [...texts].map((reason) => `<p class="reason">${esc(reason)}</p>`).join('');
	}
	return blocked.map(([label, reason]) => `<p class="reason">${esc(label)}: ${esc(reason)}</p>`).join('');
}

function matchBadge(isMatch) {
	if (isMatch === true) return badge('ok', '모터와 맞음');
	if (isMatch === false) return badge('danger', '모터와 다름');
	return badge('neutral', '확인 전');
}

function nameReason(name, files, isNewOnly) {
	if (!name) return '이름을 적으세요';
	if (!NAME_RE.test(name)) return '이름은 영문 · 숫자 · _ · - 로 40자까지입니다';
	if (isNewOnly && files.some((file) => file.id === name)) return '이미 있는 이름입니다. 다른 이름을 쓰세요';
	return null;
}

function jointLabel(name) {
	return JOINT_NAMES[name.replace(/^arm_/, '')] ?? name;
}

const percent = (tick) => `${(Math.min(Math.max(tick, 0), TICKS) / TICKS) * 100}%`;

/**
 * 한쪽 (팔로워 · 리더) 판. 상태가 올 때마다 다시 그린다.
 *
 * @param {HTMLElement} root 판을 넣을 곳.
 * @param {object} ctx app.js 의 화면 문맥.
 * @param {'follower'|'leader'} side
 */
function createSide(root, ctx, side) {
	const config = SIDES[side];
	root.innerHTML = template(side);
	const k = refs(root);
	let pending = null;    // 응답을 기다리는 버튼 (data-k)
	let rangeKeys = '';    // 범위 표의 관절 이름 (바뀔 때만 다시 만든다)
	// 서버가 거절한 글 (칸마다, 다음 요청까지 남긴다): 토스트는 사라지므로 버튼 옆에도 둔다
	const refusals = { file: '', step: '', restore: '' };
	const groupOf = (key) => (key === 'restore' ? 'restore' : FILE_KEYS.has(key) ? 'file' : 'step');
	let lastState = null;   // 보정 상태가 바뀌면 지난 거절 글은 맞지 않는다: 지운다

	/** 이 쪽의 보정 상태를 한 모양으로: {cal, current, isMatch, reason, files, session}. */
	function view() {
		const status = ctx.status;
		const cal = status?.calibration?.[side] ?? null;
		if (side === 'follower') {
			return {
				cal,
				current: cal?.robot_id ?? status?.teleop_arm?.follower?.robot_id ?? null,
				isMatch: cal?.is_match ?? null,
				reason: cal?.reason ?? null,
				files: Array.isArray(cal?.files) ? cal.files : [],
				session: cal?.session ?? null,
			};
		}
		const agent = status?.agent;
		const leader = agent?.is_connected === true ? agent.leader : null;
		// '모터와 맞춰 보기' 전에는 에이전트가 알린 이름만 있다 (맞는지 모름)
		const ids = Array.isArray(agent?.calibration_ids) ? agent.calibration_ids : [];
		const files = Array.isArray(cal?.files) ? cal.files : ids.map((id) => ({ id, is_match: null }));
		const current = leader?.state === 'connected' ? leader.calibration_id : null;
		return {
			cal,
			current,
			isMatch: files.find((file) => file.id === current)?.is_match ?? null,
			reason: null,
			files,
			session: cal?.session ?? null,
			agent,
		};
	}

	/** 모든 버튼에 먼저 걸리는 이유. */
	function commonBlock(v) {
		const reason = ctx.blockReason();
		if (reason) return reason;
		if (!ctx.status) return '로봇 상태를 기다리는 중입니다';
		if (side === 'leader' && v.agent?.is_connected !== true) return NO_AGENT;
		if (!v.cal) return '보정 상태를 기다리는 중입니다 (게이트웨이가 아직 알리지 않습니다)';
		if (pending) return '요청을 처리하는 중입니다';
		return null;
	}

	/** 팔로워: 팔을 손으로 움직이고 로봇 기능을 끄고 켜려면 팔 토크가 꺼져 있고 지도 작성 · 자율 주행이 꺼져 있어야 한다. */
	function followerBlock() {
		const arm = ctx.status?.teleop_arm;
		if (arm?.is_session_active) return '팔 원격 조작 중입니다. 조작 화면에서 먼저 ‘멈춤’과 ‘팔 힘 빼기’를 누르세요';
		if (arm?.is_torque_maybe_on || arm?.arm_ready === true) {
			return '팔 토크가 켜져 있습니다. 조작 화면에서 먼저 ‘팔 힘 빼기’를 누르세요';
		}
		const processes = ctx.status?.processes;
		if (isOn(processes?.mapping) || isOn(processes?.navigation)) return '지도 작성 · 자율 주행을 먼저 끄세요';
		return null;
	}

	function isRunning(v) {
		return RUNNING.has(v.session?.state);
	}

	/** 팔로워 복원 (되돌리기) 세션: robot_id 가 없다. 끝나면 cancelled = 되돌렸음. */
	function isRestoreSession(v) {
		return side === 'follower' && Boolean(v.session) && !v.session.robot_id;
	}

	/** 파일 · 새로 보정 버튼에 더 걸리는 이유: 팔로워는 복원이 먼저, 리더는 팔 원격 조작 중이 아닐 때 (서버 409 와 같게). */
	function sideBlock(v) {
		if (side === 'follower') return v.cal?.restore_pending ? RESTORE_FIRST : null;
		return ctx.status?.teleop_arm?.is_session_active ? ARM_SESSION : null;
	}

	function restoreBlock(v) {
		return commonBlock(v) ?? (isRunning(v) ? '보정 프로그램이 도는 중입니다' : null) ?? followerBlock();
	}

	function selectBlock(v) {
		const file = v.files.find((item) => item.id === selected[side]);
		if (side === 'leader') {   // 이 브라우저의 조작 화면 기본값만 바꾼다 (명령을 보내지 않는다)
			if (!file) return '파일을 고르세요';
			if (file.id === getLeaderCalibration()) return '이미 조작 화면 기본값입니다';
			if (file.is_match === null || file.is_match === undefined) return '먼저 ‘모터와 맞춰 보기’를 누르세요';
			return file.is_match ? null : '모터와 맞지 않는 파일입니다';
		}
		const reason = commonBlock(v) ?? (isRunning(v) ? '새로 보정하는 중입니다' : null) ?? sideBlock(v);
		if (reason) return reason;
		if (!file) return '파일을 고르세요';
		if (file.id === v.current) return '이미 쓰는 보정입니다';
		if (!v.cal.motors) return '모터 보정 값을 받지 못해 맞는지 볼 수 없습니다 (로봇 기능을 켜세요)';   // 서버 503
		if (file.is_match !== true) return '모터와 다르거나 읽을 수 없는 파일입니다. 모터와 맞는 파일을 고르세요';
		return followerBlock();
	}

	function fromMotorsBlock(v) {
		const reason = commonBlock(v) ?? (isRunning(v) ? '새로 보정하는 중입니다' : null) ?? sideBlock(v);
		if (reason) return reason;
		if (side === 'follower' && !v.cal.motors) return '모터 보정 값을 받지 못했습니다 (로봇 기능 · 팔 쓰기를 켜세요)';
		return null;
	}

	function startBlock(v) {
		const reason = commonBlock(v) ?? (isRunning(v) ? '새로 보정하는 중입니다' : null) ?? sideBlock(v);
		if (reason) return reason;
		if (side === 'follower') {
			const arm = followerBlock();
			if (arm) return arm;
		}
		return nameReason(k.newName.value.trim(), v.files, config.isStartNewOnly);
	}

	function renderNow(v) {
		const rows = [];
		if (side === 'follower') {
			rows.push(['지금 쓰는 보정', v.current ? `<span class="mono">${esc(v.current)}</span>` : DASH]);
			rows.push(['모터와', v.cal ? matchBadge(v.isMatch) : DASH]);
		} else {
			const leader = v.agent?.is_connected === true ? v.agent.leader : null;
			rows.push(['리더암', leader?.state === 'connected'
				? `연결됨 · <span class="mono">${esc(v.current ?? DASH)}</span>` : '연결 안 됨']);
			const preferred = getLeaderCalibration();
			rows.push(['조작 화면 기본값', preferred ? `<span class="mono">${esc(preferred)}</span>` : 'leader (기본)']);
		}
		putHtml(k.now, rows.map(([label, value]) => `<div><dt>${label}</dt><dd>${value}</dd></div>`).join(''));
		// 서버의 이유 (파일 없음 · 레지스터를 못 받음 · 다름). 복원이 남았으면 아래 상자가 대신 말한다
		put(k.nowReason, v.reason && !v.cal?.restore_pending ? v.reason : '');
		if (k.runBox) k.runBox.hidden = v.agent?.is_connected === true;
		k.nowReason.classList.toggle('is-calm', v.isMatch !== false);
		const isMatch = v.cal ? v.isMatch : null;
		let head = v.current ? matchBadge(isMatch) : '';
		if (isRunning(v)) head = badge('warn', isRestoreSession(v) ? '복원하는 중' : '보정하는 중');
		else if (v.cal?.restore_pending) head = badge('danger', '복원 필요');
		putHtml(k.headBadge, head);
		if (k.restoreBox) {
			k.restoreBox.hidden = !v.cal?.restore_pending || isRunning(v);
			const restore = restoreBlock(v);
			k.restore.disabled = Boolean(restore);
			k.restore.textContent = pending === 'restore' ? '복원하는 중…' : '복원';
			putHtml(k.restoreReasons, reasonsHtml([['복원', restore]]));
			put(k.restoreRefusal, refusals.restore);
		}
	}

	function renderFiles(v) {
		if (!selected[side] || !v.files.some((file) => file.id === selected[side])) {
			selected[side] = v.files.find((file) => file.id === v.current)?.id ?? v.files[0]?.id ?? null;
		}
		const preferred = side === 'leader' ? getLeaderCalibration() : null;
		if (!v.files.length) {
			const empty = side === 'leader' && v.agent?.is_connected !== true
				? 'PC 에이전트가 연결되면 노트북의 보정 파일을 보여 줍니다'
				: '보정 파일이 없습니다. ‘모터 값으로 파일 만들기’나 새로 보정으로 만드세요';
			putHtml(k.files, `<li class="is-calm">${v.cal || side === 'leader' ? empty : '보정 상태를 기다리는 중입니다'}</li>`);
		} else {
			putHtml(k.files, v.files.map((file) => {
				const tags = [];
				if (file.id === v.current) tags.push(badge('accent', side === 'leader' ? '연결된 보정' : '지금 씀'));
				if (preferred && file.id === preferred) tags.push(badge('accent', '조작 화면 기본값'));
				tags.push(matchBadge(file.is_match));
				const date = isNum(file.modified)
					? new Date(file.modified * 1000).toLocaleString('ko-KR', DATE_FORMAT) : '';
				return `<li><button type="button" class="pick" data-id="${esc(file.id)}"
					aria-pressed="${file.id === selected[side]}">
					<span class="pick-name mono">${esc(file.id)}</span>
					<span class="pick-tags">${tags.join('')}</span>
					${date ? `<span class="pick-meta">${date}</span>` : ''}
				</button></li>`;
			}).join(''));
		}

		const select = selectBlock(v);
		const fromMotors = fromMotorsBlock(v);
		const pairs = [['이 보정 쓰기', select], ['모터 값으로 파일 만들기', fromMotors]];
		k.select.disabled = Boolean(select);
		k.fromMotors.disabled = Boolean(fromMotors) || !k.nameForm.hidden;
		if (k.check) {
			const check = commonBlock(v) ?? (isRunning(v) ? '새로 보정하는 중입니다' : null) ?? sideBlock(v);
			k.check.disabled = Boolean(check);
			k.check.textContent = pending === 'check' ? '맞춰 보는 중…' : '모터와 맞춰 보기';
			pairs.unshift(['모터와 맞춰 보기', check]);
		}
		if (k.restPose) {
			const rest = commonBlock(v) ?? (isRunning(v) ? '새로 보정하는 중입니다' : null) ?? sideBlock(v)
				?? followerBlock();
			k.restPose.disabled = Boolean(rest);
			k.restPose.textContent = pending === 'restPose' ? '저장하는 중…' : '접은 자세 저장';
			pairs.push(['접은 자세 저장', rest]);
			const ticks = v.cal?.rest_pose_ticks;
			const values = Array.isArray(ticks) ? ticks : Object.values(ticks ?? {});
			put(k.restPoseLine, values.length ? `저장된 접은 자세 (tick): ${values.join(' · ')}` : '');
			// 새로 보정이 끝나면 서버가 접은 자세를 지우고 done 이유로 알린다: 그 글을 이 버튼 곁에 남긴다
			const isJustCalibrated = v.session?.state === 'done' && v.session.robot_id && v.session.reason;
			put(k.restPoseNotice, !v.cal || values.length ? '' : (isJustCalibrated ? v.session.reason : NO_REST_POSE));
		}
		k.select.textContent = pending === 'select' ? '바꾸는 중…' : '이 보정 쓰기';
		putHtml(k.fileReasons, reasonsHtml(pairs));
		put(k.fileRefusal, refusals.file);
		const name = k.fileName.value.trim();
		const nameBlock = nameReason(name, v.files, true);
		k.fileCreate.disabled = Boolean(nameBlock) || Boolean(fromMotors);
		put(k.fileNameReason, name ? (nameBlock ?? '') : '');
	}

	function renderSteps(v) {
		// 시작 요청을 보낸 뒤 서버가 starting 을 알리기 전에도 '취소'가 보이게
		const state = pending === 'start' && !isRunning(v) ? 'starting' : v.session?.state;
		const current = STEP_OF[state] ?? (state === 'done' ? 5 : 1);
		for (const item of k.steps.children) {
			const step = Number(item.dataset.step);
			item.dataset.state = step < current ? 'done' : (step === current ? 'current' : 'todo');
			if (step === current) item.setAttribute('aria-current', 'step');
			else item.removeAttribute('aria-current');
		}
		const running = RUNNING.has(state);
		const isRestore = isRestoreSession(v);
		k.steps.hidden = isRestore && running;
		k.prepare.hidden = running;
		k.run.hidden = !running;
		put(k.stepRefusal, refusals.step);
		if (!running) {
			const start = startBlock(v);
			k.start.disabled = Boolean(start);
			k.start.textContent = pending === 'start' ? '시작하는 중…' : '새로 보정 시작';
			putHtml(k.startReasons, reasonsHtml([['새로 보정 시작', start]]));
			const session = v.session;
			const id = session?.[config.idKey] ?? session?.robot_id ?? session?.calibration_id;
			let result = '';
			const reason = esc(session?.reason ?? '');
			if (state === 'done') result = badge('ok', `${id ?? ''} 보정을 저장했습니다`) + reason;
			if (state === 'error') result = badge('danger', isRestore ? '복원하지 못했습니다' : '보정하지 못했습니다') + reason;
			if (state === 'cancelled') {
				result = (isRestore ? badge('ok', '예전 보정 값을 복원했습니다') : badge('neutral', '취소했습니다')) + reason;
			}
			putHtml(k.result, result);
			return;
		}
		const instruction = isRestore ? '예전 보정 값을 모터에 되돌리는 중입니다 (로봇 기능을 끄고 보정 프로그램으로)…' : {
			starting: '준비하는 중입니다 (버스를 여는 중)…',
			waiting_middle: '팔을 모든 관절 범위의 가운데로 옮긴 뒤 ‘제로-포지션 위치 시키기’를 누르세요.',
			recording: '모든 관절을 한 번씩 끝까지 천천히 움직이세요 (손목 회전 제외). 다 했으면 ‘끝내기’를 누르세요.',
			saving: '모터에 쓰고 파일로 저장하는 중입니다…',
		}[state];
		put(k.instruction, v.session?.reason ? `${instruction} ${v.session.reason}` : instruction);
		k.middleGuide.hidden = isRestore || state !== 'waiting_middle';
		k.middle.hidden = isRestore || state !== 'waiting_middle';
		k.finish.hidden = isRestore || state !== 'recording';
		k.cancel.hidden = isRestore;   // 복원은 끝까지 둔다 (반쯤 쓴 값을 남기지 않게)
		const block = commonBlock(v);
		k.middle.disabled = Boolean(block);
		k.finish.disabled = Boolean(block);
		// 취소는 늘 (요청 중에도): 서버가 쓴 가운데 값을 되돌리고 버스를 닫는다
		k.cancel.disabled = Boolean(ctx.blockReason());
		putHtml(k.runReasons, reasonsHtml([['제로-포지션 위치 시키기 · 끝내기', block], ['취소', ctx.blockReason()]]));
		renderRanges(isRestore ? null : v.session?.joints, state);
	}

	function renderRanges(joints, state) {
		const names = Object.keys(joints ?? {});
		const keys = names.join(',');
		if (keys !== rangeKeys) {
			rangeKeys = keys;
			k.ranges.innerHTML = names.map((name) => `
				<li class="range" data-joint="${esc(name)}">
					<div class="range-head">
						<span class="range-name">${esc(jointLabel(name))}</span>
						<span class="range-nums" data-r="nums"></span>
					</div>
					<div class="range-bar${isFullTurn(name) ? ' is-full' : ''}" aria-hidden="true">
						<span class="range-fill"></span><span class="range-now"></span>
					</div>
				</li>`).join('');
		}
		for (const row of k.ranges.children) {
			const name = row.dataset.joint;
			const { now, min, max } = joints[name] ?? {};
			const isFull = isFullTurn(name);
			const isRecording = state === 'recording' && !isFull;
			const nums = isFull ? `지금 ${isNum(now) ? now : DASH} · 한 바퀴 (0~4095, 재지 않음)`
				: `지금 ${isNum(now) ? now : DASH}`
					+ (isRecording || state === 'saving' ? ` · 최소 ${isNum(min) ? min : DASH} · 최대 ${isNum(max) ? max : DASH}` : '');
			put(row.querySelector('[data-r=nums]'), nums);
			const bar = row.querySelector('.range-bar');
			const lo = isFull ? 0 : min;
			const hi = isFull ? TICKS : max;
			const hasRange = isNum(lo) && isNum(hi) && (isRecording || state === 'saving' || isFull);
			bar.style.setProperty('--lo', hasRange ? percent(lo) : '0%');
			bar.style.setProperty('--hi', hasRange ? percent(hi) : '0%');
			bar.style.setProperty('--now', isNum(now) ? percent(now) : '-10%');
		}
	}

	function render() {
		const v = view();
		const state = `${v.session?.state}:${v.cal?.restore_pending}`;
		if (state !== lastState) {
			if (lastState !== null) Object.keys(refusals).forEach((group) => { refusals[group] = ''; });
			lastState = state;
		}
		renderNow(v);
		renderFiles(v);
		renderSteps(v);
	}

	async function send(key, path, body = {}) {
		const group = groupOf(key);
		pending = key;
		refusals[group] = '';
		render();
		const result = await ctx.conn.post(`${config.api}/${path}`, body);
		pending = null;
		refusals[group] = result.ok ? '' : result.message;
		render();
		return result;
	}

	if (k.command) {
		put(k.command, agentCommand());
		k.copy.addEventListener('click', async () => {
			const isCopied = await copyText(k.command);
			ctx.conn.toast(isCopied ? '실행 명령을 복사했습니다' : '복사하지 못했습니다. 골라 둔 명령을 Ctrl+C 로 복사하세요',
				isCopied ? 'ok' : 'error');
		});
	}

	k.files.addEventListener('click', (event) => {
		const button = event.target.closest('[data-id]');
		if (!button) return;
		selected[side] = button.dataset.id;
		render();
	});
	k.select.addEventListener('click', async () => {
		const id = selected[side];
		if (side === 'leader') {
			setLeaderCalibration(id);
			ctx.conn.toast(`조작 화면의 리더 보정 기본값을 ${id} 로 했습니다 (이 브라우저)`, 'ok');
			render();
			return;
		}
		if (await ctx.confirm(config.selectQuestion(id))) send('select', 'select', { robot_id: id });
	});
	k.fromMotors.addEventListener('click', () => {
		k.nameForm.hidden = false;
		k.fileName.focus();
		render();
	});
	k.fileCancel.addEventListener('click', () => {
		k.nameForm.hidden = true;
		k.fileName.value = '';
		render();
	});
	k.fileName.addEventListener('input', render);
	k.nameForm.addEventListener('submit', async (event) => {
		event.preventDefault();
		const name = k.fileName.value.trim();
		if (nameReason(name, view().files, true) || fromMotorsBlock(view())) return;
		const result = await send('fromMotors', 'from_motors', { [config.idKey]: name });
		if (result.ok) {
			k.nameForm.hidden = true;
			k.fileName.value = '';
			selected[side] = name;
			render();
		}
	});
	k.restPose?.addEventListener('click', async () => {
		const isConfirmed = await ctx.confirm({
			title: '지금 자세를 접은 자세로 저장할까요?',
			message: '팔의 지금 자세(tick)를 접은 자세로 저장합니다. 로봇을 끄기 전 팔 접기가 이 자세로 갑니다. '
				+ '팔을 접은 자세로 손으로 옮겨 둔 뒤 저장하세요. 로봇 기능을 다시 켤 때 팔로워가 읽습니다.',
			confirmLabel: '접은 자세 저장',
		});
		if (isConfirmed) send('restPose', 'rest_pose');
	});
	k.check?.addEventListener('click', () => send('check', 'check'));
	k.newName.addEventListener('input', render);
	k.start.addEventListener('click', async () => {
		const name = k.newName.value.trim();
		if (startBlock(view())) return;
		const exists = !config.isStartNewOnly && view().files.some((file) => file.id === name);
		const isConfirmed = await ctx.confirm({
			title: config.startQuestion,
			message: `${config.startNote}\n${DATASET_WARNING}.`
				+ (exists ? `\n같은 이름 ${name}.json 은 ${name}.json.bak-<날짜시각> 으로 남기고 새로 씁니다.` : ''),
			confirmLabel: '새로 보정 시작',
		});
		if (!isConfirmed) return;
		const result = await send('start', 'start', { [config.idKey]: name });
		if (result.ok) k.newName.value = '';   // 끝나면 그 이름은 있는 이름이 된다
	});
	k.middle.addEventListener('click', () => send('middle', 'middle'));
	k.finish.addEventListener('click', () => send('finish', 'finish'));
	k.cancel.addEventListener('click', async () => {
		pending = null;   // 취소는 기다리는 요청과 상관없이 보낸다
		const result = await ctx.conn.post(`${config.api}/cancel`, {});
		refusals.step = result.ok ? '' : result.message;
		render();
	});
	k.restore?.addEventListener('click', async () => {
		const isConfirmed = await ctx.confirm({
			title: '예전 보정 값으로 복원할까요?',
			message: '지난 새로 보정이 끝나지 않아 모터 보정 값이 반쯤 쓰였을 수 있습니다. 새로 보정을 시작하기 전의 '
				+ '모터 값을 다시 씁니다.\n복원하는 동안 로봇 기능(카메라 · 바퀴 · 라이다)이 꺼집니다.',
			confirmLabel: '복원',
		});
		if (isConfirmed) send('restore', 'restore');
	});
	return render;
}

export function mount(el, ctx) {
	el.innerHTML = `
		<header class="page-head"><h1>캘리브레이션</h1></header>
		<p class="guide">lerobot 보정이 모터 EEPROM (Homing_Offset · Min/Max_Position_Limit)과 맞는지 보고, 맞는 파일을
			고르거나 새로 만듭니다. 명령은 조작 권한이 있어야 보냅니다.</p>
		<div class="calib-grid">
			<div data-k="follower"></div>
			<div data-k="leader"></div>
		</div>`;
	const k = refs(el);
	const renders = [createSide(k.follower, ctx, 'follower'), createSide(k.leader, ctx, 'leader')];
	ctx.on('update', () => renders.forEach((render) => render()));
}

export function unmount() {}
