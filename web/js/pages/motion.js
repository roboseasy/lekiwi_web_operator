// 이동 설계: 직선 왕복 · 원을 브라우저가 odom · IMU(자이로)를 보며 계산해서 그린다. 로봇 쪽 코드는 그대로다.
// 제어 로직은 ../motion/*.js (DOM 없음, tests/js 에서 시뮬레이션으로 시험). 이 파일은 화면 · 안전 조건 · 전송만 맡는다.
// 안전: 센서가 1 s 끊기거나, 창이 가려지거나, 권한 · 토크 · 비상 정지가 바뀌면 멈춘다. 서버도 teleop 이 0.3 s 끊기면 0 이다.

import { DASH, degrees, fixed, isExploring, put, putHtml, readout, refs } from '../format.js';
import { capCommand, maxCircleSpeed, maxLineSpeed, toTeleop } from '../motion/command.js';
import { DEFAULT_TAU } from '../motion/estimator.js';
import { toCsv } from '../motion/metrics.js';
import { MotionRun } from '../motion/run.js';
import { readSample } from '../motion/sample.js';

export const title = '이동 설계';
export const channels = ['odom', 'imu'];

const SEND_MS = 100;          // teleop 전송 주기 (서버는 0.3 s 끊기면 0)
const COUNTDOWN = 3;          // [s] 시작 전 카운트다운
const STOP_HOLD = 1.0;        // [s] 멈춘 뒤 0 을 더 보내는 시간
const COMMAND_MAX_AGE = 0.6;  // [s] 센서 샘플이 이보다 오래 안 오면 마지막 명령을 되풀이하지 않고 0 을 보낸다 (눈 감고 달리지 않게)
const CSV_COLUMNS = [
	't', 'odomX', 'odomY', 'odomYaw', 'gyroZ', 'gyroYaw', 'fusedX', 'fusedY', 'fusedYaw',
	'vx', 'vy', 'wz', 'primaryError', 'headingError',
];
const DEFAULT_SPEED = { line: 0.10, circle: 0.12 };
const ZERO = { vx: 0, vy: 0, wz: 0 };
const POWER_ON_QUESTION = {
	title: '조작을 켤까요?',
	message: '바퀴 토크를 켭니다. 바퀴가 움직입니다. 주변에 사람 · 케이블이 없고, 처음에는 바퀴를 띄웠는지 확인하세요.',
	confirmLabel: '조작 켜기',
};

let cleanup = () => {};

export function mount(el, ctx) {
	el.innerHTML = `
		<header class="page-head"><h1>이동 설계</h1></header>
		<div class="motion">
			<section class="panel" aria-labelledby="motion-plan">
				<div class="panel-head"><h2 id="motion-plan">동작</h2></div>
				<label class="field-label" for="motion-kind">동작</label>
				<select id="motion-kind" class="input" data-k="kind">
					<option value="line">직선 왕복</option>
					<option value="circle">원</option>
				</select>
				<div data-k="lineFields">
					<label class="field-label" for="motion-distance">거리 (m) — 앞으로 갔다가 같은 거리 돌아오기가 1회</label>
					<input id="motion-distance" class="input" type="number" min="0.2" max="3" step="0.1" value="1.0" data-k="distance">
				</div>
				<div data-k="circleFields" hidden>
					<label class="field-label" for="motion-radius">반지름 (m)</label>
					<input id="motion-radius" class="input" type="number" min="0.2" max="2" step="0.1" value="0.5" data-k="radius">
					<label class="field-label" for="motion-direction">방향</label>
					<select id="motion-direction" class="input" data-k="direction">
						<option value="1">왼쪽 (반시계)</option>
						<option value="-1">오른쪽 (시계)</option>
					</select>
				</div>
				<label class="field-label" for="motion-speed">속도 (m/s)</label>
				<input id="motion-speed" class="input" type="number" min="0.02" max="0.23" step="0.01" value="0.10" data-k="speed">
				<p class="hint" data-k="limit"></p>
				<label class="field-label" for="motion-repeat">횟수</label>
				<select id="motion-repeat" class="input" data-k="repeat">
					<option value="once">1회</option>
					<option value="forever">무한</option>
				</select>
				<label class="check"><input type="checkbox" data-k="useImu" checked> IMU 방향 사용 (끄면 odom 방향만)</label>
				<label class="field-label" for="motion-tau">방향 보정 시간 τ (초) — 작으면 odom, 크면 자이로를 믿음</label>
				<input id="motion-tau" class="input" type="number" min="5" max="1000" step="5" value="${DEFAULT_TAU}" data-k="tau">
			</section>
			<section class="panel" aria-labelledby="motion-run">
				<div class="panel-head"><h2 id="motion-run">실행</h2></div>
				<div class="actions">
					<button type="button" class="btn" data-k="control"></button>
					<button type="button" class="btn" data-k="power"></button>
				</div>
				<div class="actions">
					<button type="button" class="btn btn-primary" data-k="start">시작</button>
					<button type="button" class="btn btn-danger" data-k="stop">정지</button>
				</div>
				<p class="notice" data-k="reason" role="status"></p>
				<dl class="readouts readouts-3">
					${readout('state', '상태')}
					${readout('progress', '진행')}
					${readout('primary', '반지름/가로 오차', 'm')}
					${readout('heading', '방향 오차', '°')}
					${readout('x', '위치 x', 'm')}
					${readout('y', '위치 y', 'm')}
					${readout('fusedYaw', '융합 방향', '°')}
					${readout('odomYaw', 'odom 방향', '°')}
					${readout('gyroYaw', '자이로 적분', '°')}
				</dl>
				<h3 class="sub">결과 (odom · 융합 방향 기준)</h3>
				<ul class="motion-laps" data-k="laps"></ul>
				<div class="actions">
					<button type="button" class="btn btn-small" data-k="csv">로그 CSV 저장</button>
				</div>
			</section>
		</div>`;
	const k = refs(el);

	// ── 상태 ──
	let isMounted = true;
	let phase = 'idle';              // 'idle' | 'countdown' | 'running' | 'stopping'
	let countdownEnd = 0;
	let stopUntil = 0;
	let message = '';
	let messageTone = '';
	let latest = null;               // 가장 최근 신선한 센서 샘플 { t, odom, gyroZ }
	let lastSampleT = null;
	let lastArrival = 0;             // 그 샘플이 도착한 브라우저 시각 [s]
	let command = ZERO;
	let commandAt = 0;               // 그 명령을 계산한 브라우저 시각 [s]
	let plan = null;                 // 시작 때 읽은 매개변수
	let run = null;                  // MotionRun
	let isPowerBusy = false;

	const now = () => performance.now() / 1000;
	const levels = () => ctx.status?.teleop?.pads?.drive?.levels ?? [];
	const limits = () => ctx.status?.teleop?.limits ?? null;
	const serverNow = () => (lastSampleT ?? 0) + (now() - lastArrival);

	function send(cmd) {
		const input = toTeleop(capCommand(cmd, levels()), levels());
		if (!input) return false;
		ctx.conn.send({ type: 'teleop', ...input, pad: 'drive' });
		return true;
	}

	function sendZero() {
		ctx.conn.send({ type: 'teleop', x: 0, y: 0, yaw: 0, level: 0, pad: 'drive' });
	}

	/** 화면의 입력을 읽는다. 잘못되면 { error }. */
	function readPlan() {
		const kind = k.kind.value;
		const speed = Number(k.speed.value);
		const repeat = k.repeat.value === 'forever' ? Infinity : 1;
		const tau = Number(k.tau.value);
		if (!(tau >= 5 && tau <= 1000)) return { error: 'τ 는 5~1000 초로 정하세요' };
		const base = { kind, speed, useImu: k.useImu.checked, tau };
		if (kind === 'circle') {
			const radius = Number(k.radius.value);
			if (!(radius >= 0.2 && radius <= 2)) return { error: '반지름은 0.2~2 m 로 정하세요' };
			const cap = maxCircleSpeed(radius, limits());
			if (!(speed >= 0.02 && speed <= cap + 1e-9)) return { error: `속도는 0.02 이상 ${fixed(cap, 2)} m/s 이하로 정하세요 (반지름 ${radius} m 의 상한)` };
			return { ...base, radius, direction: Number(k.direction.value), laps: repeat };
		}
		const distance = Number(k.distance.value);
		if (!(distance >= 0.2 && distance <= 3)) return { error: '거리는 0.2~3 m 로 정하세요' };
		const cap = maxLineSpeed(limits());
		if (!(speed >= 0.02 && speed <= cap + 1e-9)) return { error: `속도는 0.02 이상 ${fixed(cap, 2)} m/s 이하로 정하세요` };
		return { ...base, distance, rounds: repeat };
	}

	/** 시작할 수 없는 이유. 시작해도 되면 null. */
	function startBlock() {
		const status = ctx.status;
		if (!ctx.isConnected) return '서버에 연결되지 않았습니다';
		if (document.hidden) return '창이 가려져 있습니다 (이 창이 보여야 계산이 돕니다)';
		const blocked = ctx.blockReason();
		if (blocked) return blocked;
		if (!ctx.isMine()) return '조작 권한을 먼저 받으세요';
		if (status?.estop?.is_latched === true) return '비상 정지가 걸려 있습니다';
		if (isExploring(status?.exploration)) return '자동 주행 중에는 시작할 수 없습니다';
		if (status?.motor?.ready !== true) return '‘조작 켜기’를 눌러 바퀴 토크를 켜세요';
		if (status?.teleop?.is_allowed !== true) return status?.teleop?.blocked_reason ?? '지금은 조작할 수 없습니다';
		if (!status?.imu) return 'IMU 값이 오지 않아 시작할 수 없습니다 (IMU 가 필수입니다)';
		if (!latest || now() - lastArrival > 1.0) return '신선한 odom · IMU 값을 기다리는 중입니다';
		const parsed = readPlan();
		if (parsed.error) return parsed.error;
		const nominal = { vx: parsed.speed, vy: 0, wz: parsed.kind === 'circle' ? parsed.speed / parsed.radius : 0 };
		if (!toTeleop(nominal, levels())) return '이 속도 · 반지름은 조작 화면의 속도 단계 상한을 넘습니다';
		return null;
	}

	// ── 시작 · 정지 ──
	async function requestStart() {
		if (phase !== 'idle' || startBlock()) return;
		const wanted = readPlan();
		const clearance = wanted.kind === 'circle' ? wanted.radius * 2 + 0.6 : wanted.distance + 0.6;
		const confirmed = await ctx.confirm({
			title: '이동을 시작할까요?',
			message: `카운트다운 뒤 로봇이 스스로 움직입니다. ${wanted.kind === 'circle' ? `가로세로 ${clearance.toFixed(1)} m` : `앞뒤 ${clearance.toFixed(1)} m`} 이상 빈 바닥이 필요합니다. `
				+ '멈추려면 Space · Esc · 정지 버튼 · 상단 비상 정지를 누르세요. 이 창이 가려지면 멈춥니다.',
			confirmLabel: '시작',
		});
		if (!confirmed || !isMounted || phase !== 'idle' || startBlock()) return;   // 기다리는 동안 상태가 바뀌었을 수 있다
		plan = readPlan();
		phase = 'countdown';
		countdownEnd = now() + COUNTDOWN;
		setMessage('', '');
		render();
	}

	function begin() {
		if (!latest || now() - lastArrival > 1.0) return cancel('센서 값이 오지 않아 시작하지 못했습니다');
		run = new MotionRun(plan, latest);
		command = ZERO;
		commandAt = now();
		phase = 'running';
		render();
	}

	function cancel(text) {
		phase = 'idle';
		setMessage(text, 'warn');
		render();
	}

	function stop(reason, tone = 'warn') {
		if (phase === 'idle' || phase === 'stopping') return;
		command = ZERO;
		sendZero();
		if (phase === 'countdown') {
			cancel(reason);
			return;
		}
		phase = 'stopping';
		stopUntil = now() + STOP_HOLD;
		setMessage(reason, tone);
		render();
	}

	function setMessage(text, tone) {
		message = text;
		messageTone = tone;
	}

	// ── 새 센서 샘플 ──
	function onSample(sample) {
		try {
			if (phase === 'running') {
				const outcome = run.step(sample);
				command = outcome.command;
				commandAt = now();
				if (outcome.stop) stop(outcome.stop.reason, outcome.stop.tone);
			} else if (phase === 'stopping' && run) {
				run.settle(sample);
			}
		} catch (error) {
			console.error(error);
			stop('내부 오류로 멈췄습니다', 'danger');   // 예외가 나면 마지막 명령을 계속 보내지 않는다
		}
	}

	function onUpdate() {
		const sample = readSample(ctx.status);
		if (sample) {
			if (lastSampleT !== null && sample.t < lastSampleT) {   // 서버 시계가 거꾸로 갔다
				if (phase === 'running') stop('서버 시계가 거꾸로 가서 멈췄습니다', 'danger');
				lastSampleT = null;
			}
			if (sample.t !== lastSampleT) {
				lastSampleT = sample.t;
				lastArrival = now();
				latest = sample;
				onSample(sample);
			}
		}
		render();
	}

	// ── 0.1 s 주기 ──
	function tick() {
		if (phase === 'countdown') {
			const startError = startBlock();
			if (startError) return cancel(startError);
			// 서버는 비상 정지 · 조작 끄기 뒤에 0 입력을 한 번 받을 때까지 방향 입력을 버린다 (teleop.py _is_zero_required).
			// 카운트다운 동안 0 을 보내 그 잠금을 푼다 (조작 화면 teleop.js 도 권한이 있는 동안 0.1 s 마다 0 을 보낸다)
			sendZero();
			if (now() >= countdownEnd) begin();
		} else if (phase === 'running') {
			const reason = run.checkStale(serverNow());
			if (reason) return stop(reason, 'danger');
			if (now() - commandAt > COMMAND_MAX_AGE) sendZero();   // 새 샘플이 늦으면 눈 감고 달리지 않는다 (1 s 정지 한도 전에 먼저 선다)
			else if (!send(command)) return stop('속도가 조작 화면의 단계 상한을 넘어 멈췄습니다', 'danger');
		} else if (phase === 'stopping') {
			sendZero();
			if (now() >= stopUntil) {
				phase = 'idle';
				run?.finish();   // 멈춘 자리에서 마지막 바퀴 결과를 낸다
			}
		}
		render();
	}

	// ── 화면 ──
	function render() {
		const status = ctx.status;
		const isMine = ctx.isMine();
		const isReady = status?.motor?.ready === true;
		const isBusy = phase !== 'idle';

		// 실행 중에 조건이 바뀌면 멈춘다
		if (phase === 'running') {
			if (!isMine) stop('조작 권한을 잃어 멈췄습니다', 'danger');
			else if (!isReady) stop('바퀴 토크가 꺼져 멈췄습니다', 'danger');
			else if (status?.estop?.is_latched === true) stop('비상 정지로 멈췄습니다', 'danger');
			else if (isExploring(status?.exploration)) stop('자동 주행이 시작되어 멈췄습니다', 'danger');
			else if (status?.teleop?.is_allowed !== true) stop(status?.teleop?.blocked_reason ?? '조작이 막혀 멈췄습니다', 'danger');
		}

		const isCircle = k.kind.value === 'circle';
		k.lineFields.hidden = isCircle;
		k.circleFields.hidden = !isCircle;
		for (const input of el.querySelectorAll('.motion input, .motion select')) input.disabled = isBusy;
		const cap = isCircle ? maxCircleSpeed(Number(k.radius.value) || 0.5, limits()) : maxLineSpeed(limits());
		put(k.limit, `속도 상한 ${fixed(cap, 2)} m/s (바퀴 상한의 80 %)`);

		k.control.textContent = isMine ? '조작 권한 놓기' : '조작 권한 받기';
		k.control.classList.toggle('btn-primary', !isMine);
		k.control.disabled = isBusy || !ctx.isConnected || Boolean(ctx.info?.is_read_only)
			|| (!isMine && Boolean(status?.control?.holder));
		k.power.textContent = isPowerBusy ? '조작 켜는 중…' : (isReady ? '조작 끄기' : '조작 켜기');
		k.power.classList.toggle('btn-primary', !isReady);
		// 토크 끄기는 실행 중에도 늘 누를 수 있다 (안전한 쪽)
		k.power.disabled = isReady ? false : (isPowerBusy || isBusy || Boolean(ctx.blockReason()) || status?.teleop?.is_allowed !== true);
		const block = phase === 'idle' ? startBlock() : null;
		k.start.disabled = isBusy || Boolean(block);
		k.stop.disabled = !isBusy;
		k.stop.textContent = phase === 'countdown' ? '취소' : '정지';

		const text = phase === 'idle' ? (block ?? message) : message;   // 대기 중에는 지금 시작할 수 없는 이유가 먼저다
		k.reason.dataset.tone = phase === 'idle' && block ? 'info' : messageTone;
		if (k.reason.textContent !== text) k.reason.textContent = text;

		const info = run?.info ?? null;
		const states = { idle: '대기', countdown: `${Math.max(0, Math.ceil(countdownEnd - now()))}초 뒤 시작`, running: '진행 중', stopping: '멈추는 중' };
		put(k.state, states[phase]);
		const target = plan ? (plan.kind === 'circle' ? plan.laps : plan.rounds) : 0;
		const total = Number.isFinite(target) ? target : '∞';
		put(k.progress, info ? (plan.kind === 'circle'
			? `${fixed(info.angle / (2 * Math.PI), 2)} / ${total} 바퀴`
			: `왕복 ${run.lapsDone} / ${total} · ${info.leg}`) : DASH);
		put(k.primary, info ? fixed(plan.kind === 'circle' ? info.radiusError : info.lateralError, 3) : DASH);
		put(k.heading, info ? degrees(info.headingError, 1) : DASH);
		const live = run && phase !== 'idle';
		put(k.x, live ? fixed(run.estimator.pose.x, 3) : DASH);
		put(k.y, live ? fixed(run.estimator.pose.y, 3) : DASH);
		put(k.fusedYaw, live ? degrees(run.estimator.pose.yaw, 1) : DASH);
		put(k.odomYaw, live ? degrees(run.estimator.odomYaw, 1) : DASH);
		put(k.gyroYaw, live ? degrees(run.estimator.gyroYaw, 1) : DASH);
		const results = run?.results ?? [];
		putHtml(k.laps, results.length ? results.map((line) => `<li>${line}</li>`).join('') : '<li class="hint">아직 없습니다</li>');
		k.csv.disabled = !run || run.rows.length === 0;
	}

	// ── 이벤트 ──
	async function setPower(isEnabled) {
		if (!isEnabled) stop('조작 끄기로 멈췄습니다');   // 응답을 기다리지 않는다
		if (isEnabled) {
			if (isPowerBusy || !(await ctx.confirm(POWER_ON_QUESTION))) return;
			isPowerBusy = true;
			render();
		}
		await ctx.conn.post('/api/motor/power', { enabled: isEnabled });
		if (isEnabled) {
			isPowerBusy = false;
			render();
		}
	}

	function downloadCsv() {
		const rows = run.rows;
		const header = run.results.map((line) => `# ${line}\n`).join('');   // pandas 는 read_csv(comment='#')
		const url = URL.createObjectURL(new Blob([header, toCsv(rows, CSV_COLUMNS)], { type: 'text/csv' }));
		const link = document.createElement('a');
		link.href = url;
		link.download = `motion-${plan?.kind ?? 'log'}-${new Date().toISOString().replace(/[:.]/g, '-')}.csv`;
		link.click();
		setTimeout(() => URL.revokeObjectURL(url), 1000);   // 바로 풀면 일부 브라우저가 내려받기를 취소한다
	}

	function onKeyDown(event) {
		if (phase !== 'idle' && (event.code === 'Space' || event.code === 'Escape')) {
			event.preventDefault();
			stop('정지 키로 멈췄습니다');
		}
	}

	const onHidden = () => {
		if (document.hidden) stop('창이 가려져 멈췄습니다 (이 창이 보여야 계산이 돕니다)');
	};
	const onPageHide = () => stop('페이지를 떠나 멈췄습니다');

	k.kind.addEventListener('change', () => {
		k.speed.value = String(DEFAULT_SPEED[k.kind.value]);
		render();
	});
	for (const input of [k.radius, k.speed, k.distance, k.tau, k.direction, k.repeat, k.useImu]) input.addEventListener('input', render);
	k.control.addEventListener('click', () => (ctx.isMine() ? ctx.release() : ctx.acquire()));
	k.power.addEventListener('click', () => setPower(ctx.status?.motor?.ready !== true));
	k.start.addEventListener('click', requestStart);
	k.stop.addEventListener('click', () => stop(phase === 'countdown' ? '취소했습니다' : '정지 버튼으로 멈췄습니다', 'info'));
	k.csv.addEventListener('click', downloadCsv);
	window.addEventListener('keydown', onKeyDown);
	document.addEventListener('visibilitychange', onHidden);
	window.addEventListener('pagehide', onPageHide);
	ctx.on('update', onUpdate);
	ctx.on('estop', () => stop('비상 정지로 멈췄습니다', 'danger'));
	const timer = setInterval(tick, SEND_MS);

	k.speed.value = String(DEFAULT_SPEED.line);
	onUpdate();

	cleanup = () => {
		isMounted = false;
		if (phase !== 'idle') {
			for (let i = 0; i < 3; i += 1) sendZero();
		}
		clearInterval(timer);
		window.removeEventListener('keydown', onKeyDown);
		document.removeEventListener('visibilitychange', onHidden);
		window.removeEventListener('pagehide', onPageHide);
		phase = 'idle';
	};
}

export function unmount() {
	cleanup();
	cleanup = () => {};
}
