// 상태 값을 화면 글자로 바꾸는 도구. 값이 없거나 숫자가 아니면 em dash 를 보여 준다.

export const DASH = '—';

const ESCAPES = { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', '\'': '&#39;' };
const ON_STATES = new Set(['starting', 'running', 'stopping', 'external']);
const EXPLORING_STATES = new Set(['starting', 'exploring', 'waiting']);   // 'saving' 은 멈춘 뒤
const PROCESS_STATES = {
	stopped: ['neutral', '꺼짐'],
	starting: ['warn', '켜는 중'],
	running: ['ok', '실행 중'],
	stopping: ['warn', '끄는 중'],
	external: ['info', '외부 실행'],
	failed: ['danger', '실패'],
	idle: ['neutral', '대기'],          // save_map 만
	succeeded: ['ok', '저장함'],        // save_map 만
};
const lastHtml = new WeakMap();

/** 유한한 숫자인지. */
export function isNum(value) {
	return typeof value === 'number' && Number.isFinite(value);
}

/** 소수 digits 자리 글자. '-0.00' 은 '0.00' 으로. */
export function fixed(value, digits = 1) {
	if (!isNum(value)) return DASH;
	const text = value.toFixed(digits);
	return /^-0(\.0+)?$/.test(text) ? text.slice(1) : text;
}

/** 라디안을 도 글자로. */
export function degrees(radians, digits = 1) {
	return isNum(radians) ? fixed(radians * 180 / Math.PI, digits) : DASH;
}

/** 초를 '3일 4시간' · '12분 5초' 처럼. */
export function duration(seconds) {
	if (!isNum(seconds) || seconds < 0) return DASH;
	const total = Math.floor(seconds);
	const days = Math.floor(total / 86400);
	const hours = Math.floor(total % 86400 / 3600);
	const minutes = Math.floor(total % 3600 / 60);
	if (days > 0) return `${days}일 ${hours}시간`;
	if (hours > 0) return `${hours}시간 ${minutes}분`;
	if (minutes > 0) return `${minutes}분 ${total % 60}초`;
	return `${total % 60}초`;
}

/** 바깥에서 온 글자(SSID · 노드 이름 · 경고)를 HTML 에 넣기 전에 바꾼다. */
export function esc(value) {
	return String(value ?? '').replace(/[&<>"']/g, (char) => ESCAPES[char]);
}

/** data-k 이름으로 요소를 찾아 둔다. */
export function refs(root) {
	const found = {};
	for (const el of root.querySelectorAll('[data-k]')) found[el.dataset.k] = el;
	return found;
}

/** 글자가 바뀔 때만 쓴다. DASH 이면 is-empty (단위 표시를 숨긴다). */
export function put(el, text) {
	const value = String(text ?? DASH);
	if (el.textContent !== value) el.textContent = value;
	el.classList.toggle('is-empty', value === DASH);
}

/** HTML 이 바뀔 때만 쓴다 (4 Hz 갱신에서 선택 · 포커스가 풀리지 않게). */
export function putHtml(el, html) {
	if (lastHtml.get(el) === html) return;
	el.innerHTML = html;
	lastHtml.set(el, html);
}

/** 읽음 칸 하나 (이름 + 값 + 단위). 값은 data-k 로 찾아 put() 으로 바꾼다. */
export function readout(key, label, unit = '') {
	const unitAttr = unit ? ` data-unit="${unit}"` : '';
	return `<div class="readout"><dt>${label}</dt>`
		+ `<dd class="is-empty" data-k="${key}"${unitAttr}>${DASH}</dd></div>`;
}

/** 점과 글자로 된 상태 표시 (색만으로 알리지 않는다). 상단 바는 kind 'chip'. */
export function badge(tone, text, kind = 'badge') {
	return `<span class="${kind}" data-tone="${tone}"><span class="dot"></span>${esc(text)}</span>`;
}

/** 기능(launch)이 켜져 있거나 켜지는 · 꺼지는 중인지. */
export function isOn(process) {
	return ON_STATES.has(process?.state);
}

/** 자동 주행 맵 생성(status.exploration)이 로봇을 움직이는 중인지. */
export function isExploring(exploration) {
	return EXPLORING_STATES.has(exploration?.state);
}

/** 기능 상태 배지. */
export function processBadge(process) {
	const [tone, text] = PROCESS_STATES[process?.state] ?? ['neutral', DASH];
	return badge(tone, text);
}
