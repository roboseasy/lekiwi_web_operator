// 로그: 웹에서 켠 기능의 실시간 출력 (마지막 1000줄, 맨 아래 따라가기) (SPEC §11-7)

import { processBadge, putHtml, refs } from '../format.js';

export const title = '로그';
export const channels = ['logs/bringup'];

const PROCESSES = [
	['bringup', '로봇'],
	['mapping', '지도 작성'],
	['navigation', '자율 주행'],
	['save_map', '지도 저장'],
];
const MAX_LINES = 1000;

let selected = 'bringup';   // 화면을 다시 열어도 기억

export function mount(el, ctx) {
	const buttons = PROCESSES.map(([name, label]) => `
		<button type="button" data-name="${name}" aria-pressed="${name === selected}">
			${label}
		</button>`).join('');
	el.innerHTML = `
		<header class="page-head"><h1>로그</h1><span data-k="badge"></span></header>
		<div class="log-bar">
			<div class="seg" role="group" aria-label="프로세스" data-k="picker">${buttons}</div>
			<label class="check"><input type="checkbox" data-k="follow" checked> 맨 아래 따라가기</label>
		</div>
		<pre class="log" data-k="log" tabindex="0" aria-label="로그"></pre>`;
	const k = refs(el);
	let lines = [];
	let hasBacklog = false;

	function paint() {
		let text = lines.join('\n');
		if (!lines.length && !ctx.isConnected) text = '서버에 연결되면 로그를 보여 줍니다';
		else if (!lines.length && !hasBacklog) text = '로그를 불러오는 중입니다…';
		else if (!lines.length) text = '아직 로그가 없습니다. 웹에서 켠 기능의 출력만 보입니다.';
		if (k.log.textContent !== text) k.log.textContent = text;
		if (k.follow.checked) k.log.scrollTop = k.log.scrollHeight;
	}

	function render() {
		putHtml(k.badge, processBadge(ctx.status?.processes?.[selected]));
		paint();
	}

	function select(name) {
		// 같은 채널을 다시 구독하면 서버가 backlog 를 다시 보내지 않으므로 지금 줄을 지우지 않는다
		if (name === selected && hasBacklog) return;
		selected = name;
		lines = [];
		hasBacklog = false;
		for (const button of k.picker.querySelectorAll('[data-name]')) {
			button.setAttribute('aria-pressed', String(button.dataset.name === name));
		}
		ctx.setChannels([`logs/${name}`]);   // 서버가 구독 순간 backlog 를 먼저 보낸다
		render();
	}

	k.picker.addEventListener('click', (event) => {
		const button = event.target.closest('[data-name]');
		if (button) select(button.dataset.name);
	});
	k.follow.addEventListener('change', paint);
	ctx.on('log', (message) => {
		if (message.process !== selected || !Array.isArray(message.lines)) return;
		if (message.is_backlog) {
			lines = [...message.lines];
			hasBacklog = true;
		} else {
			lines.push(...message.lines);
		}
		if (lines.length > MAX_LINES) lines = lines.slice(-MAX_LINES);
		paint();
	});
	ctx.on('update', render);
	select(selected);
}

export function unmount() {}
