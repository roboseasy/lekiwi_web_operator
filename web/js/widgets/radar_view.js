// 로봇 중심 레이더: base_footprint 스캔 점, 위쪽이 로봇 앞(+x), 0.5 m 고리 (SPEC §11)

import { drawRobot, observeCanvas, themeColors } from './canvas.js';

const RANGE = 3;              // [m] 화면 반지름
const RING_STEP = 0.5;        // [m]
const ROBOT_RADIUS = 0.19;    // [m]
const STALE_MS = 1500;        // 스캔이 이만큼 안 오면 '스캔 없음'

/**
 * @param {HTMLElement} root 그릴 곳 (.well).
 * @returns {{setScan: (scan: object | null) => void, destroy: () => void}}
 */
export function createRadarView(root) {
	root.classList.add('radar-view');
	root.innerHTML = `
		<canvas role="img" aria-label="레이더. 로봇 둘레 ${RANGE} m 의 라이다 점, 위쪽이 로봇 앞"></canvas>
		<p class="canvas-empty" data-k="empty">스캔 없음</p>`;
	const empty = root.querySelector('[data-k="empty"]');
	let width = 0;
	let height = 0;
	let points = [];
	let lastAt = 0;
	let isQueued = false;
	const { g, stop } = observeCanvas(root.querySelector('canvas'), (w, h) => {
		width = w;
		height = h;
		queue();
	});

	function queue() {
		if (isQueued) return;
		isQueued = true;
		requestAnimationFrame(draw);
	}

	function draw() {
		isQueued = false;
		g.clearRect(0, 0, width, height);
		if (!width || !height) return;
		const color = themeColors(root);
		const cx = width / 2;
		const cy = height / 2;
		const scale = (Math.min(width, height) / 2 - 10) / RANGE;
		g.font = `12px ${color('--font')}`;
		g.textBaseline = 'top';
		for (let step = 1; step * RING_STEP <= RANGE; step += 1) {
			const radius = step * RING_STEP * scale;
			const isMeter = step % 2 === 0;
			g.beginPath();
			g.arc(cx, cy, radius, 0, Math.PI * 2);
			g.lineWidth = 1;
			g.strokeStyle = color(isMeter ? '--line-2' : '--line');
			g.stroke();
			if (isMeter) {
				g.textAlign = 'left';
				g.fillStyle = color('--ink-3');
				g.fillText(`${step * RING_STEP} m`, cx + 5, cy - radius + 3);
			}
		}
		g.beginPath();
		g.moveTo(cx, cy - RANGE * scale);
		g.lineTo(cx, cy + RANGE * scale);
		g.moveTo(cx - RANGE * scale, cy);
		g.lineTo(cx + RANGE * scale, cy);
		g.strokeStyle = color('--line');
		g.stroke();
		g.textAlign = 'right';
		g.fillStyle = color('--accent-text');
		g.fillText('앞', cx - 5, cy - RANGE * scale + 3);
		// 로봇 앞(+x)이 화면 위, 왼쪽(+y)이 화면 왼쪽
		g.fillStyle = color('--scan');
		for (let i = 0; i + 1 < points.length; i += 2) {
			const x = points[i];
			const y = points[i + 1];
			if (Math.hypot(x, y) > RANGE) continue;
			g.fillRect(cx - y * scale - 1.5, cy - x * scale - 1.5, 3, 3);
		}
		drawRobot(g, cx, cy, Math.max(ROBOT_RADIUS * scale, 6), -Math.PI / 2, color('--accent'));
	}

	function checkStale() {
		const isStale = performance.now() - lastAt > STALE_MS;
		empty.hidden = !isStale;
		if (isStale && points.length) {
			points = [];
			queue();
		}
	}

	const timer = setInterval(checkStale, 500);
	document.addEventListener('themechange', queue);
	return {
		setScan(scan) {
			if (!Array.isArray(scan?.points)) return;
			points = scan.points;
			lastAt = performance.now();
			empty.hidden = true;
			queue();
		},
		destroy() {
			clearInterval(timer);
			stop();
			document.removeEventListener('themechange', queue);
		},
	};
}
