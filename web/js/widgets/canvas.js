// 지도 · 레이더 canvas 가 함께 쓰는 도구 (크기 맞추기 · 테마 색 · 로봇 그림)

/**
 * canvas 픽셀 크기를 화면 크기 × devicePixelRatio 로 맞춘다. 그리기 좌표는 CSS px.
 *
 * @param {HTMLCanvasElement} canvas 대상.
 * @param {(width: number, height: number) => void} onResize 크기가 바뀔 때 (CSS px).
 * @returns {{g: CanvasRenderingContext2D, stop: () => void}}
 */
export function observeCanvas(canvas, onResize) {
	const g = canvas.getContext('2d');
	const observer = new ResizeObserver(() => {
		const rect = canvas.getBoundingClientRect();
		const ratio = window.devicePixelRatio || 1;
		canvas.width = Math.max(1, Math.round(rect.width * ratio));
		canvas.height = Math.max(1, Math.round(rect.height * ratio));
		g.setTransform(ratio, 0, 0, ratio, 0, 0);
		onResize(rect.width, rect.height);
	});
	observer.observe(canvas);
	return { g, stop: () => observer.disconnect() };
}

/** 지금 밝기의 CSS 변수 값을 읽는 함수. */
export function themeColors(el) {
	const css = getComputedStyle(el);
	return (name) => css.getPropertyValue(name).trim();
}

/**
 * 로봇: 몸체 원 (+ 앞으로 튀어나온 접은 팔 상자) + 가운데에서 몸 밖까지 뻗은 앞 방향 화살표.
 * (몸 안의 작은 삼각형은 거의 정삼각형이라 방향이 헷갈려서 화살표로 그린다.)
 *
 * @param {CanvasRenderingContext2D} g 그릴 곳.
 * @param {number} x 화면 x.
 * @param {number} y 화면 y.
 * @param {number} radius 화면 반지름.
 * @param {number} angle 앞 방향 화면 각도 (0 = 오른쪽, 시계 방향 +).
 * @param {string} color 색.
 * @param {{reach: number, halfWidth: number} | null} arm 팔 상자: 중심 → 팔 끝, 좌우 반폭
 *     (몸체 반지름에 대한 비, halfWidth < 1 < reach). null 이면 원만.
 */
export function drawRobot(g, x, y, radius, angle, color, arm = null) {
	g.save();
	g.translate(x, y);
	g.rotate(angle);
	g.beginPath();
	if (arm) {
		// 바깥 선 하나: 팔 상자 옆변이 원과 만나는 곳에서 원을 뒤로 돌아 팔 끝으로
		const half = arm.halfWidth * radius;
		const start = Math.asin(arm.halfWidth);
		g.arc(0, 0, radius, start, Math.PI * 2 - start);
		g.lineTo(arm.reach * radius, -half);
		g.lineTo(arm.reach * radius, half);
		g.closePath();
	} else {
		g.arc(0, 0, radius, 0, Math.PI * 2);
	}
	g.fillStyle = color;
	g.globalAlpha = 0.22;
	g.fill();
	g.globalAlpha = 1;
	g.lineWidth = 2;
	g.strokeStyle = color;
	g.stroke();
	const tip = radius * (arm?.reach ?? 1) + 10;
	g.beginPath();
	g.moveTo(0, 0);
	g.lineTo(tip - 6, 0);
	g.lineWidth = 3;
	g.lineCap = 'round';
	g.stroke();
	g.beginPath();
	g.moveTo(tip, 0);
	g.lineTo(tip - 9, -5.5);
	g.lineTo(tip - 9, 5.5);
	g.closePath();
	g.fill();
	g.beginPath();
	g.arc(0, 0, 2.5, 0, Math.PI * 2);
	g.fill();
	g.restore();
}
