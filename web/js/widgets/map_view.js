// 지도 canvas: 지도 PNG · 1 m 격자 · 경로 · 목표 · 스캔 · 로봇, 끌어서 이동 · 휠 · 핀치 확대, 위치 도구.
// 지도 작성 · 자율 주행 화면이 함께 쓴다 (SPEC §11).
// 좌표: 지도 픽셀 (u, v) ↔ 월드 x = ox + (u + 0.5)·res, y = oy + (H − v − 0.5)·res (png 첫 줄이 +y 끝).

import { isNum, refs } from '../format.js';
import { drawRobot, observeCanvas, themeColors } from './canvas.js';

const MIN_SCALE = 2;          // [px/m]
const MAX_SCALE = 400;        // [px/m]
const ROBOT_RADIUS = 0.19;    // [m] 몸체 · 바퀴 원 (config.EXPLORE_BODY_RADIUS)
// 접은 팔: 중심 → 팔 끝 0.22 m, 좌우 반폭 0.10 m (config.EXPLORE_ARM_REACH · EXPLORE_ARM_HALF_WIDTH).
// 자동 주행이 충돌을 판단하는 모양을 그대로 보인다 (몸체 반지름에 대한 비)
const ARM = { reach: 0.22 / ROBOT_RADIUS, halfWidth: 0.10 / ROBOT_RADIUS };
const ROBOT_VIEW = 5;         // [m] '로봇에 맞춤' 때 화면 짧은 변에 담는 거리
const GRID_MIN_SCALE = 8;     // [px/m] 이보다 작게 보면 격자가 너무 촘촘해 생략
const DRAG_MIN_PX = 8;        // 이보다 짧게 끌면 방향 0 (RViz 와 같다)
const TOOLS = [['pan', '이동'], ['initial', '초기 위치'], ['goal', '목표']];

function rgb(hex) {
	const value = Number.parseInt(hex.replace('#', ''), 16);
	return [value >> 16 & 255, value >> 8 & 255, value & 255];
}

function isPoint(point) {
	return isNum(point?.x) && isNum(point?.y);
}

function isPose(pose) {
	return isPoint(pose) && isNum(pose.yaw);
}

function isMapInfo(info) {
	return isNum(info?.width) && info.width > 0 && isNum(info.height) && info.height > 0
		&& isNum(info.resolution) && info.resolution > 0
		&& isNum(info.origin?.x) && isNum(info.origin?.y);
}

/** 회색 지도(흰색 빈 곳 · 검은색 벽)를 지금 밝기의 두 색 사이로 다시 칠한다. */
function tint(image, free, occupied) {
	const canvas = document.createElement('canvas');
	canvas.width = image.naturalWidth;
	canvas.height = image.naturalHeight;
	const g = canvas.getContext('2d');
	g.drawImage(image, 0, 0);
	const pixels = g.getImageData(0, 0, canvas.width, canvas.height);
	const data = pixels.data;
	const dark = rgb(occupied);
	const light = rgb(free);
	for (let i = 0; i < data.length; i += 4) {
		const t = data[i] / 255;
		data[i] = dark[0] + (light[0] - dark[0]) * t;
		data[i + 1] = dark[1] + (light[1] - dark[1]) * t;
		data[i + 2] = dark[2] + (light[2] - dark[2]) * t;
	}
	g.putImageData(pixels, 0, 0);
	return canvas;
}

/**
 * @param {HTMLElement} root 그릴 곳 (.well).
 * @param {((tool: string, pose: {x: number, y: number, yaw: number}) => void) | null} onPose
 *     위치 도구('initial' · 'goal')를 놓았을 때. null 이면 도구 없이 보기만 한다.
 */
export function createMapView(root, onPose = null) {
	root.classList.add('map-view');
	const toolButtons = TOOLS.map(([tool, label]) => `
		<button type="button" data-tool="${tool}" aria-pressed="${tool === 'pan'}">
			${label}
		</button>`).join('');
	const toolBar = onPose
		? `<div class="seg seg-float" role="group" aria-label="지도 도구">${toolButtons}</div>`
		: '<span></span>';
	root.innerHTML = `
		<canvas data-tool="pan" aria-label="지도. 끌어서 옮기고 휠이나 두 손가락으로 확대합니다"></canvas>
		<div class="map-bar">
			${toolBar}
			<button type="button" class="btn btn-small btn-float" data-k="fit">로봇에 맞춤</button>
		</div>
		<p class="map-note" data-k="note" hidden>지도 원점이 회전돼 있어(yaw ≠ 0) 회전은 무시하고 그립니다</p>
		<p class="canvas-empty" data-k="empty">지도를 기다리는 중입니다</p>`;
	const k = refs(root);
	const canvas = root.querySelector('canvas');
	const view = { cx: 0, cy: 0, scale: 40 };
	const pointers = new Map();   // 이동 · 핀치: pointerId → [x, y]
	let width = 0;
	let height = 0;
	let map = null;               // { info, image, tinted, tintKey }
	let mapMessage = null;        // 마지막으로 받은 map 메시지 (읽는 중이어도)
	let loadToken = 0;
	let isFitted = false;
	let robot = null;
	let scan = null;              // { points, pose } pose = 그 스캔을 받았을 때 로봇 자세 (map)
	let plan = null;
	let goal = null;
	let tool = 'pan';
	let isToolsEnabled = false;
	let drag = null;              // 위치 도구: { pointerId, x, y, yaw, sx, sy, last }
	let isQueued = false;
	const { g, stop } = observeCanvas(canvas, (w, h) => {
		width = w;
		height = h;
		if (!isFitted) fitMap();
		queue();
	});

	const toScreen = (x, y) => [
		width / 2 + (x - view.cx) * view.scale,
		height / 2 - (y - view.cy) * view.scale,
	];
	const toWorld = (sx, sy) => [
		view.cx + (sx - width / 2) / view.scale,
		view.cy - (sy - height / 2) / view.scale,
	];
	const clampScale = (scale) => Math.min(Math.max(scale, MIN_SCALE), MAX_SCALE);

	function queue() {
		if (isQueued) return;
		isQueued = true;
		requestAnimationFrame(draw);
	}

	function fitMap() {
		if (!map || !width || !height) return;
		const { width: w, height: h, resolution: res, origin } = map.info;
		view.scale = clampScale(Math.min(width / (w * res), height / (h * res)) * 0.92);
		view.cx = origin.x + w * res / 2;
		view.cy = origin.y + h * res / 2;
		isFitted = true;
	}

	function fitRobot() {
		if (robot) {
			view.scale = clampScale(Math.min(width, height) / ROBOT_VIEW);
			view.cx = robot.x;
			view.cy = robot.y;
		} else {
			fitMap();
		}
		queue();
	}

	function zoomAt(sx, sy, factor) {
		const [wx, wy] = toWorld(sx, sy);
		view.scale = clampScale(view.scale * factor);
		view.cx = wx - (sx - width / 2) / view.scale;
		view.cy = wy + (sy - height / 2) / view.scale;
	}

	function setTool(next) {
		tool = next;
		canvas.dataset.tool = tool;
		for (const button of root.querySelectorAll('button[data-tool]')) {
			button.setAttribute('aria-pressed', String(button.dataset.tool === tool));
		}
	}

	function drawGrid(color) {
		if (view.scale < GRID_MIN_SCALE) return;
		const [left, top] = toWorld(0, 0);
		const [right, bottom] = toWorld(width, height);
		g.beginPath();
		for (let x = Math.ceil(left); x <= right; x += 1) {
			const sx = Math.round(toScreen(x, 0)[0]) + 0.5;
			g.moveTo(sx, 0);
			g.lineTo(sx, height);
		}
		for (let y = Math.ceil(bottom); y <= top; y += 1) {
			const sy = Math.round(toScreen(0, y)[1]) + 0.5;
			g.moveTo(0, sy);
			g.lineTo(width, sy);
		}
		g.lineWidth = 1;
		g.strokeStyle = color;
		g.stroke();
	}

	function drawPath(points, color) {
		g.beginPath();
		for (let i = 0; i + 1 < points.length; i += 2) {
			const [sx, sy] = toScreen(points[i], points[i + 1]);
			if (i === 0) g.moveTo(sx, sy);
			else g.lineTo(sx, sy);
		}
		g.lineWidth = 3;
		g.lineJoin = 'round';
		g.lineCap = 'round';
		g.strokeStyle = color;
		g.stroke();
	}

	/** 위치 + 방향 화살표 (목표 · 도구로 끄는 중인 자세). 방향이 없으면 (자동 주행 목표) 고리. */
	function drawPose(pose, color) {
		const [sx, sy] = toScreen(pose.x, pose.y);
		g.save();
		g.translate(sx, sy);
		g.fillStyle = color;
		g.strokeStyle = color;
		g.lineWidth = 3;
		g.beginPath();
		g.arc(0, 0, 5, 0, Math.PI * 2);
		g.fill();
		g.beginPath();
		if (isNum(pose.yaw)) {
			g.rotate(-pose.yaw);   // 화면 y 가 아래로 커서 부호가 바뀐다
			g.moveTo(0, 0);
			g.lineTo(26, 0);
			g.stroke();
			g.beginPath();
			g.moveTo(36, 0);
			g.lineTo(24, -7);
			g.lineTo(24, 7);
			g.closePath();
			g.fill();
		} else {
			g.arc(0, 0, 12, 0, Math.PI * 2);
			g.stroke();
		}
		g.restore();
	}

	/** base_footprint 스캔 점을 스캔을 받을 때 서버가 알던 로봇 자세(map, tf2_echo 0.1 s 값)로 옮겨 찍는다. */
	function drawScan(color) {
		const { points, pose } = scan;
		const cos = Math.cos(pose.yaw);
		const sin = Math.sin(pose.yaw);
		g.fillStyle = color;
		for (let i = 0; i + 1 < points.length; i += 2) {
			const x = pose.x + cos * points[i] - sin * points[i + 1];
			const y = pose.y + sin * points[i] + cos * points[i + 1];
			const [sx, sy] = toScreen(x, y);
			g.fillRect(sx - 1.5, sy - 1.5, 3, 3);
		}
	}

	function draw() {
		isQueued = false;
		g.clearRect(0, 0, width, height);
		if (!width || !height) return;
		const color = themeColors(root);
		if (map) {
			const key = color('--map-free') + color('--map-occupied');
			if (map.tintKey !== key) {
				map.tinted = tint(map.image, color('--map-free'), color('--map-occupied'));
				map.tintKey = key;
			}
			const { width: w, height: h, resolution: res, origin } = map.info;
			const [left, top] = toScreen(origin.x, origin.y + h * res);
			g.imageSmoothingEnabled = view.scale * res < 1;   // 칸이 1 px 보다 크면 또렷하게
			g.drawImage(map.tinted, left, top, w * res * view.scale, h * res * view.scale);
		}
		drawGrid(color('--grid'));
		if (plan) drawPath(plan, color('--info'));
		if (goal) drawPose(goal, color('--ok'));
		if (robot && scan) drawScan(color('--scan'));   // 로봇을 그리지 않을 때(꺼짐)는 스캔도 숨긴다
		if (robot) {
			const [sx, sy] = toScreen(robot.x, robot.y);
			const radius = Math.max(ROBOT_RADIUS * view.scale, 7);
			drawRobot(g, sx, sy, radius, -robot.yaw, color('--accent'), ARM);
		}
		if (drag) drawPose(drag, color(tool === 'goal' ? '--ok' : '--warn'));
	}

	function local(event) {
		const rect = canvas.getBoundingClientRect();
		return [event.clientX - rect.left, event.clientY - rect.top];
	}

	function onPointerDown(event) {
		const point = local(event);
		canvas.setPointerCapture(event.pointerId);
		// 위치 도구는 손가락 하나 · 왼쪽 버튼으로만 놓는다 (오른쪽 · 가운데 버튼은 이동)
		if (tool !== 'pan' && isToolsEnabled && event.button === 0 && !drag && !pointers.size) {
			const [x, y] = toWorld(point[0], point[1]);
			const [sx, sy] = point;
			drag = { pointerId: event.pointerId, x, y, yaw: 0, sx, sy, last: point };
			queue();
			return;
		}
		if (drag) {
			// 두 번째 손가락은 핀치다: 놓던 자세는 보내지 않고 버린 뒤 두 손가락 확대로 넘긴다 (도구는 그대로)
			pointers.set(drag.pointerId, drag.last);
			drag = null;
			queue();
		}
		pointers.set(event.pointerId, point);
	}

	function onPointerMove(event) {
		const point = local(event);
		if (drag?.pointerId === event.pointerId) {
			// 누른 점이 위치, 끈 방향이 yaw (RViz 와 같다)
			drag.last = point;
			const dx = point[0] - drag.sx;
			const dy = point[1] - drag.sy;
			drag.yaw = Math.hypot(dx, dy) < DRAG_MIN_PX ? 0 : Math.atan2(-dy, dx);
			queue();
			return;
		}
		const last = pointers.get(event.pointerId);
		if (!last) return;
		if (pointers.size === 1) {
			view.cx -= (point[0] - last[0]) / view.scale;
			view.cy += (point[1] - last[1]) / view.scale;
		} else {
			// 핀치: 다른 손가락을 고정점으로 두 손가락 거리 비율만큼 확대
			const other = [...pointers].find(([id]) => id !== event.pointerId)[1];
			const before = Math.hypot(last[0] - other[0], last[1] - other[1]);
			const after = Math.hypot(point[0] - other[0], point[1] - other[1]);
			if (before > 0) zoomAt(other[0], other[1], after / before);
		}
		pointers.set(event.pointerId, point);
		queue();
	}

	function onPointerEnd(event) {
		pointers.delete(event.pointerId);
		if (drag?.pointerId !== event.pointerId) return;
		const pose = { x: drag.x, y: drag.y, yaw: drag.yaw };
		const usedTool = tool;
		drag = null;
		setTool('pan');   // 한 번 쓰면 이동으로 돌아간다 (실수로 목표를 또 보내지 않게)
		queue();
		if (event.type === 'pointerup' && isToolsEnabled) onPose(usedTool, pose);
	}

	function onWheel(event) {
		event.preventDefault();
		const delta = event.deltaMode === 1 ? event.deltaY * 16 : event.deltaY;
		const [sx, sy] = local(event);
		zoomAt(sx, sy, Math.exp(-delta * 0.0015));
		queue();
	}

	canvas.addEventListener('pointerdown', onPointerDown);
	canvas.addEventListener('pointermove', onPointerMove);
	for (const type of ['pointerup', 'pointercancel', 'lostpointercapture']) {
		canvas.addEventListener(type, onPointerEnd);
	}
	canvas.addEventListener('wheel', onWheel, { passive: false });
	root.querySelector('.seg')?.addEventListener('click', (event) => {
		const button = event.target.closest('[data-tool]');
		if (button && !button.disabled) setTool(button.dataset.tool);
	});
	k.fit.addEventListener('click', fitRobot);
	document.addEventListener('themechange', queue);

	return {
		/** WS map 메시지 {seq, info, png(base64)}. null 이면 지도를 지우고 빈 화면 글자를 보인다. */
		setMap(message) {
			if (!message) {
				loadToken += 1;   // 읽는 중인 지도도 버린다
				mapMessage = null;
				if (!map) return;
				map = null;
				isFitted = false;   // 다음 지도가 오면 다시 화면에 맞춘다
				k.note.hidden = true;
				k.empty.hidden = false;
				queue();
				return;
			}
			// seq 가 아니라 메시지로 구별한다: 게이트웨이가 다시 켜지면 seq 는 1 부터 다시 센다
			if (message === mapMessage) return;
			if (!isMapInfo(message.info) || typeof message.png !== 'string') return;
			mapMessage = message;
			const token = ++loadToken;
			const image = new Image();
			image.onload = () => {
				if (token !== loadToken) return;   // 더 새 지도가 먼저 왔다
				map = { info: message.info, image, tinted: null, tintKey: '' };
				k.note.hidden = !message.info.origin.yaw;
				k.empty.hidden = true;
				if (!isFitted) fitMap();
				queue();
			};
			image.src = `data:image/png;base64,${message.png}`;
		},
		/** 로봇 자세 (map 프레임). 모르면 null. */
		setRobot(pose) {
			robot = isPose(pose) ? pose : null;
			queue();
		},
		/** WS scan 메시지. 점은 status 의 자세가 아니라 메시지에 실린 그때 자세(pose.map)로 찍는다. */
		setScan(message) {
			const pose = message?.pose?.map;
			const isUsable = Array.isArray(message?.points) && isPose(pose);
			scan = isUsable ? { points: message.points, pose } : null;
			queue();
		},
		setPlan(message) {
			const points = message?.points;
			plan = Array.isArray(points) && points.length >= 4 ? points : null;
			queue();
		},
		/** 목표 {x, y, yaw} (map). yaw 가 없으면 방향 없는 점 (자동 주행 목표). */
		setGoal(pose) {
			goal = isPoint(pose) ? pose : null;
			queue();
		},
		setEmptyText(text) {
			k.empty.textContent = text;
		},
		setToolsEnabled(isEnabled) {
			isToolsEnabled = isEnabled;
			for (const button of root.querySelectorAll('button[data-tool]')) {
				button.disabled = !isEnabled && button.dataset.tool !== 'pan';
			}
			if (!isEnabled && tool !== 'pan') {
				drag = null;
				setTool('pan');
				queue();
			}
		},
		destroy() {
			stop();
			document.removeEventListener('themechange', queue);
		},
	};
}
