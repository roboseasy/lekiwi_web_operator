// 조작: 전방 카메라 크게 · 손목 작게(바꾸기) · 레이더 · 조작 패드 · 텔레오퍼레이션 (팔) · 지금 보내는 속도
// (SPEC §11-2). 회전은 °/s 로 보인다 (상태 · 단계는 rad/s)

import { createArmTeleop } from '../arm_teleop.js';
import { DASH, degrees, fixed, put, readout, refs } from '../format.js';
import { createTeleop } from '../teleop.js';
import { createCameraView } from '../widgets/camera_view.js';
import { createRadarView } from '../widgets/radar_view.js';

export const title = '조작';
// 'arm': status.teleop_arm 의 팔로워 상태 · 관절 · arm_ready (Pi 에서 메시지 약 60개/s 를 더 받는다)
export const channels = ['camera/front', 'camera/wrist', 'scan', 'arm'];

let parts = [];

export function mount(el, ctx) {
	el.innerHTML = `
		<header class="page-head"><h1>조작</h1></header>
		<div class="drive">
			<div class="drive-main">
				<div class="camera-stage">
					<div class="well camera-well cam-main" data-k="front"></div>
					<div class="well camera-well cam-pip" data-k="wrist"></div>
					<button type="button" class="btn btn-small btn-float cam-swap" data-k="swap">
						카메라 바꾸기
					</button>
				</div>
				<div class="drive-bottom">
					<div class="well radar-well" data-k="radar"></div>
					<section class="panel" aria-labelledby="speed-title">
						<h2 id="speed-title">지금 내보내는 속도</h2>
						<dl class="readouts readouts-3">
							${readout('vx', '앞뒤 vx', 'm/s')}
							${readout('vy', '좌우 vy', 'm/s')}
							${readout('wz', '회전 wz', '°/s')}
							${readout('moving', '상태')}
						</dl>
						<p class="hint" data-k="limits"></p>
					</section>
				</div>
			</div>
			<div class="drive-side">
				<section class="panel" data-k="teleop" aria-label="조작"></section>
				<section class="panel" data-k="arm" aria-labelledby="arm-title"></section>
			</div>
		</div>`;
	const k = refs(el);
	const front = createCameraView(k.front, '전방 카메라');
	const wrist = createCameraView(k.wrist, '손목 카메라');
	const radar = createRadarView(k.radar);
	parts = [front, wrist, radar, createTeleop(k.teleop, ctx, false, 'drive'), createArmTeleop(k.arm, ctx)];
	k.swap.addEventListener('click', () => {
		for (const view of [k.front, k.wrist]) {
			view.classList.toggle('cam-main');
			view.classList.toggle('cam-pip');
		}
	});
	ctx.on('camera', ({ camera, blob }) => (camera === 'front' ? front : wrist).setFrame(blob));
	ctx.on('scan', (scan) => radar.setScan(scan));

	function render() {
		const teleop = ctx.status?.teleop;
		const command = teleop?.command;
		put(k.vx, fixed(command?.vx, 2));
		put(k.vy, fixed(command?.vy, 2));
		put(k.wz, degrees(command?.wz, 0));
		put(k.moving, teleop ? (teleop.is_active ? '움직이는 중' : '정지') : DASH);
		// 이 화면 패드('drive')가 고를 수 있는 가장 빠른 단계 = 베이스 상한과 패드 상한 중 낮은 것
		const pad = teleop?.pads?.drive;
		const top = pad?.levels?.[pad.max_level];
		const source = teleop?.limits?.source === 'base_controller' ? '베이스 컨트롤러 설정' : '기본값';
		put(k.limits, top ? `상한 ${fixed(top[0], 2)} m/s · 옆 ${fixed(pad.max_linear_y, 2)} m/s · `
			+ `${degrees(top[1], 0)}°/s (${source})` : '');
	}

	ctx.on('update', render);
	render();
}

export function unmount() {
	for (const part of parts) part.destroy();
	parts = [];
}
