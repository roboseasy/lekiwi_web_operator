// 게이트웨이 연결: WebSocket 자동 재연결 · heartbeat · 채널 구독 · 카메라 프레임 · REST (SPEC §3 · §4)

const HEARTBEAT_MS = 500;
const RETRY_FIRST_MS = 500;
const RETRY_MAX_MS = 5000;
// 상태는 0.25 s 마다 온다. 이만큼 아무것도 안 오면 죽은 연결로 보고 다시 연결한다
// (와이파이가 끊기면 브라우저는 한참 동안 close 를 알려 주지 않는다).
const SILENCE_MS = 3000;
const CAMERAS = { 1: 'front', 2: 'wrist' };
const LINK_ERROR = '서버에 연결할 수 없습니다. 로봇 전원과 네트워크를 확인하세요';
// 성공하면 서버가 모든 화면에 이벤트로 알리는 명령 (SPEC §7-15): 결과 토스트까지 띄우면 두 번 보인다
const EVENT_PATHS = new Set([
	'/api/control/acquire', '/api/control/release', '/api/motor/power', '/api/estop',
	'/api/process/start', '/api/process/stop', '/api/map/save',
	'/api/explore/start', '/api/explore/stop', '/api/arm/start', '/api/arm/stop', '/api/arm/relax',
]);

export class Connection {
	/**
	 * @param {(message: string, tone: string) => void} toast 명령 결과를 보여 줄 함수.
	 */
	constructor(toast) {
		this.toast = toast;
		this.ws = null;
		// 끊긴 뒤에도 남긴다: 서버가 아직 끊김을 모르면 비상 정지가 통한다.
		this.clientId = null;
		this.isOpen = false;            // welcome 을 받은 뒤부터 끊길 때까지
		this.linkState = 'connecting';  // 'connecting' | 'open' | 'closed'
		this.channels = [];
		this.listeners = new Map();
		this.retryMs = RETRY_FIRST_MS;
		this.lastHeardAt = 0;
		setInterval(() => this.tick(), HEARTBEAT_MS);
	}

	/** type 이벤트를 받는다. 돌려준 함수를 부르면 그만 받는다. */
	on(type, callback) {
		if (!this.listeners.has(type)) this.listeners.set(type, new Set());
		this.listeners.get(type).add(callback);
		return () => this.listeners.get(type).delete(callback);
	}

	emit(type, data) {
		for (const callback of this.listeners.get(type) ?? []) {
			try {
				callback(data);
			} catch (error) {
				// 한 화면의 오류가 상단 바 · 다른 구독자를 멈추지 않게 한다
				console.error(error);
			}
		}
	}

	connect() {
		const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
		const ws = new WebSocket(`${scheme}://${location.host}/ws`);
		ws.binaryType = 'arraybuffer';
		ws.onmessage = (event) => this.receive(event.data);
		ws.onclose = () => this.drop(ws);
		this.ws = ws;
		this.lastHeardAt = Date.now();
	}

	drop(ws) {
		if (this.ws !== ws) return;
		this.ws = null;
		this.isOpen = false;
		ws.onclose = null;
		ws.onmessage = null;
		ws.close();
		this.setLinkState('closed');
		setTimeout(() => this.connect(), this.retryMs);
		this.retryMs = Math.min(this.retryMs * 2, RETRY_MAX_MS);
	}

	setLinkState(linkState) {
		this.linkState = linkState;
		this.emit('link', linkState);
	}

	receive(data) {
		this.lastHeardAt = Date.now();
		if (data instanceof ArrayBuffer) {
			const camera = data.byteLength > 1 ? CAMERAS[new Uint8Array(data, 0, 1)[0]] : null;
			if (camera) {
				const blob = new Blob([new Uint8Array(data, 1)], { type: 'image/jpeg' });
				this.emit('camera', { camera, blob });
			}
			return;
		}
		let message;
		try {
			message = JSON.parse(data);
		} catch {
			console.warn('JSON 이 아닌 메시지를 버립니다', data);
			return;
		}
		if (message.type === 'welcome') {
			this.clientId = message.client_id;
			this.isOpen = true;
			this.retryMs = RETRY_FIRST_MS;
			this.send({ type: 'subscribe', channels: this.channels });
			this.emit('welcome', message);
			this.setLinkState('open');
			return;
		}
		this.emit(message.type, message);
	}

	tick() {
		if (this.ws && Date.now() - this.lastHeardAt > SILENCE_MS) {
			this.drop(this.ws);
			return;
		}
		if (this.isOpen) this.send({ type: 'heartbeat' });
	}

	/** JSON 메시지 보내기. 소켓이 열려 있지 않으면 false. */
	send(message) {
		if (this.ws?.readyState !== WebSocket.OPEN) return false;
		this.ws.send(JSON.stringify(message));
		return true;
	}

	/** 구독 채널을 통째로 바꾼다. 다시 연결되면 welcome 뒤에 다시 보낸다. */
	subscribe(channels) {
		this.channels = [...channels];
		if (this.isOpen) this.send({ type: 'subscribe', channels: this.channels });
	}

	/** GET JSON. 실패하면 토스트를 띄우고 null. */
	async get(path) {
		try {
			const response = await fetch(path, { cache: 'no-store' });
			const data = await response.json().catch(() => null);
			if (response.ok && data) return data;
			this.toast(data?.message ?? `요청이 실패했습니다 (HTTP ${response.status})`, 'error');
		} catch {
			this.toast(LINK_ERROR, 'error');
		}
		return null;
	}

	/**
	 * 명령 POST. 결과 dict 를 돌려준다. 결과 message 는 토스트로 보여 주되,
	 * 성공을 이벤트가 알리는 명령은 이벤트를 받을 수 있을 때(WS 연결) 실패만 보여 준다.
	 * @param {boolean} isQuietFailure 실패는 띄우지 않는다 (부른 쪽이 알릴지 정한다).
	 */
	async post(path, body = {}, isQuietFailure = false) {
		let result = null;
		try {
			const response = await fetch(path, {
				method: 'POST',
				headers: {
					'Content-Type': 'application/json',
					'X-Client-Id': this.clientId ?? '',
				},
				body: JSON.stringify(body),
			});
			result = await response.json().catch(() => null);
			if (typeof result?.ok !== 'boolean') {
				const message = `서버 응답을 읽지 못했습니다 (HTTP ${response.status})`;
				result = { ok: false, code: 'failed', message };
			}
		} catch {
			result = { ok: false, code: 'failed', message: LINK_ERROR };
		}
		const isShown = result.ok ? !this.isOpen || !EVENT_PATHS.has(path) : !isQuietFailure;
		if (isShown) this.toast(result.message, result.ok ? 'ok' : 'error');
		return result;
	}
}
