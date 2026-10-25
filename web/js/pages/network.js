// 네트워크: 호스트 · 인터페이스 · 와이파이 · 유선 상태, 와이파이 검색 · 연결, AP 모드 켜기 · 끄기 · 이름 바꾸기
// (SPEC §11-6, docs/ap_mode_spec.md 5장). 로봇이 움직일 수 있는 동안의 거절은 서버가 판정하고 화면은 그 이유만 보인다.

import { DASH, badge, esc, isNum, put, putHtml, readout, refs } from '../format.js';

export const title = '네트워크';
export const channels = [];

const OPEN_SECURITY = '개방';
const AP_HOST = '192.168.4.1';   // lekiwi-ap 프로필의 로봇 주소 (network.py AP_HOST)
const AP_SCAN_REASON = 'AP 모드에서는 와이파이를 검색할 수 없습니다. 아래에 공유기 이름(SSID)과 비밀번호를 직접 넣으세요';
// 서버 network.NO_ROUTER_REASON 과 같은 글 (이미지 직후 카드: 저장된 공유기가 없다)
const NO_ROUTER_REASON = '저장된 공유기가 없어 공유기 모드로 돌아갈 수 없습니다. 공유기 이름과 비밀번호를 입력해 연결하세요';
const AP_PASSWORD = /^[\x20-\x7e]{8,63}$/;   // WPA2: 인쇄 가능한 ASCII 8~63자
// 서버 str.isprintable() 이 거절하는 글자: 제어 · 서식 문자와 일반 공백이 아닌 공백 (network.py UNPRINTABLE_TEXT)
const UNPRINTABLE = /(?! )[\p{C}\p{Z}]/u;
const UNPRINTABLE_TEXT = '제어 문자나 특수 공백(줄바꿈 · 탭 · 전각 공백 등)을 쓸 수 없습니다';
const MODE_TEXT = { ap: 'AP 모드', client: '공유기 모드', none: '공유기 모드' };

function signalText(dbm) {
	if (!isNum(dbm)) return DASH;
	if (dbm >= -60) return `${dbm} dBm · 강함`;
	if (dbm >= -70) return `${dbm} dBm · 보통`;
	return `${dbm} dBm · 약함`;
}

function bandName(mhz) {
	if (mhz < 3000) return '2.4 GHz';
	return mhz < 5925 ? '5 GHz' : '6 GHz';
}

function bandText(mhz) {
	return isNum(mhz) ? `${bandName(mhz)} · ${mhz} MHz` : DASH;
}

/** AP 칸 상태 한 줄: '켜짐 · 이름 · 2.4 GHz 채널 6 · 접속 n대' (모르는 값은 뺀다), 공유기 모드면 꺼짐. */
function apStateBadge(network) {
	if (!network?.mode) return badge('neutral', DASH);
	if (network.mode !== 'ap') return badge('neutral', '꺼짐 (공유기 모드)');
	const ap = network.ap ?? {};
	const parts = ['켜짐', ap.ssid ?? DASH];
	if (isNum(ap.freq_mhz)) parts.push(bandName(ap.freq_mhz) + (isNum(ap.channel) ? ` 채널 ${ap.channel}` : ''));
	if (isNum(ap.clients)) parts.push(`접속 ${ap.clients}대`);
	return badge('ok', parts.join(' · '));
}

function utf8Length(text) {
	return new TextEncoder().encode(text).length;
}

/** 공유기 SSID · 비밀번호의 문제. 서버 network.credentials_problem 과 같은 규칙 · 순서 · 문구 (서버가 다시 본다). */
function routerCredentialsProblem(ssid, password) {
	if (UNPRINTABLE.test(ssid) || UNPRINTABLE.test(password)) return `SSID · 비밀번호에 ${UNPRINTABLE_TEXT}`;
	if (ssid.trim() !== ssid || password.trim() !== password) return 'SSID · 비밀번호의 앞뒤에 공백을 둘 수 없습니다';
	if (ssid.includes(';')) return 'SSID 에 ;(세미콜론)는 쓸 수 없습니다';   // keyfile ssid= 의 목록 구분자
	if (/["\\]/.test(ssid + password)) {
		return 'SSID · 비밀번호에 큰따옴표(")와 역슬래시(\\)는 쓸 수 없습니다 (아직 시험하지 않은 글자입니다)';
	}
	const ssidBytes = utf8Length(ssid);
	if (ssidBytes < 1 || ssidBytes > 32) return 'SSID 는 1~32 바이트여야 합니다';
	const passwordBytes = utf8Length(password);
	if (passwordBytes < 8 || passwordBytes > 63) {
		return '비밀번호는 8~63 바이트여야 합니다 (한글은 한 글자 3 바이트, 비밀번호 없는 와이파이는 지원하지 않습니다)';
	}
	return null;
}

/** AP 이름 · 비밀번호의 문제. 서버 network.ap_credentials_problem 과 같은 규칙 · 순서 · 문구 (서버가 다시 본다). */
function apCredentialsProblem(ssid, password) {
	if (UNPRINTABLE.test(ssid) || UNPRINTABLE.test(password)) return `AP 이름 · 비밀번호에 ${UNPRINTABLE_TEXT}`;
	if (ssid.trim() !== ssid || password.trim() !== password) {
		return 'AP 이름 · 비밀번호의 앞뒤에 공백을 둘 수 없습니다';
	}
	if (ssid.includes(';')) return 'AP 이름에 ;(세미콜론)는 쓸 수 없습니다';   // keyfile ssid= 의 목록 구분자
	if (ssid.includes('\\')) return 'AP 이름에 역슬래시(\\)는 쓸 수 없습니다';
	const bytes = utf8Length(ssid);
	if (bytes < 1 || bytes > 32) return 'AP 이름은 1~32 바이트여야 합니다 (한글은 한 글자 3 바이트)';
	if (!AP_PASSWORD.test(password)) return 'AP 비밀번호는 영문 · 숫자 · 기호 8~63자여야 합니다 (한글은 쓸 수 없습니다)';
	return null;
}

/** last_switch 가 실패면 '마지막 전환 실패 (시:분): …' (로봇 시계 기준). 성공 · 없음 · 다른 칸의 동작이면 ''. */
function switchFailure(network, isConnect) {
	const last = network?.last_switch;
	if (!last || last.ok || (last.action === 'connect') !== isConnect) return '';
	const at = new Date(last.at * 1000);
	const clock = `${String(at.getHours()).padStart(2, '0')}:${String(at.getMinutes()).padStart(2, '0')}`;
	return `마지막 전환 실패 (${clock}): ${last.message}`;
}

function networkItem(item, index, chosen) {
	const meta = `${signalText(item.signal_dbm)} · ${bandText(item.freq_mhz)}`;
	return `
		<li><button type="button" class="pick" data-index="${index}"
			aria-pressed="${item === chosen}">
			<span class="pick-name">${esc(item.ssid)}</span>
			${badge(item.security === OPEN_SECURITY ? 'warn' : 'neutral', item.security ?? DASH)}
			<span class="pick-meta">${meta}</span>
		</button></li>`;
}

export function mount(el, ctx) {
	el.innerHTML = `
		<header class="page-head"><h1>네트워크</h1></header>
		<div class="network-grid">
			<section class="panel" aria-labelledby="link-title">
				<h2 id="link-title">지금 연결</h2>
				<dl class="readouts readouts-text">
					${readout('mode', '모드')}
					${readout('hostname', '호스트 이름')}
					${readout('wifiName', '와이파이')}
					${readout('signal', '신호')}
					${readout('band', '주파수')}
				</dl>
				<h3 class="sub">유선 (이더넷)</h3>
				<dl class="readouts readouts-text">
					${readout('ethLink', '케이블')}
					${readout('ethAddress', '유선 주소')}
					${readout('ethGateway', '게이트웨이')}
					${readout('route', '인터넷 경로')}
				</dl>
				<p class="guide" data-k="ethHint"></p>
				<h3 class="sub">인터페이스</h3>
				<table class="table">
					<thead><tr>
						<th scope="col">이름</th><th scope="col">상태</th><th scope="col">IPv4</th>
					</tr></thead>
					<tbody data-k="interfaces"></tbody>
				</table>
			</section>
			<section class="panel wifi-panel" aria-labelledby="wifi-title">
				<div class="panel-head">
					<h2 id="wifi-title">와이파이 바꾸기</h2>
					<button type="button" class="btn btn-small" data-k="scan">와이파이 검색</button>
				</div>
				<p class="guide">공유기 이름(SSID)과 비밀번호를 넣고 연결합니다. 검색해서 고르면 이름이 채워집니다.</p>
				<p class="reason" data-k="scanReason"></p>
				<ul class="pick-list" data-k="networks"></ul>
				<form class="wifi-form" data-k="form">
					<label class="field-label" for="wifi-ssid">공유기 이름 (SSID)</label>
					<input id="wifi-ssid" class="input" type="text" autocomplete="off" spellcheck="false"
						maxlength="32" data-k="ssid">
					<label class="field-label" for="wifi-password">비밀번호 (8~63자)</label>
					<input id="wifi-password" class="input" type="password" autocomplete="off"
						minlength="8" maxlength="63" data-k="password">
					<div class="actions">
						<button type="submit" class="btn btn-primary" data-k="connect">연결</button>
						<button type="button" class="btn" data-k="unchoose" hidden>고르기 취소</button>
					</div>
					<p class="reason" data-k="connectReason"></p>
					<p class="arm-error" data-k="connectRefusal" role="status"></p>
					<p class="arm-error" data-k="connectFailure" role="status"></p>
				</form>
			</section>
			<section class="panel" aria-labelledby="ap-title">
				<div class="panel-head">
					<h2 id="ap-title">AP 모드</h2>
					<span data-k="apState" role="status"></span>
				</div>
				<p class="guide">로봇이 직접 와이파이를 만들어 폰 · 노트북을 바로 붙이는 방식입니다. 공유기가 없는 곳에서 씁니다.</p>
				<h3 class="sub" data-k="apUrlsTitle"></h3>
				<ul class="url-list" data-k="apUrls"></ul>
				<div class="actions panel-actions">
					<button type="button" class="btn btn-primary" data-k="apToggle">AP 모드 켜기</button>
				</div>
				<p class="reason" data-k="apReason"></p>
				<p class="arm-error" data-k="apRefusal" role="status"></p>
				<p class="arm-error" data-k="apFailure" role="status"></p>
				<form class="wifi-form" data-k="apForm">
					<label class="field-label" for="ap-ssid">AP 이름 (1~32 바이트)</label>
					<input id="ap-ssid" class="input" type="text" autocomplete="off" spellcheck="false"
						maxlength="32" data-k="apSsid">
					<label class="field-label" for="ap-password">새 비밀번호 (영문 · 숫자 · 기호 8~63자)</label>
					<input id="ap-password" class="input" type="password" autocomplete="off"
						minlength="8" maxlength="63" data-k="apPassword">
					<p class="guide">8자 이상이면 됩니다. AP 비밀번호를 아는 사람은 누구나 이 화면으로 로봇을 조작할 수 있습니다.</p>
					<p class="guide">지금 비밀번호는 보여 주지 않습니다. 잊었으면 새로 정하세요.</p>
					<div class="actions">
						<button type="submit" class="btn" data-k="apSave">저장</button>
					</div>
					<p class="reason" data-k="apFormReason"></p>
					<p class="arm-error" data-k="apFormRefusal" role="status"></p>
				</form>
			</section>
		</div>`;
	const k = refs(el);
	let networks = [];
	let chosen = null;
	let hasScanned = false;
	let isScanning = false;
	let isConnecting = false;
	let isApSending = false;
	let isApNameFilled = false;
	// 서버가 거절한 이유 (움직이는 중 · 다른 전환 중 등). 다음에 보낼 때 지운다
	let connectRefusal = '';
	let apRefusal = '';
	let apFormRefusal = '';
	// 전환하면 연결이 끊기고 app.js 는 status 를 지운다: 안내는 마지막으로 받은 network 로 보인다
	let lastNetwork = null;
	// ok 로 시작한 전환 {mode: 목표 모드, name: 붙을 와이파이, before: 보낼 때의 last_switch.at}. 새 결과가 오면 지운다
	let pending = null;

	function currentNetwork() {
		const network = ctx.status?.network;
		if (network) lastNetwork = network;
		return network ?? lastNetwork;
	}

	/** 다시 접속할 주소: AP 주소와 로봇 이름 주소 (이 게이트웨이 포트), 붙을 AP · 공유기 이름. */
	function addresses() {
		const network = currentNetwork();
		const port = location.port || 80;
		const host = network?.hostname ?? ctx.info?.host;
		return {
			ap: `http://${network?.ap?.ip ?? AP_HOST}:${port}`,
			local: host ? `http://${host}.local:${port}` : null,
			// info 는 연결할 때 한 번만 온다: 바꾼 AP 이름은 상태(ap_ssid)로 본다
			apName: network?.ap?.ssid ?? network?.ap_ssid ?? ctx.info?.network_features?.ap_ssid ?? host ?? DASH,
			routerName: network?.router_ssid ?? null,
		};
	}

	/** ok 로 시작한 전환을 기억한다: 화면이 끊긴 동안 '전환 중 — 와이파이 ‘X’에 붙은 뒤 이 주소로'. */
	function startPending(mode, name) {
		pending = { mode, name, before: currentNetwork()?.last_switch?.at ?? null };
	}

	function connectBlock() {
		const reason = ctx.blockReason();
		if (reason) return reason;
		if (ctx.info?.network_features?.can_connect_wifi === false) {
			return '이 로봇에서는 와이파이 연결 도구(~/lekiwi_net 스크립트 · NetworkManager · sudo)를 쓸 수 없습니다';
		}
		if (chosen?.security === OPEN_SECURITY) return '비밀번호 없는 와이파이는 지원하지 않습니다';
		if (chosen?.is_hidden && !k.ssid.value) return '숨긴 네트워크는 이름(SSID)을 직접 넣으세요';
		if (isConnecting) return '연결을 보내는 중입니다';
		return routerCredentialsProblem(k.ssid.value, k.password.value);
	}

	/** AP 칸 (켜기 · 끄기 · 저장) 을 지금 못 누르는 이유. 움직이는지는 서버가 본다 (거절하면 그 이유를 보인다). */
	function apBlock() {
		const reason = ctx.blockReason();
		if (reason) return reason;
		const features = ctx.info?.network_features;
		if (features?.can_ap_mode !== true) return features?.ap_mode_reason ?? '이 로봇에서는 AP 모드를 쓸 수 없습니다';
		if (isApSending) return '보내는 중입니다';
		return null;
	}

	function renderForm() {
		k.unchoose.hidden = !chosen;
		const reason = connectBlock();
		k.connect.disabled = Boolean(reason);
		put(k.connectReason, reason ?? '');
		put(k.connectRefusal, connectRefusal);
		put(k.connectFailure, switchFailure(currentNetwork(), true));
	}

	function renderAp() {
		const network = currentNetwork();
		const features = ctx.info?.network_features;
		const isAp = network?.mode === 'ap';
		const savedName = network?.ap_ssid ?? features?.ap_ssid;
		if (!isApNameFilled && savedName) {   // 한 번만 채운다 (치는 중인 이름을 덮지 않게)
			k.apSsid.value = savedName;
			isApNameFilled = true;
		}
		const fresh = ctx.status?.network;
		if (pending && fresh && (fresh.last_switch?.at ?? null) !== pending.before) pending = null;   // 전환이 끝났다
		putHtml(k.apState, apStateBadge(network));
		const { ap, local, apName } = addresses();
		let urls = [ap, local];
		if (pending) {
			put(k.apUrlsTitle, `전환 중 — 와이파이 ‘${pending.name}’에 붙은 뒤 이 주소로`);
			if (pending.mode === 'client') urls = [local];
		} else {
			put(k.apUrlsTitle, isAp ? `접속 주소 (와이파이 ‘${apName}’)` : `AP 모드를 켜면 쓸 주소 (와이파이 ‘${apName}’)`);
		}
		const isLink = isAp || Boolean(pending);
		putHtml(k.apUrls, urls.filter(Boolean).map((url) => (isLink
			? `<li><a href="${esc(url)}"><b>${esc(url)}</b></a></li>`
			: `<li class="mono">${esc(url)}</li>`)).join(''));
		const block = apBlock();
		put(k.apToggle, isAp ? '공유기 모드로 돌아가기' : 'AP 모드 켜기');
		let toggleBlock = block;
		if (!toggleBlock && !network?.mode) toggleBlock = '지금 와이파이 모드를 알 수 없습니다';
		else if (!toggleBlock && isAp && !network.router_ssid) toggleBlock = NO_ROUTER_REASON;
		k.apToggle.disabled = Boolean(toggleBlock);
		put(k.apReason, toggleBlock ?? '');
		put(k.apRefusal, apRefusal);
		put(k.apFailure, switchFailure(network, false));
		const formBlock = block ?? apCredentialsProblem(k.apSsid.value, k.apPassword.value);
		k.apSave.disabled = Boolean(formBlock);
		put(k.apFormReason, formBlock ?? '');
		put(k.apFormRefusal, apFormRefusal);
	}

	function renderNetworks() {
		const items = networks.map((item, index) => networkItem(item, index, chosen));
		const empty = hasScanned ? '<li class="is-calm">찾은 와이파이가 없습니다</li>' : '';
		putHtml(k.networks, items.join('') || empty);
	}

	function render() {
		const network = ctx.status?.network;
		const wifi = network?.wifi;
		const isAp = network?.mode === 'ap';
		put(k.mode, MODE_TEXT[network?.mode] ?? DASH);
		put(k.hostname, network?.hostname ?? DASH);
		let ssid = DASH;
		if (network && !wifi) ssid = '무선 인터페이스 없음';
		else if (isAp) ssid = `AP로 동작 중 (${network.ap?.ssid ?? DASH})`;
		else if (wifi?.is_connected) ssid = wifi.ssid ?? DASH;
		else if (wifi) ssid = `연결 안 됨 (${wifi.interface ?? DASH})`;
		put(k.wifiName, ssid);
		put(k.signal, signalText(wifi?.signal_dbm));
		put(k.band, bandText(isAp ? network.ap?.freq_mhz : wifi?.freq_mhz));
		// shortcut: Pi 4 는 유선이 하나다 (케이블이 꽂힌 것, 없으면 첫 것). 여럿이면 표로 바꾼다
		const wired = network?.ethernet ?? [];
		const eth = wired.find((item) => item.is_link) ?? wired[0];
		let link = DASH;
		if (network && !eth) link = '유선 인터페이스 없음';
		else if (eth?.is_link) link = `연결됨 · ${eth.interface}${isNum(eth.speed_mbps) ? ` · ${eth.speed_mbps} Mb/s` : ''}`;
		else if (eth) link = `케이블 없음 (${eth.interface})`;
		put(k.ethLink, link);
		put(k.ethAddress, eth?.ipv4?.join(', ') || DASH);
		put(k.ethGateway, eth?.gateway ?? DASH);
		const route = network?.default_route;
		let routeText = network ? '없음' : DASH;
		if (route && route === eth?.interface) routeText = `유선 (${route})`;
		else if (route && route === wifi?.interface) routeText = `와이파이 (${route})`;
		else if (route) routeText = route;
		put(k.route, routeText);
		const address = eth?.is_link ? eth.ipv4?.[0] : null;
		put(k.ethHint, address
			? `유선으로 접속: http://${address}:${location.port || 80} · PC 에이전트는 --robot ${address}` : '');
		const rows = (network?.interfaces ?? []).map((item) => `<tr>
			<td class="mono">${esc(item.name)}</td>
			<td>${item.is_up ? badge('ok', '켜짐') : badge('neutral', '꺼짐')}</td>
			<td class="mono">${esc((item.ipv4 ?? []).join(', ') || DASH)}</td></tr>`);
		const empty = `<tr><td colspan="3">${DASH}</td></tr>`;
		putHtml(k.interfaces, rows.length ? rows.join('') : empty);

		const features = ctx.info?.network_features;
		let scanReason = null;
		if (!ctx.isConnected) scanReason = '서버에 연결되지 않았습니다';
		else if (ctx.info?.is_read_only) scanReason = '보기 전용 화면이라 와이파이를 다룰 수 없습니다';
		else if (isAp) scanReason = AP_SCAN_REASON;
		else if (features?.can_scan_wifi === false) scanReason = '이 로봇에는 와이파이 검색 도구(iw)가 없습니다';
		else if (isScanning) scanReason = '검색하는 중입니다 (최대 15초)';
		k.scan.disabled = Boolean(scanReason);
		put(k.scanReason, scanReason ?? '');
		renderForm();
		renderAp();
	}

	k.scan.addEventListener('click', async () => {
		isScanning = true;
		render();
		const result = await ctx.conn.post('/api/network/scan', {});
		isScanning = false;
		if (result.ok) {
			networks = Array.isArray(result.networks) ? result.networks : [];
			chosen = null;
			hasScanned = true;
			renderNetworks();
		}
		render();
	});
	k.networks.addEventListener('click', (event) => {
		const button = event.target.closest('[data-index]');
		if (!button) return;
		chosen = networks[Number(button.dataset.index)] ?? null;
		k.ssid.value = chosen && !chosen.is_hidden ? chosen.ssid : '';   // 숨긴 네트워크는 이름을 직접 넣는다
		k.password.value = '';
		connectRefusal = '';
		renderNetworks();
		renderForm();
		(k.ssid.value ? k.password : k.ssid).focus();
	});
	k.unchoose.addEventListener('click', () => {
		chosen = null;
		k.ssid.value = '';
		k.password.value = '';
		connectRefusal = '';
		renderNetworks();
		renderForm();
	});
	k.ssid.addEventListener('input', () => {
		if (chosen && !chosen.is_hidden && k.ssid.value !== chosen.ssid) {   // 고른 것과 다른 이름을 직접 넣었다
			chosen = null;
			renderNetworks();
		}
		renderForm();
	});
	k.password.addEventListener('input', renderForm);
	k.form.addEventListener('submit', async (event) => {
		event.preventDefault();
		if (connectBlock()) return;
		const ssid = k.ssid.value;
		const { ap, local, apName } = addresses();
		const isConfirmed = await ctx.confirm({
			title: `‘${ssid}’에 연결할까요?`,
			message: `연결이 바뀌면 이 화면이 끊깁니다. 새 와이파이에서 ${local ?? '로봇 이름 주소'} 로 다시 접속하세요. `
				+ `붙지 못하면 AP 모드(와이파이 ‘${apName}’, ${ap})로 돌아옵니다. 랜선으로 연결돼 있으면 안전합니다.`,
			confirmLabel: '연결',
		});
		if (!isConfirmed) return;
		isConnecting = true;
		connectRefusal = '';
		renderForm();
		const result = await ctx.conn.post('/api/network/connect', {
			ssid,
			password: k.password.value,
			confirm: true,
		});
		isConnecting = false;
		k.password.value = '';   // 비밀번호를 화면에 남기지 않는다
		connectRefusal = result.ok ? '' : result.message;
		if (result.ok) startPending('client', ssid);
		render();
	});

	k.apToggle.addEventListener('click', async () => {
		const mode = ctx.status?.network?.mode;
		if (apBlock() || !mode) return;
		const isAp = mode === 'ap';
		const { ap, local, apName, routerName } = addresses();
		if (isAp && !routerName) return;   // 저장된 공유기가 없다 (버튼이 꺼져 있다)
		const where = local ? `${ap} 또는 ${local}` : ap;
		const isConfirmed = await ctx.confirm(isAp ? {
			title: `공유기 ‘${routerName}’로 돌아갈까요?`,
			message: `저장된 공유기 ‘${routerName}’에 다시 붙고 AP ‘${apName}’를 끕니다. 이 화면이 끊기면 그 공유기 `
				+ `와이파이에서 ${local ?? '로봇 이름 주소'} 또는 공유기가 준 주소로 다시 접속하세요. 공유기에 붙지 `
				+ `못하면 AP 모드로 돌아옵니다 (${ap}). 한번 AP로 돌아오면 공유기가 다시 켜져도 AP에 남습니다.`,
			confirmLabel: '공유기 모드로',
		} : {
			title: 'AP 모드를 켤까요?',
			message: `공유기 연결을 끊고 로봇이 와이파이 ‘${apName}’를 만듭니다. 이 화면이 끊기면 폰 · 노트북의 `
				+ `와이파이를 ‘${apName}’로 옮긴 뒤 ${where} 로 다시 접속하세요 (폰이 ‘인터넷 없음’을 알리면 연결을 `
				+ '유지하세요). AP 모드는 재부팅해도, 공유기가 있어도 ‘공유기 모드로 돌아가기’를 누를 때까지 남습니다.',
			confirmLabel: 'AP 모드 켜기',
		});
		if (!isConfirmed) return;
		isApSending = true;
		apRefusal = '';
		renderAp();
		const result = await ctx.conn.post(isAp ? '/api/network/ap_off' : '/api/network/ap_on', { confirm: true });
		isApSending = false;
		apRefusal = result.ok ? '' : result.message;
		if (result.ok) startPending(isAp ? 'client' : 'ap', isAp ? routerName : apName);
		renderAp();
	});
	k.apSsid.addEventListener('input', renderAp);
	k.apPassword.addEventListener('input', renderAp);
	k.apForm.addEventListener('submit', async (event) => {
		event.preventDefault();
		const ssid = k.apSsid.value;
		if (apBlock() || apCredentialsProblem(ssid, k.apPassword.value)) return;
		const { ap } = addresses();
		const isConfirmed = await ctx.confirm({
			title: 'AP 이름 · 비밀번호를 바꿀까요?',
			message: `AP 이름을 ‘${ssid}’로, 비밀번호를 새 값으로 저장합니다. AP 모드가 켜져 있으면 AP를 다시 켜서 이 화면이 `
				+ `끊깁니다: 폰 · 노트북을 새 이름 · 비밀번호로 다시 붙인 뒤 ${ap} 로 접속하세요. 비밀번호는 다시 보여 주지 `
				+ '않으니 적어 두세요.',
			confirmLabel: '저장',
		});
		if (!isConfirmed) return;
		isApSending = true;
		apFormRefusal = '';
		renderAp();
		const result = await ctx.conn.post('/api/network/ap_config', {
			ssid,
			password: k.apPassword.value,
			confirm: true,
		});
		isApSending = false;
		k.apPassword.value = '';   // 쓰기 전용: 보낸 비밀번호를 화면에 남기지 않는다 (기획서 U5)
		apFormRefusal = result.ok ? '' : result.message;
		if (result.ok && currentNetwork()?.mode === 'ap') startPending('ap', ssid);   // AP 를 다시 켜 끊긴다
		renderAp();
	});

	ctx.on('update', render);
	render();
}

export function unmount() {}
