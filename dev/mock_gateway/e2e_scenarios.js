// 브라우저 콘솔(또는 DevTools 도구)에서 http://127.0.0.1:8091/#/motion 을 연 뒤 통째로 붙여 넣는다.
// 가짜 게이트웨이(mock_gateway.py)에 이상 상황을 주입하고, 페이지가 멈추는지 · 얼마나 빨리 멈추는지 · 멈춘 뒤 로봇이 안 움직이는지를 잰다.
// 사용: await window.__scenario('이름', { inject: 'freeze_odom=1' })   (kind · imu · inject · action · runSeconds · watchSeconds)
const wait = (ms) => new Promise((resolve) => setTimeout(resolve, ms));
const q = (key) => document.querySelector(`[data-k="${key}"]`);
window.__scenario = async (name, { kind = 'circle', imu = true, inject = null, action = null, runSeconds = 3.5, watchSeconds = 5 }) => {
	await fetch('/mock/reset'); await wait(1200);
	q('control').click(); await wait(900);
	q('power').click(); await wait(300); document.getElementById('confirm-ok').click(); await wait(900);
	q('kind').value = kind; q('kind').dispatchEvent(new Event('change', { bubbles: true }));
	q('repeat').value = 'forever'; q('repeat').dispatchEvent(new Event('input', { bubbles: true }));
	q('useImu').checked = imu; q('useImu').dispatchEvent(new Event('input', { bubbles: true }));
	await wait(200);
	if (q('start').disabled) return { name, error: `시작 불가: ${q('reason').textContent}` };
	q('start').click(); await wait(300); document.getElementById('confirm-ok').click();
	await wait(3000 + runSeconds * 1000);
	const before = q('state').textContent;
	const t0 = (await (await fetch('/mock/truth')).json()).now;
	if (inject) await fetch(`/mock/inject?${inject}`);
	if (action) await action();
	let stoppedAfter = null;
	for (let i = 0; i < watchSeconds * 10; i += 1) {
		await wait(100);
		if (q('state').textContent !== '진행 중') { stoppedAfter = (i + 1) / 10; break; }
	}
	const reason = q('reason').textContent;
	await wait(1500);
	const truth = (await (await fetch('/mock/truth')).json()).truth;
	const last = truth[truth.length - 1];
	const prev = truth.filter((p) => p[0] >= last[0] - 1.0)[0];
	const speedAfter = Math.hypot(last[1] - prev[1], last[2] - prev[2]) / Math.max(last[0] - prev[0], 0.01);
	const teleop = (await (await fetch('/mock/teleop')).json()).teleop.filter((m) => m[0] >= t0);
	await fetch('/mock/inject?freeze_odom=0&clock_offset=0&exploring=0&drop_status=0&gyro_sign=1&stuck_odom=0');
	return {
		name, before, stoppedAfterSec: stoppedAfter, reason, robotSpeedAfter: Number(speedAfter.toFixed(4)),
		lastTeleopAllZero: teleop.slice(-5).every((m) => m[1] === 0 && m[2] === 0 && m[3] === 0),
	};
};

// 기대: 모두 stoppedAfterSec 가 2 이하, robotSpeedAfter 0, lastTeleopAllZero true.
//   await window.__scenario('센서 동결', { inject: 'freeze_odom=1' });            // ≈1.5 s, '센서 값이 1 초 넘게 오지 않아'
//   await window.__scenario('서버 시계 역행', { inject: 'clock_offset=-3600' });   // ≈0.2 s
//   await window.__scenario('자동 주행 시작', { inject: 'exploring=1' });
//   await window.__scenario('status 끊김', { inject: 'drop_status=1' });          // ≈1 s
//   await window.__scenario('자이로 부호 반대', { inject: 'gyro_sign=-1' });        // ≈1 s, '부호'
//   await window.__scenario('비상 정지', { action: async () => document.querySelector('[data-estop]').click() });
//   IMU 방향 끔: await window.__scenario('IMU 끔', { imu: false })  → stoppedAfterSec 가 null (멈추면 안 된다)

// ── 비상 정지 · 조작 끄기 뒤 다시 켜고 시작하는 흐름 (서버의 0 래치): 시작하면 로봇이 실제로 움직여야 한다 ──
// 기대: state '진행 중', rejected 0, movedMeters > 0.3
window.__latchFlow = async (label) => {
	await fetch('/mock/reset'); await wait(1200);
	q('control').click(); await wait(900);
	q('power').click(); await wait(300); document.getElementById('confirm-ok').click(); await wait(900);   // 켜기
	q('power').click(); await wait(900);                                                                    // 끄기 → 서버가 0 입력을 받을 때까지 방향 입력을 버린다
	q('power').click(); await wait(300); document.getElementById('confirm-ok').click(); await wait(900);   // 다시 켜기
	q('kind').value = 'circle'; q('kind').dispatchEvent(new Event('change', { bubbles: true }));
	q('repeat').value = 'forever'; q('repeat').dispatchEvent(new Event('input', { bubbles: true }));
	await wait(200);
	q('start').click(); await wait(300); document.getElementById('confirm-ok').click();
	await wait(8000);
	const state = q('state').textContent;
	const teleop = await (await fetch('/mock/teleop')).json();
	const truth = (await (await fetch('/mock/truth')).json()).truth;
	const last = truth[truth.length - 1];
	window.dispatchEvent(new KeyboardEvent('keydown', { code: 'Space', bubbles: true }));
	await wait(1300);
	return { label, state, rejected: teleop.rejected, movedMeters: Number(Math.hypot(last[1], last[2]).toFixed(3)) };
};
//   status 가 0.7 s 끊겼다 돌아오는 경우: '/mock/inject?drop_status=1' → 0.7 s 뒤 '=0' → 끊긴 동안 0 이 나가고(0.6 s 부터) 정지 없이 이어서 달린다
//   odom 이 값은 새로 오는데 위치가 얼어붙음: 'stuck_odom=1' → 진행 감시가 3 s 안팎에 정지 ('명령대로 움직이지 않습니다')
