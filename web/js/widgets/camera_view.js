// 카메라 한 대: JPEG 프레임 → <img> (Blob URL), 2 s 동안 프레임이 없으면 '신호 없음', fps 표시

const STALE_MS = 2000;

/**
 * @param {HTMLElement} root 그릴 곳 (.well).
 * @param {string} label 카메라 이름 (코드에 적은 글자만 넣는다).
 * @returns {{setFrame: (blob: Blob) => void, destroy: () => void}}
 */
export function createCameraView(root, label) {
	root.classList.add('camera-view');
	root.innerHTML = `
		<img alt="${label} 영상">
		<p class="canvas-empty">신호 없음</p>
		<p class="camera-tag">${label} <span data-k="fps"></span></p>`;
	const img = root.querySelector('img');
	const fps = root.querySelector('[data-k="fps"]');
	let url = null;
	let times = [];

	function refresh() {
		const now = performance.now();
		times = times.filter((time) => now - time <= STALE_MS);
		root.classList.toggle('is-live', times.length > 0);
		const span = times.length > 1 ? times.at(-1) - times[0] : 0;
		fps.textContent = span > 0 ? `${Math.round((times.length - 1) * 1000 / span)} fps` : '';
	}

	const timer = setInterval(refresh, 500);
	return {
		setFrame(blob) {
			const next = URL.createObjectURL(blob);
			img.src = next;
			if (url) URL.revokeObjectURL(url);   // 이전 프레임 메모리를 바로 돌려준다
			url = next;
			times.push(performance.now());
			refresh();
		},
		destroy() {
			clearInterval(timer);
			if (url) URL.revokeObjectURL(url);
		},
	};
}
