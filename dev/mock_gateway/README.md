# 가짜 게이트웨이 (이동 설계 페이지 시험용)

실제 로봇 없이 `web/` 를 브라우저에서 돌려 보는 도구다. 로봇 · 네트워크에 닿지 않고 `127.0.0.1` 에서만 돈다.
`teleop.py` 의 규칙(0.3 s 끊기면 0, 목표가 0 이면 출력도 바로 0, 램프, 단계 곱셈)과 `odom` · 자이로를 흉내 낸다.

```
python3 dev/mock_gateway/mock_gateway.py web 8091      # 저장소 루트에서
# 브라우저: http://127.0.0.1:8091/#/motion
```

- 이상 상황 주입: `GET /mock/inject?freeze_odom=1` · `clock_offset=-3600` · `exploring=1` · `drop_status=1` · `gyro_sign=-1` ·
  `speed_scale` · `odom_scale` · `yaw_drift` · `gyro_bias`. 되돌리기 `/mock/reset`.
- 확인: `/mock/truth` (참 경로), `/mock/teleop` (받은 teleop).
- `e2e_scenarios.js` 를 페이지 콘솔에 붙여 넣으면 정지 시나리오를 자동으로 돌린다.
- `status_template.json` · `info.json` 은 실제 로봇 응답에서 따왔고 Wi‑Fi 이름 · IP 는 가짜로 바꿨다.
  게이트웨이 status 구조가 바뀌면 다시 따서 갱신한다 (`/api/info`, WS `status`).
