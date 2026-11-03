# 새 미션 초기화와 PICK·PLACE 시퀀스 선택

현재 웹 미션은 다음 순서를 실행합니다. 선택 결과만 표시하며 실제 운반은 대기합니다.

```text
새 미션 버튼 → ARM_RESET(팔 초기 자세) → LEFT/RIGHT QR
            → 직진·펄스 접근 → 30±1cm 정지·바퀴 잠금
            → ARM_WAIT(기본 1초) → QR_TURNING(BASE만 회전)
            → SIDE_QR_WAIT(PICK·PLACE 확인) → SEQUENCE_READY(선택 표시·대기)
```

새 미션은 팔의 회전·어깨·팔꿈치·집게를 서버 시작 자세(`ServoCtrl.initPos`, 기본 모두 90°)로 부드럽게 돌립니다. 도착 전 프레임은 출발 QR로 확정하지 않습니다. 초기화 중 전체 중지를 누르면 현재 자세를 유지하고 수동 모드로 전환합니다.

정지 후 중앙 BASE 90° 기준 **LEFT는 180°, RIGHT는 0°**로 돌아봅니다. 실물에서 RIGHT QR 후 왼쪽으로 돌던 기존 기본값을 반대로 바로잡았습니다. 이때 카메라에 PICK·PLACE QR이 함께 들어오도록 팔꿈치를 초기 자세에서 30° 내리고(`qr_prepare_elbow_offset_deg`, 음수=내림, 0=BASE만 회전), 어깨·집게는 유지합니다. 회전 속도는 기존 속도 설정·슬라이더를 적용하고 진행 중인 이동을 중간에 가속하지 않습니다. `qr_prepare_left_base_deg`·`qr_prepare_right_base_deg` 설정으로 목표각을 바꿀 수 있으며 관절 범위를 검사합니다. 사이트에 별도 회전각 저장 항목은 추가하지 않았습니다.

카메라에서 **RIGHT를 볼 때 화면 왼쪽이 A, LEFT를 볼 때 화면 오른쪽이 A**입니다. A는 모니터·출발 QR 쪽, B는 출발선 쪽이라는 물리 기준을 유지합니다. 같은 쪽의 PICK과 PLACE QR을 동시에 읽고 두 중심의 좌우 순서를 비교합니다. 같은 결과가 `layout_frames`(기본 3) 프레임 연속 나와야 선택합니다. 한 코드만 보이거나 중복 코드·너무 가까운 중심·오래된 프레임이면 확정하지 않고 기다립니다. 회전 완료 후 `side_qr_settle_seconds`(기본 0.3초) 이전 프레임과 진행 중이던 이전 QR 판독도 버립니다.

| 선택 | 순서 |
|---|---|
| 1 · LEFT-A | LEFT-A PICK → RIGHT-A PLACE → RIGHT-B PICK → LEFT-B PLACE |
| 2 · LEFT-B | LEFT-B PICK → RIGHT-B PLACE → RIGHT-A PICK → LEFT-A PLACE |
| 3 · RIGHT-A | RIGHT-A PICK → LEFT-A PLACE → LEFT-B PICK → RIGHT-B PLACE |
| 4 · RIGHT-B | RIGHT-B PICK → LEFT-B PLACE → LEFT-A PICK → RIGHT-A PLACE |

`mission_config.json`의 `arm_enabled=true`이고 `/poses`의 필요한 자세(네 자리 × approach·pick·place, 공통 집게 열기·닫기, `center_carry`)가 모두 저장돼 있으면 확정 직후 `ARM_TRANSFER`로 넘어가 이 순서대로 운반합니다. 예전 `arm_poses.json`의 자세와 `calibration_confirmed`는 쓰지 않으며, 속도·대기·시간 한도만 그 파일에서 읽습니다. 조건이 맞지 않으면 `SEQUENCE_READY`에서 이유를 표시하고 대기합니다. 선택 결과는 다음 새 미션까지 유지하며, 새 미션을 시작하면 QR·선택·시간을 함께 지웁니다. 연결이 끊기면 화면은 기존 선택을 현재 결과로 표시하지 않습니다.

정지 시 초음파 거리값을 고정하고 이후 조향·전진·후진을 잠급니다. 전체 중지는 주행·QR·팔 이동 작업을 종료하고 `/poses`에서 즉시 수동 조작을 허용합니다. 새 미션 버튼은 수동 작업을 먼저 종료한 뒤 초기 자세 복원을 시작합니다. 기존 자세 이름을 가진 `arm_poses.json`도 관절 범위·속도만 읽어 수동 조절·자리 저장에 사용할 수 있으며 원본 파일을 새 형식으로 덮어쓰지 않습니다. `arm_positions.json`·설정 파일도 자동으로 다시 저장하지 않습니다.

제조사 서버 시작 시의 서보 초기화는 그대로입니다. 테스트는 가상 드라이버로 수행했으며 실제 로봇에서 90° 회전하는 경로와 카메라 시야는 현장에서 확인해야 합니다.
