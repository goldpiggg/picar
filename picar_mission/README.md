# PiCar QR 인식 1차 버전

> 통합 실행은 저장소 루트 README를 따르세요. 기존 `Server/WebServer.py`는 기본 설정에서 QR을 읽으면 자동 접근·정지까지 실행합니다. 아래의 `qr_scan.py` 독립 실행만 QR 확인 전용이며 모터를 움직이지 않습니다. 공통 설정은 루트 `mission_config.json`에서 읽습니다.

`qr_scan.py` 한 파일을 라즈베리파이에 복사하면 됩니다. Python 3.8+ 및 OpenCV 4.5+가 필요합니다. 모터/서보/초음파 모듈은 불러오지 않습니다.

## 이번 버전 범위

- 목표 거리 35.0cm, 거리 보정값 0.0cm를 설정값으로 둡니다. 아직 거리 측정이나 정지 제어는 구현하지 않았습니다.
- `LEFT`, `RIGHT`, `PICK`, `PLACE`를 인식합니다. 대소문자와 앞뒤 공백만 정규화하고 다른 내용은 명령으로 해석하지 않습니다.
- 출발 모드는 기본 설정에서 LEFT/RIGHT 한 개를 1회 읽으면 방향을 한 번 확정한 뒤 종료합니다. 기본 추가 대기 시간은 0초입니다.
- 누락, 방향 변경, 두 출발 QR의 동시 인식, 기본 1.5초 초과의 관측 간격은 확인을 다시 시작하게 합니다. PICK/PLACE만 보이면 출발을 확정하지 않습니다.
- 각 QR의 꼭짓점, 중심 좌표와 정규화한 중심 좌표를 기록합니다. 좌표는 영상 기준이고 실제 거리/팔 각도는 아닙니다.
- `start_confirmed` 출력도 정보 이벤트일 뿐이며 주행 명령을 보내지 않습니다.

## 기본 실행: 공식 웹 영상 재사용

공식 웹 제어 프로그램이 이미 실행 중이고 영상이 나오는 상태에서 라즈베리파이 SSH 터미널에 입력합니다.

```bash
python3 -c "import cv2; print(cv2.__version__)"
python3 qr_scan.py
```

기본 영상 주소는 `http://127.0.0.1:5000/video_feed`입니다. 제조사 웹 서버의 영상 출력을 재사용하므로 웹 서버와 USB 카메라를 서로 차지하지 않습니다. 제조사 모듈을 import하면 서보 초기화 등 부수 동작이 있을 수 있어, 이번 파일은 해당 모듈을 import하지 않습니다.

실행 후 카메라 앞에 첨부 자료의 LEFT 또는 RIGHT QR을 보여줍니다. 정상 출력 예시는 아래와 같습니다.

```json
{"event":"start_confirmed","direction":"LEFT","target_distance_cm":35.0,"distance_offset_cm":0.0,"motor_control":false}
```

실제 출력에는 프레임 수와 확인 시간도 포함됩니다. 성공 후 프로그램은 종료됩니다. 재시험하려면 다시 실행하세요. 인식값은 화면에 보여주는 QR로 결정하며, 물리적인 왼쪽/오른쪽 위치로 추측하지 않습니다.

전체 QR 관찰만 하려면:

```bash
python3 qr_scan.py --mode inspect --timeout 120
```

노트북에서 실행하려면 OpenCV가 필요하고, 영상 주소를 로봇 IP로 바꿉니다.

```bash
python qr_scan.py --source http://로봇IP:5000/video_feed
```

노트북에 복사한 파일을 PowerShell에서 전송하는 예시:

```powershell
scp .\qr_scan.py 계정명@로봇IP:~/
```

## 선택 실행

```bash
# 정지 이미지: 단일 이미지 결과만 출력, 출발 확정 없음
python3 qr_scan.py --image Left.png

# USB 카메라 직접 사용: 다른 프로그램이 카메라를 사용하지 않을 때만
python3 qr_scan.py --source 0

# 모니터가 직접 연결된 GUI 환경에서만 영상 창 표시
python3 qr_scan.py --preview
```

일반 SSH에는 `--preview`를 붙이지 않습니다. OpenCV가 없다면 라즈베리파이에서 `sudo apt install python3-opencv`를 사용합니다. 이미 설치됐다면 다시 설치하지 않습니다.

## 실패와 한계

- 웹 화면의 영상이 안 나오면 QR 코드 전에 공식 카메라 설정부터 확인합니다. 이 프로그램은 웹 서버를 자동 시작/재시작하거나 시스템 서비스를 변경하지 않습니다.
- HTTP 입력은 FFmpeg 백엔드와 열기/읽기 타임아웃을 사용합니다. 지원하지 않는 OpenCV에서는 명시적 오류로 종료하며 무제한 대기로 우회하지 않습니다. HTTP 열기는 최대 약 5초, 읽기는 약 3초 단위 제한이며 전체 테스트 시간에는 마지막 읽기/인식 처리 시간이 더해질 수 있습니다. USB 장치 백엔드의 블로킹 시간은 보장하지 않습니다.
- HTTP 연결 실패, 영상 읽기 실패 시 종료 코드 1. 출발 인식 시간초과/QR 없는 이미지 2. 정상 확인 0. Ctrl+C 또는 q 취소 130.
- 연속 관측은 영상에서 반복 인식했다는 뜻이며, 스트림 자체가 과거 프레임을 반복 송출하거나 버퍼에 오래된 프레임이 쌓이는 문제까지 검출하지 않습니다. 실제 주행 연결 전에는 촬영 시각·최신 프레임 처리와 출발 허용 상태를 추가해야 합니다.
- 읽지 못한 두 번째 QR은 충돌 판정 대상이 될 수 없습니다. 화면에는 출발 QR 하나만 표시하는 경기 조건을 전제로 합니다.
- 조명, 모니터 반사/깜빡임, QR 크기, 웹캠 초점·거리·각도는 실제 카메라에서 검증해야 합니다. 이번 버전은 원본/합성 이미지 검증과 실물 검증을 구분합니다.

## 테스트

```bash
python3 -m unittest discover -s picar_mission -p 'test_*.py' -v
```

`test_qr_scan.py`를 함께 둔 경우 코드 폴더에서 `python3 -m unittest test_qr_scan -v`로도 실행할 수 있습니다. 첨부 이미지가 없는 환경에서는 이미지 테스트가 생략되고 상태 판단 테스트는 실행됩니다.

## 참고한 공식 코드/API

- [Adeept app.py](https://github.com/adeept/Adeept_PiCar-Pro/blob/main/Server/app.py): `/video_feed` 영상 주소.
- [OpenCV QRCodeDetector](https://docs.opencv.org/4.x/de/dc3/classcv_1_1QRCodeDetector.html): 다중 QR 해독.
- [OpenCV VideoCapture 속성](https://docs.opencv.org/4.x/d4/d15/group__videoio__flags__base.html): FFmpeg 열기/읽기 타임아웃.

다음 단계는 이 출발 방향을 미션 상태 관리기로 전달하고, 보정값 0인 초음파 35cm 정지 단계를 별도로 연결하는 것입니다.
