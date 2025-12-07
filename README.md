# EdgeAI-RTP (Zynq + EdgeTPU + YOLOv8) — README

> **프로젝트 요약**  
> Zynq-7000(Zybo) 보드에서 **Google Coral USB EdgeTPU**로 YOLOv8 추론을 수행하고,  
> **GStreamer**로 카메라 영상을 **RTP(JPEG)** 로 PC에 스트리밍하며,  
> **UDP(JSON)** 으로 **BBOX 메타데이터(CXYWH, center-based)** 를 전송해 PC에서 **오버레이**로 시각화하는 시스템입니다.  
> 짐벌 제어(서보 PWM), IMU(자세 안정화), 발사 제어(안전 게이팅)까지 포함한 **실시간 임베디드 비전 파이프라인**입니다.

(시스템 전체 사진)  
(보드/카메라/EdgeTPU 연결 사진)  
(PC 오버레이 화면 캡쳐)

---

## 1) 아키텍처 개요

- **Zynq 보드(PetaLinux 2024.1)**
  - 카메라 입력(V4L2, 640×480, YUY2)
  - GStreamer 파이프라인: `v4l2src → tee → appsink(AI) + jpegenc→rtpjpegpay→udpsink`
  - 전처리(리사이즈/색변환) → **TFLite(EdgeTPU delegate)** → 후처리(+NMS) → 멀티오브젝트 트래킹(간단 매칭/확장 예정)
  - **BBox 메타데이터(센터 기반 CXYWH)** 를 **UDP(JSON)** 으로 PC에 전송
  - 짐벌/센서/발사 제어 루프(모듈화)
- **PC**
  - RTP(JPEG) 수신, 프레임 디코드
  - UDP(JSON) 메타 수신 → BBOX 오버레이
  - 성능/FPS 측정, 발표/디버깅 툴

(아키텍처 다이어그램)

---

## 2) 주요 특징

- **온보드 NPU 추론**: EdgeTPU로 YOLOv8n EdgeTPU 모델 실시간 추론
- **경량 스트리밍**: 영상은 RTP(JPEG)로, 메타는 UDP(JSON)로 분리 전송(낮은 오버헤드)
- **좌표 체계 일관화**: **CXYWH(center, width, height)** 전송, 수신단에서 해상도 스케일링
- **후처리 품질 향상**: OpenCV **`cv::dnn::NMSBoxes`** 기반 NMS 적용
- **실측 성능 계측**: 전처리/추론/후처리/트래킹/전송 **구간별 ms** + **FPS** 로그
- **임베디드 제약 대응**: 하드-플로트, 낮은 병렬, GStreamer 기반 저메모리 파이프라인

(성능 로그 스크린샷)  
(오버레이 예시 캡쳐)

---

## 3) 하드웨어 / 소프트웨어 요구사항

### 하드웨어
- **Zybo Z7-7010** (Zynq-7000)
- **USB 카메라** (V4L2, 640×480 @30fps, YUY2)
- **Google Coral USB Accelerator (EdgeTPU)**
- **서보/IMU/송신 모듈**(선택)
- **안정된 전원 분리** 권장(카메라/IMU 버스 노이즈 방지)

### 보드 소프트웨어 (PetaLinux 2024.1)
- `libtensorflow-lite.a` (TFLite 2.5.0, XNNPACK OFF)  
- `libedgetpu.so.1` (EdgeTPU runtime 20210726 계열), `/usr/lib/` 배치
- **OpenCV 4.x**(imgproc, videoio, highgui, **dnn**), **GStreamer 1.0**, GLib
- udev/usbutils/v4l-utils 등 런타임 유틸(선택)

### PC 환경
- Python 3.9+ / 3.10 (Windows 또는 Linux)
- `opencv-python`(FFmpeg 포함 빌드 권장), `numpy`
- **ffmpeg/ffplay**(진단용)
- Windows 방화벽 UDP **영상 포트(예: 5600)**, **메타 포트(예: 5000)** 허용

---

## 5) 빌드(보드용) — **Known-Good Cross Compile**

1) **SDK 환경 스크립트 로드**
```bash
source ~/Desktop/zybo_sdk_2024_1/environment-setup-cortexa9t2hf-neon-xilinx-linux-gnueabi
export SYSROOT=$SDKTARGETSYSROOT
export PKG_CONFIG_SYSROOT_DIR=$SYSROOT
export PKG_CONFIG_PATH=$SYSROOT/usr/lib/pkgconfig:$SYSROOT/usr/share/pkgconfig
```

2) **OpenCV/GStreamer 경로 확인**
```bash
pkg-config --cflags opencv4
pkg-config --libs   opencv4
pkg-config --cflags gstreamer-1.0
pkg-config --libs   gstreamer-1.0
```

3) **하드-플로트/NEON 플래그 & 링킹**  
Makefile 예시(핵심만):
```make
SDK_SYSROOT       ?= $(SDKTARGETSYSROOT)
CXX               ?= arm-xilinx-linux-gnueabi-g++
CXXFLAGS_COMMON   := -O2 -pipe -fPIC -Wall -Wextra -fno-exceptions \
                     -mfpu=neon -mfloat-abi=hard -mcpu=cortex-a9 -mthumb -D__ARM_PCS_VFP
INCLUDE_PATHS     := \
  -Iinclude -I. \
  -I$(SDK_SYSROOT)/usr/include/opencv4 \
  -I$(SDK_SYSROOT)/usr/include/gstreamer-1.0 \
  -I$(SDK_SYSROOT)/usr/include/glib-2.0 \
  -I$(SDK_SYSROOT)/usr/lib/glib-2.0/include

OPENCV_LIBS       := $(shell pkg-config --libs opencv4)
GST_LIBS          := $(shell pkg-config --libs gstreamer-1.0) -lgstapp-1.0
LIBS              := $(OPENCV_LIBS) -lopencv_dnn $(GST_LIBS) -lstdc++ -lpthread -lm -lc -lrt -latomic

LDFLAGS          := --sysroot=$(SDK_SYSROOT) -Wl,-dynamic-linker=/lib/ld-linux-armhf.so.3
```
> **중요**: `-lopencv_dnn` 링크 추가( `cv::dnn::NMSBoxes` 사용 ), 패키지에 dnn 모듈 포함되어야 함.

4) **TFLite / EdgeTPU**
- 정적 `libtensorflow-lite.a`(2.5.0) 경로 지정
- `libedgetpu.so.1`는 런타임 배치(`/usr/lib/`), 코드에서 `dlopen("libedgetpu.so.1")`

5) **빌드**
```bash
make -j1
```

---

## 6) 실행 방법

### 보드 → 송출
- **모델 입력**: 기본 224×224, 코드가 인터프리터에서 **in_w/in_h** 자동 획득
- **RTP(JPEG) 영상 포트**: 5600 (예시)
- **BBox/상태 메타 포트(JSON)**: 5000

```bash
# 예시 파이프라인 로그 + 추론 + 메타 송출 (vision_main)
./build/vision_main
# 또는 이진 배포물:
# /usr/local/bin/tpu_stream.out /dev/video0 <PC_IP> 5600 /usr/local/share/models/yolov8n_full_integer_quant_edgetpu.tflite
```

GStreamer 파이프라인(코드 내부):
```
v4l2src device=/dev/video0 !
  video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! tee name=t
  t. ! queue max-size-buffers=1 leaky=downstream ! appsink name=appsink drop=true sync=false
  t. ! queue ! jpegenc quality=50 ! rtpjpegpay ! udpsink host=<PC_IP> port=5600
```

### PC → 수신/오버레이

1) **ffplay로 스트림 확인 (진단)**
```bash
ffplay -protocol_whitelist "file,udp,rtp" -i stream.sdp
```
`stream.sdp` 예시:
```
v=0
o=- 0 0 IN IP4 <PC_IP>
s=RTP JPEG
c=IN IP4 <PC_IP>
t=0 0
m=video 5600 RTP/AVP 26
a=rtpmap:26 JPEG/90000
```

2) **Python 오버레이 수신기**
```powershell
# Windows PowerShell 환경변수(FFmpeg 허용)
$env:OPENCV_FFMPEG_CAPTURE_OPTIONS="protocol_whitelist;file,crypto,data,udp,rtp|safe;0"

# 실행
python apps/overlay_receiver_cv.py --sdp stream.sdp --meta-port 5000 --model-input 224x224
```
- **메타는 UDP(JSON)**, 포트 5000 바인드
- **CXYWH(center-based)** 를 프레임 해상도에 맞게 스케일링 후 그려줌

(PC 오버레이 스크린샷)

---

## 7) BBox 메타 데이터 규격(송수신)

- **좌표 포맷**: **CXYWH** (center_x, center_y, width, height)  
- **스케일 기준**: 모델 입력(예: 224×224)에서 검출된 **TLWH**를 **CXYWH**로 변환 →  
  보드에서 **640×480** 기준으로 스케일 업 → 전송
- **네임스페이스**: `BBoxData{ frame_id, ts_ms(optional), objects[ {id, class_name, confidence, priority, bbox[CXYWH]} ] }`

예시(JSON):
```json
{
  "frame_id": 1234,
  "ts_ms": 1733555550000,
  "objects": [
    { "id": 42, "class": "0", "confidence": 0.78, "priority": 1, "bbox": [320, 240, 60, 48] }
  ]
}
```

---

## 8) 모델 준비 (YOLOv8 → TFLite → EdgeTPU)

1) **YOLO 학습(커스텀 데이터)**
2) **TFLite 변환** (정수 양자화)
3) **EdgeTPU 컴파일** → `*_edgetpu.tflite`

> **주의**  
> - **추론 입력 해상도**(예: 224×224)는 코드에서 `interpreter.input()`에서 읽어 자동 적용됩니다.  
> - **학습/변환 해상도**와 **런타임 전처리 해상도**를 일치시키세요.  
> - 사각형 입력을 쓰려면, 전체 파이프라인/스케일러가 그 가정에 맞아야 합니다.

(모델 변환 로그/EdgeTPU 컴파일 스크린샷)

---

## 9) 성능 계측(보드 로그)

`vision_main`의 **process_thread** 내에 구간 타이머가 내장되어, 30프레임마다:
```
[PERF] pre= 1.10ms  inf= 6.40ms  post= 0.70ms  trk=0.15ms  send=0.20ms  total= 9.10ms  FPS=110.0
```
- `pre`: 전처리(BGR→RGB, resize)
- `inf`: TFLite Invoke(EdgeTPU)
- `post`: 후처리(+NMS)
- `trk`: 트래킹
- `send`: 직렬화/UDP 전송
- `total`: 프레임 처리 총합 → **FPS** 계산

PC 수신기도 **도착 간격 기반 FPS** 를 표시하도록 옵션 제공.

---

## 10) 트래킹

- 기본: IoU 기반 간단 매칭(프레임 간 ID 유지)
- **계획**: ByteTrack / BoT-SORT 통합(헝가리안 + 칼만)  
  (트래커 모듈 인터페이스 유지, 내부 알고리즘 교체 가능)

(트래킹 다이어그램)  
(칼만/헝가리안 개념도 사진)

---

## 11) 트러블슈팅

- **OpenCV FFmpeg “`Protocol 'rtp' not on whitelist`” (Windows)**
  - PowerShell:  
    ` $env:OPENCV_FFMPEG_CAPTURE_OPTIONS="protocol_whitelist;file,crypto,data,udp,rtp|safe;0" `
  - SDP는 **절대경로 file: URL** 도 유효:  
    `file:C:/path/to/stream.sdp?protocol_whitelist=file,crypto,data,udp,rtp`
- **RTP는 뜨는데 오버레이가 안 보임**
  - 방화벽에서 **메타 포트(5000)** Inbound 허용
  - PC IP/포트가 **보드 코드와 동일**한지 확인
- **검은 화면에서도 검출됨(오탐)**
  - **CONF_TH** 상향(예: 0.60+), **NMS_TH** 조정, **MIN_W/H** 확대로 작은 노이즈 박스 제거
  - 학습 데이터/증강/밝기 범위 확인
- **I2C 드랍/IMU 끊김**
  - 발사/서보 전류 스파이크로 GND 바운스 가능 → **전원 분리/필터링**, 센서 라인 풀업/실드, 공통 GND 처리 개선
- **libedgetpu.so.1 로드 실패**
  - `/usr/lib/libedgetpu.so.1` 배치, 실행 사용자 권한/의존성 확인
- **x264 인코더 미탑재**
  - 본 파이프라인은 **RTP JPEG** 사용(추가 인코더 불필요)

---

## 12) 향후 로드맵

- [ ] ByteTrack/BoT-SORT 통합(헝가리안+칼만)
- [ ] 짐벌 폐루프 튜닝(PID, 데드존/레이트 리밋, 안전 게이팅)
- [ ] RTP H.264 경량화 옵션 추가(x264enc zerolatency)
- [ ] 온보드 로깅/리플레이 툴
- [ ] 프레임 동기화(메타↔영상 타임스탬프 정합 강화)

---

## 13) 라이선스 / 크레딧

- 코드 라이선스: **MIT** (예시)  
- 모델/데이터셋은 각 저작권/사용권을 따릅니다.  
- Google Coral / TensorFlow Lite / OpenCV / GStreamer 커뮤니티에 감사드립니다.

---

## 14) 부록 — 좌표/스케일 정리

- **모델 출력**: 보통 224×224 기준 **TLWH**(x, y, w, h, 좌상단 기준)
- **전송 포맷**: **CXYWH** 로 변환 후, **640×480** 로 스케일 업  
  ```
  cx = (x + w/2) * (640 / 224)
  cy = (y + h/2) * (480 / 224)
  w' = w * (640 / 224)
  h' = h * (480 / 224)
  ```
- **PC 표시**: CXYWH → TLWH 변환 후 사각형 그리기

(좌표 변환 도식 이미지)

---

## 15) 연락처 / 문서

- (팀/담당자 사진)  
- (이메일/노션/위키/이슈 트래커 링크 자리)

---

> 참고: 사진은 **(사진)** 표기된 곳에 추가해 주세요.  
> 문서/스크립트/경로명은 실제 사용 중인 저장소 기준으로 맞춰 주시면 됩니다.
