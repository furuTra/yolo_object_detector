# yolo_object_detector

M5Stack Module LLM(AX630C / StackFlowプロトコル)を使い、**JPEG画像を渡すと物体検出の座標列を返す**
移植可能なPlatformIOローカルライブラリ。カメラやLCDには一切依存しないので、Module LLM を接続した
別のM5Stackプロジェクト(アバター表示など)へこのフォルダごとコピーして再利用できる。

## 何をするパッケージか

- **INPUT**: JPEG圧縮済み画像のバイト列 + 長さ(`const uint8_t*`, `size_t`)。
  カメラドライバやJPEG圧縮方式には依存しない(呼び出し側が用意する)。
- **OUTPUT**: 検出物体のリスト。各要素は `DetectedObject`
  (`class_name`, `confidence`, `x1`, `y1`, `x2`, `y2`)。
  座標は **入力JPEG画像のピクセル空間** における `[x1, y1, x2, y2]`(左上・右下の対角コーナー)。
  この座標系は実機検証済みで確定した公開契約(幅/高さが必要なら `w = x2 - x1`, `h = y2 - y1`)。

内部では UART 経由の StackFlow 通信・ボーレート切替・`yolo.setup` / `yolo.inferenceAndWaitResult`
呼び出し・検出結果JSONのパースまでを面倒見る。

## 依存関係

`library.json` に宣言済み。利用側プロジェクトの `platformio.ini` の `lib_deps` にも同じものを足すこと。

- [`m5stack/M5Module-LLM`](https://github.com/m5stack/M5Module-LLM) — StackFlow の Arduino クライアント
- [`bblanchon/ArduinoJson`](https://github.com/bblanchon/ArduinoJson) — 検出結果JSONのパース
- [`m5stack/M5Unified`](https://github.com/m5stack/M5Unified) — UARTピンの自動解決(`M5.getPin`)

ハードウェア要件:

- M5Stack Module LLM(YOLOモデル `yolo11n` がインストール済みであること)
- **M5Stack製品前提**。UART RX/TX ピンは `M5.getPin(port_c_rxd/txd)` で自動解決するため、
  明示的なピン指定のオーバーロードは用意していない。`begin()` を呼ぶ前に `M5.begin()`(または
  `M5.getPin()` が有効になる初期化)を済ませておくこと。

## 使い方

```cpp
#include <M5Unified.h>
#include <yolo_object_detector.h>

using yolo_object_detector::Detector;
using yolo_object_detector::DetectedObject;
using yolo_object_detector::kYoloBaudRate;

Detector detector;

void setup() {
    M5.begin();  // これより後で detector.begin() を呼ぶこと(M5.getPin が有効になる)

    detector.begin(Serial2);        // UART初期化(ピンは M5.getPin で自動解決)
    detector.connectAutoBaud();     // 接続できるまでブロック。接続後は必ず115200へ戻る
    detector.resetModule();         // sys.reset()
    detector.setBaudRate(kYoloBaudRate);  // 高スループット通信用に1.5Mbpsへ
    detector.setupYolo();           // yolo.setup。既定モデル "yolo11n"
}

void loop() {
    // 呼び出し側でカメラ等からJPEGを用意する
    const uint8_t* jpeg = /* ... */;
    size_t jpeg_len     = /* ... */;

    detector.detect(jpeg, jpeg_len, [](const DetectedObject& d) {
        // 検出1件ごとに呼ばれる。座標は入力JPEGのピクセル空間、[x1,y1,x2,y2] 対角コーナー
        Serial.printf("%s %.2f (%d,%d)-(%d,%d)\n",
                      d.class_name.c_str(), d.confidence, d.x1, d.y1, d.x2, d.y2);
    });
}
```

呼び出し順序(重要):
`begin()` → `connectAutoBaud()` → `resetModule()` → `setBaudRate(kYoloBaudRate)` → `setupYolo()`
→ ループで `detect()`。

`connectAutoBaud()` は 115200 ↔ `kYoloBaudRate` を交互に ping して接続し、**接続後は必ず115200へ戻す**。
ホスト側だけ再起動してモジュールが前回の1.5Mbpsのまま残っていても、この順序なら復帰できる。

## 公開API

```cpp
namespace yolo_object_detector {

extern const char* const kDefaultYoloModel;  // "yolo11n"
extern const uint32_t    kYoloBaudRate;      // 1500000

struct DetectedObject {
    String class_name;
    float  confidence = 0.0f;
    int    x1 = 0, y1 = 0, x2 = 0, y2 = 0;   // 入力JPEGのピクセル空間、対角コーナー
};

class Detector {
public:
    bool begin(HardwareSerial& serial, uint32_t baud_rate = 115200);
    void waitForConnection();
    void connectAutoBaud();
    void resetModule();
    bool setBaudRate(uint32_t baud_rate);
    bool setupYolo(const String& model = kDefaultYoloModel);
    bool isReady() const;
    const String& yoloWorkId() const;
    bool detect(const uint8_t* jpeg_data, size_t jpeg_len,
                const std::function<void(const DetectedObject&)>& onDetection,
                uint32_t timeout_ms = 500);
};

}  // namespace yolo_object_detector
```

`detect()` の `timeout_ms` は応答が途切れてからのタイムアウト。検出0件でモジュールが finish を
返さない場合の空振り時間になり、ライブフレームレートに影響するので実機で調整すること。

## 他プロジェクトへの移植手順

1. `lib/yolo_object_detector/` フォルダを移植先プロジェクトの `lib/` 配下へ丸ごとコピーする
   (PlatformIO は `lib/*` を自動的にライブラリとして認識する)。
2. 移植先の `platformio.ini` の `lib_deps` に依存を足す:
   ```ini
   lib_deps =
       m5stack/M5Unified
       m5stack/M5Module-LLM
       bblanchon/ArduinoJson
   ```
3. `#include <yolo_object_detector.h>` して上記「使い方」の順序で呼ぶ。
