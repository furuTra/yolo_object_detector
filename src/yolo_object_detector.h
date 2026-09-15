/**
 * @file yolo_object_detector.h
 * @brief M5Stack Module LLM (StackFlow protocol) を使ったYOLO物体検出の移植可能パッケージ。
 *
 * JPEG画像バイト列を渡すと Module LLM 上の YOLO で推論し、検出物体
 * ({class_name, confidence, x1, y1, x2, y2}) をコールバックで返す。カメラ/LCD等の
 * 周辺機器には依存しない(JPEGバイト列は呼び出し元が渡す)ため、Module LLM を接続した
 * 別のM5Stackプロジェクトへフォルダごとコピーして再利用できる。詳細は同梱のREADME.md参照。
 *
 * 本ラッパーはStackFlowの詳細に立ち入らず、`M5ModuleLLM`ライブラリの呼び出しを薄くラップする。
 *
 * UARTピンは `M5.getPin(port_c_rxd/txd)` で自動解決する(M5Stack製品前提・M5Unified依存)。
 */
#pragma once

#include <Arduino.h>
#include <M5ModuleLLM.h>
#include <functional>

namespace yolo_object_detector {

// YOLO物体検出の既定モデル。リファレンス(yolo_example.ino)/ YOLO_CoreS3.ino が
// yolo.setup() を引数なしで呼ぶときの既定 "yolo11n" に合わせる。
extern const char* const kDefaultYoloModel;    // "yolo11n"

// YOLOの高スループット通信用ボーレート。リファレンスに合わせて 1.5Mbps。
// フレーム毎にJPEGをUART送信するため、既定の115200bpsでは表示レートが不足する。
extern const uint32_t kYoloBaudRate;           // 1500000

// YOLOが返す検出結果1件。座標はYOLOへ渡した画像(=入力JPEG)のピクセル空間。
// bboxは [x1,y1,x2,y2] の対角コーナー座標(実機検証済みで確定した公開契約)。
struct DetectedObject {
    String class_name;
    float confidence = 0.0f;
    int x1 = 0;
    int y1 = 0;
    int x2 = 0;
    int y2 = 0;
};

/**
 * @brief M5Stack Module LLM (StackFlow) との通信をラップするYOLO物体検出クライアント。
 * カメラ/LCD等の周辺機器には依存しない(JPEGバイト列は呼び出し元が渡す)。
 *
 * 呼び出し順序(reference/yolo_example.ino の setup() と同じ):
 *   Detector detector;
 *   detector.begin(Serial2);
 *   detector.connectAutoBaud();          // 接続できるまでブロック(接続後は115200へ戻す)
 *   detector.resetModule();
 *   detector.setBaudRate(yolo_object_detector::kYoloBaudRate);
 *   detector.setupYolo();
 *
 * 呼び出し順序(loop()、撮影→推論):
 *   detector.detect(jpeg, jpeg_len, [](const DetectedObject& d) { ... });
 */
class Detector {
public:
    Detector()  = default;
    ~Detector() = default;

    /**
     * @brief UART(Serial2)を初期化し、Module LLMへ接続する。
     * ピンは M5.getPin(port_c_rxd/txd) で自動解決する(リファレンス準拠)。
     * M5.begin()より後に呼ぶこと。
     * @return 現状は常にtrue(HardwareSerial::begin()に失敗シグナルがなく失敗を検出できないため)。
     *         boolはAPI互換のため維持する(将来の失敗検出用)。
     */
    bool begin(HardwareSerial& serial, uint32_t baud_rate = 115200);

    /**
     * @brief 接続できるまでブロックする(リファレンスの `while(1) if(checkConnection()) break;`
     * と同じ、タイムアウトなし)。呼び出し前にステータス表示するのは呼び出し側の役目。
     */
    void waitForConnection();

    /**
     * @brief ボーレートを自動検出して接続する(115200 ↔ kYoloBaudRate を交互に試す)。
     *
     * 背景: `setBaudRate()` で上げたボーレートはモジュール側の電源を切るまで保持されるため、
     * ホスト側だけ再起動すると「モジュール=1.5Mbps / ホスト=115200」で永久に接続できなくなる
     * (`checkConnection()`は1回2秒待つため、止まって見える)。本メソッドは各候補ボーレートで
     * pingを試し、接続できたら**必ず115200へ戻して**から返る。以降は
     * `resetModule()` → `setBaudRate(kYoloBaudRate)` → `setupYolo()` の順序を前提とする。
     * 接続できるまでブロックする(タイムアウトなし)。
     */
    void connectAutoBaud();

    /// Module LLMをリセットする(戻り値は確認しない。リファレンス準拠)。
    void resetModule();

    /**
     * @brief モジュール側とホスト側(UART)双方のボーレートを変更する。
     * リファレンスと同じ手順: `sys` 経由でモジュールへ新ボーレートを通知し、
     * こちら側のSerialも同じボーレートで開き直す。begin() の後に呼ぶこと。
     * @return モジュール側の設定成功でtrue。
     */
    bool setBaudRate(uint32_t baud_rate);

    /**
     * @brief `yolo.setup` を呼び、YOLOのwork_idを保持する。
     * @return 成功時true(work_idが空でない)。
     */
    bool setupYolo(const String& model = kDefaultYoloModel);

    /// setupYolo()が成功しているか。
    bool isReady() const { return yolo_work_id_.length() > 0 && yolo_work_id_ != "yolo"; }

    const String& yoloWorkId() const { return yolo_work_id_; }

    /**
     * @brief JPEG画像1枚をYOLOで推論し、検出ごとに onDetection を呼ぶ(同期・ブロッキング)。
     * ライブラリの `ApiYolo::inferenceAndWaitResult()` を使い、返ってくる検出JSON
     * ({"class","confidence","bbox":[x1,y1,x2,y2]})を DetectedObject にパースする。
     * finish時の空チャンクや不正JSONは onDetection を呼ばずにスキップする。
     *
     * @param timeout_ms 応答が途切れてからのタイムアウト。検出0件でモジュールがfinishを
     *        返さない場合の空振り時間になるため、ライブフレームレートに影響する(要実機調整)。
     * @return true=正常完了、false=未setup/タイムアウト。
     */
    bool detect(const uint8_t* jpeg_data, size_t jpeg_len,
                const std::function<void(const DetectedObject&)>& onDetection,
                uint32_t timeout_ms = 500);

private:
    M5ModuleLLM module_;
    String yolo_work_id_;
    // setBaudRate()/connectAutoBaud() でSerialを開き直すために begin() で受け取ったSerialを保持する。
    HardwareSerial* serial_ = nullptr;

    // serial_ を指定ボーレートで開き直し、module_.begin() し直す(RXD/TXDは毎回M5.getPin()で解決)。
    void reopenSerial(uint32_t baud_rate);
};

}  // namespace yolo_object_detector
