#include <Arduino.h>
#include <AlfredoCRSF.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

#include "Logger.h"
#include "PocketMaster.h"
#include "DualTB6612.h"
#include "PFServo.h"

#if __has_include("secrets.h")
  // secrets.h hanya ada di komputer lokal dan di-ignore oleh Git agar
  // SSID/password Wi-Fi tidak ikut ter-upload ke repository publik.
  #include "secrets.h"
#else
  // Project tetap bisa di-compile tanpa credential. Dalam kondisi ini
  // kontrol RC tetap bekerja, tetapi fitur Wi-Fi/WebSerial dinonaktifkan.
  #define WIFI_SSID ""
  #define WIFI_PASSWORD ""
#endif

// ============================================================
// Hardware setup
// ============================================================
constexpr int PIN_RX = 21;
constexpr int PIN_TX = 20;

constexpr int PIN_SDA = 8;
constexpr int PIN_SCL = 9;

constexpr int PIN_STBY = -1;

constexpr int PIN_DIR1_XL1 = 1;
constexpr int PIN_DIR2_XL1 = 0;
constexpr int PIN_PWM_XL1  = 7;

constexpr int PIN_DIR1_XL2 = 2;
constexpr int PIN_DIR2_XL2 = 3;
constexpr int PIN_PWM_XL2  = 6;

constexpr int PIN_DIR1_L = 5;
constexpr int PIN_DIR2_L = 4;

constexpr int PIN_SERVO = 10;

constexpr int CH_PWM_XL1 = 0;
constexpr int CH_PWM_XL2 = 1;
constexpr int CH_SERVO   = 2;

DualTB6612::ChannelPins chA{PIN_DIR1_XL1, PIN_DIR2_XL1, PIN_PWM_XL1, CH_PWM_XL1};
DualTB6612::ChannelPins chB{PIN_DIR1_XL2, PIN_DIR2_XL2, PIN_PWM_XL2, CH_PWM_XL2};

// Motor memakai PWM 20 kHz, resolusi 10-bit (duty 0..1023).
// ZeroMode::Brake membuat kedua input H-bridge aktif saat command = 0.
DualTB6612 motors(PIN_STBY, chA, chB, 20000, 10, ZeroMode::Brake);

// Servo LEGO PF menggunakan LEDC channel 2. Default PFServo adalah
// 50 Hz, resolusi 16-bit, dan pulse width 1000..2000 microseconds.
PFServo steering(PIN_SERVO, CH_SERVO);

// CRSF memakai UART1 terpisah dari USB Serial sehingga log debug tidak
// mengganggu komunikasi receiver ExpressLRS pada 420000 baud.
HardwareSerial crsfSerial(1);
AlfredoCRSF crsf;
PocketMaster PocketRadio(crsf);

// AsyncWebServer melayani halaman WebSerial pada port HTTP standar (80).
AsyncWebServer server(80);

// Web server hanya dimulai sekali setelah ESP32 pertama kali mendapat IP.
bool webSerialStarted = false;
bool wifiEnabled = false;
wl_status_t previousWiFiStatus = WL_IDLE_STATUS;
unsigned long lastWiFiAttemptMs = 0;

// State machine menggantikan delay(800/500/500) yang sebelumnya memblokir
// pembacaan CRSF. Dengan millis(), receiver dan Wi-Fi tetap diproses.
enum class FailsafeStage : uint8_t {
  Idle,
  Braking,
  Coasting
};

FailsafeStage failsafeStage = FailsafeStage::Idle;
unsigned long failsafeStageStartedMs = 0;

void debugPrintChannels();
void startWiFi();
void serviceWiFi();
void resetFailsafe();
void serviceFailsafe();

// Semua macro LOGx() berakhir di fungsi ini. Serial USB selalu menerima
// log; browser ikut menerima log setelah halaman WebSerial tersedia.
void writeLogLine(const char* line) {
  Serial.print(line);
  if (webSerialStarted) {
    WebSerial.print(line);
  }
}

void startWiFi() {
  // String kosong berarti include/secrets.h belum dibuat.
  wifiEnabled = WIFI_SSID[0] != '\0';

  if (!wifiEnabled) {
    LOGW("WIFI", "Disabled: copy secrets.example.h to include/secrets.h and fill credentials");
    return;
  }

  WiFi.mode(WIFI_STA);
  // Station mode berarti ESP32 bergabung ke router IndiHome; ESP32 tidak
  // membuat access point sendiri.
  WiFi.setAutoReconnect(true);
  // Jangan tulis credential berulang kali ke flash internal.
  WiFi.persistent(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastWiFiAttemptMs = millis();

  LOGI("WIFI", "Connecting to %s...", WIFI_SSID);
}

void serviceWiFi() {
  // Fungsi ini dipanggil setiap loop dan tidak menunggu koneksi secara blocking.
  if (!wifiEnabled) return;

  const wl_status_t status = WiFi.status();

  if (status == WL_CONNECTED) {
    if (previousWiFiStatus != WL_CONNECTED) {
      LOGI("WIFI", "Connected. IP: %s", WiFi.localIP().toString().c_str());

      if (!webSerialStarted) {
        // WebSerial mendaftarkan route /webserial ke AsyncWebServer.
        WebSerial.begin(&server);
        server.begin();
        webSerialStarted = true;
        LOGI("WEB", "Open http://%s/webserial", WiFi.localIP().toString().c_str());
      }
    }
  } else {
    if (previousWiFiStatus == WL_CONNECTED) {
      LOGW("WIFI", "Connection lost; retrying in background");
    }

    if (millis() - lastWiFiAttemptMs >= 10000UL) {
      // Batasi percobaan reconnect menjadi sekali setiap 10 detik.
      lastWiFiAttemptMs = millis();
      WiFi.reconnect();
    }
  }

  previousWiFiStatus = status;

  if (webSerialStarted) {
    // Membersihkan client WebSocket dan mengirim buffer log secara periodik.
    WebSerial.loop();
  }
}

void resetFailsafe() {
  // Dipanggil segera setelah link CRSF kembali tersedia.
  failsafeStage = FailsafeStage::Idle;
}

void serviceFailsafe() {
  const unsigned long now = millis();

  switch (failsafeStage) {
    case FailsafeStage::Idle:
      // Tahap pertama: disarm dan active-brake kedua motor secepat mungkin.
      if (PocketRadio.isArmed) {
        LOGW("FAILSAFE", "Receiver lost -> disarmed");
        PocketRadio.isArmed = false;
      } else {
        LOGI("CRSF", "Receiver disconnected");
      }

      motors.setSpeedA(0);
      motors.setSpeedB(0);
      failsafeStage = FailsafeStage::Braking;
      failsafeStageStartedMs = now;
      break;

    case FailsafeStage::Braking:
      // Setelah 800 ms braking, lepaskan motor ke mode coast.
      if (now - failsafeStageStartedMs >= 800UL) {
        motors.coastA();
        motors.coastB();
        failsafeStage = FailsafeStage::Coasting;
        failsafeStageStartedMs = now;
      }
      break;

    case FailsafeStage::Coasting:
      // Pertahankan coast selama 1000 ms, lalu ulangi siklus pemantauan.
      if (now - failsafeStageStartedMs >= 1000UL) {
        failsafeStage = FailsafeStage::Idle;
      }
      break;
  }
}

void setup() {
  // USB Serial digunakan untuk upload/debug lokal pada baud 115200.
  Serial.begin(115200);

  const unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 1500UL)) {
    delay(10);
  }

  LOGI("BOOT", "Serial ready. Starting CRSF...");

  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
  crsf.begin(crsfSerial);
  LOGI("CRSF", "UART1 @ %d baud (RX=%d TX=%d)",
       (int)CRSF_BAUDRATE, PIN_RX, PIN_TX);

  // Namespace "rc-cal" adalah lokasi penyimpanan nilai kalibrasi di NVS.
  PocketRadio.begin("rc-cal");
  // Pergerakan kecil di sekitar center (+/-20 us) dianggap netral.
  PocketRadio.setDeadzoneUs(20);
  PocketRadio.loadCalibration();

  if (PocketRadio.isCallibrated) {
    LOGI("LED", "Calibration found -> Blink 3x");
  }

  steering.begin();
  // setInput(0) adalah center karena PFServo menerima -1000..+1000.
  steering.setInput(0);

  motors.begin();
  motors.setZeroMode(ZeroMode::Brake);
  // Safety: remain braked at boot until CRSF is linked and SA is armed.

  startWiFi();
}

void loop() {
  // Harus dipanggil sesering mungkin agar frame CRSF dan status link terbarui.
  PocketRadio.update();
  // Wi-Fi/WebSerial dirawat tanpa menghentikan proses kontrol RC.
  serviceWiFi();

  if (PocketRadio.isLinkUp()) {
    resetFailsafe();

    if (PocketRadio.val(SA) == 1) {
      // SA posisi atas = armed. Motor dan steering baru mengikuti transmitter.
      if (!PocketRadio.isArmed) {
        PocketRadio.isArmed = true;
        LOGI("RC", "SA UP -> system armed");

        if (!PocketRadio.isCallibrated) {
          PocketRadio.loadCalibration();
        }
      }

      debugPrintChannels();
      // THROTTLE sudah dinormalisasi PocketMaster ke 0..1000.
      motors.setSpeedA(PocketRadio.val(THROTTLE));
      motors.setSpeedB(PocketRadio.val(THROTTLE));

      // ROLL dari PocketMaster bernilai 0..1000 dengan center 500,
      // sedangkan PFServo membutuhkan -1000..+1000 dengan center 0.
      const int steeringInput = ((int)PocketRadio.val(ROLL) - 500) * 2;
      steering.setInput(steeringInput);
    } else if (PocketRadio.val(SA) == 3) {
      // SA posisi bawah = disarmed sekaligus mode pengambilan min/max kalibrasi.
      if (PocketRadio.isArmed) {
        LOGI("RC", "SA DOWN -> system disarmed, entering calibration mode");
      }

      PocketRadio.isArmed = false;
      PocketRadio.startCalibration();

      if (PocketRadio.isCalibSaved()) {
        LOGI("CAL",
             "ROLL[max:%4u min:%4u] PITCH[max:%4u min:%4u] YAW[max:%4u min:%4u] THR[max:%4u min:%4u] S1[max:%4u min:%4u]",
             PocketRadio.max(ROLL), PocketRadio.min(ROLL),
             PocketRadio.max(PITCH), PocketRadio.min(PITCH),
             PocketRadio.max(YAW), PocketRadio.min(YAW),
             PocketRadio.max(THROTTLE), PocketRadio.min(THROTTLE),
             PocketRadio.max(S1), PocketRadio.min(S1));
      }
    }
  } else {
    // Tidak ada delay panjang di sini; failsafe diproses sebagai state machine.
    serviceFailsafe();
  }

  delay(10);
}

void debugPrintChannels() {
#if defined(DEBUG)
  static unsigned long lastPrintMs = 0;
  const unsigned long now = millis();

  // Kontrol tetap diperbarui sekitar setiap 10 ms, tetapi tampilan telemetry
  // dibatasi 10 Hz supaya buffer WebSerial dan CPU tidak dibanjiri log.
  if (now - lastPrintMs < 100UL) return;
  lastPrintMs = now;

  LOGD("CRSF",
       "R:%4u P:%4u T:%4u Y:%4u | SA:%d SB:%d SC:%d SD:%d SE:%d | S1:%4u | LQ:%3u",
       PocketRadio.val(ROLL),
       PocketRadio.val(PITCH),
       PocketRadio.val(THROTTLE),
       PocketRadio.val(YAW),
       PocketRadio.val(SA),
       PocketRadio.val(SB),
       PocketRadio.val(SC),
       PocketRadio.val(SD),
       PocketRadio.val(SE),
       PocketRadio.val(S1),
       PocketRadio.linkQuality());
#endif
}
