// ============================================================
//  ROBOT SENSOR TANAH - ESP32
//  Komponen:
//    - ESP32
//    - HC-SR04 (obstacle detection)
//    - Sensor pH Tanah (analog)
//    - Servo MG996R (lengan sensor)
//    - OLED SSD1306 I2C 128x64
//    - Tracker Sensor (obstacle)
//    - Motor DC + L298N
//  Navigasi : timing-based (sesuaikan MS_PER_METER)
//  Storage  : WiFi -> Google Sheets via Apps Script
// ============================================================

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <time.h>

// ============================================================
// KONFIGURASI — SESUAIKAN BAGIAN INI
// ============================================================

// WiFi
const char* WIFI_SSID     = "NAMA_WIFI_KAMU";
const char* WIFI_PASSWORD = "PASSWORD_WIFI_KAMU";

// Google Apps Script URL (isi setelah deploy script)
const char* APPS_SCRIPT_URL = "https://script.google.com/macros/s/XXXXXXX/exec";

// Timezone (WIB = UTC+7)
const long  GMT_OFFSET_SEC  = 7 * 3600;
const int   DAYLIGHT_OFFSET = 0;

// Navigasi — KALIBRASI INI setelah lihat robot bergerak
// Ukur: jalankan motor 5 detik, ukur berapa meter, hitung MS_PER_METER
const long MS_PER_METER  = 3000;  // ms untuk jarak 1 meter (estimasi awal)
const long TURN_90_MS    = 1500;  // ms untuk putar 90 derajat (estimasi awal)

// Titik sensor dalam meter (dari titik awal tiap sisi)
const float SENSOR_POINT_1 = 2.5;
const float SENSOR_POINT_2 = 7.5;
const float SIDE_LENGTH    = 10.0;

// Servo
const int SERVO_UP   = 0;    // derajat posisi atas (istirahat)
const int SERVO_DOWN = 90;   // derajat turun ke tanah
const int SERVO_DELAY_MS = 2000; // waktu tunggu sensor di tanah (ms)

// ============================================================
// PIN DEFINITIONS
// ============================================================

// Motor L298N
#define MOTOR_IN1  25
#define MOTOR_IN2  26
#define MOTOR_IN3  27
#define MOTOR_IN4  14
#define MOTOR_ENA  32   // PWM speed kiri
#define MOTOR_ENB  33   // PWM speed kanan

// HC-SR04
#define TRIG_PIN   5
#define ECHO_PIN   18

// Tracker Sensor (obstacle, digital)
#define TRACKER_PIN 19

// Servo
#define SERVO_PIN  13

// pH Tanah (analog)
#define PH_PIN     34   // GPIO34 ADC1

// OLED I2C
#define OLED_SDA   21
#define OLED_SCL   22
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_ADDR  0x3C

// Motor speed (0-255)
#define MOTOR_SPEED 180

// ============================================================
// OBJEK
// ============================================================

Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Servo armServo;

// ============================================================
// STATE ROBOT
// ============================================================

// Rute: 4 sisi, tiap sisi 2 titik sensor = 8 titik total per sesi
// Urutan gerak: bawah (kiri->kanan) -> putar -> atas (kanan->kiri) dst
// Posisi awal: pojok kiri bawah, hadap kanan

struct SensorData {
  String waktu;
  String lokasi;  // contoh: "Sisi1-P1"
  float  ph;
};

SensorData dataLog[8];   // simpan 8 titik per sesi
int dataCount = 0;

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);

  // Pin motor
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_ENB, OUTPUT);
  motorStop();

  // Pin sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRACKER_PIN, INPUT);

  // Servo
  armServo.attach(SERVO_PIN);
  armServo.write(SERVO_UP);

  // OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED tidak terdeteksi!");
  }
  oledShow("Booting...", "", "", "");

  // WiFi
  connectWiFi();

  // Sinkron waktu via NTP
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET, "pool.ntp.org");
  oledShow("Sync waktu...", "", "", "");
  delay(2000);

  oledShow("Robot Siap", "Menunggu", "jadwal...", "");
  Serial.println("Setup selesai. Menunggu jadwal 08:00 / 20:00");
}

// ============================================================
// LOOP UTAMA
// ============================================================

void loop() {
  if (isScheduleTime()) {
    Serial.println("=== MULAI SESI SENSOR ===");
    oledShow("SESI DIMULAI", getCurrentTime(), "", "");
    delay(2000);

    dataCount = 0;
    jalankanRute();

    Serial.println("=== KIRIM DATA ===");
    oledShow("Kirim data...", "", "", "");
    kirimDataKeSheets();

    oledShow("Sesi selesai", "Data terkirim", "Tunggu sesi", "berikutnya...");
    Serial.println("=== SESI SELESAI ===");

    // Tunggu 60 detik agar tidak trigger 2x di menit yang sama
    delay(60000);
  }

  delay(30000); // cek jadwal tiap 30 detik
}

// ============================================================
// CEK JADWAL
// ============================================================

bool isScheduleTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return false;

  int jam   = timeinfo.tm_hour;
  int menit = timeinfo.tm_min;

  // Trigger pada jam 8 pagi atau 8 malam, menit 0
  return ((jam == 8 || jam == 20) && menit == 0);
}

String getCurrentTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "--:--";
  char buf[20];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M", &timeinfo);
  return String(buf);
}

// ============================================================
// RUTE NAVIGASI
// ============================================================
// Layout 10x10m, 4 sisi, tiap sisi sensor di 2.5m dan 7.5m
// Pola gerak: zig-zag searah jarum jam dari pojok kiri bawah

void jalankanRute() {
  for (int sisi = 1; sisi <= 4; sisi++) {
    oledShow("Sisi " + String(sisi) + "/4", "Menuju P1...", "", "");

    // Maju ke titik sensor 1 (2.5m)
    majuDenganObstacle(SENSOR_POINT_1);

    // Lakukan sensor
    lakukanSensor(sisi, 1);

    oledShow("Sisi " + String(sisi) + "/4", "Menuju P2...", "", "");

    // Maju ke titik sensor 2 (5m lagi = 7.5m total)
    majuDenganObstacle(SENSOR_POINT_2 - SENSOR_POINT_1);

    // Lakukan sensor
    lakukanSensor(sisi, 2);

    // Maju sisa jalan ke ujung sisi (2.5m lagi)
    majuDenganObstacle(SIDE_LENGTH - SENSOR_POINT_2);

    // Putar 90 derajat ke kiri (searah jarum jam dari atas)
    if (sisi < 4) {
      oledShow("Putar...", "", "", "");
      putar90();
    }
  }

  // Kembali ke posisi awal setelah 4 sisi
  // (setelah 4 putaran 90° = sudah kembali ke arah asal)
  motorStop();
  armServo.write(SERVO_UP);
}

// ============================================================
// SENSOR
// ============================================================

void lakukanSensor(int sisi, int titik) {
  String lokasi = "S" + String(sisi) + "-P" + String(titik);
  oledShow("SENSOR", lokasi, "Turunkan lengan", "");

  // Turunkan servo 90 derajat
  armServo.write(SERVO_DOWN);
  delay(SERVO_DELAY_MS); // tunggu sensor stabil di tanah

  // Baca pH
  float phValue = bacaPH();

  // Naikkan servo
  armServo.write(SERVO_UP);
  delay(500);

  // Simpan data
  String waktu = getCurrentTime();
  if (dataCount < 8) {
    dataLog[dataCount] = {waktu, lokasi, phValue};
    dataCount++;
  }

  // Tampil OLED
  oledShow("pH: " + String(phValue, 2), lokasi, waktu, "");
  Serial.println("Sensor " + lokasi + " pH=" + String(phValue, 2));
  delay(2000);
}

// ============================================================
// BACA SENSOR pH
// ============================================================

float bacaPH() {
  // Baca ADC rata-rata 10 sample
  long total = 0;
  for (int i = 0; i < 10; i++) {
    total += analogRead(PH_PIN);
    delay(10);
  }
  float adcVal = total / 10.0;

  // Konversi ADC ke tegangan (ESP32 ADC 12-bit, 3.3V)
  float voltage = adcVal * (3.3 / 4095.0);

  // Konversi tegangan ke pH
  // PERLU KALIBRASI dengan larutan buffer pH 4, 7, 9
  // Rumus linear estimasi: pH = 7 + ((2.5 - voltage) / 0.18)
  // Sesuaikan slope dan offset setelah kalibrasi
  float ph = 7.0 + ((2.5 - voltage) / 0.18);

  // Clamp nilai pH ke range valid
  ph = constrain(ph, 0.0, 14.0);
  return ph;
}

// ============================================================
// GERAK MOTOR
// ============================================================

void majuDenganObstacle(float meter) {
  long durasi = (long)(meter * MS_PER_METER);
  long mulai  = millis();

  motorMaju();

  while (millis() - mulai < durasi) {
    // Cek obstacle
    if (adaHalangan()) {
      motorStop();
      oledShow("HALANGAN!", "Berhenti...", "Tunggu 3 detik", "");
      delay(3000);

      // Coba mundur sedikit lalu cek lagi
      motorMundur();
      delay(500);
      motorStop();
      delay(500);

      // Cek lagi
      if (adaHalangan()) {
        // Masih ada halangan — tunggu lebih lama
        oledShow("HALANGAN!", "Tunggu...", "", "");
        delay(5000);
      }

      // Lanjut maju, timer tidak di-reset (estimasi posisi tetap)
      motorMaju();
    }
    delay(50);
  }

  motorStop();
}

void putar90() {
  // Putar kanan: motor kiri maju, motor kanan mundur
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, HIGH);
  analogWrite(MOTOR_ENA, MOTOR_SPEED);
  analogWrite(MOTOR_ENB, MOTOR_SPEED);
  delay(TURN_90_MS);
  motorStop();
  delay(300);
}

void motorMaju() {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, HIGH);
  digitalWrite(MOTOR_IN4, LOW);
  analogWrite(MOTOR_ENA, MOTOR_SPEED);
  analogWrite(MOTOR_ENB, MOTOR_SPEED);
}

void motorMundur() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, HIGH);
  analogWrite(MOTOR_ENA, MOTOR_SPEED);
  analogWrite(MOTOR_ENB, MOTOR_SPEED);
}

void motorStop() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, LOW);
  analogWrite(MOTOR_ENA, 0);
  analogWrite(MOTOR_ENB, 0);
}

// ============================================================
// DETEKSI HALANGAN
// ============================================================

bool adaHalangan() {
  // HC-SR04
  float jarak = bacaJarak();
  if (jarak > 0 && jarak < 30) return true; // halangan < 30cm

  // Tracker sensor (LOW = deteksi, sesuaikan logika jika terbalik)
  if (digitalRead(TRACKER_PIN) == LOW) return true;

  return false;
}

float bacaJarak() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long durasi = pulseIn(ECHO_PIN, HIGH, 30000); // timeout 30ms
  if (durasi == 0) return -1; // tidak ada pantulan
  return durasi * 0.034 / 2.0; // cm
}

// ============================================================
// OLED DISPLAY
// ============================================================

void oledShow(String baris1, String baris2, String baris3, String baris4) {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);

  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println(baris1);
  oled.setCursor(0, 16);
  oled.println(baris2);
  oled.setCursor(0, 32);
  oled.println(baris3);
  oled.setCursor(0, 48);
  oled.println(baris4);

  oled.display();
}

// ============================================================
// WIFI
// ============================================================

void connectWiFi() {
  oledShow("Konek WiFi...", WIFI_SSID, "", "");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int coba = 0;
  while (WiFi.status() != WL_CONNECTED && coba < 20) {
    delay(500);
    Serial.print(".");
    coba++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    oledShow("WiFi OK", WiFi.localIP().toString(), "", "");
    Serial.println("\nWiFi terhubung: " + WiFi.localIP().toString());
  } else {
    oledShow("WiFi GAGAL", "Cek SSID/Pass", "", "");
    Serial.println("WiFi gagal terhubung!");
  }
  delay(1000);
}

// ============================================================
// KIRIM DATA KE GOOGLE SHEETS
// ============================================================

void kirimDataKeSheets() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  for (int i = 0; i < dataCount; i++) {
    if (WiFi.status() != WL_CONNECTED) break;

    HTTPClient http;
    http.begin(APPS_SCRIPT_URL);
    http.addHeader("Content-Type", "application/json");

    String payload = "{";
    payload += "\"waktu\":\"" + dataLog[i].waktu + "\",";
    payload += "\"lokasi\":\"" + dataLog[i].lokasi + "\",";
    payload += "\"ph\":" + String(dataLog[i].ph, 2);
    payload += "}";

    int httpCode = http.POST(payload);

    if (httpCode == 200 || httpCode == 302) {
      Serial.println("Data " + dataLog[i].lokasi + " terkirim");
    } else {
      Serial.println("Gagal kirim " + dataLog[i].lokasi + " code:" + String(httpCode));
    }

    http.end();
    delay(500); // jeda antar request
  }
}
