// ============================================================
//  ROBOT SENSOR TANAH - ESP32
//  Komponen:
//    - ESP32
//    - HC-SR04 (obstacle detection)
//    - Sensor pH Tanah (analog)
//    - Servo MG996R (lengan sensor, terpasang menghadap depan)
//    - OLED SSD1306 I2C 128x64
//    - Tracker Sensor (obstacle)
//    - Motor DC + L298N
//  Navigasi : timing-based (sesuaikan MS_PER_METER & TURN_90_MS)
//  Storage  : WiFi -> Google Sheets via Apps Script
//  Obstacle : mundur -> belok kiri -> lurus -> belok kanan -> lanjut
//  Sensor   : berhenti -> belok kanan 90° -> servo turun -> baca pH
//             -> servo naik -> belok kiri 90° -> lanjut jalan
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

const char* WIFI_SSID       = "NAMA_WIFI_KAMU";
const char* WIFI_PASSWORD   = "PASSWORD_WIFI_KAMU";
const char* APPS_SCRIPT_URL = "https://script.google.com/macros/s/AKfycbxlhb9j4nRgBZwSQ8Ard6_-UTtttLHMErGjsQCDpPM35znoRUBZ4Q0FJ-wY871Dbips/exec";

const long  GMT_OFFSET_SEC  = 7 * 3600;
const int   DAYLIGHT_OFFSET = 0;

// ============================================================
// KALIBRASI NAVIGASI — SESUAIKAN SETELAH TEST FISIK
// ============================================================

const long MS_PER_METER  = 3000; // ms untuk 1 meter lurus
const long TURN_90_MS    = 1500; // ms untuk putar 90 derajat di tempat

// Obstacle avoidance timing
const long OBS_MUNDUR_MS      = 400;
const long OBS_BELOK_KIRI_MS  = 600;
const long OBS_LURUS_MS       = 800;
const long OBS_BELOK_KANAN_MS = 600;

// ============================================================
// KALIBRASI SERVO — SESUAIKAN SETELAH TEST FISIK
// ============================================================

// State istirahat: tangan menghadap ke depan (sejajar badan robot)
// State sensor   : tangan turun ke tanah (setelah robot belok kanan)
// Sesuaikan nilai ini dengan test_servo.ino terlebih dahulu
const int SERVO_DEPAN = 0;   // derajat posisi istirahat (menghadap depan)
const int SERVO_BAWAH = 90;  // derajat posisi turun ke tanah
const int SERVO_DELAY_MS = 2000; // waktu tunggu sensor di tanah (ms)

// Titik sensor per sisi
const float SENSOR_POINT_1 = 2.5;
const float SENSOR_POINT_2 = 7.5;
const float SIDE_LENGTH    = 10.0;

// ============================================================
// PIN DEFINITIONS
// ============================================================

#define MOTOR_IN1   25
#define MOTOR_IN2   26
#define MOTOR_IN3   27
#define MOTOR_IN4   14
#define MOTOR_ENA   32
#define MOTOR_ENB   33

#define TRIG_PIN    5
#define ECHO_PIN    18
#define TRACKER_PIN 19
#define SERVO_PIN   13
#define PH_PIN      34

#define OLED_SDA    21
#define OLED_SCL    22
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_ADDR   0x3C

#define MOTOR_SPEED 180

// ============================================================
// OBJEK & STATE
// ============================================================

Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Servo armServo;

struct SensorData {
  String waktu;
  String lokasi;
  float  ph;
};

SensorData dataLog[8];
int dataCount = 0;

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_IN1, OUTPUT); pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT); pinMode(MOTOR_IN4, OUTPUT);
  pinMode(MOTOR_ENA, OUTPUT); pinMode(MOTOR_ENB, OUTPUT);
  motorStop();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRACKER_PIN, INPUT);

  armServo.attach(SERVO_PIN);
  armServo.write(SERVO_DEPAN); // posisi awal: tangan menghadap depan
  delay(500);

  Wire.begin(OLED_SDA, OLED_SCL);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED tidak terdeteksi!");
  }
  oledShow("Booting...", "", "", "");

  connectWiFi();

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

    oledShow("Kirim data...", "", "", "");
    kirimDataKeSheets();

    oledShow("Sesi selesai", "Data terkirim", "Tunggu sesi", "berikutnya...");
    Serial.println("=== SESI SELESAI ===");
    delay(60000);
  }

  delay(30000);
}

// ============================================================
// CEK JADWAL
// ============================================================

bool isScheduleTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return false;
  return ((timeinfo.tm_hour == 8 || timeinfo.tm_hour == 20) && timeinfo.tm_min == 0);
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

void jalankanRute() {
  for (int sisi = 1; sisi <= 4; sisi++) {
    oledShow("Sisi " + String(sisi) + "/4", "Menuju P1...", "", "");
    majuDenganObstacle(SENSOR_POINT_1);
    lakukanSensor(sisi, 1);

    oledShow("Sisi " + String(sisi) + "/4", "Menuju P2...", "", "");
    majuDenganObstacle(SENSOR_POINT_2 - SENSOR_POINT_1);
    lakukanSensor(sisi, 2);

    majuDenganObstacle(SIDE_LENGTH - SENSOR_POINT_2);

    if (sisi < 4) {
      oledShow("Putar kiri...", "", "", "");
      putarKiri90();
    }
  }

  motorStop();
  armServo.write(SERVO_DEPAN); // pastikan tangan kembali ke posisi depan
}

// ============================================================
// PROSEDUR SENSOR
// Urutan: berhenti -> belok kanan -> servo turun -> baca pH
//         -> servo naik -> belok kiri -> lanjut
// ============================================================

void lakukanSensor(int sisi, int titik) {
  String lokasi = "S" + String(sisi) + "-P" + String(titik);

  // 1. Pastikan sudah berhenti
  motorStop();
  delay(300);
  oledShow("SENSOR " + lokasi, "Belok ke tanaman", "", "");
  Serial.println("Sensor " + lokasi + " - belok kanan ke tanaman");

  // 2. Belok kanan 90° untuk menghadap tanaman (sisi kanan jalur)
  putarKanan90();
  delay(300);

  // 3. Turunkan servo ke tanah
  oledShow("SENSOR " + lokasi, "Turunkan probe...", "", "");
  Serial.println("Servo turun");
  armServo.write(SERVO_BAWAH);
  delay(SERVO_DELAY_MS); // tunggu probe stabil di tanah

  // 4. Baca pH
  float phValue = bacaPH();
  String waktu = getCurrentTime();

  // 5. Simpan data
  if (dataCount < 8) {
    dataLog[dataCount] = {waktu, lokasi, phValue};
    dataCount++;
  }

  // 6. Tampil di OLED
  oledShow("pH: " + String(phValue, 2), lokasi, waktu, "");
  Serial.println("pH=" + String(phValue, 2) + " @ " + lokasi);
  delay(1500); // tampilkan sebentar di OLED

  // 7. Naikkan servo kembali ke posisi depan
  oledShow("SENSOR " + lokasi, "Naikkan probe...", "", "");
  Serial.println("Servo naik");
  armServo.write(SERVO_DEPAN);
  delay(800); // beri waktu servo bergerak penuh

  // 8. Belok kiri 90° untuk kembali ke arah semula
  oledShow("SENSOR " + lokasi, "Kembali ke jalur", "", "");
  Serial.println("Belok kiri kembali ke jalur");
  putarKiri90();
  delay(300);

  Serial.println("Sensor selesai, lanjut jalan");
}

// ============================================================
// BACA SENSOR pH
// ============================================================

float bacaPH() {
  long total = 0;
  for (int i = 0; i < 10; i++) {
    total += analogRead(PH_PIN);
    delay(10);
  }
  float voltage = (total / 10.0) * (3.3 / 4095.0);

  // Rumus estimasi — KALIBRASI dengan larutan buffer pH 4 dan 7
  // Ukur voltage saat di buffer, hitung slope dan offset
  float ph = 7.0 + ((2.5 - voltage) / 0.18);
  return constrain(ph, 0.0, 14.0);
}

// ============================================================
// NAVIGASI DENGAN OBSTACLE AVOIDANCE
// ============================================================

void majuDenganObstacle(float meter) {
  long durasi     = (long)(meter * MS_PER_METER);
  long waktuMulai = millis();

  motorMaju();

  while (true) {
    if ((millis() - waktuMulai) >= durasi) break;

    if (adaHalangan()) {
      motorStop();
      long waktuTerpakai = millis() - waktuMulai;
      hindariHalangan();
      long waktuHindar = millis() - waktuMulai - waktuTerpakai;
      waktuMulai += waktuHindar;
      motorMaju();
    }

    delay(50);
  }

  motorStop();
}

// ============================================================
// OBSTACLE AVOIDANCE
// Hindari ke kiri — tanaman selalu di kanan, jangan rusak
// ============================================================

void hindariHalangan() {
  oledShow("HALANGAN!", "Mundur...", "", "");
  Serial.println("Halangan terdeteksi!");

  motorMundur();
  delay(OBS_MUNDUR_MS);
  motorStop();
  delay(200);

  oledShow("HALANGAN!", "Belok kiri...", "", "");
  belokKiri();
  delay(OBS_BELOK_KIRI_MS);
  motorStop();
  delay(200);

  oledShow("HALANGAN!", "Lewati...", "", "");
  motorMaju();
  delay(OBS_LURUS_MS);
  motorStop();
  delay(200);

  oledShow("HALANGAN!", "Kembali jalur...", "", "");
  belokKanan();
  delay(OBS_BELOK_KANAN_MS);
  motorStop();
  delay(200);

  oledShow("Lanjut...", "", "", "");
  Serial.println("Halangan terlewati");
}

// ============================================================
// MOTOR PRIMITIVES
// ============================================================

// Putar kiri 90° di tempat (navigasi sisi & kembali dari sensor)
void putarKiri90() {
  // Motor kiri mundur, kanan maju = pivot kiri
  digitalWrite(MOTOR_IN1, LOW);  digitalWrite(MOTOR_IN2, HIGH);
  digitalWrite(MOTOR_IN3, HIGH); digitalWrite(MOTOR_IN4, LOW);
  analogWrite(MOTOR_ENA, MOTOR_SPEED);
  analogWrite(MOTOR_ENB, MOTOR_SPEED);
  delay(TURN_90_MS);
  motorStop();
  delay(300);
}

// Putar kanan 90° di tempat (menghadap tanaman saat sensor)
void putarKanan90() {
  // Motor kiri maju, kanan mundur = pivot kanan
  digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);  digitalWrite(MOTOR_IN4, HIGH);
  analogWrite(MOTOR_ENA, MOTOR_SPEED);
  analogWrite(MOTOR_ENB, MOTOR_SPEED);
  delay(TURN_90_MS);
  motorStop();
  delay(300);
}

// Belok kiri halus (untuk obstacle avoidance)
void belokKiri() {
  digitalWrite(MOTOR_IN1, LOW);  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, HIGH); digitalWrite(MOTOR_IN4, LOW);
  analogWrite(MOTOR_ENA, 0);
  analogWrite(MOTOR_ENB, MOTOR_SPEED);
}

// Belok kanan halus (untuk obstacle avoidance)
void belokKanan() {
  digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);  digitalWrite(MOTOR_IN4, LOW);
  analogWrite(MOTOR_ENA, MOTOR_SPEED);
  analogWrite(MOTOR_ENB, 0);
}

void motorMaju() {
  digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, HIGH); digitalWrite(MOTOR_IN4, LOW);
  analogWrite(MOTOR_ENA, MOTOR_SPEED);
  analogWrite(MOTOR_ENB, MOTOR_SPEED);
}

void motorMundur() {
  digitalWrite(MOTOR_IN1, LOW);  digitalWrite(MOTOR_IN2, HIGH);
  digitalWrite(MOTOR_IN3, LOW);  digitalWrite(MOTOR_IN4, HIGH);
  analogWrite(MOTOR_ENA, MOTOR_SPEED);
  analogWrite(MOTOR_ENB, MOTOR_SPEED);
}

void motorStop() {
  digitalWrite(MOTOR_IN1, LOW); digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW); digitalWrite(MOTOR_IN4, LOW);
  analogWrite(MOTOR_ENA, 0);    analogWrite(MOTOR_ENB, 0);
}

// ============================================================
// DETEKSI HALANGAN
// ============================================================

bool adaHalangan() {
  float jarak = bacaJarak();
  if (jarak > 0 && jarak < 30) return true;
  if (digitalRead(TRACKER_PIN) == LOW) return true;
  return false;
}

float bacaJarak() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 30000);
  if (dur == 0) return -1;
  return dur * 0.034 / 2.0;
}

// ============================================================
// OLED
// ============================================================

void oledShow(String b1, String b2, String b3, String b4) {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0,  0); oled.println(b1);
  oled.setCursor(0, 16); oled.println(b2);
  oled.setCursor(0, 32); oled.println(b3);
  oled.setCursor(0, 48); oled.println(b4);
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
    delay(500); Serial.print("."); coba++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    oledShow("WiFi OK", WiFi.localIP().toString(), "", "");
    Serial.println("\nWiFi: " + WiFi.localIP().toString());
  } else {
    oledShow("WiFi GAGAL", "Cek SSID/Pass", "", "");
    Serial.println("WiFi gagal!");
  }
  delay(1000);
}

// ============================================================
// KIRIM KE GOOGLE SHEETS
// ============================================================

void kirimDataKeSheets() {
  if (WiFi.status() != WL_CONNECTED) connectWiFi();

  for (int i = 0; i < dataCount; i++) {
    if (WiFi.status() != WL_CONNECTED) break;

    HTTPClient http;
    http.begin(APPS_SCRIPT_URL);
    http.addHeader("Content-Type", "application/json");

    String payload = "{";
    payload += "\"waktu\":\""  + dataLog[i].waktu  + "\",";
    payload += "\"lokasi\":\"" + dataLog[i].lokasi + "\",";
    payload += "\"ph\":"       + String(dataLog[i].ph, 2);
    payload += "}";

    int code = http.POST(payload);
    Serial.println((code == 200 || code == 302)
      ? "Terkirim: " + dataLog[i].lokasi
      : "Gagal: "    + dataLog[i].lokasi + " (" + String(code) + ")");

    http.end();
    delay(500);
  }
}
