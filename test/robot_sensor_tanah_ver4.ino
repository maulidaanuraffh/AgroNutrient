// ============================================================
//  ROBOT SENSOR TANAH - ESP32
//  
//  Komponen final:
//    - NodeMCU ESP32
//    - HC-SR04 (deteksi halangan)
//    - Sensor pH Tanah analog 3 probe
//    - Servo MG996R (lengan, terpasang menghadap depan)
//    - OLED SSD1306 I2C 0.96"
//    - 2x Motor DC Gearbox + Track Wheel Tank
//    - Driver Motor L298N
//    - Breadboard + Cable Jumper
//
//  Navigasi  : timing-based, sesuaikan MS_PER_METER & TURN_90_MS
//  Storage   : WiFi -> Google Sheets via Apps Script
//  Obstacle  : HC-SR04 saja, hindari ke kiri
//  Prosedur sensor:
//    berhenti -> belok kanan 90° -> servo turun -> baca pH
//    -> servo naik -> belok kiri 90° -> lanjut jalan
// ============================================================

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <time.h>

// ============================================================
//  KONFIGURASI — ISI BAGIAN INI SEBELUM UPLOAD
// ============================================================

const char* WIFI_SSID       = "NAMA_WIFI_KAMU";
const char* WIFI_PASSWORD   = "PASSWORD_WIFI_KAMU";
const char* APPS_SCRIPT_URL = "https://script.google.com/macros/s/AKfycbxlhb9j4nRgBZwSQ8Ard6_-UTtttLHMErGjsQCDpPM35znoRUBZ4Q0FJ-wY871Dbips/exec";

// Timezone WIB (UTC+7)
const long GMT_OFFSET_SEC  = 7 * 3600;
const int  DAYLIGHT_OFFSET = 0;

// ============================================================
//  KALIBRASI NAVIGASI
//  Cara kalibrasi MS_PER_METER:
//    1. Set MS_PER_METER = 1000
//    2. Jalankan motor 1 detik, ukur jarak tempuh (misal 30cm)
//    3. MS_PER_METER baru = 1000 / 0.30 = 3333
//  Cara kalibrasi TURN_90_MS:
//    1. Jalankan putar, ukur sudut aktual
//    2. Sesuaikan proporsional
// ============================================================

const long MS_PER_METER = 3000; // ms per 1 meter (estimasi awal)
const long TURN_90_MS   = 1500; // ms untuk putar 90 derajat di tempat

// Obstacle avoidance — sesuaikan setelah test dengan botol 8x8cm
const long OBS_MUNDUR_MS      = 400;
const long OBS_BELOK_KIRI_MS  = 600;
const long OBS_LURUS_MS       = 800;
const long OBS_BELOK_KANAN_MS = 600;

// ============================================================
//  KALIBRASI SERVO
//  Gunakan test_servo.ino untuk cari nilai yang pas secara fisik
// ============================================================

const int SERVO_DEPAN    = 0;    // posisi istirahat (tangan ke depan)
const int SERVO_BAWAH    = 90;   // posisi sensor (tangan turun ke tanah)
const int SERVO_DELAY_MS = 2000; // waktu tunggu probe di tanah (ms)

// Titik sensor tiap sisi
const float SENSOR_POINT_1 = 2.5;  // meter
const float SENSOR_POINT_2 = 7.5;  // meter
const float SIDE_LENGTH    = 10.0; // meter

// ============================================================
//  PIN — L298N + ESP32
//  Jika pakai L293D: pinout sama, ganti IC saja
//  Catatan: ENA & ENB di L298N harus dilepas jumpernya
//           agar bisa dikontrol PWM dari ESP32
// ============================================================

// Motor kiri  → IN1, IN2, ENA
// Motor kanan → IN3, IN4, ENB
#define MOTOR_IN1  25
#define MOTOR_IN2  26
#define MOTOR_IN3  27
#define MOTOR_IN4  14
#define MOTOR_ENA  32   // PWM speed motor kiri
#define MOTOR_ENB  33   // PWM speed motor kanan

// HC-SR04
#define TRIG_PIN   5
#define ECHO_PIN   18

// Servo
#define SERVO_PIN  13

// Sensor pH tanah (analog, 3 probe)
// Hubungkan: VCC->3.3V, GND->GND, AOUT->GPIO34
#define PH_PIN     34   // GPIO34 = ADC1_CH6, input only

// OLED I2C
#define OLED_SDA   21
#define OLED_SCL   22
#define OLED_ADDR  0x3C
#define SCREEN_W   128
#define SCREEN_H   64

// Kecepatan motor (0-255)
#define MOTOR_SPD  180

// ============================================================
//  OBJEK
// ============================================================

Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &Wire, -1);
Servo armServo;

struct SensorData {
  String waktu;
  String lokasi;
  float  ph;
};

SensorData dataLog[8]; // 4 sisi x 2 titik = 8 data per sesi
int dataCount = 0;

// ============================================================
//  SETUP
// ============================================================

void setup() {
  Serial.begin(115200);

  // Motor
  pinMode(MOTOR_IN1, OUTPUT); pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT); pinMode(MOTOR_IN4, OUTPUT);
  pinMode(MOTOR_ENA, OUTPUT); pinMode(MOTOR_ENB, OUTPUT);
  motorStop();

  // HC-SR04
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Servo — posisi awal tangan menghadap depan
  armServo.attach(SERVO_PIN);
  armServo.write(SERVO_DEPAN);
  delay(500);

  // OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED tidak terdeteksi! Cek wiring SDA/SCL.");
  }
  oledShow("Booting...", "", "", "");

  // WiFi + NTP
  connectWiFi();
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET, "pool.ntp.org");
  oledShow("Sync waktu...", "", "", "");
  delay(2000);

  oledShow("Robot Siap", "Menunggu jadwal", "08:00 / 20:00", "");
  Serial.println("Setup selesai.");
}

// ============================================================
//  LOOP UTAMA
// ============================================================

void loop() {
  if (isScheduleTime()) {
    Serial.println("=== SESI DIMULAI ===");
    oledShow("SESI DIMULAI", getCurrentTime(), "", "");
    delay(2000);

    dataCount = 0;
    jalankanRute();

    oledShow("Kirim data...", "", "", "");
    kirimDataKeSheets();

    oledShow("Sesi selesai", "Data terkirim", "Tunggu sesi", "berikutnya...");
    Serial.println("=== SESI SELESAI ===");

    delay(60000); // tunggu 60 detik agar tidak trigger 2x
  }

  delay(30000); // cek jadwal tiap 30 detik
}

// ============================================================
//  CEK JADWAL (08:00 dan 20:00)
// ============================================================

bool isScheduleTime() {
  struct tm t;
  if (!getLocalTime(&t)) return false;
  return ((t.tm_hour == 8 || t.tm_hour == 20) && t.tm_min == 0);
}

String getCurrentTime() {
  struct tm t;
  if (!getLocalTime(&t)) return "--:--";
  char buf[20];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M", &t);
  return String(buf);
}

// ============================================================
//  RUTE NAVIGASI
//  Pola: searah jarum jam mengelilingi tepi area 10x10m
//  Sisi 1: kiri bawah -> kanan bawah (hadap kanan)
//  Sisi 2: kanan bawah -> kanan atas (hadap atas)
//  Sisi 3: kanan atas -> kiri atas   (hadap kiri)
//  Sisi 4: kiri atas -> kiri bawah   (hadap bawah)
//  Tanaman selalu di sisi kanan robot
// ============================================================

void jalankanRute() {
  for (int sisi = 1; sisi <= 4; sisi++) {

    // Menuju titik sensor pertama (2.5m)
    oledShow("Sisi " + String(sisi) + "/4", "Menuju 2.5m...", "", "");
    majuDenganObstacle(SENSOR_POINT_1);
    lakukanSensor(sisi, 1);

    // Menuju titik sensor kedua (7.5m)
    oledShow("Sisi " + String(sisi) + "/4", "Menuju 7.5m...", "", "");
    majuDenganObstacle(SENSOR_POINT_2 - SENSOR_POINT_1);
    lakukanSensor(sisi, 2);

    // Maju ke ujung sisi
    majuDenganObstacle(SIDE_LENGTH - SENSOR_POINT_2);

    // Putar kiri 90° ke sisi berikutnya (kecuali sisi terakhir)
    if (sisi < 4) {
      oledShow("Sisi " + String(sisi) + "/4", "Putar...", "", "");
      putarKiri90();
    }
  }

  motorStop();
  armServo.write(SERVO_DEPAN);
  Serial.println("Rute selesai.");
}

// ============================================================
//  PROSEDUR SENSOR
//  berhenti -> belok kanan -> servo turun -> baca pH
//  -> servo naik -> belok kiri -> lanjut
// ============================================================

void lakukanSensor(int sisi, int titik) {
  String lokasi = "S" + String(sisi) + "-P" + String(titik);
  motorStop();
  delay(300);

  // 1. Belok kanan menghadap tanaman
  oledShow(lokasi, "Hadap tanaman...", "", "");
  Serial.println(lokasi + " - belok kanan");
  putarKanan90();
  delay(300);

  // 2. Turunkan probe ke tanah
  oledShow(lokasi, "Probe turun...", "", "");
  Serial.println(lokasi + " - servo turun");
  armServo.write(SERVO_BAWAH);
  delay(SERVO_DELAY_MS);

  // 3. Baca pH
  float ph = bacaPH();
  String waktu = getCurrentTime();

  // 4. Simpan
  if (dataCount < 8) {
    dataLog[dataCount++] = {waktu, lokasi, ph};
  }

  // 5. Tampil di OLED
  oledShow(lokasi, "pH: " + String(ph, 2), waktu, "");
  Serial.println(lokasi + " pH=" + String(ph, 2));
  delay(1500);

  // 6. Naikkan probe
  oledShow(lokasi, "Probe naik...", "", "");
  Serial.println(lokasi + " - servo naik");
  armServo.write(SERVO_DEPAN);
  delay(800);

  // 7. Belok kiri kembali ke jalur semula
  oledShow(lokasi, "Kembali jalur...", "", "");
  Serial.println(lokasi + " - belok kiri, lanjut");
  putarKiri90();
  delay(300);
}

// ============================================================
//  BACA SENSOR pH
//  Rata-rata 10 sampel untuk stabilisasi
//  WAJIB DIKALIBRASI dengan larutan buffer pH 4.0 dan 7.0:
//    1. Celup ke buffer pH 7.0, catat voltage → V7
//    2. Celup ke buffer pH 4.0, catat voltage → V4
//    3. slope  = (7.0 - 4.0) / (V7 - V4)
//    4. offset = 7.0 - slope * V7
//    5. Ganti rumus: ph = slope * voltage + offset
// ============================================================

float bacaPH() {
  long total = 0;
  for (int i = 0; i < 10; i++) {
    total += analogRead(PH_PIN);
    delay(10);
  }
  float voltage = (total / 10.0) * (3.3 / 4095.0);

  // Rumus estimasi sebelum kalibrasi — GANTI setelah kalibrasi
  float ph = 7.0 + ((2.5 - voltage) / 0.18);
  return constrain(ph, 0.0, 14.0);
}

// ============================================================
//  MAJU DENGAN OBSTACLE AVOIDANCE
//  Timer dikompensasi selama menghindar agar estimasi
//  posisi di jalur tetap akurat
// ============================================================

void majuDenganObstacle(float meter) {
  long durasi     = (long)(meter * MS_PER_METER);
  long waktuMulai = millis();

  motorMaju();

  while ((millis() - waktuMulai) < durasi) {
    if (adaHalangan()) {
      motorStop();
      long sebelumHindar = millis();
      hindariHalangan();
      // Kompensasi: tambahkan waktu menghindar ke timer
      waktuMulai += (millis() - sebelumHindar);
      motorMaju();
    }
    delay(50);
  }

  motorStop();
}

// ============================================================
//  OBSTACLE AVOIDANCE
//  Selalu hindari ke kiri — tanaman di kanan jangan dirusak
//  Pola: mundur -> belok kiri -> lurus -> belok kanan
// ============================================================

void hindariHalangan() {
  Serial.println("Halangan! Menghindar ke kiri...");

  oledShow("HALANGAN!", "Mundur...", "", "");
  motorMundur();
  delay(OBS_MUNDUR_MS);
  motorStop();
  delay(200);

  oledShow("HALANGAN!", "Belok kiri...", "", "");
  belokKiriHalus();
  delay(OBS_BELOK_KIRI_MS);
  motorStop();
  delay(200);

  oledShow("HALANGAN!", "Lewati...", "", "");
  motorMaju();
  delay(OBS_LURUS_MS);
  motorStop();
  delay(200);

  oledShow("HALANGAN!", "Kembali jalur...", "", "");
  belokKananHalus();
  delay(OBS_BELOK_KANAN_MS);
  motorStop();
  delay(200);

  Serial.println("Halangan terlewati.");
  oledShow("Lanjut...", "", "", "");
}

// ============================================================
//  DETEKSI HALANGAN — HC-SR04 saja
// ============================================================

bool adaHalangan() {
  float jarak = bacaJarak();
  return (jarak > 0 && jarak < 30); // halangan < 30cm
}

float bacaJarak() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 30000);
  if (dur == 0) return -1;
  return dur * 0.034 / 2.0; // cm
}

// ============================================================
//  MOTOR PRIMITIVES
// ============================================================

// Putar kiri 90° di tempat (navigasi antar sisi & kembali sensor)
void putarKiri90() {
  digitalWrite(MOTOR_IN1, LOW);  digitalWrite(MOTOR_IN2, HIGH); // kiri mundur
  digitalWrite(MOTOR_IN3, HIGH); digitalWrite(MOTOR_IN4, LOW);  // kanan maju
  analogWrite(MOTOR_ENA, MOTOR_SPD);
  analogWrite(MOTOR_ENB, MOTOR_SPD);
  delay(TURN_90_MS);
  motorStop();
  delay(300);
}

// Putar kanan 90° di tempat (menghadap tanaman saat sensor)
void putarKanan90() {
  digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW);  // kiri maju
  digitalWrite(MOTOR_IN3, LOW);  digitalWrite(MOTOR_IN4, HIGH); // kanan mundur
  analogWrite(MOTOR_ENA, MOTOR_SPD);
  analogWrite(MOTOR_ENB, MOTOR_SPD);
  delay(TURN_90_MS);
  motorStop();
  delay(300);
}

// Belok kiri halus — untuk obstacle avoidance (satu roda berhenti)
void belokKiriHalus() {
  digitalWrite(MOTOR_IN1, LOW);  digitalWrite(MOTOR_IN2, LOW);  // kiri stop
  digitalWrite(MOTOR_IN3, HIGH); digitalWrite(MOTOR_IN4, LOW);  // kanan maju
  analogWrite(MOTOR_ENA, 0);
  analogWrite(MOTOR_ENB, MOTOR_SPD);
}

// Belok kanan halus — untuk obstacle avoidance (satu roda berhenti)
void belokKananHalus() {
  digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW);  // kiri maju
  digitalWrite(MOTOR_IN3, LOW);  digitalWrite(MOTOR_IN4, LOW);  // kanan stop
  analogWrite(MOTOR_ENA, MOTOR_SPD);
  analogWrite(MOTOR_ENB, 0);
}

void motorMaju() {
  digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, HIGH); digitalWrite(MOTOR_IN4, LOW);
  analogWrite(MOTOR_ENA, MOTOR_SPD);
  analogWrite(MOTOR_ENB, MOTOR_SPD);
}

void motorMundur() {
  digitalWrite(MOTOR_IN1, LOW);  digitalWrite(MOTOR_IN2, HIGH);
  digitalWrite(MOTOR_IN3, LOW);  digitalWrite(MOTOR_IN4, HIGH);
  analogWrite(MOTOR_ENA, MOTOR_SPD);
  analogWrite(MOTOR_ENB, MOTOR_SPD);
}

void motorStop() {
  digitalWrite(MOTOR_IN1, LOW); digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW); digitalWrite(MOTOR_IN4, LOW);
  analogWrite(MOTOR_ENA, 0);    analogWrite(MOTOR_ENB, 0);
}

// ============================================================
//  OLED DISPLAY
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
//  WIFI
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
//  KIRIM DATA KE GOOGLE SHEETS
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
      : "Gagal: "    + dataLog[i].lokasi + " (HTTP " + String(code) + ")");

    http.end();
    delay(500);
  }
}
