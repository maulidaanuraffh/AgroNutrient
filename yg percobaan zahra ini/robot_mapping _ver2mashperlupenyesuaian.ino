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
//  Obstacle : mundur -> belok kiri -> lurus -> belok kanan -> lanjut
//             (hindari ke kiri, tanaman selalu di kanan)
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

// Navigasi — kalibrasi setelah lihat robot bergerak
const long MS_PER_METER = 3000; // ms untuk 1 meter
const long TURN_90_MS   = 1500; // ms untuk putar 90 derajat

// Obstacle avoidance timing
// Asumsi halangan ~8x8cm, robot menghindar ke kiri
const long OBS_MUNDUR_MS     = 400;  // mundur sebentar biar ada ruang belok
const long OBS_BELOK_KIRI_MS = 600;  // belok kiri ~45 derajat
const long OBS_LURUS_MS      = 800;  // lurus melewati halangan
const long OBS_BELOK_KANAN_MS= 600;  // belok kanan kembali ke jalur
// Sesuaikan nilai di atas setelah test fisik

// Titik sensor
const float SENSOR_POINT_1 = 2.5;
const float SENSOR_POINT_2 = 7.5;
const float SIDE_LENGTH    = 10.0;

// Servo
const int SERVO_UP       = 0;
const int SERVO_DOWN     = 90;
const int SERVO_DELAY_MS = 2000;

// ============================================================
// PIN DEFINITIONS
// ============================================================

#define MOTOR_IN1    25
#define MOTOR_IN2    26
#define MOTOR_IN3    27
#define MOTOR_IN4    14
#define MOTOR_ENA    32
#define MOTOR_ENB    33

#define TRIG_PIN     5
#define ECHO_PIN     18
#define TRACKER_PIN  19
#define SERVO_PIN    13
#define PH_PIN       34

#define OLED_SDA     21
#define OLED_SCL     22
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_ADDR    0x3C

#define MOTOR_SPEED  180

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
  armServo.write(SERVO_UP);

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
      oledShow("Putar...", "", "", "");
      putar90();
    }
  }

  motorStop();
  armServo.write(SERVO_UP);
}

// ============================================================
// SENSOR
// ============================================================

void lakukanSensor(int sisi, int titik) {
  String lokasi = "S" + String(sisi) + "-P" + String(titik);
  oledShow("SENSOR", lokasi, "Turunkan lengan", "");

  armServo.write(SERVO_DOWN);
  delay(SERVO_DELAY_MS);

  float phValue = bacaPH();

  armServo.write(SERVO_UP);
  delay(500);

  String waktu = getCurrentTime();
  if (dataCount < 8) {
    dataLog[dataCount] = {waktu, lokasi, phValue};
    dataCount++;
  }

  oledShow("pH: " + String(phValue, 2), lokasi, waktu, "");
  Serial.println("Sensor " + lokasi + " pH=" + String(phValue, 2));
  delay(2000);
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
  float ph = 7.0 + ((2.5 - voltage) / 0.18);
  return constrain(ph, 0.0, 14.0);
}

// ============================================================
// NAVIGASI DENGAN OBSTACLE AVOIDANCE
// ============================================================

void majuDenganObstacle(float meter) {
  long durasi       = (long)(meter * MS_PER_METER);
  long waktuMulai   = millis();
  long waktuSisa;

  motorMaju();

  while (true) {
    waktuSisa = durasi - (millis() - waktuMulai);
    if (waktuSisa <= 0) break;

    if (adaHalangan()) {
      motorStop();

      // Catat waktu yang sudah berjalan sebelum halangan
      long waktuTerpakai = millis() - waktuMulai;

      hindariHalangan();

      // Setelah menghindar, perbarui timer:
      // tambahkan waktu yang dipakai untuk menghindar ke waktu mulai
      // sehingga sisa jarak tetap terjaga
      long waktuHindar = millis() - waktuMulai - waktuTerpakai;
      waktuMulai += waktuHindar;

      motorMaju();
    }

    delay(50);
  }

  motorStop();
}

// ============================================================
// ALGORITMA HINDARI HALANGAN
// Urutan: mundur -> belok kiri -> lurus -> belok kanan -> selesai
// Tanaman selalu di kanan, hindari ke kiri
// ============================================================

void hindariHalangan() {
  oledShow("HALANGAN!", "Mundur...", "", "");
  Serial.println("Halangan! Mundur...");

  // 1. Mundur sedikit untuk beri ruang belok
  motorMundur();
  delay(OBS_MUNDUR_MS);
  motorStop();
  delay(200);

  // 2. Belok kiri (~45 derajat, setengah dari TURN_90_MS)
  oledShow("HALANGAN!", "Belok kiri...", "", "");
  Serial.println("Belok kiri...");
  belokKiri();
  delay(OBS_BELOK_KIRI_MS);
  motorStop();
  delay(200);

  // 3. Lurus melewati halangan
  oledShow("HALANGAN!", "Lewati...", "", "");
  Serial.println("Lurus melewati halangan...");
  motorMaju();
  delay(OBS_LURUS_MS);
  motorStop();
  delay(200);

  // 4. Belok kanan kembali ke jalur semula
  oledShow("HALANGAN!", "Kembali jalur...", "", "");
  Serial.println("Belok kanan kembali...");
  belokKanan();
  delay(OBS_BELOK_KANAN_MS);
  motorStop();
  delay(200);

  Serial.println("Halangan terlewati, lanjut...");
  oledShow("Lanjut...", "", "", "");
}

// ============================================================
// MOTOR PRIMITIVES
// ============================================================

void putar90() {
  // Putar kanan di tempat: kiri maju, kanan mundur
  digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);  digitalWrite(MOTOR_IN4, HIGH);
  analogWrite(MOTOR_ENA, MOTOR_SPEED);
  analogWrite(MOTOR_ENB, MOTOR_SPEED);
  delay(TURN_90_MS);
  motorStop();
  delay(300);
}

void belokKiri() {
  // Kiri pelan / berhenti, kanan maju = belok kiri
  digitalWrite(MOTOR_IN1, LOW);  digitalWrite(MOTOR_IN2, LOW);   // kiri stop
  digitalWrite(MOTOR_IN3, HIGH); digitalWrite(MOTOR_IN4, LOW);   // kanan maju
  analogWrite(MOTOR_ENA, 0);
  analogWrite(MOTOR_ENB, MOTOR_SPEED);
}

void belokKanan() {
  // Kiri maju, kanan pelan / berhenti = belok kanan
  digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW);   // kiri maju
  digitalWrite(MOTOR_IN3, LOW);  digitalWrite(MOTOR_IN4, LOW);   // kanan stop
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
