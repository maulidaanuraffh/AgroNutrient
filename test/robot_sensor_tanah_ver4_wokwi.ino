// ============================================================
//  ROBOT SENSOR TANAH - VERSI WOKWI (SIMULASI)
//
//  Perbedaan dari versi asli:
//  - WiFi & NTP dinonaktifkan
//  - Sensor pH diganti potentiometer di GPIO34
//  - Tombol hijau di GPIO35 = trigger sensor manual
//  - Jadwal otomatis dinonaktifkan
//  - Serial Monitor menampilkan semua state robot
//
//  Cara pakai di Wokwi:
//  1. Jalankan simulasi (tombol Play)
//  2. Robot otomatis mulai rute setelah 3 detik
//  3. Putar potentiometer untuk ubah nilai pH
//  4. Dekatin objek ke HC-SR04 untuk simulasi halangan
//  5. Tekan tombol hijau untuk trigger sensor manual
// ============================================================

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

// ============================================================
//  KALIBRASI — sama seperti versi asli
//  Sesuaikan ini setelah test hardware fisik
// ============================================================

const long MS_PER_METER  = 3000;
const long TURN_90_MS    = 1500;

const long OBS_MUNDUR_MS      = 400;
const long OBS_BELOK_KIRI_MS  = 600;
const long OBS_LURUS_MS       = 800;
const long OBS_BELOK_KANAN_MS = 600;

const int SERVO_DEPAN    = 0;
const int SERVO_BAWAH    = 90;
const int SERVO_DELAY_MS = 1500; // dipercepat untuk simulasi

const float SENSOR_POINT_1 = 2.5;
const float SENSOR_POINT_2 = 7.5;
const float SIDE_LENGTH    = 10.0;

// ============================================================
//  PIN
// ============================================================

#define MOTOR_IN1   25
#define MOTOR_IN2   26
#define MOTOR_IN3   27
#define MOTOR_IN4   14
#define MOTOR_ENA   32
#define MOTOR_ENB   33

#define TRIG_PIN    5
#define ECHO_PIN    18
#define SERVO_PIN   13
#define PH_PIN      34   // potentiometer di Wokwi
#define BTN_PIN     35   // tombol hijau = trigger manual

#define OLED_SDA    21
#define OLED_SCL    22
#define OLED_ADDR   0x3C
#define SCREEN_W    128
#define SCREEN_H    64

#define MOTOR_SPD   180

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

SensorData dataLog[8];
int dataCount = 0;

// ============================================================
//  SETUP
// ============================================================

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_IN1, OUTPUT); pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT); pinMode(MOTOR_IN4, OUTPUT);
  pinMode(MOTOR_ENA, OUTPUT); pinMode(MOTOR_ENB, OUTPUT);
  motorStop();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BTN_PIN, INPUT_PULLDOWN);

  armServo.attach(SERVO_PIN);
  armServo.write(SERVO_DEPAN);
  delay(300);

  Wire.begin(OLED_SDA, OLED_SCL);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED tidak terdeteksi!");
  }

  oledShow("WOKWI MODE", "Robot Sensor", "Tanah", "Mulai 3 detik...");
  Serial.println("=== ROBOT SENSOR TANAH - WOKWI ===");
  Serial.println("Putar potentiometer = ubah pH");
  Serial.println("Dekatkan objek ke HC-SR04 = halangan");
  Serial.println("Tekan tombol hijau = trigger sensor manual");
  Serial.println("Memulai rute dalam 3 detik...");
  delay(3000);
}

// ============================================================
//  LOOP
// ============================================================

void loop() {
  // Tombol manual — trigger satu siklus sensor langsung
  if (digitalRead(BTN_PIN) == HIGH) {
    Serial.println("\n[TOMBOL] Trigger manual sensor!");
    oledShow("MANUAL", "Trigger sensor", "", "");
    delay(300);
    lakukanSensor(0, 0); // sisi 0 titik 0 = test manual
    delay(500);
    return;
  }

  // Jalankan rute penuh
  Serial.println("\n=== MULAI RUTE ===");
  oledShow("MULAI RUTE", "4 sisi x 2 titik", "= 8 sensor", "");
  delay(1000);

  dataCount = 0;
  jalankanRute();

  // Tampil ringkasan di Serial Monitor
  Serial.println("\n=== RINGKASAN DATA ===");
  for (int i = 0; i < dataCount; i++) {
    Serial.println(dataLog[i].lokasi + " | pH: " + String(dataLog[i].ph, 2));
  }

  oledShow("Rute selesai", String(dataCount) + " data", "Loop ulang", "5 detik...");
  Serial.println("Rute selesai. Ulang dalam 5 detik...");
  delay(5000);
}

// ============================================================
//  RUTE NAVIGASI
// ============================================================

void jalankanRute() {
  for (int sisi = 1; sisi <= 4; sisi++) {
    oledShow("Sisi " + String(sisi) + "/4", "Menuju 2.5m...", "", "");
    Serial.println("\n-- Sisi " + String(sisi) + " --");
    majuDenganObstacle(SENSOR_POINT_1);
    lakukanSensor(sisi, 1);

    oledShow("Sisi " + String(sisi) + "/4", "Menuju 7.5m...", "", "");
    majuDenganObstacle(SENSOR_POINT_2 - SENSOR_POINT_1);
    lakukanSensor(sisi, 2);

    majuDenganObstacle(SIDE_LENGTH - SENSOR_POINT_2);

    if (sisi < 4) {
      oledShow("Sisi " + String(sisi) + "/4", "Putar kiri...", "", "");
      Serial.println("Putar kiri 90°");
      putarKiri90();
    }
  }

  motorStop();
  armServo.write(SERVO_DEPAN);
  Serial.println("Rute selesai.");
}

// ============================================================
//  PROSEDUR SENSOR
// ============================================================

void lakukanSensor(int sisi, int titik) {
  String lokasi = (sisi == 0) ? "MANUAL" : "S" + String(sisi) + "-P" + String(titik);
  motorStop();
  delay(200);

  Serial.println("[SENSOR] " + lokasi + " - belok kanan");
  oledShow(lokasi, "Hadap tanaman...", "", "");
  putarKanan90();
  delay(200);

  Serial.println("[SENSOR] servo turun");
  oledShow(lokasi, "Probe turun...", "", "");
  armServo.write(SERVO_BAWAH);
  delay(SERVO_DELAY_MS);

  float ph = bacaPH();
  String waktu = "SIM " + String(millis() / 1000) + "s";

  if (dataCount < 8 && sisi > 0) {
    dataLog[dataCount++] = {waktu, lokasi, ph};
  }

  Serial.println("[SENSOR] " + lokasi + " pH=" + String(ph, 2));
  oledShow(lokasi, "pH: " + String(ph, 2), waktu, "OK");
  delay(1500);

  Serial.println("[SENSOR] servo naik");
  oledShow(lokasi, "Probe naik...", "", "");
  armServo.write(SERVO_DEPAN);
  delay(600);

  Serial.println("[SENSOR] belok kiri, lanjut");
  oledShow(lokasi, "Kembali jalur...", "", "");
  putarKiri90();
  delay(200);
}

// ============================================================
//  BACA pH (potentiometer di Wokwi)
//  Putar ke kiri = pH rendah (asam)
//  Putar ke kanan = pH tinggi (basa)
//  Tengah = sekitar pH 7 (netral)
// ============================================================

float bacaPH() {
  long total = 0;
  for (int i = 0; i < 10; i++) {
    total += analogRead(PH_PIN);
    delay(5);
  }
  float adcVal  = total / 10.0;
  float voltage = adcVal * (3.3 / 4095.0);
  float ph = 7.0 + ((2.5 - voltage) / 0.18);
  return constrain(ph, 0.0, 14.0);
}

// ============================================================
//  NAVIGASI DENGAN OBSTACLE AVOIDANCE
// ============================================================

void majuDenganObstacle(float meter) {
  long durasi     = (long)(meter * MS_PER_METER);
  long waktuMulai = millis();

  Serial.println("Maju " + String(meter) + "m (" + String(durasi) + "ms)");
  motorMaju();

  while ((millis() - waktuMulai) < durasi) {
    if (digitalRead(BTN_PIN) == HIGH) {
      // Tombol ditekan saat jalan = pause dan sensor manual
      motorStop();
      Serial.println("[BTN] Pause - sensor manual");
      lakukanSensor(0, 0);
      motorMaju();
    }

    if (adaHalangan()) {
      motorStop();
      long sebelumHindar = millis();
      hindariHalangan();
      waktuMulai += (millis() - sebelumHindar);
      motorMaju();
    }
    delay(50);
  }

  motorStop();
}

// ============================================================
//  OBSTACLE AVOIDANCE
// ============================================================

void hindariHalangan() {
  Serial.println("[HALANGAN] Terdeteksi! Menghindar ke kiri...");
  oledShow("HALANGAN!", "Mundur...", "", "");
  motorMundur();
  delay(OBS_MUNDUR_MS);
  motorStop(); delay(150);

  oledShow("HALANGAN!", "Belok kiri...", "", "");
  belokKiriHalus();
  delay(OBS_BELOK_KIRI_MS);
  motorStop(); delay(150);

  oledShow("HALANGAN!", "Lewati...", "", "");
  motorMaju();
  delay(OBS_LURUS_MS);
  motorStop(); delay(150);

  oledShow("HALANGAN!", "Kembali jalur...", "", "");
  belokKananHalus();
  delay(OBS_BELOK_KANAN_MS);
  motorStop(); delay(150);

  Serial.println("[HALANGAN] Terlewati.");
  oledShow("Lanjut...", "", "", "");
}

// ============================================================
//  DETEKSI HALANGAN
// ============================================================

bool adaHalangan() {
  float jarak = bacaJarak();
  if (jarak > 0 && jarak < 30) {
    Serial.println("[HC-SR04] Jarak: " + String(jarak) + "cm — HALANGAN!");
    return true;
  }
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
//  MOTOR PRIMITIVES
// ============================================================

void putarKiri90() {
  digitalWrite(MOTOR_IN1, LOW);  digitalWrite(MOTOR_IN2, HIGH);
  digitalWrite(MOTOR_IN3, HIGH); digitalWrite(MOTOR_IN4, LOW);
  analogWrite(MOTOR_ENA, MOTOR_SPD);
  analogWrite(MOTOR_ENB, MOTOR_SPD);
  delay(TURN_90_MS);
  motorStop(); delay(200);
}

void putarKanan90() {
  digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);  digitalWrite(MOTOR_IN4, HIGH);
  analogWrite(MOTOR_ENA, MOTOR_SPD);
  analogWrite(MOTOR_ENB, MOTOR_SPD);
  delay(TURN_90_MS);
  motorStop(); delay(200);
}

void belokKiriHalus() {
  digitalWrite(MOTOR_IN1, LOW);  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, HIGH); digitalWrite(MOTOR_IN4, LOW);
  analogWrite(MOTOR_ENA, 0);
  analogWrite(MOTOR_ENB, MOTOR_SPD);
}

void belokKananHalus() {
  digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);  digitalWrite(MOTOR_IN4, LOW);
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
//  OLED
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
