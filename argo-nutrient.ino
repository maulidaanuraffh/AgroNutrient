/*
  ARGO NUTRIENT MONITOR ROBOT - FULL VERSION
  ---------------------------------------------------------------
  Sesuai flowchart:
  1. Inisialisasi Sistem (Motor, Sensor, Servo, WiFi)
  2. Baca Data Sensor Mapping Garis/Jalur (tracker sensor)
  3. Jika jalur TIDAK terdeteksi -> Cari Jalur (putar/scan)
  4. Jika jalur terdeteksi -> Gerakkan robot sesuai jalur
  5. Cek apakah sudah waktunya sampling tanah (estimasi jarak via waktu)
     - Jika belum -> lanjut baca jalur lagi
     - Jika sudah:
         a. Berhenti sejenak
         b. Servo turun & tancapkan sensor ke tanah
         c. Baca sensor pH & kelembapan tanah
         d. Kirim data ke website via WiFi
         e. Servo naik, kembali ke posisi awal
         f. Lanjutkan gerak robot

  CATATAN PENTING:
  - Trigger sampling pakai ESTIMASI WAKTU/JARAK (bukan sensor khusus),
    karena belum ada encoder roda. WAJIB dikalibrasi (lihat variabel
    WAKTU_PER_METER_MS di bawah).
  - WiFi SSID, password, dan URL server WAJIB diisi sebelum upload.
  - Sensor pH tanah di sini masih baca nilai RAW/voltage (belum
    dikonversi ke skala pH asli). Kalibrasi 2-titik (pH4 & pH7)
    perlu dilakukan terpisah dan rumus konversi ditambahkan di
    fungsi bacaPH().
  - "Kelembapan tanah" di flowchart: jika sensor pH tanah kamu HANYA
    baca pH (bukan sensor 2-in-1 pH+moisture), maka bagian kelembapan
    untuk saat ini di-skip dan diberi nilai placeholder. Konfirmasi
    ke tim elektronika apakah sensor yang dipakai 2-in-1 atau bukan.
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <HTTPClient.h>

// ===================== KONFIGURASI WIFI & SERVER =====================
const char* WIFI_SSID     = "NAMA_WIFI_KAMU";       // GANTI
const char* WIFI_PASSWORD = "PASSWORD_WIFI_KAMU";   // GANTI
const char* SERVER_URL    = "http://example.com/api/data"; // GANTI sesuai endpoint website kamu

// ===================== KONFIGURASI OLED =====================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ===================== DEKLARASI PIN =====================
// Sensor Ultrasonik HC-SR04 (cadangan, bisa dipakai untuk deteksi halangan tambahan)
const int trigPin = 5;
const int echoPin = 18;

// Tracker Sensor (IR line tracker) - 2 sensor: kiri & kanan
// HIGH = kena garis, LOW = tidak kena garis (SESUAIKAN dengan modul kamu,
// kalau logikanya kebalik, tinggal tukar pengecekan HIGH/LOW di bawah)
const int trackerKiriPin  = 32;
const int trackerKananPin = 33;

// Motor Driver (L298N) - SESUAIKAN dengan pinout dari tim elektronika
const int enA = 25; // Kecepatan Motor Kanan
const int in1 = 26; // Arah Motor Kanan
const int in2 = 27; // Arah Motor Kanan
const int in3 = 14; // Arah Motor Kiri
const int in4 = 12; // Arah Motor Kiri
const int enB = 13; // Kecepatan Motor Kiri

// Servo MG996R (lengan probe sensor pH)
const int servoPin = 19;
Servo servoPH;
const int SERVO_POSISI_ATAS  = 0;   // posisi servo saat terangkat
const int SERVO_POSISI_TANAH = 90;  // posisi servo saat menancap ke tanah (sesuaikan)

// Sensor pH Tanah (analog)
const int phPin = 34; // pakai pin ADC ESP32 (32-39)

// ===================== PARAMETER LOGIC =====================
const int SPEED_MOTOR       = 200;  // PWM 0-255, kecepatan saat jalan lurus
const int SPEED_PUTAR_CARI  = 150;  // PWM saat putar mencari jalur

// !!! WAJIB DIKALIBRASI - lihat catatan di file sebelumnya !!!
const unsigned long WAKTU_PER_METER_MS = 2500; // GANTI sesuai hasil test
const float JARAK_TARGET_METER = 5.0;
const unsigned long WAKTU_TARGET_MS = (unsigned long)(WAKTU_PER_METER_MS * JARAK_TARGET_METER);

// ===================== VARIABEL GLOBAL =====================
unsigned long waktuMulaiJalan = 0;

void setup() {
  Serial.begin(115200);

  // --- Setup Sensor Ultrasonik ---
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // --- Setup Tracker Sensor ---
  pinMode(trackerKiriPin, INPUT);
  pinMode(trackerKananPin, INPUT);

  // --- Setup Motor ---
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // --- Setup Servo ---
  servoPH.attach(servoPin);
  servoPH.write(SERVO_POSISI_ATAS);

  // --- Setup OLED ---
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED gagal terdeteksi!");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Argo Nutrient Robot");
    display.println("Inisialisasi...");
    display.display();
  }

  // --- Setup WiFi ---
  hubungkanWiFi();

  delay(1000);
  waktuMulaiJalan = millis();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Sistem Siap!");
  display.println("Mulai jalan...");
  display.display();
  delay(1000);
}

void loop() {
  // ----- CEK APAKAH SUDAH WAKTUNYA SAMPLING -----
  if (millis() - waktuMulaiJalan >= WAKTU_TARGET_MS) {
    lakukanSampling();
    waktuMulaiJalan = millis(); // reset patokan, hitung 5 meter lagi
    return;
  }

  // ----- BACA TRACKER SENSOR (DETEKSI JALUR) -----
  bool kiriKenaGaris  = bacaTrackerKiri();
  bool kananKenaGaris = bacaTrackerKanan();

  if (!kiriKenaGaris && !kananKenaGaris) {
    // Jalur TIDAK terdeteksi -> Cari Jalur (putar/scan)
    cariJalur();
  } else {
    // Jalur terdeteksi -> Gerakkan robot sesuai jalur
    gerakSesuaiJalur(kiriKenaGaris, kananKenaGaris);
  }
}

// ===================== FUNGSI SAMPLING PH =====================
void lakukanSampling() {
  Serial.println("== Estimasi jarak tercapai, mulai sampling ==");

  berhenti();
  delay(300);

  // Servo turun & tancapkan sensor
  servoPH.write(SERVO_POSISI_TANAH);
  delay(800);

  // Baca sensor pH & kelembapan
  float nilaiPH = bacaPH();
  float kelembapan = bacaKelembapan(); // placeholder, lihat catatan di fungsinya

  Serial.print("pH: ");
  Serial.print(nilaiPH);
  Serial.print(" | Kelembapan: ");
  Serial.println(kelembapan);

  // Tampilkan ke OLED
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Hasil Sampling:");
  display.print("pH   : ");
  display.println(nilaiPH);
  display.print("Lembap: ");
  display.println(kelembapan);
  display.display();

  // Kirim data ke website via WiFi
  kirimDataKeServer(nilaiPH, kelembapan);

  delay(1500); // waktu tampil hasil sebelum lanjut

  // Servo naik kembali ke posisi awal
  servoPH.write(SERVO_POSISI_ATAS);
  delay(800);

  Serial.println("== Sampling selesai, lanjut jalan ==");
}

// ===================== FUNGSI WIFI =====================
void hubungkanWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Menghubungkan ke WiFi");

  int percobaan = 0;
  while (WiFi.status() != WL_CONNECTED && percobaan < 20) {
    delay(500);
    Serial.print(".");
    percobaan++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi terhubung!");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nGagal terhubung WiFi, robot tetap jalan tanpa kirim data.");
  }
}

void kirimDataKeServer(float ph, float kelembapan) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi tidak terhubung, data tidak terkirim.");
    return;
  }

  HTTPClient http;
  http.begin(SERVER_URL);
  http.addHeader("Content-Type", "application/json");

  String payload = "{\"ph\":" + String(ph) +
                    ",\"kelembapan\":" + String(kelembapan) + "}";

  int responseCode = http.POST(payload);

  if (responseCode > 0) {
    Serial.print("Data terkirim, response code: ");
    Serial.println(responseCode);
  } else {
    Serial.print("Gagal kirim data, error: ");
    Serial.println(http.errorToString(responseCode));
  }

  http.end();
}

// ===================== FUNGSI SENSOR =====================
bool bacaTrackerKiri() {
  return digitalRead(trackerKiriPin) == HIGH;
}

bool bacaTrackerKanan() {
  return digitalRead(trackerKananPin) == HIGH;
}

float bacaPH() {
  int nilaiRaw = analogRead(phPin);
  float voltage = nilaiRaw * (3.3 / 4095.0);

  // PLACEHOLDER: belum dikalibrasi ke skala pH asli (0-14).
  // Setelah kalibrasi 2-titik (pH4 & pH7), ganti baris di bawah
  // dengan rumus konversi yang sesuai, contoh:
  // float ph = (voltage - offset) / slope;
  float ph = voltage; // sementara masih voltage mentah

  return ph;
}

float bacaKelembapan() {
  // PLACEHOLDER: isi fungsi ini kalau sensor pH tanah kamu
  // ternyata 2-in-1 (pH + moisture). Kalau cuma sensor pH biasa,
  // bagian ini bisa dihapus dari sistem, atau diganti sensor
  // kelembapan tanah terpisah nantinya.
  return 0.0;
}

int ukurJarakUltrasonik() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long durasi = pulseIn(echoPin, HIGH);
  int jarakCm = durasi * 0.034 / 2;

  if (jarakCm == 0) {
    jarakCm = 100;
  }
  return jarakCm;
}

// ===================== FUNGSI LOGIC JALUR =====================
void gerakSesuaiJalur(bool kiri, bool kanan) {
  if (kiri && kanan) {
    // kedua sensor kena garis -> lurus
    maju();
  } else if (kiri && !kanan) {
    // hanya kiri kena garis -> garis ada di kiri, belok kiri sedikit
    belokKiri();
  } else if (!kiri && kanan) {
    // hanya kanan kena garis -> garis ada di kanan, belok kanan sedikit
    belokKanan();
  }
}

void cariJalur() {
  Serial.println("Jalur hilang, mencari...");
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Mencari jalur...");
  display.display();

  // Putar di tempat (searah jarum jam) sambil scan tracker
  analogWrite(enA, SPEED_PUTAR_CARI);
  analogWrite(enB, SPEED_PUTAR_CARI);
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  // motor kanan maju
  digitalWrite(in3, LOW);  digitalWrite(in4, HIGH); // motor kiri mundur

  delay(150); // putar sedikit, lalu cek lagi di loop berikutnya (tidak full blocking lama)
  berhenti();
}

// ===================== FUNGSI GERAKAN MOTOR =====================
void maju() {
  analogWrite(enA, SPEED_MOTOR); analogWrite(enB, SPEED_MOTOR);
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
}

void mundur() {
  analogWrite(enA, SPEED_MOTOR); analogWrite(enB, SPEED_MOTOR);
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
}

void belokKanan() {
  analogWrite(enA, SPEED_MOTOR); analogWrite(enB, SPEED_MOTOR);
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW); // motor kanan maju
  digitalWrite(in3, LOW);  digitalWrite(in4, HIGH); // motor kiri mundur
}

void belokKiri() {
  analogWrite(enA, SPEED_MOTOR); analogWrite(enB, SPEED_MOTOR);
  digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); // motor kanan mundur
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);  // motor kiri maju
}

void berhenti() {
  analogWrite(enA, 0); analogWrite(enB, 0);
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
}
