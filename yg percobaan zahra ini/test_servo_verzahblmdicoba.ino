// ============================================================
//  TEST SERVO MG996R (LENGAN)
//  Buka Serial Monitor 115200 baud
//  Ketik perintah:
//    u = naik (posisi UP / istirahat)
//    d = turun (posisi DOWN / sensor)
//    t = test otomatis: naik -> turun -> tahan 2 detik -> naik
//    angka 0-180 = gerak ke derajat tertentu langsung
// ============================================================

#include <ESP32Servo.h>

#define SERVO_PIN  13

Servo armServo;

int POS_UP   = 0;   // derajat posisi istirahat — ubah kalau perlu
int POS_DOWN = 90;  // derajat posisi turun ke tanah — ubah kalau perlu

void setup() {
  Serial.begin(115200);
  armServo.attach(SERVO_PIN);
  armServo.write(POS_UP);
  delay(500);
  Serial.println("=== TEST SERVO ===");
  printHelp();
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "u") {
      armServo.write(POS_UP);
      Serial.println("NAIK -> " + String(POS_UP) + " derajat");

    } else if (input == "d") {
      armServo.write(POS_DOWN);
      Serial.println("TURUN -> " + String(POS_DOWN) + " derajat");

    } else if (input == "t") {
      Serial.println("TEST OTOMATIS...");
      Serial.println("Naik -> " + String(POS_UP) + " deg");
      armServo.write(POS_UP);
      delay(1000);

      Serial.println("Turun -> " + String(POS_DOWN) + " deg");
      armServo.write(POS_DOWN);
      delay(2000); // simulasi waktu sensor di tanah

      Serial.println("Naik lagi -> " + String(POS_UP) + " deg");
      armServo.write(POS_UP);
      delay(500);
      Serial.println("Test selesai.");

    } else if (input == "?") {
      printHelp();

    } else {
      // Coba parse sebagai angka
      int derajat = input.toInt();
      if (derajat >= 0 && derajat <= 180) {
        armServo.write(derajat);
        Serial.println("Gerak ke " + String(derajat) + " derajat");
      } else {
        Serial.println("Perintah tidak dikenal. Ketik ? untuk bantuan.");
      }
    }
  }
}

void printHelp() {
  Serial.println("-------------------");
  Serial.println("u       = naik (UP)");
  Serial.println("d       = turun (DOWN / sensor)");
  Serial.println("t       = test otomatis");
  Serial.println("0-180   = gerak ke derajat tertentu");
  Serial.println("?       = tampilkan ini");
  Serial.println("");
  Serial.println("POS_UP   = " + String(POS_UP));
  Serial.println("POS_DOWN = " + String(POS_DOWN));
  Serial.println("Ubah POS_UP / POS_DOWN di kode");
  Serial.println("sesuai posisi fisik lengan kamu.");
  Serial.println("-------------------");
}
