#include <ESP32Servo.h>

// Deklarasi objek servo
Servo servoTest;

// Tentukan pin PWM ESP32 yang akan digunakan (misal Pin 19)
const int pinServo = 19; 

void setup() {
  Serial.begin(115200);
  Serial.println("--- Menguji Motor Servo ---");

  // Alokasikan semua timer PWM ESP32 untuk servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Set frekuensi servo standar (50Hz)
  servoTest.setPeriodHertz(50);    

  // Hubungkan servo ke pin dengan pulse width standar (544us - 2400us)
  servoTest.attach(pinServo, 544, 2400); 
  
  // Kembalikan ke posisi awal (0 derajat)
  servoTest.write(0);
  delay(1000);
}

void loop() {
  Serial.println("Gerak menuju 120 derajat...");
  for (int posisi = 0; posisi <= 120; posisi += 1) {
    servoTest.write(posisi);
    delay(15); // Mengatur kecepatan gerakan servo
  }

  delay(1000); // Diam sebentar di posisi 180 derajat

  Serial.println("Gerak kembali ke 0 derajat...");
  for (int posisi = 120; posisi >= 0; posisi -= 1) {
    servoTest.write(posisi);
    delay(15);
  }

  delay(1000); // Diam sebentar di posisi 0 derajat
}