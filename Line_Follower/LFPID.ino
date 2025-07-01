const int pinSensor[] = {13, 12, 14, 27, 26};
int sensor[5];
const int weight[] = {2, 1, 0, -1, -2};  // Kanan positif, kiri negatif
const int jumlahSensor = sizeof(pinSensor) / sizeof(pinSensor[0]);

// PID Variables
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
float output = 0;
float posisi = 0;
float lastPosisi = 0;

// Tuning PID
float Kp = 15;
float Ki = 0;
float Kd = 12;

const int freq = 1000;
const int resolusi = 8;

// Motor pin
const int motorKiriPWM = 5;
const int motorKananPWM = 18;
const int motorKiriIN1 = 2;
const int motorKiriIN2 = 4;
const int motorKananIN1 = 19;
const int motorKananIN2 = 21;

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < jumlahSensor; i++) {
    pinMode(pinSensor[i], INPUT);
  }

  pinMode(motorKiriIN1, OUTPUT);
  pinMode(motorKiriIN2, OUTPUT);
  pinMode(motorKananIN1, OUTPUT);
  pinMode(motorKananIN2, OUTPUT);

  ledcSetup(0, freq, resolusi);
  ledcAttachPin(motorKiriPWM, 0);

  ledcSetup(1, freq, resolusi);
  ledcAttachPin(motorKananPWM, 1);
}

void loop() {
  int sensorAktif = 0;
  int weightSensor = 0;

  for (int i = 0; i < jumlahSensor; i++) {
    sensor[i] = digitalRead(pinSensor[i]);
    if (sensor[i] == 0) {
      weightSensor += weight[i];
      sensorAktif++;
    }
  }

  // Update posisi
  if (sensorAktif > 0) {
    posisi = (float)weightSensor / sensorAktif;
    lastPosisi = posisi;
  } else {
    posisi = lastPosisi;
    integral = 0; // reset anti windup saat off track
  }

  // Posisi smoothing (optional)
  float posisiFiltered = (posisi + lastPosisi) / 2.0;

  // PID
  error = 0 - posisiFiltered;
  integral += error;
  integral = constrain(integral, -100, 100);
  derivative = error - lastError;
  
  // Kp Dynamic
  float factor_kp = 5;
  float Kp_Dyn = 13 + abs(error) * factor_kp;

  // Kd Dynamic
  float factor_kd = 5;
  float Kd_Dyn = 12 + abs(error) * factor_kd;

  // Logic Kp & Kd Dynamic 
  if (abs(error) > 1.5) {
    output = Kp_Dyn * error + Ki * integral + Kd_Dyn * derivative;
  } else {
    output = Kp * error + Ki * integral + Kd * derivative;
  }

  output = constrain(output, -60, 60);

  lastError = error;

  // Base speed dynamic
  int baseSpeed = 110 - abs(error) * 25;
  baseSpeed = constrain(baseSpeed, 60, 100);

  int kecepatanKiri = baseSpeed + output;
  int kecepatanKanan = baseSpeed - output;

  // Biar motor tetap muter
  kecepatanKiri = constrain(kecepatanKiri, 40, 255);
  kecepatanKanan = constrain(kecepatanKanan, 40, 255);

  ledcWrite(0, kecepatanKiri);
  ledcWrite(1, kecepatanKanan);
  allMotorsON();

  // Debugging
  Serial.print("Posisi: "); Serial.print(posisi);
  Serial.print(" | Filtered: "); Serial.print(posisiFiltered);
  Serial.print(" | Error: "); Serial.print(error);
  Serial.print(" | Output: "); Serial.print(output);
  Serial.print(" | Kiri: "); Serial.print(kecepatanKiri);
  Serial.print(" | Kanan: "); Serial.println(kecepatanKanan);

  delay(10);
}

void allMotorsON() {
  digitalWrite(motorKiriIN1, HIGH);
  digitalWrite(motorKiriIN2, LOW);
  digitalWrite(motorKananIN1, HIGH);
  digitalWrite(motorKananIN2, LOW);
}
