#define PWMA 19
#define PWMB 18
#define LOWA 21
#define LOWB 5

const int pinSensor[] = {34, 35, 32, 33, 25, 26, 27, 14, 12, 13};
const int jumlahSensor = sizeof(pinSensor) / sizeof(pinSensor[0]);

const int weightSensor[] = {-5, -4, -3, -2, -1, 1, 2, 3, 4, 5};

int valueSensor[10];

const int freq = 2000;     
const int resolusi = 8;    

int kp = 28;
int kd = 24;
static int lastError = 0; // nyimpen error sebelumnya

void setup() {
  Serial.begin(115200); // buat debug

  for (int i = 0; i < jumlahSensor; i++) {
    pinMode(pinSensor[i], INPUT);
  }

  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(LOWA, OUTPUT);
  pinMode(LOWB, OUTPUT);

  ledcSetup(0, freq, resolusi);
  ledcAttachPin(PWMA, 0);

  ledcSetup(1, freq, resolusi);
  ledcAttachPin(PWMB, 1);

  digitalWrite(LOWA, LOW);
  digitalWrite(LOWB, LOW);
}

void loop() {
  int sensorAktif = 0;
  int totalWeight = 0;

  // Baca semua sensor
  for (int i = 0; i < jumlahSensor; i++) {
    valueSensor[i] = digitalRead(pinSensor[i]);
    if (valueSensor[i] == 0) { // deteksi garis hitam
      totalWeight += weightSensor[i];
      sensorAktif++;
    }
    Serial.print(valueSensor[i]);
    Serial.print(" ");
  }
  Serial.println();

  int error = 0;
  if (sensorAktif > 0) {
    error = totalWeight / sensorAktif;
  }

  int derivative = error - lastError;
  lastError = error;

  int output = kp * error + kd * derivative;

  PWMMotor(output);
}

void PWMMotor(int output) {
  int baseSpeed = 110;
  int kanan = baseSpeed + output;
  int kiri = baseSpeed - output;
  kanan = constrain(kanan, 0, 255);
  kiri = constrain(kiri, 0, 255);

  ledcWrite(0, kanan);
  ledcWrite(1, kiri);
}
