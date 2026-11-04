// =============================================================================
// stm32_imu.ino — IMU BNO085 em um STM32F401 ("Black Pill"), core STM32duino
// =============================================================================
// Lê a BNO085 por I2C e transmite pela USB CDC (aparece como /dev/ttyACM*):
//
//   I <yaw> <pitch> <roll> <gx> <gy> <gz> <ax> <ay> <az>\n     (50 Hz)
//
//   yaw/pitch/roll : rad   — orientação do Game Rotation Vector (fusão
//                            acelerômetro + giroscópio, sem magnetômetro:
//                            imune a interferência magnética em ambiente
//                            interno; o yaw é relativo à posição no boot)
//   gx/gy/gz       : rad/s — giroscópio calibrado
//   ax/ay/az       : m/s²  — acelerômetro COM gravidade (convenção REP-145)
//   # <texto>               — log informativo
//
// Os eixos seguem o chip; monte a placa com X para a frente e Z para cima
// (ou ajuste o imu_joint no URDF).
//
// Ligações: BNO085 em I2C1 — SCL=PB6, SDA=PB7, endereço 0x4A. LED em PC13.
// Dependência: Adafruit BNO08x (Arduino Library Manager).
// =============================================================================

#include <Wire.h>
#include <Adafruit_BNO08x.h>

#define OUT              SerialUSB
#define LED              PC13          // ativo em nível baixo
#define BNO_ADDR         0x4A
#define REPORT_PERIOD_MS 20            // 50 Hz
#define SENSOR_PERIOD_US 10000         // sensores internos a 100 Hz
#define DATA_TIMEOUT_MS  500           // sem dados -> reinicializa a BNO085

TwoWire i2c(PB7, PB6);                 // (SDA, SCL)
Adafruit_BNO08x bno;
sh2_SensorValue_t value;

struct {
  float yaw, pitch, roll;
  float gx, gy, gz;
  float ax, ay, az;
} imu = {};

uint32_t last_data_ms = 0;

// ── Configuração dos relatórios da BNO085 ────────────────────────────────────
bool enableReports() {
  return bno.enableReport(SH2_GAME_ROTATION_VECTOR, SENSOR_PERIOD_US) &&
         bno.enableReport(SH2_GYROSCOPE_CALIBRATED, SENSOR_PERIOD_US) &&
         bno.enableReport(SH2_ACCELEROMETER,        SENSOR_PERIOD_US);
}

bool startBno() {
  if (!bno.begin_I2C(BNO_ADDR, &i2c)) return false;
  return enableReports();
}

// ── Quaternion -> ângulos de Euler (Z-Y-X) ───────────────────────────────────
void quaternionToEuler(float qr, float qi, float qj, float qk) {
  imu.roll  = atan2f(2.0f * (qr * qi + qj * qk), 1.0f - 2.0f * (qi * qi + qj * qj));
  float s   = 2.0f * (qr * qj - qk * qi);
  imu.pitch = fabsf(s) >= 1.0f ? copysignf(M_PI / 2.0f, s) : asinf(s);
  imu.yaw   = atan2f(2.0f * (qr * qk + qi * qj), 1.0f - 2.0f * (qj * qj + qk * qk));
}

void readBno() {
  if (bno.wasReset()) {
    OUT.println("# BNO085 reiniciou, reativando relatorios");
    enableReports();
  }
  while (bno.getSensorEvent(&value)) {
    last_data_ms = millis();
    switch (value.sensorId) {
      case SH2_GAME_ROTATION_VECTOR:
        quaternionToEuler(value.un.gameRotationVector.real,
                          value.un.gameRotationVector.i,
                          value.un.gameRotationVector.j,
                          value.un.gameRotationVector.k);
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        imu.gx = value.un.gyroscope.x;
        imu.gy = value.un.gyroscope.y;
        imu.gz = value.un.gyroscope.z;
        break;
      case SH2_ACCELEROMETER:
        imu.ax = value.un.accelerometer.x;
        imu.ay = value.un.accelerometer.y;
        imu.az = value.un.accelerometer.z;
        break;
    }
  }
}

// print(float, casas) em vez de printf("%f"): o newlib-nano do STM32duino
// não formata float no printf sem uma opção extra de compilação.
void printFrame() {
  const float fields[9] = { imu.yaw, imu.pitch, imu.roll,
                            imu.gx, imu.gy, imu.gz,
                            imu.ax, imu.ay, imu.az };
  OUT.print('I');
  for (uint8_t i = 0; i < 9; i++) {
    OUT.print(' ');
    OUT.print(fields[i], 4);
  }
  OUT.println();
}

void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  OUT.begin(115200);
  i2c.begin();
  i2c.setClock(400000);
  delay(500);

  while (!startBno()) {
    OUT.println("# BNO085 nao encontrada (SCL=PB6 SDA=PB7 0x4A), tentando novamente");
    digitalWrite(LED, !digitalRead(LED));
    delay(500);
  }
  last_data_ms = millis();
  digitalWrite(LED, LOW);
  OUT.println("# BNO085 pronta");
}

void loop() {
  static uint32_t last_report = 0;
  readBno();

  uint32_t now = millis();
  if (now - last_data_ms > DATA_TIMEOUT_MS) {
    OUT.println("# BNO085 sem dados, reinicializando");
    startBno();
    last_data_ms = now;
  }
  if (now - last_report >= REPORT_PERIOD_MS) {
    last_report = now;
    printFrame();
  }
}
