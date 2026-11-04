// =============================================================================
// esp32_motor_controller.ino — controle de 2 motores BLDC (hoverboard)
// ESP32 + SimpleFOC, controle de velocidade em malha fechada com sensores Hall
// =============================================================================
// Protocolo com o ROS 2 (hardware/hardware_node), ASCII, 115200 bps:
//
//   host -> ESP32 :  C <vel_esq> <vel_dir>\n          (rad/s)
//   ESP32 -> host :  M <vel_esq> <vel_dir> <pos_esq> <pos_dir> R\n   (50 Hz)
//                    # <texto>\n                       (log)
//
// Canal com o host: Serial2 (RX=GPIO16, TX=GPIO17) através de um conversor
// USB-serial. A USB nativa (Serial) fica livre para depuração.
// Para usar a USB como canal principal, defina HOST_ON_USB como 1.
//
// Segurança: sem comando por WATCHDOG_MS ms, a velocidade-alvo volta a zero.
//
// Dependência: biblioteca Simple FOC (Arduino Library Manager).
// =============================================================================

#include <SimpleFOC.h>

#define HOST_ON_USB 0
#if HOST_ON_USB
  #define HOST Serial
#else
  #define HOST Serial2
  #define HOST_RX_PIN 16
  #define HOST_TX_PIN 17
#endif

#define BAUDRATE         115200
#define REPORT_PERIOD_MS 20      // telemetria a 50 Hz
#define WATCHDOG_MS      500
#define LED_STATUS       2

// ── Parâmetros do controle ───────────────────────────────────────────────────
#define POLE_PAIRS        15
#define SUPPLY_VOLTAGE    24.0f
#define VOLTAGE_LIMIT     24.0f
#define ALIGN_VOLTAGE     6.0f
#define PID_P             0.9f
#define PID_I             2.1f
#define PID_D             0.0f
#define PID_RAMP          5000.0f
#define VELOCITY_LPF_TF   0.01f

// ── Motor 1 (direito) ────────────────────────────────────────────────────────
BLDCMotor motorR(POLE_PAIRS);
BLDCDriver3PWM driverR(27, 26, 25);
HallSensor hallR(13, 14, 15, POLE_PAIRS);
void hallR_A() { hallR.handleA(); }
void hallR_B() { hallR.handleB(); }
void hallR_C() { hallR.handleC(); }

// ── Motor 2 (esquerdo) ───────────────────────────────────────────────────────
BLDCMotor motorL(POLE_PAIRS);
BLDCDriver3PWM driverL(23, 19, 18);
HallSensor hallL(32, 33, 34, POLE_PAIRS);
void hallL_A() { hallL.handleA(); }
void hallL_B() { hallL.handleB(); }
void hallL_C() { hallL.handleC(); }

// ── Estado compartilhado entre loop() e a tarefa de comunicação ──────────────
volatile float target_left  = 0.0f;
volatile float target_right = 0.0f;
volatile unsigned long last_cmd_ms = 0;
volatile bool foc_ready = false;

// ── Recepção de comandos ─────────────────────────────────────────────────────
static char rx_buf[48];
static uint8_t rx_len = 0;

static void handleLine(const char* line) {
  float l, r;
  if (line[0] == 'C' && sscanf(line + 1, " %f %f", &l, &r) == 2) {
    target_left  = l;
    target_right = r;
    last_cmd_ms  = millis();
  }
}

static void readHost() {
  while (HOST.available()) {
    char c = (char)HOST.read();
    if (c == '\n' || c == '\r') {
      if (rx_len > 0) {
        rx_buf[rx_len] = '\0';
        handleLine(rx_buf);
        rx_len = 0;
      }
    } else if (rx_len < sizeof(rx_buf) - 1) {
      rx_buf[rx_len++] = c;
    }
  }
}

// ── Tarefa de comunicação (núcleo 0): telemetria em período fixo ─────────────
// loop() fica dedicado ao loopFOC() no núcleo 1.
void commsTask(void*) {
  TickType_t last_wake = xTaskGetTickCount();
  unsigned long last_heartbeat = 0;

  for (;;) {
    readHost();

    if (!foc_ready) {
      // Durante a calibração inicial só sinaliza que está vivo.
      if (millis() - last_heartbeat > 1000) {
        HOST.println(F("# calibrando motores..."));
        last_heartbeat = millis();
      }
    } else {
      if (millis() - last_cmd_ms > WATCHDOG_MS) {
        target_left  = 0.0f;
        target_right = 0.0f;
      }
      // Sinais escolhidos para que valor positivo = roda girando para a frente
      // do robô, de acordo com a montagem dos motores.
      HOST.printf("M %.4f %.4f %.4f %.4f R\n",
                  -motorL.shaft_velocity, motorR.shaft_velocity,
                  -motorL.shaft_angle,    motorR.shaft_angle);
    }
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(REPORT_PERIOD_MS));
  }
}

// ── Inicialização de um motor ────────────────────────────────────────────────
static void setupMotor(BLDCMotor& motor, BLDCDriver3PWM& driver, HallSensor& hall,
                       void (*a)(), void (*b)(), void (*c)()) {
  hall.init();
  hall.enableInterrupts(a, b, c);
  motor.linkSensor(&hall);

  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.voltage_limit = VOLTAGE_LIMIT;
  driver.init();
  motor.linkDriver(&driver);

  motor.controller = MotionControlType::velocity;
  motor.voltage_sensor_align = ALIGN_VOLTAGE;
  motor.velocity_index_search = 2;
  motor.voltage_limit = VOLTAGE_LIMIT;
  motor.PID_velocity.P = PID_P;
  motor.PID_velocity.I = PID_I;
  motor.PID_velocity.D = PID_D;
  motor.PID_velocity.output_ramp = PID_RAMP;
  motor.LPF_velocity.Tf = VELOCITY_LPF_TF;

  motor.init();
  motor.initFOC();
}

void setup() {
  Serial.begin(115200);  // depuração
#if HOST_ON_USB
  HOST.begin(BAUDRATE);
#else
  HOST.begin(BAUDRATE, SERIAL_8N1, HOST_RX_PIN, HOST_TX_PIN);
#endif
  pinMode(LED_STATUS, OUTPUT);

  // A comunicação sobe antes da calibração para o host ver sinal de vida.
  xTaskCreatePinnedToCore(commsTask, "comms", 8192, NULL, 1, NULL, 0);

  Serial.println("calibrando motor direito...");
  setupMotor(motorR, driverR, hallR, hallR_A, hallR_B, hallR_C);
  Serial.println("calibrando motor esquerdo...");
  setupMotor(motorL, driverL, hallL, hallL_A, hallL_B, hallL_C);

  last_cmd_ms = millis();
  foc_ready = true;
  HOST.println(F("# motores prontos"));
  Serial.println("motores prontos");
}

void loop() {
  motorR.loopFOC();
  motorL.loopFOC();

  // Cada motor recebe o alvo conforme a sua orientação de montagem.
  motorR.move(target_left);
  motorL.move(-target_right);

  static unsigned long led_t = 0;
  if (millis() - led_t > 400) {
    digitalWrite(LED_STATUS, !digitalRead(LED_STATUS));
    led_t = millis();
  }
}
