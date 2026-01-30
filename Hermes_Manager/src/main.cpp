#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include <Preferences.h>

// ------------------------ USER WIRING ------------------------
static constexpr int I2C_SDA = 21;
static constexpr int I2C_SCL = 22;
static constexpr uint32_t I2C_HZ = 400000;

static constexpr int PIN_U  = 25;
static constexpr int PIN_V  = 26;
static constexpr int PIN_W  = 27;
static constexpr int PIN_EN = 33;   // set -1 if your driver has no enable

static constexpr int POLE_PAIRS = 7;
// ------------------------------------------------------------

// -------------------- CONTROL / TUNING -----------------------
static constexpr uint32_t PRINT_PERIOD_MS = 10;

static constexpr float VOLTAGE_LIMIT_V      = 12.0f;
static constexpr float VELOCITY_LIMIT_DPS   = 720.0f;
static constexpr float P_ANGLE_P            = 12.0f;
static constexpr float P_ANGLE_D            = 0.5f;

static constexpr float PID_VEL_P            = 0.2f;
static constexpr float PID_VEL_I            = 2.0f;
static constexpr float PID_VEL_D            = 0.0f;
// ------------------------------------------------------------

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_U, PIN_V, PIN_W, PIN_EN);

Preferences prefs;

static uint32_t g_next_print_ms = 0;
static bool g_enabled   = false;
static bool g_foc_ready = false;

// REL frame offset (deg), stored in NVS.
// rel_deg = wrap(abs_deg - offset_abs_deg)
static float g_offset_abs_deg = 0.0f;
static bool  g_offset_valid   = false;

// Target bias (deg) applied in REL frame, stored in NVS.
// commanded_rel = wrap(user_target_rel + bias_deg)
static float g_bias_deg = 0.0f;
static bool  g_bias_valid = false;

// User target in REL degrees [0,360)
static volatile float g_target_rel_deg = 180.0f;

String inputString = "";

// ---------- Helpers ----------
static float norm360(float deg) {
  float x = fmodf(deg, 360.0f);
  if (x < 0.0f) x += 360.0f;
  return x;
}

// Wrap-aware difference a - b into (-180, 180]
static float wrapDiffDeg(float a_deg, float b_deg) {
  float d = norm360(a_deg) - norm360(b_deg);
  if (d > 180.0f) d -= 360.0f;
  if (d <= -180.0f) d += 360.0f;
  return d;
}

static bool isAngleCommand(const String& s) {
  char *endptr = nullptr;
  const String t = String(s);
  const char* c = t.c_str();
  (void)strtod(c, &endptr);
  if (c == endptr) return false;
  while (*endptr == ' ' || *endptr == '\t') endptr++;
  return *endptr == '\0';
}

static float encAbsDegFresh() {
  sensor.update();
  return norm360(sensor.getAngle() * RAD_TO_DEG);
}

static float absToRelDeg(float abs_deg) {
  if (!g_offset_valid) return norm360(abs_deg);
  return norm360(abs_deg - g_offset_abs_deg);
}

static float relToAbsDeg(float rel_deg) {
  if (!g_offset_valid) return norm360(rel_deg);
  return norm360(rel_deg + g_offset_abs_deg);
}

// ---------- NVS ----------
static void loadCal() {
  prefs.begin("hermes", true);

  g_offset_valid   = prefs.getBool("off_ok", false);
  g_offset_abs_deg = prefs.getFloat("off_abs", 0.0f);

  g_bias_valid = prefs.getBool("bias_ok", false);
  g_bias_deg   = prefs.getFloat("bias_deg", 0.0f);

  prefs.end();
}

static void saveOffset() {
  prefs.begin("hermes", false);
  prefs.putBool("off_ok", g_offset_valid);
  prefs.putFloat("off_abs", g_offset_abs_deg);
  prefs.end();
}

static void saveBias() {
  prefs.begin("hermes", false);
  prefs.putBool("bias_ok", g_bias_valid);
  prefs.putFloat("bias_deg", g_bias_deg);
  prefs.end();
}

static void setOffsetNowAndSave() {
  const float abs_deg = encAbsDegFresh();
  g_offset_abs_deg = abs_deg;
  g_offset_valid = true;
  saveOffset();

  Serial.print("\nOffset set & saved. off_abs_deg=");
  Serial.print(g_offset_abs_deg, 2);
  Serial.println(" (AS5600 frame). REL angle now = 0.00 deg");
}

static void clearOffset() {
  prefs.begin("hermes", false);
  prefs.putBool("off_ok", false);
  prefs.putFloat("off_abs", 0.0f);
  prefs.end();
  g_offset_valid = false;
  g_offset_abs_deg = 0.0f;

  Serial.println("\nOffset cleared. REL frame == ABS frame now.");
}

static void clearBias() {
  prefs.begin("hermes", false);
  prefs.putBool("bias_ok", false);
  prefs.putFloat("bias_deg", 0.0f);
  prefs.end();
  g_bias_valid = false;
  g_bias_deg = 0.0f;

  Serial.println("\nBias cleared. commanded_rel == target_rel now.");
}

// ---------- UI ----------
static void printHelp() {
  Serial.println("\nCommands:");
  Serial.println("  o            -> set REL offset now (save). Makes current position REL=0");
  Serial.println("  x            -> clear REL offset (REL==ABS)");
  Serial.println("  k            -> learn bias from current position (makes final==target)");
  Serial.println("  b            -> clear bias");
  Serial.println("  e            -> enable motor (runs initFOC once per boot/session)");
  Serial.println("  c            -> run initFOC now (calibrate this session)");
  Serial.println("  d            -> disable motor");
  Serial.println("  p            -> print status");
  Serial.println("  <number>     -> set target REL angle (deg), wrapped to 0..360");
  Serial.println();
}

static void printStatus() {
  const float abs_deg = encAbsDegFresh();
  const float rel_deg = absToRelDeg(abs_deg);

  Serial.println("\n--- Status ---");
  Serial.print("offset_ok: "); Serial.println(g_offset_valid ? "true" : "false");
  Serial.print("offset_abs_deg: "); Serial.println(g_offset_abs_deg, 2);
  Serial.print("bias_ok: "); Serial.println(g_bias_valid ? "true" : "false");
  Serial.print("bias_deg: "); Serial.println(g_bias_deg, 2);
  Serial.print("enc_abs_deg: "); Serial.println(abs_deg, 2);
  Serial.print("enc_rel_deg: "); Serial.println(rel_deg, 2);
  Serial.print("enabled: "); Serial.println(g_enabled ? "true" : "false");
  Serial.print("foc_ready: "); Serial.println(g_foc_ready ? "true" : "false");
  if (g_foc_ready) {
    Serial.print("sensor_offset_deg: "); Serial.println(motor.sensor_offset * RAD_TO_DEG, 4);
  }
  Serial.println("--------------");
}

// ---------- Motor control ----------
static void disableMotor() {
  driver.disable();
  g_enabled = false;
  Serial.println("\nMotor DISABLED");
}

static void runFOCOnceThisSession() {
  if (g_foc_ready) {
    Serial.println("\nFOC already initialized this session.");
    return;
  }

  driver.enable();
  g_enabled = true;

  motor.initFOC();
  g_foc_ready = true;

  Serial.print("\nFOC initialized (this session). sensor_offset_deg=");
  Serial.println(motor.sensor_offset * RAD_TO_DEG, 4);
}

static void enableMotor() {
  g_target_rel_deg = norm360(g_target_rel_deg);
  if (!g_foc_ready) runFOCOnceThisSession();

  Serial.print("\nMotor ENABLED. Target(rel)=");
  Serial.print(g_target_rel_deg, 2);
  Serial.print(" deg, bias=");
  Serial.print(g_bias_deg, 2);
  Serial.println(" deg");
}

// Learn bias so that (target + bias) lands on the current position.
// Call this when the motor has settled at the target (or whenever you want to correct).
static void learnBiasNow() {
  const float abs_deg = encAbsDegFresh();
  const float rel_deg = absToRelDeg(abs_deg);

  // error = final - target (wrap-aware)
  const float err = wrapDiffDeg(rel_deg, g_target_rel_deg);

  // We want new commanded = target + bias to reduce error to ~0.
  // If final = target + err, then bias should subtract err.
  g_bias_deg = norm360(g_bias_deg + err);
  g_bias_valid = true;
  saveBias();

  Serial.print("\nBias learned. err(final-target)=");
  Serial.print(err, 2);
  Serial.print(" deg -> new bias=");
  Serial.print(g_bias_deg, 2);
  Serial.println(" deg");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_HZ);

  sensor.init(&Wire);

  driver.voltage_power_supply = 12.0f;
  driver.pwm_frequency = 20000;

  // If your driver enable is ACTIVE-LOW, uncomment:
  // driver.enable_active_high = false;

  driver.init();
  driver.disable(); // keep off during boot/upload/reset

  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);

  motor.controller = MotionControlType::angle;
  motor.voltage_limit  = VOLTAGE_LIMIT_V;
  motor.velocity_limit = VELOCITY_LIMIT_DPS * DEG_TO_RAD;

  motor.P_angle.P = P_ANGLE_P;
  motor.P_angle.D = P_ANGLE_D;

  motor.PID_velocity.P = PID_VEL_P;
  motor.PID_velocity.I = PID_VEL_I;
  motor.PID_velocity.D = PID_VEL_D;
  motor.PID_velocity.limit = motor.voltage_limit;

  motor.init();

  loadCal();

  // Initialize target to current REL (so enable won't jump)
  const float abs_deg = encAbsDegFresh();
  g_target_rel_deg = absToRelDeg(abs_deg);

  Serial.println("\n=== Hermes Angle Control (REL inputs + saved offset + bias) ===");
  Serial.println("All user inputs are REL degrees, wrapped to 0..360.");
  Serial.println("Use 'o' to set REL=0 at current position (saved).");
  Serial.println("If final angle is consistently offset from target, settle then press 'k' to learn bias.");
  printHelp();
  printStatus();

  g_next_print_ms = millis() + PRINT_PERIOD_MS;
}

void loop() {
  sensor.update();

  // -------- Serial input --------
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputString.length() > 0) {
        String cmd = inputString;
        cmd.trim();
        inputString = "";

        if (cmd.equalsIgnoreCase("o")) {
          setOffsetNowAndSave();
          g_target_rel_deg = absToRelDeg(encAbsDegFresh()); // avoid step
        } else if (cmd.equalsIgnoreCase("x")) {
          clearOffset();
          g_target_rel_deg = absToRelDeg(encAbsDegFresh());
        } else if (cmd.equalsIgnoreCase("k")) {
          learnBiasNow();
        } else if (cmd.equalsIgnoreCase("b")) {
          clearBias();
        } else if (cmd.equalsIgnoreCase("e")) {
          enableMotor();
        } else if (cmd.equalsIgnoreCase("c")) {
          runFOCOnceThisSession();
        } else if (cmd.equalsIgnoreCase("d")) {
          disableMotor();
        } else if (cmd.equalsIgnoreCase("p")) {
          printStatus();
        } else if (isAngleCommand(cmd)) {
          float requested = cmd.toFloat();
          g_target_rel_deg = norm360(requested);

          Serial.print("\nRequested(rel)=");
          Serial.print(requested, 2);
          Serial.print(" -> wrapped=");
          Serial.print(g_target_rel_deg, 2);
          Serial.println(" deg");
        } else if (cmd.equalsIgnoreCase("h") || cmd.equalsIgnoreCase("help")) {
          printHelp();
        } else {
          Serial.println("\nUnknown command. Type 'h' for help.");
        }
      }
    } else {
      inputString += c;
    }
  }

  // -------- Motor control --------
  if (g_enabled && g_foc_ready) {
    // Apply bias in REL frame
    const float commanded_rel_deg = norm360(g_target_rel_deg + g_bias_deg);

    // REL -> ABS -> rad
    const float target_abs_deg = relToAbsDeg(commanded_rel_deg);
    const float target_abs_rad = target_abs_deg * DEG_TO_RAD;

    // ABS -> shaft frame (SimpleFOC internal) using sensor_offset
    const float target_shaft_rad = target_abs_rad - motor.sensor_offset;

    motor.loopFOC();
    motor.move(target_shaft_rad);
  }

  // -------- Print angles (REL frame, parity) --------
  const uint32_t now = millis();
  if ((int32_t)(now - g_next_print_ms) >= 0) {
    g_next_print_ms = now + PRINT_PERIOD_MS;

    const float abs_deg = norm360(sensor.getAngle() * RAD_TO_DEG);
    const float rel_deg = absToRelDeg(abs_deg);
    const float commanded_rel = norm360(g_target_rel_deg + g_bias_deg);

    Serial.print("enc_deg=");
    Serial.print(rel_deg, 2);
    Serial.print(" motor_deg=");
    Serial.print(rel_deg, 2);
    Serial.print(" target=");
    Serial.print(g_target_rel_deg, 2);
    Serial.print(" cmd=");
    Serial.print(commanded_rel, 2);
    Serial.print(" bias=");
    Serial.print(g_bias_deg, 2);
    Serial.print(" en=");
    Serial.print(g_enabled ? 1 : 0);
    Serial.print("   \r");
  }
}
