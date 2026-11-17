// MyoPlayBand - ESP32-S3  (v2: angulo completo + topic /live para la pagina web)
// - MPU6050: angulo respecto a la posicion de reposo (0-180°) -> barra de LEDs, OLED, LED y motor
// - EMG: filtrado + deteccion de activacion muscular -> buzzer
// - Boton: recalibra el EMG y la posicion de reposo del angulo
// - MQTT: /live cada 100 ms (angulo, velocidad, EMG), /estado cada 1 s, comandos por /cmd
//
// El EMG corre en su propia tarea (nucleo 0) a 1000 Hz, asi la OLED y el
// MPU6050 (que son lentos) no le quitan muestras a los filtros.
//
// Libreria extra necesaria: "PubSubClient" de Nick O'Leary (Library Manager)
//
// CAMBIOS v2 (respecto a la version anterior):
//  1) Angulo completo 0-180°: se mide el giro desde la POSICION DE REPOSO, que se
//     guarda al calibrar (al encender, con el boton o con el comando "calibrar").
//     Usa un filtro complementario: acelerometro (sin deriva) + giroscopio (sin ruido).
//  2) Velocidad angular en °/s (positiva alejandose del reposo, negativa volviendo).
//  3) Topic nuevo /live con angulo, velocidad y EMG cada 100 ms (para animar el monito).
//  Todo lo que hace la placa sola (LEDs, motor, buzzer, OLED, boton) funciona igual,
//  con o sin WiFi.

#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_MPU6050 mpu;

// ================== PINES ==================
// --- I2C (MPU6050 + OLED) ---
const int I2C_SDA = 8;
const int I2C_SCL = 9;

// --- Componentes del codigo original ---
const int LED_PIN       = 2;
const int PIN_NEOPIXEL  = 4;
const int BUZZER_PIN    = 5;
const int VIB_MOTOR_PIN = 6;   // usar transistor o modulo

// --- EMG y boton (reubicados para no chocar) ---
const int PIN_EMG   = 1;       // ADC1_CH0
const int PIN_BOTON = 7;       // un lado al pin, el otro a GND

// ================== CONFIGURACION ==================
// --- NeoPixel ---
const int NUMPIXELS = 8;
Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// --- OLED ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- Angulo ---
const float UMBRAL_ANGULO = 45.0;     // angulo objetivo (grados desde el reposo)
const float ALPHA_COMP    = 0.90;     // filtro complementario: peso del giroscopio (0-1)
const int   MUESTRAS_REF  = 10;       // lecturas promediadas para guardar el reposo (~1 s)

// --- Buzzer ---
// true  = buzzer ACTIVO (suena con solo darle voltaje)
// false = buzzer PASIVO (usa tone())
const bool BUZZER_ACTIVO = true;
const int  FREC_BUZZER   = 1000;

// --- EMG ---
const bool  MOSTRAR_PLOTTER = true;   // envia la senal al Serial Plotter
const float FACTOR_ON  = 2.5;         // activa sobre 2.5x el reposo
const float FACTOR_OFF = 1.6;         // desactiva bajo 1.6x el reposo
const unsigned long TIEMPO_MIN_ON_MS = 150;
const float ALPHA_ENV = 0.01;
const float FS = 1000.0;              // 1 muestra por tick de FreeRTOS (1 ms)

// ================== WIFI / MQTT ==================
// Red WiFi de 2.4 GHz con usuario y clave simples (el hotspot del celular sirve).
// Las redes con login de la universidad (portal o usuario/clave por persona) NO funcionan.
const char* WIFI_SSID  = "NOMBRE_DE_TU_RED";
const char* WIFI_PASS  = "CLAVE_DE_TU_RED";

const char* MQTT_BROKER = "broker.emqx.io";   // broker publico gratuito, sin cuenta
const int   MQTT_PORT   = 1883;

#define TOPIC_BASE "myoplayband/grupo8_basti_jt"
const char* TOPIC_ESTADO = TOPIC_BASE "/estado";   // JSON completo cada 1 s
const char* TOPIC_LIVE   = TOPIC_BASE "/live";     // NUEVO: JSON liviano cada 100 ms
const char* TOPIC_EVENTO = TOPIC_BASE "/evento";   // aviso en cada contraccion
const char* TOPIC_CMD    = TOPIC_BASE "/cmd";      // comandos: "calibrar" o "reset"
const char* TOPIC_ONLINE = TOPIC_BASE "/online";   // "online" / "offline"

const unsigned long PERIODO_PUBLICACION_MS = 1000;
const unsigned long PERIODO_LIVE_MS        = 100;

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// ================== FILTRO BIQUAD ==================
struct Biquad {
  float b0, b1, b2, a1, a2;
  float x1 = 0, x2 = 0, y1 = 0, y2 = 0;

  void setHighPass(float fc, float fs, float Q = 0.707) {
    float w0 = 2 * PI * fc / fs, c = cos(w0), alpha = sin(w0) / (2 * Q);
    float a0 = 1 + alpha;
    b0 = (1 + c) / 2 / a0;  b1 = -(1 + c) / a0;  b2 = b0;
    a1 = -2 * c / a0;       a2 = (1 - alpha) / a0;
  }

  void setNotch(float f0, float fs, float Q = 10) {
    float w0 = 2 * PI * f0 / fs, c = cos(w0), alpha = sin(w0) / (2 * Q);
    float a0 = 1 + alpha;
    b0 = 1 / a0;      b1 = -2 * c / a0;  b2 = b0;
    a1 = -2 * c / a0; a2 = (1 - alpha) / a0;
  }

  float process(float x) {
    float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1; x1 = x; y2 = y1; y1 = y;
    return y;
  }
};

Biquad hp, notch50, notch100;
float envolvente = 0;
float umbralOn = 0, umbralOff = 0;

// Estado compartido con el loop principal (para la OLED)
volatile bool musculoActivo = false;
volatile bool calibrando = false;

// Estado compartido para MQTT
volatile float emgActual = 0;                 // envolvente actual
volatile unsigned long contracciones = 0;     // contracciones detectadas
volatile bool nuevaContraccion = false;       // aviso para publicar evento
volatile bool pedirCalibracion = false;       // comando remoto "calibrar"
volatile bool pedirRefAngulo = false;         // NUEVO: guardar la posicion de reposo
float anguloActual = 0;                       // NUEVO: grados desde el reposo (0-180)
float velAngular = 0;                         // NUEVO: °/s
bool anguloOK = false;

// ================== BUZZER ==================
void buzzerOn() {
  if (BUZZER_ACTIVO) digitalWrite(BUZZER_PIN, HIGH);
  else tone(BUZZER_PIN, FREC_BUZZER);
}

void buzzerOff() {
  if (BUZZER_ACTIVO) digitalWrite(BUZZER_PIN, LOW);
  else noTone(BUZZER_PIN);
}

// ================== EMG ==================
float procesarMuestra() {
  float x = analogRead(PIN_EMG);
  float y = hp.process(x);
  y = notch50.process(y);
  y = notch100.process(y);
  envolvente += ALPHA_ENV * (fabs(y) - envolvente);
  return envolvente;
}

void calibrarEMG() {
  calibrando = true;
  musculoActivo = false;
  buzzerOff();
  Serial.println("Calibrando... manten el brazo relajado en la posicion inicial");

  // Dos pitidos cortos = empieza
  buzzerOn(); vTaskDelay(pdMS_TO_TICKS(80)); buzzerOff(); vTaskDelay(pdMS_TO_TICKS(80));
  buzzerOn(); vTaskDelay(pdMS_TO_TICKS(80)); buzzerOff();
  vTaskDelay(pdMS_TO_TICKS(500));

  // NUEVO: el loop guarda esta posicion como "reposo" del angulo (0°)
  pedirRefAngulo = true;

  TickType_t t = xTaskGetTickCount();

  // 1 s para estabilizar filtros
  for (int i = 0; i < 1000; i++) { vTaskDelayUntil(&t, 1); procesarMuestra(); }

  // 2 s midiendo reposo
  const int N = 2000;
  float suma = 0, sumaCuad = 0;
  for (int i = 0; i < N; i++) {
    vTaskDelayUntil(&t, 1);
    float e = procesarMuestra();
    suma += e;
    sumaCuad += e * e;
  }
  float media = suma / N;
  float desv  = sqrt(max(0.0f, sumaCuad / N - media * media));

  umbralOn  = max(media * FACTOR_ON,  media + 6 * desv);
  umbralOff = max(media * FACTOR_OFF, media + 3 * desv);

  Serial.print("Listo. Reposo: ");  Serial.print(media);
  Serial.print("  Umbral ON: ");    Serial.print(umbralOn);
  Serial.print("  Umbral OFF: ");   Serial.println(umbralOff);

  // Pitido largo = termino
  buzzerOn(); vTaskDelay(pdMS_TO_TICKS(300)); buzzerOff();
  calibrando = false;
}

// --- Boton con antirrebote ---
bool estadoBoton = HIGH, lecturaAnterior = HIGH;
unsigned long tCambioBoton = 0;

bool botonPresionado() {
  bool lectura = digitalRead(PIN_BOTON);
  unsigned long ahora = millis();
  if (lectura != lecturaAnterior) { tCambioBoton = ahora; lecturaAnterior = lectura; }
  if (ahora - tCambioBoton > 50 && lectura != estadoBoton) {
    estadoBoton = lectura;
    if (estadoBoton == LOW) return true;
  }
  return false;
}

// --- Tarea EMG (nucleo 0, 1000 Hz) ---
void tareaEMG(void *param) {
  calibrarEMG();

  TickType_t t = xTaskGetTickCount();
  unsigned long tActivacion = 0;
  int contadorPlot = 0;

  for (;;) {
    vTaskDelayUntil(&t, 1);   // 1 tick = 1 ms
    float e = procesarMuestra();
    emgActual = e;
    unsigned long ahora = millis();

    // Boton: recalibrar
    if (botonPresionado()) {
      buzzerOff();
      musculoActivo = false;
      while (digitalRead(PIN_BOTON) == LOW) vTaskDelay(pdMS_TO_TICKS(10));
      calibrarEMG();
      t = xTaskGetTickCount();
      continue;
    }

    // Comando remoto por MQTT: recalibrar
    if (pedirCalibracion) {
      pedirCalibracion = false;
      calibrarEMG();
      t = xTaskGetTickCount();
      continue;
    }

    // Deteccion con histeresis -> buzzer
    if (!musculoActivo && e > umbralOn) {
      musculoActivo = true;
      tActivacion = ahora;
      contracciones++;
      nuevaContraccion = true;
      buzzerOn();
    } else if (musculoActivo && e < umbralOff &&
               ahora - tActivacion >= TIEMPO_MIN_ON_MS) {
      musculoActivo = false;
      buzzerOff();
    }

    // Serial Plotter, 10 veces por segundo
    if (MOSTRAR_PLOTTER && ++contadorPlot >= 100) {
      contadorPlot = 0;
      Serial.print("senal:");      Serial.print(e);
      Serial.print(",umbralOn:");  Serial.print(umbralOn);
      Serial.print(",umbralOff:"); Serial.print(umbralOff);
      Serial.print(",activo:");    Serial.println(musculoActivo ? umbralOn : 0);
    }
  }
}

// ================== ANGULO (NUEVO) ==================
// Se estima el vector gravedad en el marco del sensor:
//  - el giroscopio lo hace girar (rapido y suave, pero se desvia con el tiempo)
//  - el acelerometro lo corrige (sin desvio, pero ruidoso al moverse)
// El angulo es cuanto se ha girado ese vector respecto al guardado en reposo.
float gx = 0, gy = 0, gz = 1;            // gravedad estimada (unitaria)
float rx = 0, ry = 0, rz = 1;            // gravedad en la posicion de reposo
bool  refValida = false;
int   nRef = 0;
float sx = 0, sy = 0, sz = 0;
float anguloPrevio = 0;
unsigned long tPrevioUs = 0;

void normalizar(float &x, float &y, float &z) {
  float n = sqrt(x * x + y * y + z * z);
  if (n > 1e-6) { x /= n; y /= n; z /= n; }
}

void actualizarAngulo(const sensors_event_t &a, const sensors_event_t &g) {
  unsigned long ahoraUs = micros();
  float dt = (tPrevioUs == 0) ? 0.1 : (ahoraUs - tPrevioUs) / 1e6;
  tPrevioUs = ahoraUs;
  if (dt <= 0 || dt > 0.5) dt = 0.1;

  float ax = a.acceleration.x, ay = a.acceleration.y, az = a.acceleration.z;
  normalizar(ax, ay, az);

  // 1) Girar la gravedad estimada con el giroscopio (rad/s): g' = g - (w x g) dt
  float wx = g.gyro.x, wy = g.gyro.y, wz = g.gyro.z;
  float px = gx - (wy * gz - wz * gy) * dt;
  float py = gy - (wz * gx - wx * gz) * dt;
  float pz = gz - (wx * gy - wy * gx) * dt;

  // 2) Mezclar con el acelerometro (filtro complementario)
  gx = ALPHA_COMP * px + (1 - ALPHA_COMP) * ax;
  gy = ALPHA_COMP * py + (1 - ALPHA_COMP) * ay;
  gz = ALPHA_COMP * pz + (1 - ALPHA_COMP) * az;
  normalizar(gx, gy, gz);

  // 3) Guardar la posicion de reposo cuando se calibra
  if (pedirRefAngulo) {
    pedirRefAngulo = false;
    nRef = 0; sx = sy = sz = 0;
  }
  if (nRef < MUESTRAS_REF) {
    sx += ax; sy += ay; sz += az;
    nRef++;
    if (nRef == MUESTRAS_REF) {
      rx = sx; ry = sy; rz = sz;
      normalizar(rx, ry, rz);
      gx = rx; gy = ry; gz = rz;       // partir el filtro desde el reposo
      refValida = true;
      anguloPrevio = 0;
      Serial.println("Posicion de reposo guardada (0 grados)");
    }
  }

  // 4) Angulo entre la gravedad actual y la de reposo
  if (!refValida) { anguloActual = 0; velAngular = 0; return; }
  float d = gx * rx + gy * ry + gz * rz;
  d = constrain(d, -1.0f, 1.0f);
  anguloActual = acos(d) * 180.0 / M_PI;

  // 5) Velocidad angular (°/s), suavizada
  float vRaw = (anguloActual - anguloPrevio) / dt;
  anguloPrevio = anguloActual;
  velAngular += 0.5 * (vRaw - velAngular);
}

// ================== MQTT ==================
// Llega un mensaje al topic de comandos
void alRecibirMensaje(char* topic, byte* payload, unsigned int largo) {
  String msg;
  for (unsigned int i = 0; i < largo; i++) msg += (char)payload[i];
  msg.trim();
  Serial.print("MQTT comando: "); Serial.println(msg);

  if (msg == "calibrar")   pedirCalibracion = true;
  else if (msg == "reset") contracciones = 0;
}

// Reintenta WiFi y MQTT sin bloquear el resto del programa
void mantenerConexion() {
  static unsigned long tUltimoIntento = 0;

  if (WiFi.status() != WL_CONNECTED) return;   // el WiFi se reconecta solo
  if (mqtt.connected()) { mqtt.loop(); return; }

  if (millis() - tUltimoIntento < 5000) return;  // reintentar cada 5 s
  tUltimoIntento = millis();

  String clientId = "myoplayband-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  Serial.print("Conectando a MQTT... ");
  // Last Will: si la placa se desconecta, el broker publica "offline"
  if (mqtt.connect(clientId.c_str(), TOPIC_ONLINE, 1, true, "offline")) {
    Serial.println("OK");
    mqtt.publish(TOPIC_ONLINE, "online", true);
    mqtt.subscribe(TOPIC_CMD);
  } else {
    Serial.print("fallo, rc="); Serial.println(mqtt.state());
  }
}

void publicarEstado() {
  static unsigned long tUltima = 0;
  static unsigned long tUltimaLive = 0;
  if (!mqtt.connected()) return;

  // Evento inmediato en cada contraccion
  if (nuevaContraccion) {
    nuevaContraccion = false;
    char ev[64];
    snprintf(ev, sizeof(ev), "{\"contraccion\":%lu}", contracciones);
    mqtt.publish(TOPIC_EVENTO, ev);
  }

  // NUEVO: datos en vivo cada 100 ms (para el monito de la pagina)
  if (millis() - tUltimaLive >= PERIODO_LIVE_MS) {
    tUltimaLive = millis();
    char live[120];
    snprintf(live, sizeof(live),
      "{\"angulo\":%.1f,\"vel\":%.1f,\"emg\":%.1f,\"activo\":%s}",
      anguloActual, velAngular, (float)emgActual,
      musculoActivo ? "true" : "false");
    mqtt.publish(TOPIC_LIVE, live);
  }

  // Estado completo cada PERIODO_PUBLICACION_MS
  if (millis() - tUltima < PERIODO_PUBLICACION_MS) return;
  tUltima = millis();

  char json[256];
  snprintf(json, sizeof(json),
    "{\"angulo\":%.1f,\"vel\":%.1f,\"umbralAngulo\":%.1f,\"anguloOK\":%s,"
    "\"emg\":%.1f,\"umbralOn\":%.1f,\"umbralOff\":%.1f,"
    "\"musculoActivo\":%s,\"calibrando\":%s,\"contracciones\":%lu}",
    anguloActual, velAngular, UMBRAL_ANGULO, anguloOK ? "true" : "false",
    (float)emgActual, umbralOn, umbralOff,
    musculoActivo ? "true" : "false",
    calibrando ? "true" : "false",
    contracciones);
  mqtt.publish(TOPIC_ESTADO, json);
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);   // I2C rapido: la OLED se actualiza mas rapido

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(BUZZER_PIN, OUTPUT);
  buzzerOff();

  pinMode(VIB_MOTOR_PIN, OUTPUT);
  digitalWrite(VIB_MOTOR_PIN, LOW);

  pinMode(PIN_BOTON, INPUT_PULLUP);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // NeoPixels
  pixels.begin();
  pixels.clear();
  pixels.show();

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("Fallo en la asignacion de SSD1306"));
    while (1) delay(10);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Iniciando MPU6050...");
  display.display();

  // MPU6050
  if (!mpu.begin()) {
    Serial.println("Error al conectar con MPU6050!");
    display.println("Error MPU6050");
    display.display();
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);     // NUEVO: hasta 500 °/s (movimientos rapidos)
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Filtros EMG
  hp.setHighPass(20, FS);
  notch50.setNotch(50, FS);
  notch100.setNotch(100, FS);

  // Arrancar la tarea EMG en el nucleo 0 (el loop corre en el nucleo 1)
  xTaskCreatePinnedToCore(tareaEMG, "EMG", 4096, NULL, 2, NULL, 0);

  // WiFi + MQTT (no bloquea: si no hay red, el resto funciona igual)
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(alRecibirMensaje);
  mqtt.setSocketTimeout(2);                     // menos espera si el broker no responde
}

// ================== LOOP (giroscopio, LEDs, OLED, motor) ==================
void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // NUEVO: angulo completo desde el reposo + velocidad
  actualizarAngulo(a, g);
  anguloOK = (anguloActual >= UMBRAL_ANGULO);

  // Barra de LEDs (se llena al llegar al angulo objetivo)
  float progreso = constrain(anguloActual / UMBRAL_ANGULO, 0.0f, 1.0f);
  int ledsAEncender = (int)(progreso * NUMPIXELS + 0.5);
  pixels.clear();
  for (int i = 0; i < ledsAEncender; i++) {
    pixels.setPixelColor(i, pixels.Color(0, 255, 0));
  }
  pixels.show();

  // OLED
  display.clearDisplay();
  display.setCursor(0, 10);
  display.setTextSize(1);
  display.println("Angulo:");
  display.setTextSize(2);
  display.print(anguloActual, 1);
  display.println(" \xF8");

  // Angulo alcanzado -> LED + motor vibrador (el buzzer NO depende del angulo)
  display.setTextSize(1);
  display.setCursor(0, 45);
  if (anguloOK) {
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(VIB_MOTOR_PIN, HIGH);
    display.println("ESTADO: ACTIVADO");
  } else {
    digitalWrite(LED_PIN, LOW);
    digitalWrite(VIB_MOTOR_PIN, LOW);
    display.println("ESTADO: NORMAL");
  }

  // Estado del EMG
  display.setCursor(0, 55);
  if (calibrando)          display.println("EMG: CALIBRANDO");
  else if (musculoActivo)  display.println("EMG: ACTIVO");
  else                     display.println("EMG: REPOSO");

  // Indicador de conexion en la esquina superior derecha
  display.setCursor(104, 0);
  if (mqtt.connected())                   display.print("MQTT");
  else if (WiFi.status() == WL_CONNECTED) display.print("WiFi");
  else                                    display.print("--");

  display.display();

  // Monitoreo remoto
  mantenerConexion();
  publicarEstado();

  delay(50);   // antes 100: el loop ahora corre a ~10-15 Hz contando la OLED
}
