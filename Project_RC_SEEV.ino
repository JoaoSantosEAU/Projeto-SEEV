/*
Nome João Santos - 2231131
Nome Afonso Fernandes - 2232108
IPLEIRIA - Instituto Politécnico de Leiria
ESTG - Escola Superior de Tecnologia e Gestão
LEAU - Licenciatura em Engenharia Automóvel
SEEV - Sistemas Elétricos e Eletrónicos de Veículos
*/


// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------- Bibliotecas -------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

#include <Bluepad32.h>
#include "driver/ledc.h"
#include "Arduino.h"
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Wire.h>     //Sensor Laser;
#include <VL53L0X.h>  //Sensor Laser;
#include <Adafruit_NeoPixel.h>


// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------- Defines -----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


ControllerPtr myControllers[BP32_MAX_GAMEPADS];  //Para o controlo do comando;

// ---------------------- MOTOR -------------------------------
#define ESC_PIN 25  //GPIO

#define MOTOR_CHANNEL LEDC_CHANNEL_1  // Para configurar o LEDC;
#define MOTOR_TIMER LEDC_TIMER_1      // Para configurar o LEDC;
#define MOTOR_FREQ 100                // Para configurar o LEDC;
#define MOTOR_RES LEDC_TIMER_16_BIT   // Para configurar o LEDC;

// Duty 16 bits motor (100 Hz)
#define NEUTRO_DUTY 9830  // 15%          //original= 9830
#define MAX_DUTY 14000    // 17% frente  //original= 11148 //Já anda mais = 14000
#define MIN_DUTY 7000     // 12% trás     //original = na sei // Já anda mais = 7000

#define MAX_DUTY_HiGH_SPEED 11000
#define MIN_DUTY_HiGH_SPEED 8660

#define DEADZONE 20
// ------------------------------------------------------------

// ----------------- Servo Motor(direção) ---------------------
#define SERVO_PIN 32  //GPIO

#define SERVO_CHANNEL LEDC_CHANNEL_0  // Para configurar o LEDC;
#define SERVO_TIMER LEDC_TIMER_0      // Para configurar o LEDC;
#define SERVO_FREQ 50                 // Para configurar o LEDC;
#define SERVO_RES LEDC_TIMER_16_BIT   // Para configurar o LEDC;

// Pulsos RC em 16 bits (20ms = 65535)
#define PULSE_MIN (uint32_t)(65535 * 0.05)   // 1.0ms  = 5%   //original= 0.05  //0.065 tá fixe precisso, mas tenho curvas apertadas só auto estrada //0.060 já é muito
#define PULSE_MID (uint32_t)(65535 * 0.075)  // 1.5ms  = 7.5% //original= 0.075
#define PULSE_MAX (uint32_t)(65535 * 0.1)    // 2.0ms  = 10%  //original= 0.10  //0.085 tá fixe precisso //0.090 já é muito

#define PULSE_MIN_HiGH_SPEED (uint32_t)(65535 * 0.065)
#define PULSE_MAX_HiGH_SPEED (uint32_t)(65535 * 0.085)
// ------------------------------------------------------------

// --------------------------- Buzzer -------------------------
#define PWM_buzzer 4  //GPIO

#define BUZZER_CHANNEL LEDC_CHANNEL_2    // Para configurar o LEDC;
#define BUZZER_TIMER LEDC_TIMER_2        // Para configurar o LEDC;
#define BUZZER_MODE LEDC_LOW_SPEED_MODE  // Para configurar o LEDC;
#define BUZZER_FREQ_HZ 2000              // Para configurar o LEDC;
#define BUZZER_RES LEDC_TIMER_8_BIT      // Para configurar o LEDC;
// -----------------------------------------------

// ---------------- Display ----------------------
#define TFT_CS 5     //GPIO
#define TFT_DC 17    //GPIO
#define TFT_MOSI 23  //GPIO
#define TFT_SCLK 18  //GPIO
#define TFT_RST 19   //GPIO
// -----------------------------------------------

// ----------------- Cores -----------------------
#define COLOR_Background ST77XX_BLACK
#define COLOR_OUTLINE ST77XX_BLUE
#define COLOR_LEFT_FILL ST77XX_GREEN
#define COLOR_CENTER_FILL ST77XX_RED
#define COLOR_CHASSIS ST77XX_BLUE
#define COLOR_ARROW ST77XX_GREEN
#define COLOR_LIGHT ST77XX_CYAN
#define COLOR_ORANGE ST77XX_ORANGE
#define COLOR_YELLOW ST77XX_YELLOW
// -----------------------------------------------

// ----------------- LDR + LEDS ------------------
#define LDR 35  //GPIO;
#define ADC_RESOLUTION 10
#define VREF_PLUS 3.3
#define VREF_MINUS 0.0
#define LED_farois 13          //GPIO;
#define LED_piscasdireita 12   //GPIO;
#define LED_piscasesquerda 14  //GPIO;
#define NUMPIXELS 10           //nº de leds na fita;
// -----------------------------------------------

// ----------------- Butôes ----------------------
#define Botao_esquerdo 15      //GPIO;
#define Botao_direito 2        //GPIO;
#define lauchcontrol_boton 16  //GPIO;
// -----------------------------------------------

// ----------------- Tasks -----------------------
void vTask_PWM_Direcao(void *pvParameters);
void vTask_PWM_Tracao(void *pvParameters);
void vTask_Sensor_Distancia(void *pvParameters);
void vTask_PWM_Buzzer(void *pvParameters);
void vTask_LDR_ADC(void *pvParameters);
void vTask_Inicializacao_Display(void *pvParameters);
void vTask_Display(void *pvParameters);
void vTask_Bluetooth(void *pvParameters);
void vTask_Farois(void *pvParameters);
void vTask_PiscasManager(void *pvParameters);
void vTask_pisca_direito(void *pvParameters);
void vTask_pisca_esquerdo(void *pvParameters);
void vTask_piscas_emergencia(void *pvParameters);
void vTask_launch_control(void *pvParameters);
static void Semaforo_give(void);
// -----------------------------------------------

// ----------------- Interrupções ----------------
void IRAM_ATTR ISR_quatro_piscas();
void IRAM_ATTR ISR_Botao_esquerdo();
void IRAM_ATTR ISR_Botao_direito();
void IRAM_ATTR ISR_launch_control();
// -----------------------------------------------

// --------------------- Queues -------------------
QueueHandle_t distanciaQueue;
QueueHandle_t piscasQueue;
QueueHandle_t luminosidadeQueue;
QueueHandle_t LuzesLigadasQueue;
QueueHandle_t Quatro_Piscas_DisplayQueue;
QueueHandle_t Steeringdata;
QueueHandle_t motordata;
QueueHandle_t drivemode;
// -----------------------------------------------

// ----------------- Structs ---------------------
typedef struct {

  bool mode_1;
  bool mode_2;
  int speed = 0;

} drivemodestruct;

typedef struct {

  int xvalue;
  int yvalue;

} Steeringdatastruct;

typedef struct {

  uint16_t throttle;
  uint16_t brake;

} motordatastruct;

typedef struct {
  int escuro;
  int LDR_value;

} luminosidade;

typedef struct {

  bool Estado;
  uint16_t cor;

} QuatroPiscas_display;
// -----------------------------------------------

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
Adafruit_NeoPixel pixel_led_esquerdo(NUMPIXELS, LED_piscasesquerda, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixel_led_direito(NUMPIXELS, LED_piscasdireita, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixel_luz(NUMPIXELS, LED_farois, NEO_GRB + NEO_KHZ800);


// --------------- Semáforos ---------------------
SemaphoreHandle_t semaforo_piscas;
SemaphoreHandle_t semaforo_launchcontrol;
// -----------------------------------------------

// -------- Task handles dos piscas --------------
TaskHandle_t pisca_esquerdo_TaskHandle = NULL;
TaskHandle_t pisca_direito_TaskHandle = NULL;
TaskHandle_t quatro_piscas_TaskHandle = NULL;
TaskHandle_t travagem_emergencia_TaskHandle = NULL;
TaskHandle_t piscaManager_Handle = NULL;
TaskHandle_t xHandleMotor = NULL;
TaskHandle_t xHandleSteering = NULL;
// -----------------------------------------------

// -----------------------------------------------
typedef enum {

  PISCA_ESQUERDO,
  PISCA_DIREITO,
  PISCA_QUATRO,
  TRAVAGEM_EMERGENCIA

} PiscaEvento_t;
// -----------------------------------------------

// ----------------- Estados ---------------------
bool esquerdo_ativo = false;
bool direito_ativo = false;
bool quatro_ativo = false;
bool travagem_emergencia_ativo = false;
// -----------------------------------------------

// ---------------- LEDC SETUP -------------------
void setupLEDC() {  //Configurar os sinais PWM;

  // ---- TIMER da DIREÇÃO (50 Hz) ----
  ledc_timer_config_t t1 = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = SERVO_RES,
    .timer_num = SERVO_TIMER,
    .freq_hz = SERVO_FREQ,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&t1);

  // ---- CANAL da DIREÇÃO ----
  ledc_channel_config_t c1 = {
    .gpio_num = SERVO_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = SERVO_CHANNEL,
    .timer_sel = SERVO_TIMER,
    .duty = PULSE_MID,
    .hpoint = 0
  };
  ledc_channel_config(&c1);

  // ---- TIMER do MOTOR (100 Hz) ----
  ledc_timer_config_t t2 = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = MOTOR_RES,
    .timer_num = MOTOR_TIMER,
    .freq_hz = MOTOR_FREQ,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&t2);

  // ---- CANAL do MOTOR ----
  ledc_channel_config_t c2 = {
    .gpio_num = ESC_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = MOTOR_CHANNEL,
    .timer_sel = MOTOR_TIMER,
    .duty = NEUTRO_DUTY,
    .hpoint = 0
  };
  ledc_channel_config(&c2);

  // ---- TIMER do Buzzer ----
  ledc_timer_config_t tb = {
    .speed_mode = BUZZER_MODE,  // Agora é LOW_SPEED
    .duty_resolution = BUZZER_RES,
    .timer_num = BUZZER_TIMER,
    .freq_hz = BUZZER_FREQ_HZ,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&tb);

  // ---- CANAL do Buzzer ----
  ledc_channel_config_t cb = {
    .gpio_num = PWM_buzzer,
    .speed_mode = BUZZER_MODE,  // Deve ser igual ao timer acima
    .channel = BUZZER_CHANNEL,
    .timer_sel = BUZZER_TIMER,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&cb);
}
// ------------------------------------------------

// --------------------------------------- Funções Display ------------------------------------------

// funções display                                     tft.drawRect(x, y, width, height, color);
void drawOutlineRect(int x, int y, int w, int h, uint16_t color, Adafruit_ST7735 tft) {
  tft.drawRect(x, y, w, h, color);
}

void drawPercentBar(int x, int y, int w, int h, int pct, uint16_t color, Adafruit_ST7735 tft) {
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;

  //limpeza interna da barra
  tft.fillRect(x + 1, y + 1, w - 2, h - 2, COLOR_Background);

  int filled = ((h - 2) * pct) / 100;
  tft.fillRect(x + 1, y + h - filled, w - 2, filled, color);
}

//Inserir valores dentro das barras
void drawCenteredText(int x, int y, int w, int h, const char *text, Adafruit_ST7735 tft) {
  // Clear text area
  tft.fillRect(x + 2, y + 2, w - 4, 10, COLOR_Background);

  // Center text
  int16_t x1, y1;
  uint16_t tw, th;
  tft.getTextBounds((char *)text, x, y, &x1, &y1, &tw, &th);

  int cx = x + (w - tw) / 2;
  int cy = y + 3;

  tft.setCursor(cx, cy);
  tft.print(text);
}

void drawChassis(int chassisX, int chassisY, int chassisW, int chassisH, Adafruit_ST7735 tft) {
  tft.drawRect(chassisX, chassisY, chassisW, chassisH, COLOR_CHASSIS);

  // rodas
  tft.fillRect(chassisX - 10, chassisY + 5, 8, 15, COLOR_CHASSIS);            //frente esquerda
  tft.fillRect(chassisX + chassisW + 2, chassisY + 5, 8, 15, COLOR_CHASSIS);  //frente direita
  tft.fillRect(chassisX - 10, chassisY + chassisH - 20, 8, 15, COLOR_CHASSIS);
  tft.fillRect(chassisX + chassisW + 2, chassisY + chassisH - 20, 8, 15, COLOR_CHASSIS);
}

void eraseArrows(int chassisX, int chassisY, int chassisW, int chassisH, Adafruit_ST7735 tft) {
  int mx = chassisX + chassisW / 2;
  int my = chassisY + chassisH / 2;
  tft.fillTriangle(mx, chassisY - 18, mx - 6, chassisY - 4, mx + 6, chassisY - 4, COLOR_Background);
  tft.fillTriangle(mx, chassisY + chassisH + 18, mx - 6, chassisY + chassisH + 4, mx + 6, chassisY + chassisH + 4, COLOR_Background);
  tft.fillTriangle(chassisX - 18, my, chassisX - 4, my - 6, chassisX - 4, my + 6, COLOR_Background);
  tft.fillTriangle(chassisX + chassisW + 18, my, chassisX + chassisW + 4, my - 6, chassisX + chassisW + 4, my + 6, COLOR_Background);
}

void drawArrowFront(int chassisX, int chassisY, int chassisW, int chassisH, Adafruit_ST7735 tft) {  //tft.fillTriangle(x1, y1, x2, y2, x3, y3, color);
  int mx = chassisX + chassisW / 2;                                                                 //meio do retangulo do chassis na horizontal
  tft.fillTriangle(mx, chassisY - 18, mx - 6, chassisY - 4, mx + 6, chassisY - 4, COLOR_ARROW);
}

void drawArrowBack(int chassisX, int chassisY, int chassisW, int chassisH, Adafruit_ST7735 tft) {
  int mx = chassisX + chassisW / 2;
  tft.fillTriangle(mx, chassisY + chassisH + 18, mx - 6, chassisY + chassisH + 4, mx + 6, chassisY + chassisH + 4, COLOR_ARROW);
}

void drawArrowLeft(int chassisX, int chassisY, int chassisW, int chassisH, Adafruit_ST7735 tft) {
  int my = chassisY + chassisH / 2;  //meio do retangulo do chassis na vertical
  tft.fillTriangle(chassisX - 18, my, chassisX - 4, my - 6, chassisX - 4, my + 6, COLOR_ARROW);
}

void drawArrowRight(int chassisX, int chassisY, int chassisW, int chassisH, Adafruit_ST7735 tft) {
  int my = chassisY + chassisH / 2;
  tft.fillTriangle(chassisX + chassisW + 18, my, chassisX + chassisW + 4, my - 6, chassisX + chassisW + 4, my + 6, COLOR_ARROW);
}

void drawEmergencyTriangle(int16_t x, int16_t y, bool ligado, Adafruit_ST7735 &tft) {
  if (ligado) {

    tft.fillTriangle(x - 8, y + 15, x + 8, y + 15, x, y, ST77XX_ORANGE);     // Desenhar o triângulo com laranja;
    tft.fillTriangle(x - 5, y + 13, x + 5, y + 13, x, y + 4, ST77XX_BLACK);  // Furar o centro a preto;

  } else {

    tft.fillTriangle(x - 8, y + 15, x + 8, y + 15, x, y, ST77XX_BLACK);  // Apaga o triângulo;
  }
}

void drawFarois(int highBeamX, int highBeamY, int highBeamR, bool on, Adafruit_ST7735 tft) {
  if (on) {
    tft.fillCircle(highBeamX, highBeamY, highBeamR, COLOR_LIGHT);
    tft.drawLine(highBeamX + 5, highBeamY - 6, highBeamX + 14, highBeamY - 12, COLOR_LIGHT);
    tft.drawLine(highBeamX + 5, highBeamY, highBeamX + 14, highBeamY, COLOR_LIGHT);
    tft.drawLine(highBeamX + 5, highBeamY + 6, highBeamX + 14, highBeamY + 12, COLOR_LIGHT);
  } else {
    tft.fillCircle(highBeamX, highBeamY, highBeamR, COLOR_Background);
    tft.drawLine(highBeamX + 5, highBeamY - 6, highBeamX + 14, highBeamY - 12, COLOR_Background);
    tft.drawLine(highBeamX + 5, highBeamY, highBeamX + 14, highBeamY, COLOR_Background);
    tft.drawLine(highBeamX + 5, highBeamY + 6, highBeamX + 14, highBeamY + 12, COLOR_Background);
  }
}
// --------------------------------------------------------------------------------------------------

// ------------------------------- Funções para ligar o comando -------------------------------------
void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
    if (myControllers[i] == nullptr) {
      myControllers[i] = ctl;
      break;
    }
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
    if (myControllers[i] == ctl)
      myControllers[i] = nullptr;
}
// -------------------------------------------------------------------------------------------------

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------- Setup -------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void setup() {

  Serial.begin(115200);

  // -------------- Iniciar comando ------------------
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);
  // ------------------------------------------------

  setupLEDC();  //Executar a função de configurar os sinais PWM;

  Serial.println("iniciei o display");

  tft.initR(INITR_BLACKTAB);  //Inicializar Display;
  tft.setRotation(1);         //Rodar display;
  
  Serial.println("iniciei-o");

  // ----------------- Criar as Queues ----------------
  Steeringdata = xQueueCreate(1, sizeof(Steeringdatastruct));  //Queue para a estrutura dos dados;
  motordata = xQueueCreate(1, sizeof(motordatastruct));
  drivemode = xQueueCreate(1, sizeof(drivemodestruct));
  distanciaQueue = xQueueCreate(1, sizeof(int));
  piscasQueue = xQueueCreate(10, sizeof(piscasQueue));
  luminosidadeQueue = xQueueCreate(1, sizeof(luminosidade));
  LuzesLigadasQueue = xQueueCreate(1, sizeof(bool));
  Quatro_Piscas_DisplayQueue = xQueueCreate(1, sizeof(QuatroPiscas_display));
  // ------------------------------------------------

  // ------------ Botões das interrupções -----------
  pinMode(0, INPUT_PULLUP);        // Boot button
  pinMode(Botao_esquerdo, INPUT);  // External pull-up
  pinMode(Botao_direito, INPUT);
  pinMode(lauchcontrol_boton, INPUT);
  // ------------------------------------------------


  // --------------- Interrupções -------------------
  attachInterrupt(digitalPinToInterrupt(0), ISR_quatro_piscas, FALLING);
  attachInterrupt(digitalPinToInterrupt(Botao_esquerdo), ISR_Botao_esquerdo, FALLING);
  attachInterrupt(digitalPinToInterrupt(Botao_direito), ISR_Botao_direito, FALLING);
  attachInterrupt(digitalPinToInterrupt(lauchcontrol_boton), ISR_launch_control, FALLING);
  // ------------------------------------------------

  // ----------------- Semáforos --------------------
  semaforo_piscas = xSemaphoreCreateCounting(10, 0);
  semaforo_launchcontrol = xSemaphoreCreateBinary();
  // ------------------------------------------------

  // ------------------- Tasks ----------------------
  xTaskCreatePinnedToCore(vTask_PWM_Direcao, "PWM_Direção", 4096, NULL, 5, &xHandleSteering, 1);
  xTaskCreatePinnedToCore(vTask_PWM_Tracao, "Task PWM_Tração", 4096, NULL, 4, &xHandleMotor, 1);
  xTaskCreatePinnedToCore(vTask_Sensor_Distancia, "Task Sensor_Distancia", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(vTask_PWM_Buzzer, "Task PWM_Buzzer", 2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(vTask_LDR_ADC, "Task ADC", 2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(vTask_PiscasManager, "Pisca_Manager", 3072, NULL, 1, &piscaManager_Handle, 1);
  xTaskCreatePinnedToCore(vTask_Farois, "Task Farois", 3072, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(vTask_Inicializacao_Display, "Task Inicialização do Display", 4096, NULL, 6, NULL, 1);
  xTaskCreatePinnedToCore(vTask_Bluetooth, "Task Bluetooth", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(vTask_launch_control, "Tasklaunch_controle", 4096, NULL, 1, NULL, 1);
  // ------------------------------------------------

  Serial.println("Setup completo!");
}


// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------- Tasks -------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void vTask_PWM_Direcao(void *pvParameters) {

  uint32_t pulse;
  Steeringdatastruct urso;
  drivemodestruct mg3;

  for (;;) {

    xQueuePeek(drivemode, &mg3, portTICK_PERIOD_MS);

    bool mode_1 = mg3.mode_1;
    uint16_t speed = mg3.speed;

    if (xQueuePeek(Steeringdata, &urso, portTICK_PERIOD_MS) == pdTRUE) {  //Segurança, caso não receba dados da Queue, mete a direção a meio(fazer uma que pare o carro);

      int x = urso.xvalue;

      if (mode_1) {

        if (abs(x) < 15) {    //esta zona é pra a direção pesada;
          pulse = PULSE_MID;  // zona morta
        } else if (x > 15) {
          pulse = map(x, 0, 512, PULSE_MID, PULSE_MAX_HiGH_SPEED);
        } else if (x < -15) {
          pulse = map(x, -512, 0, PULSE_MIN_HiGH_SPEED, PULSE_MID);
        } else {
          pulse = PULSE_MID;
        }

      } else {  //esta zona é para uma direção mais pesada;

        if (abs(x) < 15) {
          pulse = PULSE_MID;  // zona morta
        } else if (x > 15) {
          pulse = map(x, 0, 512, PULSE_MID, PULSE_MAX);
        } else if (x < -15) {
          pulse = map(x, -512, 0, PULSE_MIN, PULSE_MID);
        } else {
          pulse = PULSE_MID;
        }
      }

      ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL, pulse);
      ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL);
    }

    vTaskDelay(15 / portTICK_PERIOD_MS);
  }
}

void vTask_PWM_Tracao(void *pvParameters) {

  motordatastruct urso;  //Ponteiro para a estrutura com os dados do motor (motordatastruct);
  drivemodestruct mg3;   //Ponteiro para a estrutura com os dados dos modos de condução (drivemodestruct);

  int distancia = 200;             //Variavel para a travagem de emergência, distância;
  bool quatro_auto_ativo = false;  //Variavel para a travagem de emergência, estado dos piscas;

  uint32_t currentDuty = NEUTRO_DUTY;  //Variavel para Duty Cycle do motor;
  uint32_t targetDuty = NEUTRO_DUTY;   //Variavel para Duty Cycle do motor;


  for (;;) {

    xQueueReceive(motordata, &urso, portTICK_PERIOD_MS);
    xQueuePeek(drivemode, &mg3, portTICK_PERIOD_MS);

    uint16_t throttle = urso.throttle;
    uint16_t brake = urso.brake;

    xQueuePeek(distanciaQueue, &distancia, pdMS_TO_TICKS(10));
    distancia = constrain(distancia, 0, 600);


    if (distancia < 60) {  //Travagem de emergência;

      if (uxTaskPriorityGet(NULL) != 6) {

        vTaskPrioritySet(NULL, 6);
      }

      currentDuty = NEUTRO_DUTY;

      if (!quatro_auto_ativo) {

        PiscaEvento_t ev = PISCA_QUATRO;
        xQueueSend(piscasQueue, &ev, 0);
        quatro_auto_ativo = true;
      }
    } else {  //Caso não esteja em Travage de emergência, repõe a prioridade e desliga os piscas;

      if (uxTaskPriorityGet(NULL) != 4) vTaskPrioritySet(NULL, 4);

      if (quatro_auto_ativo) {

        PiscaEvento_t ev = PISCA_QUATRO;
        xQueueSend(piscasQueue, &ev, 0);
        quatro_auto_ativo = false;
      }


      uint32_t limite_max = MAX_DUTY;  //Tem de estar aqui se não, não funciona;
      uint32_t limite_min = MIN_DUTY;  //Tem de estar aqui se não, não funciona;

      if (mg3.mode_2) {  //Se estiver ligado calcula novos limites para o Duty Cycle máximo do motor, aqui o operador escolhe;

        limite_max = constrain(9830 + 500 + mg3.speed, 9830, 14000);
        limite_min = MIN_DUTY_HiGH_SPEED;
      }

      if (throttle > DEADZONE) {  //Calcula qual vai ser o valor final de Duty Cycle para o motor;

        targetDuty = map(throttle, DEADZONE, 1023, NEUTRO_DUTY, limite_max);

      } else if (brake > DEADZONE) {

        targetDuty = map(brake, DEADZONE, 1023, NEUTRO_DUTY, limite_min);

      } else {

        targetDuty = NEUTRO_DUTY;
      }
    }

    if ((currentDuty > NEUTRO_DUTY && targetDuty < NEUTRO_DUTY) || (currentDuty < NEUTRO_DUTY && targetDuty > NEUTRO_DUTY)) {

      currentDuty = NEUTRO_DUTY;  // sempre parar antes de inverter
    } else {

      currentDuty = targetDuty;
    }

    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR_CHANNEL, currentDuty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR_CHANNEL);

    // Serial.println(currentDuty);

    vTaskDelay(pdMS_TO_TICKS(30));
  }
}

void vTask_Sensor_Distancia(void *pvParameters) {

  int distance = 0, distance_filtered = 0, distance_tmp = 0;

  VL53L0X sensor;

  //Inicializar o I2C no ESP32 nos pinos 21 (SDA), 22 (SCL)
  Wire.begin(21, 22);

  // Iniciar o Sensor de Distancia:
  if (!sensor.init()) {

    Serial.println("Failed to detect and initialize VL53L0X!");

    /*while (!sensor.init()) {

      Serial.println("Falha ao detectar o VL53L0X! Tentando novamente...");
    }*/
  }

  //Inicializar sensor de distância
  sensor.setTimeout(500);      // previne se o sensor bloqueie se falhar
  sensor.startContinuous(50);  // Mede a cada 50ms

  Serial.println("vTask_Sensor_Distancia iniciada");


  for (;;) {

    //filtro
    distance = sensor.readRangeContinuousMillimeters();

    //distance_tmp=0.5*distance_filtered + 0.5 * distance;
    if (distance > 0.2 * distance_filtered) {

      distance_filtered = 0.9 * distance_filtered + 0.1 * distance;
    }

    distance_filtered = constrain(distance_filtered, 0, 600);  // 0 cm a 60 cm

    if (sensor.timeoutOccurred()) {

      Serial.println("Sensor timeout!");

    } else {

      xQueueOverwrite(distanciaQueue, &distance_filtered);

      // Debug Serial.println("O valor do sensor passou para a queue");
    }

    // Debug
    //Serial.print("task distancia = ");
    //Serial.println(distance_filtered);
    //

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void vTask_PWM_Buzzer(void *pvParameters) {

  int distancia = 0;

  ledc_set_duty(BUZZER_MODE, BUZZER_CHANNEL, 0);
  ledc_update_duty(BUZZER_MODE, BUZZER_CHANNEL);  //segurança para o buzzer estar desligado;


  // Serial.println("vTask_PWM_Buzzer iniciada");


  for (;;) {

    if (xQueuePeek(distanciaQueue, &distancia, 10 / portTICK_PERIOD_MS)) {

      distancia = constrain(distancia, 0, 600);  // 0 cm a 60 cm

      if (distancia < 150) {  //100 = 10cm

        Serial.println("O Buzzer vai apitar");
        ledc_set_duty(BUZZER_MODE, BUZZER_CHANNEL, 127);
        ledc_update_duty(BUZZER_MODE, BUZZER_CHANNEL);

      } else {

        ledc_set_duty(BUZZER_MODE, BUZZER_CHANNEL, 0);
        ledc_update_duty(BUZZER_MODE, BUZZER_CHANNEL);
      }
    }

    // Debug
    // Serial.println("task buzzer = ");
    // Serial.println(distancia);
    //

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void vTask_LDR_ADC(void *pvParameters) {


  analogReadResolution(ADC_RESOLUTION);  //Define a resolução do ADC;

  luminosidade msg;  //ponteiro para a estrutura Lumisidade (dados, valor LDR e o escuro);


  // -------- Auto-calibração: --------       (para adpatar ao ambiente onde estamos, usar apenas para testes)

  int amostras = 50;
  int soma = 0;

  for (int i = 0; i < amostras; i++) {

    soma += analogRead(LDR);
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }

  int luz_ambiente = soma / amostras;

  msg.escuro = luz_ambiente * 0.6;  // 60% da luz do local atual
  // ----------------------------------


  //Serial.println("vTask_Luminosidade_ADC iniciada");


  for (;;) {

    int LDR_value = analogRead(LDR);
    msg.LDR_value = LDR_value;
    xQueueOverwrite(luminosidadeQueue, &msg);

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void vTask_Farois(void *pvparameters) {

  bool luzesLigadas = false;
  luminosidade msg;  //ponteiro para a estrutura Lumisidade (dados, valor LDR e o escuro);

  pinMode(LED_farois, OUTPUT);
  digitalWrite(LED_farois, LOW);  //segurança para as luzes começarem desligadas;


  //Serial.println("vTask_Farois iniciada");


  for (;;) {

    if (xQueueReceive(luminosidadeQueue, &msg, portMAX_DELAY)) {
      pixel_luz.begin();
      int escuro = msg.escuro;
      int LDR_value = msg.LDR_value;

      if (LDR_value < escuro) {

        pixel_luz.clear();
        pixel_luz.show();
        pixel_luz.setBrightness(200);

        for (int i = 0; i < NUMPIXELS; i++) {
          pixel_luz.setPixelColor(i, pixel_luz.Color(255, 255, 255));  // branco
        }
        pixel_luz.show();

        if (!luzesLigadas) {

          luzesLigadas = true;
          xQueueOverwrite(LuzesLigadasQueue, &luzesLigadas);
        }

      } else {

        pixel_luz.clear();
        pixel_luz.show();

        if (luzesLigadas) {

          luzesLigadas = false;
          xQueueOverwrite(LuzesLigadasQueue, &luzesLigadas);
        }
      }
    }
  }
}

void vTask_Inicializacao_Display(void *pvParameters) {

  // -------- Variavéis do display ---------
  const int leftX = 5, leftY = 20, leftW = 30, leftH = 82;
  const int centerX = 40, centerY = 20, centerW = 30, centerH = 82;

  const int chassisX = 100, chassisY = 25;
  const int chassisW = 20, chassisH = 70;

  const int highBeamX = 135, highBeamY = 105;
  const int highBeamR = 8;

  const char *texto = "Engenharia\nAutomovel";

  int16_t x1, y1;  //Variaveis, a usar para centrar o texto;
  uint16_t w, h;   //Variaveis, a usar para centrar o texto;
  // ------------------------------------

  // Serial.println("vTask_Inicialização_do_Display iniciada");

  tft.fillScreen(ST77XX_BLUE);     // fundo azul
  tft.setTextColor(ST77XX_WHITE);  // texto branco
  tft.setTextSize(2);              // tamanho maior

  // -------- Valores para centrar o texto ---------  (display 160x128 em landscape);
  tft.getTextBounds(texto, 0, 0, &x1, &y1, &w, &h);  // Calcula o espaço que o texto ocupa;

  int textX = (160 - w) / 2;  //posição em x;
  int textY = (128 - h) / 2;  //posição em y;
  // ----------------------------------------------

  tft.setCursor(textX, textY - 20);
  tft.println("Engenharia");
  tft.setCursor(textX + 10, textY);
  tft.println("Automovel");
  tft.setTextSize(1);
  tft.setCursor(textX + 15, textY + 30);
  tft.println("Joao Santos");
  tft.setCursor(textX + 15, textY + 40);
  tft.println("Afonso Fernandes");
  tft.setCursor(10, textY + 65);
  tft.println("2025/2026");

  vTaskDelay(3000 / portTICK_PERIOD_MS);  // Mostrar durante 2s;

  tft.fillScreen(ST77XX_BLACK);  // Limpar ecrã;
  vTaskDelay(500 / portTICK_PERIOD_MS);

  // -------- Elementos fixos no Display ---------
  drawOutlineRect(leftX, leftY, leftW, leftH + 1, COLOR_OUTLINE, tft);      //barra dutycycle
  drawOutlineRect(centerX, centerY, leftW, leftH + 1, COLOR_OUTLINE, tft);  //barra proximidade
  drawChassis(chassisX, chassisY, chassisW, chassisH, tft);
  drawPercentBar(leftX, leftY, leftW, leftH, 255, COLOR_LEFT_FILL, tft);
  drawCenteredText(leftX, leftY, leftW, leftH, "100%", tft);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setCursor(11, leftY + leftH + 5);
  tft.println("Thr");
  tft.setCursor(43, leftY + leftH + 5);
  tft.println("Prox");
  // ----------------------------------------------

  //Serial.println("vTask_Inicializacao_do_Display prestes a levar delete");

  vTaskDelay(500 / portTICK_PERIOD_MS);

  xTaskCreatePinnedToCore(vTask_Display, "Task Display", 4096, NULL, 1, NULL, 1);  //Cria a função que vai, atualizar o Display;

  vTaskDelete(NULL);  //Elemina-se a ela própria;
}

void vTask_Display(void *pvParameters) {

  // -------- Variavéis para lementos fixos do display ---------
  const int leftX = 5, leftY = 20, leftW = 30, leftH = 82;
  const int centerX = 40, centerY = 20, centerW = 30, centerH = 82;
  const int chassisX = 100, chassisY = 25;
  const int chassisW = 20, chassisH = 70;
  const int highBeamX = 135, highBeamY = 105;
  const int highBeamR = 8;
  // -----------------------------------------------------------

  // -------- Variáveis para as coisas que variam---------
  int dist = 0;       //Distancia;
  int prevDist = -1;  //Distancia anterior;

  int duty = 0;       //dutycycle
  int prevduty = -1;  //dutycycle anterior

  motordatastruct urso;     //Ponteiro para a estrutura com os dados do motor (motordatastruct);
  Steeringdatastruct bear;  //Ponteiro para a estrutura com os dados da direção (Steeringdatastruct);

  bool msg;                           //Ponteiro para o o valor que etá dentro da Queue que diz se as luzes estão ligadas (LuzesLigadasQueue);
  bool prevEstado_das_luzes = false;  //Para usar na função das luzes, guarda o estado anterior;

  QuatroPiscas_display msg_quatro_piscas;  //Ponteiro para a estrutura QuatroPiscas_display, onde está o estado dos piscas;
  bool Estado_quatro_piscas = false;       //Variavel para saber o estado do quatro piscas;

  char ThrText[10];   //Strings pra escrever Thr;
  char distText[10];  //Strings pra escrever Dist;
  // -----------------------------------------------------------

  //Serial.println("vTask_Display iniciada");

  drawFarois(highBeamX, highBeamY, highBeamR, false, tft);  //Faz o farol das luzes, e desliga-o;

  for (;;) {

    // ------------------ Barra central com a distância ---------------------
    if (xQueueReceive(distanciaQueue, &dist, 10 / portTICK_PERIOD_MS)) {

      if (dist != prevDist) {

        int dist_pct = map(dist, 0, 600, 100, 0);  //Converte distância p/ percentagem (0–2 m → 0–100%)
        dist_pct = constrain(dist_pct, 0, 100);    //Restrige o valor entre 0 e 100;

        drawPercentBar(centerX, centerY, centerW, centerH, dist_pct, COLOR_CENTER_FILL, tft);

        sprintf(distText, "%dcm", dist / 10);                                 // convert mm → cm
        drawCenteredText(centerX, centerY, centerW, centerH, distText, tft);  //Inserir valores dentro da barra

        prevDist = dist;
      }
    }
    // ----------------------------------------------------------------

    // ------------------------- Farois -------------------------------
    if (xQueueReceive(LuzesLigadasQueue, &msg, 10 / portTICK_PERIOD_MS)) {

      bool Estado_das_luzes = msg;

      if (Estado_das_luzes != prevEstado_das_luzes) {

        drawFarois(highBeamX, highBeamY, highBeamR, Estado_das_luzes, tft);
        prevEstado_das_luzes = Estado_das_luzes;
      }
    }
    // -----------------------------------------------------------

    // ------------------ Barra da Esquerda, Throttle ------------
    if (xQueuePeek(motordata, &urso, 10 / portTICK_PERIOD_MS)) {

      duty = map(urso.throttle, 0, 1023, 0, 100);  // o mesmo que - carga_atual*100% a dividir pela resolução
      duty = constrain(duty, 0, 100);

      if (duty != prevduty) {

        drawPercentBar(leftX, leftY, leftW, leftH, duty, COLOR_LEFT_FILL, tft);  //Faz a barra com o valor de do Duty;
        sprintf(ThrText, "%d%%", duty);                                          //Escreve o valor em percentagem;
        drawCenteredText(leftX, leftY, leftW, leftH, ThrText, tft);              //Centra o valor do duty por cima da barra;

        prevduty = duty;
      }
      // ---------------------------------------------------------------

      // ------------------- Setas de direções -------------------------
      if (xQueuePeek(motordata, &urso, 10 / portTICK_PERIOD_MS) && (xQueuePeek(Steeringdata, &bear, 10 / portTICK_PERIOD_MS))) {

        eraseArrows(chassisX, chassisY, chassisW, chassisH, tft);

        if (urso.throttle > 0 && urso.throttle > urso.brake) {

          drawArrowFront(chassisX, chassisY, chassisW, chassisH, tft);
        }

        else if (urso.brake > 0 && urso.throttle < urso.brake) {

          drawArrowBack(chassisX, chassisY, chassisW, chassisH, tft);
        }

        if (bear.xvalue > 15) {

          drawArrowRight(chassisX, chassisY, chassisW, chassisH, tft);

        } else if (bear.xvalue < -15) {

          drawArrowLeft(chassisX, chassisY, chassisW, chassisH, tft);
        }
      }
    }
    // ---------------------------------------------------------------

    // ------------------------ Quatro piscas ------------------------
    if (xQueueReceive(Quatro_Piscas_DisplayQueue, &msg_quatro_piscas, 10 / portTICK_PERIOD_MS)) {

      Estado_quatro_piscas = msg_quatro_piscas.Estado;
    }

    if (Estado_quatro_piscas && (millis() % 1000 < 500)) {

      drawEmergencyTriangle(146, 5, true, tft);  //Ativa a função que faz o triangulo do pisca;

    } else {

      drawEmergencyTriangle(146, 5, false, tft);  //Desativa a função que faz o triangulo do pisca;
    }
    // ----------------------------------------------------------------

    vTaskDelay(30 / portTICK_PERIOD_MS);
  }
}

void vTask_launch_control(void *pvParameters) {

  for (;;) {

    if (xSemaphoreTake(semaforo_launchcontrol, portMAX_DELAY) == pdPASS) {

      vTaskSuspend(xHandleSteering);
      vTaskSuspend(xHandleMotor);

      Serial.println("Evento detectado! A direção está direita, durante 10s.");

      ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL, PULSE_MID);
      ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL);

      Serial.println("Já consertei a direção");

      ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR_CHANNEL, NEUTRO_DUTY);
      ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR_CHANNEL);

      vTaskDelay(pdMS_TO_TICKS(10000));

      vTaskResume(xHandleSteering);
      vTaskResume(xHandleMotor);

      Serial.println("As outras tarefas voltaram, a funcionar.");
    }
  }
}

void vTask_Bluetooth(void *pvParameters) {

  Steeringdatastruct mg1;
  motordatastruct mg2;

  bool lastButtonStateX = false;
  bool lastButtonStateTri = false;
  uint8_t lastDpadState = 0;

  drivemodestruct mg3;

  mg3.mode_1 = false;
  mg3.mode_2 = false;

  for (;;) {

    BP32.update();

    for (auto ctl : myControllers) {
      if (ctl && ctl->isConnected() && ctl->hasData()) {

        ctl->setColorLED(227, 127, 42);  //rosa salmão
        // Leitura dos inputs
        int x = ctl->axisX();
        uint16_t front = ctl->throttle();
        uint16_t back = ctl->brake();

        // Estados atuais
        bool currentButtonStateX = (ctl->buttons() & BUTTON_A);
        bool currentButtonStateTri = (ctl->buttons() & BUTTON_Y);
        uint8_t currentDpadState = ctl->dpad();

        xQueuePeek(drivemode, &mg3, 0);

        // --- LÓGICA BOTÃO X (Mode 1) ---
        if (currentButtonStateX && !lastButtonStateX) {

          mg3.mode_1 = !mg3.mode_1;  // Inverte apenas o mode_1
          ctl->playDualRumble(0, 250, 0x80, 0x40);
        }

        lastButtonStateX = currentButtonStateX;

        // --- LÓGICA BOTÃO TRIÂNGULO (Mode 2) ---
        if (currentButtonStateTri && !lastButtonStateTri) {

          mg3.mode_2 = !mg3.mode_2;  // Inverte apenas o mode_2
          ctl->playDualRumble(0, 250, 0x80, 0x40);
        }

        if (mg3.mode_2) {

          // Deteta o clique (borda de subida) para CIMA
          if ((currentDpadState & DPAD_UP) && !(lastDpadState & DPAD_UP)) {
            mg3.speed += 50;  // Agora só soma 500 UMA VEZ por clique
            Serial.printf("Velocidade aumentada: %d\n", mg3.speed);

          }
          // Deteta o clique para BAIXO
          else if ((currentDpadState & DPAD_DOWN) && !(lastDpadState & DPAD_DOWN)) {

            mg3.speed -= 100;
            Serial.printf("Velocidade diminuída: %d\n", mg3.speed);
          }
        }

        lastDpadState = currentDpadState;  // Atualiza o estado para o próximo ciclo
        lastButtonStateTri = currentButtonStateTri;


        // --- Envio de Direção e Motores ---
        mg1 = { x };
        mg2 = { front, back };

        xQueueOverwrite(drivemode, &mg3);
        xQueueOverwrite(Steeringdata, &mg1);
        xQueueOverwrite(motordata, &mg2);
      }
    }

    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}


// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------- Interrupts -----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


void IRAM_ATTR ISR_Botao_esquerdo() {  //Pisca da esquerda;

  PiscaEvento_t ev = PISCA_ESQUERDO;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (piscasQueue != NULL) {

    xQueueSendFromISR(piscasQueue, &ev, &xHigherPriorityTaskWoken);
  }
  if (xHigherPriorityTaskWoken == pdTRUE) {

    portYIELD_FROM_ISR();
  }
}

void IRAM_ATTR ISR_Botao_direito() {  //Pisca da direita;

  PiscaEvento_t ev = PISCA_DIREITO;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (piscasQueue != NULL) {

    xQueueSendFromISR(piscasQueue, &ev, &xHigherPriorityTaskWoken);
  }
  if (xHigherPriorityTaskWoken == pdTRUE) {

    portYIELD_FROM_ISR();
  }
}

void IRAM_ATTR ISR_quatro_piscas() {  //Quatro piscas;

  PiscaEvento_t ev = PISCA_QUATRO;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (piscasQueue != NULL) {

    xQueueSendFromISR(piscasQueue, &ev, &xHigherPriorityTaskWoken);
  }
  if (xHigherPriorityTaskWoken == pdTRUE) {

    portYIELD_FROM_ISR();
  }
}

void IRAM_ATTR ISR_launch_control() {  //Interrupção para ativar o launch control;

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  xSemaphoreGiveFromISR(semaforo_launchcontrol, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken) {

    portYIELD_FROM_ISR();
  }
}


// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------- Tarefas sobre piscas ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


void vTask_pisca_esquerdo(void *pvParameters) {

  pixel_led_esquerdo.begin();
  pixel_led_esquerdo.clear();
  pixel_led_esquerdo.show();

  for (;;) {
    // Tenta pegar o semáforo. Se conseguir, executa o ciclo de pisca
    if (xSemaphoreTake(semaforo_piscas, 0) == pdTRUE) {

      // --- INÍCIO DO EFEITO ESCADA ---
      pixel_led_esquerdo.clear();
      for (int i = 0; i < NUMPIXELS; i++) {

        pixel_led_esquerdo.setPixelColor(i, pixel_led_esquerdo.Color(255, 45, 0));  // Teu laranja calibrado
        pixel_led_esquerdo.show();
        vTaskDelay(40 / portTICK_PERIOD_MS);
      }

      // Apaga tudo antes de reiniciar ou terminar
      pixel_led_esquerdo.clear();
      pixel_led_esquerdo.show();

      // Pequena pausa entre piscadas (intervalo do relé)
      vTaskDelay(300 / portTICK_PERIOD_MS);
      // --- FIM DO EFEITO ESCADA ---

    } else {
      // Se o semáforo não estiver disponível, limpa e apaga a tarefa
      pixel_led_esquerdo.clear();
      pixel_led_esquerdo.show();

      pisca_esquerdo_TaskHandle = NULL;
      esquerdo_ativo = false;

      vTaskDelete(NULL);
    }
  }
}

void vTask_pisca_direito(void *pvParameters) {


  pixel_led_direito.begin();
  pixel_led_direito.clear();
  pixel_led_direito.show();

  for (;;) {
    if (xSemaphoreTake(semaforo_piscas, 0) == pdTRUE) {

      // --- INÍCIO DO EFEITO ESCADA ---
      pixel_led_direito.clear();
      for (int i = 0; i < NUMPIXELS; i++) {

        pixel_led_direito.setPixelColor(i, pixel_led_direito.Color(255, 45, 0));  // Teu laranja calibrado
        pixel_led_direito.show();
        vTaskDelay(40 / portTICK_PERIOD_MS);
      }


      // Apaga tudo antes de reiniciar ou terminar
      pixel_led_direito.clear();
      pixel_led_direito.show();

      // Pequena pausa entre piscadas (intervalo do relé)
      vTaskDelay(300 / portTICK_PERIOD_MS);
      // --- FIM DO EFEITO ESCADA ---
    } else {

      // Se o semáforo não estiver disponível, limpa e apaga a tarefa
      pixel_led_direito.clear();
      pixel_led_direito.show();
      pisca_direito_TaskHandle = NULL;
      direito_ativo = false;

      //Serial.println("Tarefa prestes a levar delete");

      vTaskDelete(NULL);
    }
  }
}

void vTask_quatro_piscas(void *pvParameters) {
  // Inicialização básica (Opcional se já feito no setup)
  pixel_led_direito.begin();
  pixel_led_esquerdo.begin();

  for (;;) {
    // --- INÍCIO DO EFEITO ESCADA ---
    // 1. Limpa ambos os lados para começar o movimento
    pixel_led_direito.clear();
    pixel_led_esquerdo.clear();

    for (int i = 0; i < NUMPIXELS; i++) {
      uint32_t corLaranja = pixel_led_direito.Color(255, 45, 0);

      // Define a cor no LED 'i' de ambos os lados
      pixel_led_direito.setPixelColor(i, corLaranja);
      pixel_led_esquerdo.setPixelColor(i, corLaranja);

      // Envia os dados para as fitas
      pixel_led_direito.show();
      pixel_led_esquerdo.show();

      // Velocidade da "subida" da escada
      vTaskDelay(pdMS_TO_TICKS(40));
    }

    // 2. Apaga tudo após completar a subida (efeito de pisca real)
    pixel_led_direito.clear();
    pixel_led_esquerdo.clear();
    pixel_led_direito.show();
    pixel_led_esquerdo.show();

    // 3. Tempo que os LEDs ficam apagados até o próximo ciclo
    vTaskDelay(pdMS_TO_TICKS(300));

    // O seu Pisca Manager provavelmente usa vTaskDelete(handle) de fora,
    // então este loop for(;;) é interrompido externamente.
  }
}

void vTask_piscas_emergencia(void *pvParameters) {
  // Inicialização básica (Opcional se já feito no setup)
  pixel_led_direito.begin();
  pixel_led_esquerdo.begin();

  for (;;) {
    // --- INÍCIO DO EFEITO ESCADA ---
    // 1. Limpa ambos os lados para começar o movimento
    pixel_led_direito.clear();
    pixel_led_esquerdo.clear();

    for (int i = 0; i < NUMPIXELS; i++) {
      uint32_t corLaranja = pixel_led_direito.Color(255, 45, 0);

      // Define a cor no LED 'i' de ambos os lados
      pixel_led_direito.setPixelColor(i, corLaranja);
      pixel_led_esquerdo.setPixelColor(i, corLaranja);

      // Envia os dados para as fitas
      pixel_led_direito.show();
      pixel_led_esquerdo.show();

      // Velocidade da "subida" da escada
      vTaskDelay(pdMS_TO_TICKS(25));
    }

    // 2. Apaga tudo após completar a subida (efeito de pisca real)
    pixel_led_direito.clear();
    pixel_led_esquerdo.clear();
    pixel_led_direito.show();
    pixel_led_esquerdo.show();

    // 3. Tempo que os LEDs ficam apagados até o próximo ciclo
    vTaskDelay(pdMS_TO_TICKS(150));

    // O seu Pisca Manager provavelmente usa vTaskDelete(handle) de fora,
    // então este loop for(;;) é interrompido externamente.
  }
}

void vTask_PiscasManager(void *pvParameters) {

  PiscaEvento_t ev;
  QuatroPiscas_display msg;
  bool QuatroPiscas_display = false;

  for (;;) {

    if (xQueueReceive(piscasQueue, &ev, portMAX_DELAY) == pdTRUE) {

      switch (ev) {
        case PISCA_ESQUERDO:

          if (!travagem_emergencia_ativo) {
            Semaforo_give();

            if (!esquerdo_ativo) {

              if (pisca_direito_TaskHandle != NULL) {  // != NULL  =  a correr

                vTaskDelete(pisca_direito_TaskHandle);
                pisca_direito_TaskHandle = NULL;
                direito_ativo = false;
                pixel_led_direito.clear();
                pixel_led_direito.show();
              }
              if (pisca_esquerdo_TaskHandle == NULL) {

                if (xTaskCreatePinnedToCore(vTask_pisca_esquerdo, "Pisca_Esquerdo", 2048, NULL, 1, &pisca_esquerdo_TaskHandle, 1) == pdPASS) {

                  esquerdo_ativo = true;
                } else {
                  pisca_esquerdo_TaskHandle = NULL;
                }
              } else {
                esquerdo_ativo = true;  // defensive
              }
            } else {  // ele faz o delete de esquerdo_ativo = 1 (if do inicio)

              if (pisca_esquerdo_TaskHandle != NULL) {  // != NULL  =  a correr
                vTaskDelete(pisca_esquerdo_TaskHandle);
                pisca_esquerdo_TaskHandle = NULL;
              }
              esquerdo_ativo = false;
              pixel_led_esquerdo.clear();
              pixel_led_esquerdo.show();
            }
          }
          break;

        case PISCA_DIREITO:

          if (!travagem_emergencia_ativo) {

            Semaforo_give();

            if (!direito_ativo) {
              if (pisca_esquerdo_TaskHandle != NULL) {

                vTaskDelete(pisca_esquerdo_TaskHandle);
                pisca_esquerdo_TaskHandle = NULL;
                esquerdo_ativo = false;
                pixel_led_esquerdo.clear();
                pixel_led_esquerdo.show();
              }
              if (pisca_direito_TaskHandle == NULL) {

                if (xTaskCreatePinnedToCore(vTask_pisca_direito, "Pisca_Direito", 2048, NULL, 1, &pisca_direito_TaskHandle, 1) == pdPASS) {
                  direito_ativo = true;
                } else {
                  pisca_direito_TaskHandle = NULL;
                }
              } else {
                direito_ativo = true;
              }
            } else {

              if (pisca_direito_TaskHandle != NULL) {
                vTaskDelete(pisca_direito_TaskHandle);
                pisca_direito_TaskHandle = NULL;
              }
              direito_ativo = false;
              pixel_led_direito.clear();
              pixel_led_direito.show();
            }
          }
          break;

        case PISCA_QUATRO:

          if (!travagem_emergencia_ativo) {
            if (!quatro_ativo) {

              if (pisca_esquerdo_TaskHandle != NULL) {

                vTaskDelete(pisca_esquerdo_TaskHandle);
                pisca_esquerdo_TaskHandle = NULL;
                pixel_led_esquerdo.clear();
                pixel_led_esquerdo.show();
              }
              if (pisca_direito_TaskHandle != NULL) {

                vTaskDelete(pisca_direito_TaskHandle);
                pisca_direito_TaskHandle = NULL;
                direito_ativo = false;
                pixel_led_direito.clear();
                pixel_led_direito.show();
              }
              if (quatro_piscas_TaskHandle == NULL) {

                if (xTaskCreatePinnedToCore(vTask_quatro_piscas, "Quatro_Piscas", 2048, NULL, 2, &quatro_piscas_TaskHandle, 1) == pdPASS) {

                  QuatroPiscas_display = true;
                  msg.Estado = QuatroPiscas_display;
                  msg.cor = COLOR_YELLOW;
                  xQueueSend(Quatro_Piscas_DisplayQueue, &msg, 0);

                  quatro_ativo = true;
                } else {

                  quatro_piscas_TaskHandle = NULL;
                }
              } else {

                quatro_ativo = true;
              }
            } else {
              if (quatro_piscas_TaskHandle != NULL) {

                vTaskDelete(quatro_piscas_TaskHandle);
                quatro_piscas_TaskHandle = NULL;

                QuatroPiscas_display = false;
                msg.Estado = QuatroPiscas_display;
                msg.cor = COLOR_Background;
                xQueueSend(Quatro_Piscas_DisplayQueue, &msg, 0);
              }

              quatro_ativo = false;
              pixel_led_direito.clear();
              pixel_led_direito.show();
              pixel_led_esquerdo.clear();
              pixel_led_esquerdo.show();
            }
          }
          break;

        case TRAVAGEM_EMERGENCIA:
          if (!travagem_emergencia_ativo) {

            if (pisca_esquerdo_TaskHandle != NULL) {

              vTaskDelete(pisca_esquerdo_TaskHandle);
              pisca_esquerdo_TaskHandle = NULL;
              pixel_led_esquerdo.clear();
              pixel_led_esquerdo.show();
            }
            if (pisca_direito_TaskHandle != NULL) {

              vTaskDelete(pisca_direito_TaskHandle);
              pisca_direito_TaskHandle = NULL;
              direito_ativo = false;
              pixel_led_direito.clear();
              pixel_led_direito.show();
            }
            if (quatro_piscas_TaskHandle != NULL) {

              vTaskDelete(quatro_piscas_TaskHandle);
              quatro_piscas_TaskHandle = NULL;
              quatro_ativo = false;
              pixel_led_direito.clear();
              pixel_led_direito.show();
              pixel_led_esquerdo.clear();
              pixel_led_esquerdo.show();
            }
            if (quatro_piscas_TaskHandle == NULL) {

              if (xTaskCreatePinnedToCore(vTask_piscas_emergencia, "Travagem_emergencia", 2048, NULL, 2, &travagem_emergencia_TaskHandle, 1) == pdPASS) {

                QuatroPiscas_display = true;
                msg.Estado = QuatroPiscas_display;
                msg.cor = COLOR_CENTER_FILL;
                xQueueSend(Quatro_Piscas_DisplayQueue, &msg, 0);  //talvez overwrite

                travagem_emergencia_ativo = true;
              } else {
                travagem_emergencia_TaskHandle = NULL;
              }
            } else {
              travagem_emergencia_ativo = true;
            }
          } else {
            if (travagem_emergencia_TaskHandle != NULL) {
              vTaskDelete(travagem_emergencia_TaskHandle);
              travagem_emergencia_TaskHandle = NULL;

              QuatroPiscas_display = false;
              msg.Estado = QuatroPiscas_display;
              msg.cor = COLOR_Background;
              xQueueSend(Quatro_Piscas_DisplayQueue, &msg, 0);  //talvez overwrite
            }
            travagem_emergencia_ativo = false;
            pixel_led_direito.clear();
            pixel_led_direito.show();
            pixel_led_esquerdo.clear();
            pixel_led_esquerdo.show();
          }
          break;
      }
    }
  }
}

static void Semaforo_give(void) {  //Semáforo contador para os piscas;

  static portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;

  for (int i = 0; i < 10; i++) {

    xSemaphoreGiveFromISR(semaforo_piscas, &xHigherPriorityTaskWoken);
  }

  if (xHigherPriorityTaskWoken == pdTRUE) {

    portYIELD_FROM_ISR();
  }
}


void loop() {
  vTaskDelete(NULL);
}
