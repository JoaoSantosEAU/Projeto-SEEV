/*
Nome João Santos - 2231131
Nome Afonso Fernandes - 2232108
IPLEIRIA - Instituto Politécnico de Leiria
ESTG - Escola Superior de Tecnologia e Gestão
LEAU - Licenciatura em Engenharia Automóvel
SEEV - Sistemas Elétricos e Eletrónicos de Veículos
*/




// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------- Bibliotecas: ------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------




#include "Arduino.h"

//SPI:
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <Adafruit_GFX.h>

//Sensor Laser:
#include <Wire.h>
 //#include <Adafruit_VL53L0X.h>
#include <VL53L0X.h>

//FreeRTOS e etc:
#include <stdio.h>
#include "freertos/portmacro.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"

//Bluetooth, app Dabble
#include <DabbleESP32.h>




// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------- Defines: ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------




// -------- L298N: --------
//Motor traseiro (tração)
#define PWM_trac  33  //EN1
#define IN1 12
#define IN2 14

//Motor Frontal (direção)
#define PWM_steer 32  //EN2
#define IN3 27
#define IN4 25

// -------- L293d: --------
//buzzer (A)
#define PWM_buzzer  4  //EN1
#define IN1_buzzer 16
#define IN2_buzzer 17

// -------- Display: --------
#define TFT_CS     5
#define TFT_DC     26
#define TFT_MOSI   23
#define TFT_SCLK   18
#define TFT_RST    -1  //3.3V
//#define TFT_VCC    3.3V
//#define TFT_BL     3.3V

// -------- Cores: --------
#define COLOR_Background   ST77XX_BLACK
#define COLOR_OUTLINE      ST77XX_BLUE
#define COLOR_LEFT_FILL    ST77XX_GREEN
#define COLOR_CENTER_FILL  ST77XX_RED
#define COLOR_CHASSIS      ST77XX_BLUE
#define COLOR_ARROW        ST77XX_GREEN
#define COLOR_LIGHT        ST77XX_CYAN

// -------- LDR + LEDS --------
#define ADC_RESOLUTION 10
#define LDR 35
#define VREF_PLUS  3.3
#define VREF_MINUS  0.0
#define LED_farois 13                //amarelos
#define LED_piscasdireita 2          //vermelhos
#define LED_piscasesquerda 19        //vermelhos

// -------- Butôes: --------
#define Botao_esquerdo 15               // botao preto  VN pin
#define Botao_direito 34               // botao vermelho


// -------- Funções PWM --------
const int freq = 10000;    // 10 kHz
const int resolution = 8;  // 8 bits -> 0-255

//Defenir Tasks
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
static void Semaforo_give( void );


// -------- Queues --------
QueueHandle_t distanciaQueue, controloQueue, piscasQueue, luminosidadeQueue, LuzesLigadasQueue;

// -------- Structs --------
typedef struct {
    int sentido_de_tracao;
    int direcao;
    int carga;
} Controlo;

typedef struct {
	int escuro;
	int LDR_value;
}luminosidade ;

// -------------------- Piscas: --------------------
                                                  //
// -------- Semáforos: --------                   //
SemaphoreHandle_t semaforo_piscas;

// -------- Task handles dos piscas: --------
TaskHandle_t pisca_esquerdo_TaskHandle  = NULL;
TaskHandle_t pisca_direito_TaskHandle   = NULL;
TaskHandle_t quatro_piscas_TaskHandle   = NULL;
TaskHandle_t piscaManager_Handle        = NULL;


typedef enum {
  PISCA_ESQUERDO,
  PISCA_DIREITO,
  PISCA_QUATRO
} PiscaEvento_t;


// -------- Estados: --------
bool esquerdo_ativo = false;
bool direito_ativo = false;
bool quatro_ativo = false;                        //
                                                  //
// -------------------------------------------------




// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------- Funções Display: --------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------




// funções display                                     tft.drawRect(x, y, width, height, color);
void drawOutlineRect(int x,int y,int w,int h,uint16_t color, Adafruit_ST7735 tft){
    tft.drawRect(x,y,w,h,color);
    }

void drawPercentBar(int x,int y,int w,int h,int pct,uint16_t color, Adafruit_ST7735 tft){
    if(pct<0) pct=0;
    if(pct>100) pct=100;

    //limpeza interna da barra
    tft.fillRect(x+1,y+1,w-2,h-2,COLOR_Background);

    int filled=((h-2)*pct)/100;
    tft.fillRect(x+1, y + h - filled, w-2, filled, color);
    }


//Inserir valores dentro das barras
void drawCenteredText(int x, int y, int w, int h, const char* text, Adafruit_ST7735 tft) {
    // Clear text area
    tft.fillRect(x+2, y+2, w-4, 10, COLOR_Background);

    // Center text
    int16_t x1, y1;
    uint16_t tw, th;
    tft.getTextBounds((char*)text, x, y, &x1, &y1, &tw, &th);

    int cx = x + (w - tw) / 2;
    int cy = y + 3;

    tft.setCursor(cx, cy);
    tft.print(text);
}


void drawChassis(int chassisX, int chassisY, int chassisW, int chassisH, Adafruit_ST7735 tft){
    tft.drawRect(chassisX, chassisY, chassisW, chassisH, COLOR_CHASSIS);

    // rodas
    tft.fillRect(chassisX-10, chassisY+5, 8, 15, COLOR_CHASSIS);                //frente esquerda
    tft.fillRect(chassisX+chassisW+2, chassisY+5, 8, 15, COLOR_CHASSIS);        //frente direita
    tft.fillRect(chassisX-10, chassisY+chassisH-20, 8, 15, COLOR_CHASSIS);
    tft.fillRect(chassisX+chassisW+2,chassisY+chassisH-20,8,15,COLOR_CHASSIS);
    }


void eraseArrows(int chassisX, int chassisY, int chassisW, int chassisH, Adafruit_ST7735 tft){
    tft.fillRect(chassisX-25,chassisY-25,chassisW+50,chassisH+50,COLOR_Background);
    drawChassis(chassisX, chassisY, chassisW, chassisH, tft);
    }


void drawArrowFront(int chassisX, int chassisY, int chassisW, int chassisH, Adafruit_ST7735 tft){                        //tft.fillTriangle(x1, y1, x2, y2, x3, y3, color);
    int mx=chassisX+chassisW/2;  //meio do retangulo do chassis na horizontal
    tft.fillTriangle(mx, chassisY-18, mx-6, chassisY-4, mx+6, chassisY-4, COLOR_ARROW);
    }


void drawArrowBack(int chassisX, int chassisY, int chassisW, int chassisH, Adafruit_ST7735 tft){
    int mx=chassisX+chassisW/2;
    tft.fillTriangle(mx, chassisY+chassisH+18, mx-6, chassisY+chassisH+4, mx+6, chassisY+chassisH+4, COLOR_ARROW);
    }


void drawArrowLeft(int chassisX, int chassisY, int chassisW, int chassisH, Adafruit_ST7735 tft){
    int my=chassisY+chassisH/2;   //meio do retangulo do chassis na vertical
    tft.fillTriangle(chassisX-18,my, chassisX-4,my-6, chassisX-4,my+6, COLOR_ARROW);
    }
void drawArrowRight(int chassisX, int chassisY, int chassisW, int chassisH, Adafruit_ST7735 tft){
    int my=chassisY+chassisH/2;
    tft.fillTriangle(chassisX+chassisW+18,my, chassisX+chassisW+4,my-6, chassisX+chassisW+4,my+6, COLOR_ARROW);
    }


void drawHighBeams(int highBeamX, int highBeamY, int highBeamR, bool on, Adafruit_ST7735 tft){
    if(on){
        tft.fillCircle(highBeamX,highBeamY,highBeamR,COLOR_LIGHT);
        tft.drawLine(highBeamX+5,highBeamY-6 , highBeamX+14 , highBeamY-12 ,COLOR_LIGHT);
        tft.drawLine(highBeamX+5,highBeamY   , highBeamX+14 , highBeamY    ,COLOR_LIGHT);
        tft.drawLine(highBeamX+5,highBeamY+6 , highBeamX+14 , highBeamY+12 ,COLOR_LIGHT);
    } else {
        tft.fillCircle(highBeamX,highBeamY,highBeamR,COLOR_Background);
        tft.drawLine(highBeamX+5,highBeamY-6 , highBeamX+14 , highBeamY-12 ,COLOR_Background);
        tft.drawLine(highBeamX+5,highBeamY   , highBeamX+14 , highBeamY    ,COLOR_Background);
        tft.drawLine(highBeamX+5,highBeamY+6 , highBeamX+14 , highBeamY+12 ,COLOR_Background);
    }
    }




// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------- Set up: -----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------




void setup(){
	Serial.begin(115200);


	// -------- Criar Queues --------
    distanciaQueue    =  xQueueCreate(1 , sizeof(int)         );
    controloQueue     =  xQueueCreate(1 , sizeof(Controlo)    );
    piscasQueue       =  xQueueCreate(10, sizeof(piscasQueue) );
    luminosidadeQueue =  xQueueCreate(1 , sizeof(luminosidade));
    LuzesLigadasQueue =  xQueueCreate(1 , sizeof(bool)        );

    // -------- Botoes das interrupções --------
    pinMode(0, INPUT_PULLUP);  // Boot button
    pinMode(Botao_esquerdo, INPUT);     // external pull-up
    pinMode(Botao_direito,  INPUT);

    // -------- Interrupções --------
    attachInterrupt(digitalPinToInterrupt(0),               &ISR_quatro_piscas ,  FALLING);
    attachInterrupt(digitalPinToInterrupt(Botao_esquerdo),  &ISR_Botao_esquerdo,  FALLING);
    attachInterrupt(digitalPinToInterrupt(Botao_direito) ,  &ISR_Botao_direito ,  FALLING);

    // -------- Semáforos --------
    semaforo_piscas = xSemaphoreCreateCounting(10, 0);


	// ---------- Tasks ----------
	xTaskCreatePinnedToCore(vTask_PWM_Direcao,            "PWM_Direção",                       4096, NULL, 5, NULL, 1);
	xTaskCreatePinnedToCore(vTask_PWM_Tracao,             "Task PWM_Tração",                   4096, NULL, 4, NULL, 1);
	xTaskCreatePinnedToCore(vTask_Sensor_Distancia,       "Task Sensor_Distancia",             4096, NULL, 3, NULL, 1);
	xTaskCreatePinnedToCore(vTask_PWM_Buzzer,             "Task PWM_Buzzer",                   2048, NULL, 3, NULL, 1);
	xTaskCreatePinnedToCore(vTask_LDR_ADC,                "Task ADC",                          2048, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(vTask_PiscasManager,          "Pisca_Manager",                     3072, NULL, 1, &piscaManager_Handle, 1),
	xTaskCreatePinnedToCore(vTask_Farois,                 "Task Farois",                       2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(vTask_Inicializacao_Display,  "Task Inicialização do Display",     4096, NULL, 6, NULL, 1);
	xTaskCreatePinnedToCore(vTask_Bluetooth,              "Task Bluetooth",                    4096, NULL, 1, NULL, 0);     //Bluetooth core 0


	Serial.println("Setup completo!");
}




// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------- Tasks: ------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------




void vTask_PWM_Direcao(void *pvParameters) {
	int sentido_de_direcao = 0;
	Controlo msg;

	pinMode(IN3, OUTPUT);
	pinMode(IN4, OUTPUT);
	ledcAttach(PWM_steer, freq, resolution);
	ledcWrite(PWM_steer , 0);

	Serial.println("vTask_PWM_Direcao iniciada");


	while(1){

	if (xQueuePeek(controloQueue, &msg, 10 / portTICK_PERIOD_MS)) {

    sentido_de_direcao = msg.direcao;

    if (sentido_de_direcao > 0){
    // Vira para DIREITA
	  digitalWrite(IN3, HIGH);
	  digitalWrite(IN4, LOW);
    } else if (sentido_de_direcao < 0){
    // Vira para ESQUERDA
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    } else {
    // Fica parado = LOW e LOW
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
    }

    if (sentido_de_direcao != 0){
    ledcWrite(PWM_steer, 240);
    } else ledcWrite(PWM_steer, 0);
	}
	vTaskDelay(30 / portTICK_PERIOD_MS);
  }
}


void vTask_PWM_Tracao(void *pvParameters) {
	int distancia = 200;
	int sentido_de_tracao = 0;
	int carga = 0;
	Controlo msg;
	bool quatro_auto_ativo = false;

	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	ledcAttach(PWM_trac , freq, resolution);
	ledcWrite(PWM_trac , 0);


	Serial.println("vTask_PWM_Tracao iniciada");


	while(1){


	// --------------------- Travagem de emergência: ----------------------------
			                                                                   //
	if (xQueuePeek(distanciaQueue, &distancia, 10 / portTICK_PERIOD_MS)) {     //
		                                                                       //
	  distancia = constrain(distancia, 0, 600);    // 0 cm a 60 cm
	}

	if (distancia < 150) {           //100 = 10cm

	  // Subir prioridade
	  if (uxTaskPriorityGet(NULL) != 6) {
	    vTaskPrioritySet(NULL, 6);
	  }

	  // Travar
	  digitalWrite(IN1, HIGH);
	  digitalWrite(IN2, HIGH);

	  //Acender 4 piscas
	  if (quatro_auto_ativo == false){
	    PiscaEvento_t ev = PISCA_QUATRO;
	    xQueueSend(piscasQueue, &ev, 0);

	    //...

	    quatro_auto_ativo = true;
		}                                                                      //
	}                                                                          // 	                                                                       //
	// --------------------------------------------------------------------------


	else {

	// Voltar à prioridade normal
	if (uxTaskPriorityGet(NULL) != 4) {
	  vTaskPrioritySet(NULL, 4);
	}

    //Desligadar 4 piscas
	if (quatro_auto_ativo == true){
	  PiscaEvento_t ev = PISCA_QUATRO;
	  xQueueSend(piscasQueue, &ev, 0);

	  //...

	  quatro_auto_ativo = false;
	}

	if (xQueuePeek(controloQueue, &msg, 10 / portTICK_PERIOD_MS)) {

	sentido_de_tracao = msg.sentido_de_tracao;
	carga = msg.carga;
	carga = constrain(carga, 175, 255);

	if (sentido_de_tracao > 0 ){
	// Roda para FRENTE
	  digitalWrite(IN1, LOW);
	  digitalWrite(IN2, HIGH);
	} else if (sentido_de_tracao < 0) {
	// Roda para TRÁS
	  digitalWrite(IN1, HIGH);
	  digitalWrite(IN2, LOW);
	} else {
    // Fica parado = LOW e LOW
	  digitalWrite(IN1, LOW);            //LOW e LOW = fica parado
	  digitalWrite(IN2, LOW);
	}

	if (sentido_de_tracao != 0){
	ledcWrite(PWM_trac, carga);
	} else ledcWrite(PWM_trac, 0);

	}
	}
	//Serial.println(uxTaskPriorityGet(NULL));
	vTaskDelay(30 / portTICK_PERIOD_MS);
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
		  for(;;){Serial.println("Restart sensor");}
		 }

		//Inicializar sensor de distância
		sensor.setTimeout(500);      // previne se o sensor bloqueie se falhar
		sensor.startContinuous(50);  // Mede a cada 50ms

		Serial.println("vTask_Sensor_Distancia iniciada");


	while(1){

    //filtro
	distance = sensor.readRangeContinuousMillimeters();
	//distance_tmp=0.5*distance_filtered + 0.5 * distance;
	if ( distance > 0.2*distance_filtered )
	  distance_filtered=0.9*distance_filtered + 0.1 * distance;

	distance_filtered = constrain(distance_filtered, 0, 600);    // 0 cm a 60 cm

	if (sensor.timeoutOccurred()) {
	  Serial.println("Sensor timeout!");
    } else {
      xQueueOverwrite(distanciaQueue, &distance_filtered);
	}

	// Debug
	//Serial.println("task distancia = ");
	//Serial.println(distance_filtered);
    //

	  vTaskDelay(50 / portTICK_PERIOD_MS);
	}
}


void vTask_PWM_Buzzer(void *pvParameters) {
    int distancia = 0;

    pinMode(IN1_buzzer, OUTPUT);
    pinMode(IN2_buzzer, OUTPUT);
    ledcAttach(PWM_buzzer,freq, resolution);
    ledcWrite(PWM_buzzer , 0);

    Serial.println("vTask_PWM_Buzzer iniciada");


	while(1){

	if (xQueuePeek(distanciaQueue, &distancia, 10 / portTICK_PERIOD_MS)) {

	  distancia = constrain(distancia, 0, 600);    // 0 cm a 60 cm


	  if (distancia < 150){           //100 = 10cm
		ledcWrite(PWM_buzzer, 255);
		digitalWrite(IN1_buzzer, HIGH);
		digitalWrite(IN2_buzzer, LOW);

	} else {
		digitalWrite(IN1_buzzer, LOW);
		digitalWrite(IN2_buzzer, LOW);
		ledcWrite(PWM_buzzer, 0);
	    }
	}

	// Debug
	//Serial.println("task buzzer = ");
	//Serial.println(distancia);
	//


	vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}


void vTask_LDR_ADC(void *pvParameters) {

	//ADC:
	analogReadResolution(ADC_RESOLUTION);

	luminosidade msg;

	// -------- Auto-calibração: --------       (usar para testes)
	int amostras = 50;
	int soma = 0;
	for (int i = 0; i < amostras; i++) {
	   soma += analogRead(LDR);
	   vTaskDelay(20 / portTICK_PERIOD_MS);
	}
	int luz_ambiente = soma / amostras;
	msg.escuro = luz_ambiente * 0.6;   // 60% da luz do local atual
	// ----------------------------------


	Serial.println("vTask_Luminosidade_ADC iniciada");


	while(1){

	int LDR_value = analogRead(LDR);
	msg.LDR_value = LDR_value;

	xQueueOverwrite(luminosidadeQueue, &msg);


	vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}


void vTask_Farois (void *pvparameters) {

	luminosidade msg;
	bool luzesLigadas = false;

	pinMode(LED_farois, OUTPUT);
	digitalWrite(LED_farois, LOW);


	Serial.println("vTask_Farois iniciada");


	while(1){

	if (xQueueReceive(luminosidadeQueue, &msg, portMAX_DELAY)) {

	  int escuro    = msg.escuro;
	  int LDR_value = msg.LDR_value;

	  if (LDR_value < escuro)	{
	    digitalWrite (LED_farois, HIGH);

	    if(!luzesLigadas){
	    luzesLigadas = true;
	    xQueueOverwrite(LuzesLigadasQueue, &luzesLigadas);
	    }
	    //Serial.println("LED is ON - Its dark");      //debug
	} else {
	    digitalWrite (LED_farois, LOW);
	    if(luzesLigadas){
	    luzesLigadas = false;
	    xQueueOverwrite(LuzesLigadasQueue, &luzesLigadas);
	    //Serial.println("LED is OFF - Its bright");   //debug
	    }
	  }
    }
    }
}


void vTask_Inicializacao_Display(void *pvParameters) {        //falta confirmar com o resto

	Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

	// -------- Variavéis display: --------
	const int leftX = 5, leftY = 20, leftW = 30, leftH = 82;
	const int centerX = 40, centerY = 20, centerW = 30, centerH = 82;

	const int chassisX = 100, chassisY = 25;
	const int chassisW = 20, chassisH = 70;

	const int highBeamX = 135, highBeamY = 105;
	const int highBeamR = 8;


	//Inicializar Display:
	tft.initR(INITR_BLACKTAB);
	tft.setRotation(1);


	Serial.println("vTask_Inicialização_do_Display iniciada");


    tft.fillScreen(ST77XX_BLUE);          // fundo azul
    tft.setTextColor(ST77XX_WHITE);       // texto branco
    tft.setTextSize(2);                   // tamanho maior

    const char *texto = "Engenharia\nAutomovel";

    // Centragem manual (display 160x128 em landscape)
    int16_t x1, y1;
    uint16_t w, h;

    tft.getTextBounds(texto, 0, 0, &x1, &y1, &w, &h);

    int textX = (160 - w) / 2;
    int textY = (128 - h) / 2;

    tft.setCursor(textX , textY - 20);
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

    // Mostrar durante 2s
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    // Limpar
    tft.fillScreen(ST77XX_BLACK);
    vTaskDelay(500 / portTICK_PERIOD_MS);


	// desenhar elementos fixos no início
	drawOutlineRect(leftX, leftY, leftW, leftH + 1, COLOR_OUTLINE, tft);      //barra dutycycle
	drawOutlineRect(centerX, centerY, leftW, leftH + 1, COLOR_OUTLINE, tft);  //barra proximidade
	drawChassis(chassisX, chassisY, chassisW, chassisH, tft);
	drawPercentBar(leftX, leftY, leftW, leftH, 255, COLOR_LEFT_FILL, tft);
	drawCenteredText(leftX, leftY, leftW, leftH, "100%", tft);
	tft.setTextColor(ST77XX_WHITE);
	tft.setTextSize(1);
	tft.setCursor(15, leftY + leftH + 5);
	tft.println("DC");
	tft.setCursor(43, leftY + leftH + 5);
	tft.println("Prox");
	drawHighBeams(highBeamX, highBeamY, highBeamR, false, tft);


	Serial.println("vTask_Inicializacao_do_Display prestes a levar delete");


	vTaskDelay(500 / portTICK_PERIOD_MS);

	// Criar task vTask_Display
	xTaskCreatePinnedToCore(vTask_Display, "Task Display", 4096, NULL, 1, NULL, 1);

	vTaskDelete(NULL);
}


void vTask_Display(void *pvParameters) {

	Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

	// -------- Variavéis display: --------
	const int leftX = 5, leftY = 20, leftW = 30, leftH = 82;
	const int centerX = 40, centerY = 20, centerW = 30, centerH = 82;

	const int chassisX = 100, chassisY = 25;
	const int chassisW = 20,  chassisH = 70;

	const int highBeamX = 135, highBeamY = 105;
	const int highBeamR = 8;

	int dist = 0;       //distancia
	int prevDist = -1;  //distancia anterior
	int duty = 0;       //dutycycle
	int prevduty = -1;  //dutycycle anterior
	Controlo prevctrl = {999,999,999};  //struct de controlo do motor anterior
	Controlo ctrl;                      //struct de controlo do motor
	bool msg;
	bool prevEstado_das_luzes = false;
	char dutyText[10];
	char distText[10];
	int distUpdateCounter = 0;


	Serial.println("vTask_Display iniciada");


	while(1){


        // ----------------------------------------------------------------
		// ------------------ BAR central : Distância ---------------------
		// ----------------------------------------------------------------


	if (xQueueReceive(distanciaQueue, &dist , 10 / portTICK_PERIOD_MS)) {

	  if (dist != prevDist){

		//Converte distância p/ percentagem (0–2 m → 0–100%)
		int dist_pct = map(dist, 0, 600, 100, 0);
		dist_pct = constrain(dist_pct, 0, 100);        // = if(dist_pct < 0)   dist_pct = 0;
	                                                   //   if(dist_pct > 100) dist_pct = 100;

          drawPercentBar(centerX, centerY, centerW, centerH, dist_pct, COLOR_CENTER_FILL, tft);

          //Inserir valores dentro da barra
          sprintf(distText, "%dcm", dist / 10);  // convert mm → cm
          drawCenteredText(centerX, centerY, centerW, centerH, distText, tft);


	  prevDist = dist;
	  }

}


	   // ----------------------------------------------------------------
	   // ------------------- HIGH BEAMS ("máximos") ---------------------
	   // ----------------------------------------------------------------


    //     -----------  ----------- está mal  ----------- -----------
	if (xQueueReceive(LuzesLigadasQueue, &msg , 10 / portTICK_PERIOD_MS)) {
	bool Estado_das_luzes = msg;
    if (Estado_das_luzes != prevEstado_das_luzes){
	  drawHighBeams(highBeamX, highBeamY, highBeamR, Estado_das_luzes, tft);

	  prevEstado_das_luzes = Estado_das_luzes;
    }
	}
    //     -----------  ----------- está mal  ----------- -----------


	   // ----------------------------------------------------------------
	   // ------------------ BAR Esquerda : Duty-cycle tração ------------
	   // ----------------------------------------------------------------


	if (xQueuePeek(controloQueue, &ctrl, 10 / portTICK_PERIOD_MS)) {
		  duty = map(ctrl.carga, 175, 255, 0, 100);              // o mesmo que - carga_atual*100% a dividir pela resolução
		  duty = constrain(duty, 0, 100);

	  if (duty != prevduty){
		  drawPercentBar(leftX, leftY, leftW, leftH, duty, COLOR_LEFT_FILL, tft);

		  //Inserir valores dentro da barra
		  sprintf(dutyText, "%d%%", duty);
		  drawCenteredText(leftX, leftY, leftW, leftH, dutyText, tft);


	    prevduty = duty;
	    }


	    // ----------------------------------------------------------------
	    // ------------------- Setas de direções --------------------------
	    // ----------------------------------------------------------------


	   if (ctrl.sentido_de_tracao != prevctrl.sentido_de_tracao ||
	       ctrl.direcao           != prevctrl.direcao){


		     eraseArrows(chassisX, chassisY, chassisW, chassisH, tft);

		     if      (ctrl.sentido_de_tracao > 0)   drawArrowFront(chassisX, chassisY, chassisW, chassisH, tft);
		     else if (ctrl.sentido_de_tracao < 0)   drawArrowBack(chassisX, chassisY, chassisW, chassisH, tft);
		     if      (ctrl.direcao > 0)             drawArrowRight(chassisX, chassisY, chassisW, chassisH, tft);
		     else if (ctrl.direcao < 0)             drawArrowLeft(chassisX, chassisY, chassisW, chassisH, tft);


	     prevctrl = ctrl;
	     }
    }
	  vTaskDelay(30 / portTICK_PERIOD_MS);
  }
}


void vTask_Bluetooth(void *pvParameters) {

	// Inicializa o Bluetooth Classic para Dabble
	Dabble.begin("ESP32_DABBLE");
	Serial.println("Dabble iniciado! Abre a app e conecta via Bluetooth.");

	Controlo msg;
	int Xmove;
	int Ymove;
    int carga = 255;


    Serial.println("vTask_Bluetooth iniciada");


	while (1) {

		Dabble.processInput();

		// -------- Andar para a frente ou para trás --------

		if (GamePad.isCrossPressed()) {
		  Ymove = +100;
	    }
	    else if (GamePad.isCirclePressed()) {
		  Ymove = -100;
	    }
		else {
		  Ymove = 0;
	    }

	    // -------- Ajustar carga (Duty-cycle) --------

	    if (GamePad.isSelectPressed()) {
	        carga += 3;
	    } else if (GamePad.isStartPressed()) {
	        carga -= 3;
	    }
	    carga = constrain(carga, 175, 255);

	    // -------- Guardar valores --------

	    msg.direcao            = GamePad.getXaxisData();
	    msg.sentido_de_tracao  = Ymove;
	    msg.carga              = carga;

	    // Debug
	    //Serial.print("[BT] X=");
	    //Serial.print(msg.direcao);
	    //Serial.print("  Y=");
	    //Serial.println(msg.sentido_de_tracao);
	    //Serial.print(" Carga=");
	    //Serial.print(msg.carga);

	xQueueOverwrite(controloQueue, &msg);

	vTaskDelay(20 / portTICK_PERIOD_MS);
	}
}




// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------- Piscas: -----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------




// -------------------------------- Interrupts: --------------------------------

// --------------------- Esquerdo: --------------------------
                                                           //
void IRAM_ATTR ISR_Botao_esquerdo() {                      //


	PiscaEvento_t ev = PISCA_ESQUERDO;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (piscasQueue != NULL) {
		xQueueSendFromISR(piscasQueue, &ev, &xHigherPriorityTaskWoken);
	}
	if (xHigherPriorityTaskWoken == pdTRUE) {
	  portYIELD_FROM_ISR();
	  //vPortYield();         // não usar porque se não dá core 1 panic'ed
	}													   //
}														   //
// ----------------------------------------------------------


// --------------------- Direito: ---------------------------
                                                           //
void IRAM_ATTR ISR_Botao_direito() {					   //

	PiscaEvento_t ev = PISCA_DIREITO;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (piscasQueue != NULL) {
	  xQueueSendFromISR(piscasQueue, &ev, &xHigherPriorityTaskWoken);
	}
	if (xHigherPriorityTaskWoken == pdTRUE) {
	  portYIELD_FROM_ISR();
	  //vPortYield();
	}													   //
}														   //
// ----------------------------------------------------------


// --------------------- Quatro: ----------------------------
														   //
void IRAM_ATTR ISR_quatro_piscas() {				       //

	PiscaEvento_t ev = PISCA_QUATRO;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (piscasQueue != NULL) {
	  xQueueSendFromISR(piscasQueue, &ev, &xHigherPriorityTaskWoken);
	}
	if (xHigherPriorityTaskWoken == pdTRUE) {
	  portYIELD_FROM_ISR();
	  //vPortYield();
	}													   //
}														   //
// ----------------------------------------------------------




// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------- Tasks Piscas: -----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------




void vTask_pisca_esquerdo(void *pvParameters) {

    pinMode(LED_piscasesquerda, OUTPUT);
    digitalWrite(LED_piscasesquerda, LOW);

    for (;;) {
        if (xSemaphoreTake(semaforo_piscas, 0) == pdTRUE) {
            digitalWrite(LED_piscasesquerda, HIGH);
            vTaskDelay(250 / portTICK_PERIOD_MS);
            digitalWrite(LED_piscasesquerda, LOW);
            vTaskDelay(250 / portTICK_PERIOD_MS);
        } else {
            digitalWrite(LED_piscasesquerda, LOW);
            pisca_esquerdo_TaskHandle = NULL;
            esquerdo_ativo = false;
            Serial.println("Tarefa prestes a levar delete");
            vTaskDelete(NULL);
        }
    }
}


void vTask_pisca_direito(void *pvParameters) {

    pinMode(LED_piscasdireita, OUTPUT);
    digitalWrite(LED_piscasdireita, LOW);

    for (;;) {
        if (xSemaphoreTake(semaforo_piscas, 0) == pdTRUE) {
            digitalWrite(LED_piscasdireita, HIGH);
            vTaskDelay(250 / portTICK_PERIOD_MS);
            digitalWrite(LED_piscasdireita, LOW);
            vTaskDelay(250 / portTICK_PERIOD_MS);
        } else {
            digitalWrite(LED_piscasdireita, LOW);
            pisca_direito_TaskHandle = NULL;
            direito_ativo = false;
            Serial.println("Tarefa prestes a levar delete");
            vTaskDelete(NULL);
        }
    }
}


void vTask_quatro_piscas(void *pvParameters) {

    pinMode(LED_piscasdireita, OUTPUT);
    pinMode(LED_piscasesquerda, OUTPUT);
    digitalWrite(LED_piscasdireita, LOW);
    digitalWrite(LED_piscasesquerda, LOW);

    for (;;) {
        digitalWrite(LED_piscasdireita, HIGH);
        digitalWrite(LED_piscasesquerda, HIGH);
        vTaskDelay(250 / portTICK_PERIOD_MS);
        digitalWrite(LED_piscasdireita, LOW);
        digitalWrite(LED_piscasesquerda, LOW);
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}




// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------- Manager dos piscas: --------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------




void vTask_PiscasManager(void *pvParameters) {

	PiscaEvento_t ev;
    for (;;) {
        if (xQueueReceive(piscasQueue, &ev, portMAX_DELAY) == pdTRUE) {

            switch (ev) {
                case PISCA_ESQUERDO:

				    Semaforo_give();

                    if (!esquerdo_ativo) {

                        if (pisca_direito_TaskHandle != NULL) {          // != NULL  =  a correr
                            vTaskDelete(pisca_direito_TaskHandle);
                            pisca_direito_TaskHandle = NULL;
                            direito_ativo = false;
                            digitalWrite(LED_piscasdireita, LOW);
                        }
                        if (pisca_esquerdo_TaskHandle == NULL) {
                            if (xTaskCreatePinnedToCore(vTask_pisca_esquerdo, "Pisca_Esquerdo", 2048, NULL, 1, &pisca_esquerdo_TaskHandle, 1) == pdPASS) {
                            	esquerdo_ativo = true;
                            } else {
                                pisca_esquerdo_TaskHandle = NULL;
                            }
                        } else {
                        	esquerdo_ativo = true; // defensive
                        }
                    } else {                                 // ele faz o delete de esquerdo_ativo = 1 (if do inicio)
                        if (pisca_esquerdo_TaskHandle != NULL) {        // != NULL  =  a correr
                            vTaskDelete(pisca_esquerdo_TaskHandle);
                            pisca_esquerdo_TaskHandle = NULL;
                        }
                        esquerdo_ativo = false;
                        digitalWrite(LED_piscasesquerda, LOW);
                    }
                    break;

                case PISCA_DIREITO:

                	Semaforo_give();

                    if (!direito_ativo) {
                        if (pisca_esquerdo_TaskHandle != NULL) {
                            vTaskDelete(pisca_esquerdo_TaskHandle);
                            pisca_esquerdo_TaskHandle = NULL;
                            esquerdo_ativo = false;
                            digitalWrite(LED_piscasesquerda, LOW);
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
                        digitalWrite(LED_piscasdireita, LOW);
                    }
                    break;

                case PISCA_QUATRO:
                    if (!quatro_ativo) {

                        if (pisca_esquerdo_TaskHandle != NULL) {
                            vTaskDelete(pisca_esquerdo_TaskHandle);
                            pisca_esquerdo_TaskHandle = NULL;
                            esquerdo_ativo = false;
                            digitalWrite(LED_piscasesquerda, LOW);
                        }
                        if (pisca_direito_TaskHandle != NULL) {
                            vTaskDelete(pisca_direito_TaskHandle);
                            pisca_direito_TaskHandle = NULL;
                            direito_ativo = false;
                            digitalWrite(LED_piscasdireita, LOW);
                        }
                        if (quatro_piscas_TaskHandle == NULL) {
                            if (xTaskCreatePinnedToCore(vTask_quatro_piscas, "Quatro_Piscas", 2048, NULL, 2, &quatro_piscas_TaskHandle, 1) == pdPASS) {

                            	//...

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
                        }
                        quatro_ativo = false;
                        digitalWrite(LED_piscasdireita, LOW);
                        digitalWrite(LED_piscasesquerda, LOW);
                    }
                    break;
            }
        }
    }
}




// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------- Gives: -----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------




static void Semaforo_give( void ){

static portBASE_TYPE xHigherPriorityTaskWoken;

  xHigherPriorityTaskWoken = pdFALSE;


  for (int i = 0; i < 10; i++) {
  	 xSemaphoreGiveFromISR( semaforo_piscas, &xHigherPriorityTaskWoken );
  }


  if ( xHigherPriorityTaskWoken == pdTRUE ) {
	portYIELD_FROM_ISR();
  }
}





void loop(){
	vTaskDelete(NULL);
}


