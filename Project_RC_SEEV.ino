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
#define LED_front 13
#define LED_rear 2

// -------- Funções PWM --------
const int freq = 10000;    // 10 kHz
const int resolution = 8;  // 8 bits -> 0-255

//Defenir Tasks
void vTask_PWM_Direcao(void *pvParameters);
void vTask_PWM_Tracao(void *pvParameters);
void vTask_Sensor_Distancia(void *pvParameters);
void vTask_PWM_Buzzer(void *pvParameters);
void vTask_Luminosidade_ADC(void *pvParameters);
void vTask_Display(void *pvParameters);
void vTask_Bluetooth(void *pvParameters);

// -------- Periféricos --------
VL53L0X sensor;
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// -------- Queues --------
QueueHandle_t distanciaQueue, controloQueue;

// -------- Structs --------
typedef struct {
    int sentido_de_tracao;
    int direcao;
    int carga;
} Controlo;

// ---------- Display Mutex ----------
SemaphoreHandle_t displayMutex;


// ---------- Flag do sensor ----------
volatile bool sensorInitialized = false;




// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------- Funções Display: --------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------




// -------- Variavéis display: --------   tft.drawRect(x, y, width, height, color);
const int leftX = 5, leftY = 20, leftW = 30, leftH = 82;
const int centerX = 40, centerY = 20, centerW = 30, centerH = 82;

const int chassisX = 100, chassisY = 25;
const int chassisW = 20,  chassisH = 70;

const int highBeamX = 135, highBeamY = 105;
const int highBeamR = 8;


// funções display (chat)
void drawOutlineRect(int x,int y,int w,int h,uint16_t color){
    tft.drawRect(x,y,w,h,color);
    }

void drawPercentBar(int x,int y,int w,int h,int pct,uint16_t color){
    if(pct<0) pct=0;
    if(pct>100) pct=100;

    //limpeza interna da barra
    tft.fillRect(x+1,y+1,w-2,h-2,COLOR_Background);

    int filled=((h-2)*pct)/100;
    tft.fillRect(x+1, y + h - filled, w-2, filled, color);
    }


//Inserir valores dentro das barras
void drawCenteredText(int x, int y, int w, int h, const char* text) {
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


void drawChassis(){
    tft.drawRect(chassisX, chassisY, chassisW, chassisH, COLOR_CHASSIS);

    // rodas
    tft.fillRect(chassisX-10, chassisY+5, 8, 15, COLOR_CHASSIS);                //frente esquerda
    tft.fillRect(chassisX+chassisW+2, chassisY+5, 8, 15, COLOR_CHASSIS);        //frente direita
    tft.fillRect(chassisX-10, chassisY+chassisH-20, 8, 15, COLOR_CHASSIS);
    tft.fillRect(chassisX+chassisW+2,chassisY+chassisH-20,8,15,COLOR_CHASSIS);
    }


void eraseArrows(){
    tft.fillRect(chassisX-25,chassisY-25,chassisW+50,chassisH+50,COLOR_Background);
    drawChassis();
    }


void drawArrowFront(){                        //tft.fillTriangle(x1, y1, x2, y2, x3, y3, color);
    int mx=chassisX+chassisW/2;  //meio do retangulo do chassis na horizontal
    tft.fillTriangle(mx, chassisY-18, mx-6, chassisY-4, mx+6, chassisY-4, COLOR_ARROW);
    }


void drawArrowBack(){
    int mx=chassisX+chassisW/2;
    tft.fillTriangle(mx, chassisY+chassisH+18, mx-6, chassisY+chassisH+4, mx+6, chassisY+chassisH+4, COLOR_ARROW);
    }


void drawArrowLeft(){
    int my=chassisY+chassisH/2;   //meio do retangulo do chassis na vertical
    tft.fillTriangle(chassisX-18,my, chassisX-4,my-6, chassisX-4,my+6, COLOR_ARROW);
    }
void drawArrowRight(){
    int my=chassisY+chassisH/2;
    tft.fillTriangle(chassisX+chassisW+18,my, chassisX+chassisW+4,my-6, chassisX+chassisW+4,my+6, COLOR_ARROW);
    }


void drawHighBeams(bool on){
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


	// Inicializa o Bluetooth Classic para Dabble
	Dabble.begin("ESP32_DABBLE");
	Serial.println("Dabble iniciado! Abre a app e conecta via Bluetooth.");


	//Criar Queues
    distanciaQueue    =  xQueueCreate(1 , sizeof(int)       );
    controloQueue     =  xQueueCreate(1 , sizeof(Controlo)  );


    // Mutex para o Display
    displayMutex = xSemaphoreCreateMutex();


	//Inicializar o I2C no ESP32 nos pinos 21 (SDA), 22 (SCL)
	Wire.begin(21, 22);


	// Iniciar o Sensor de Distancia:
	if (!sensor.init()) {
	  Serial.println("Failed to detect and initialize VL53L0X!");
	  while (1) { vTaskDelay(1000 / portTICK_PERIOD_MS); }
	  }
	  sensor.setTimeout(500);      // previne se o sensor bloqueie se falhar
	  sensor.startContinuous(50);  // Mede a cada 50ms
	  sensorInitialized = true;    // Inicialização iniciada


    //ADC:
	analogReadResolution(ADC_RESOLUTION);


	//Display:
	tft.initR(INITR_BLACKTAB);
	tft.setRotation(1);
	tft.fillScreen(ST77XX_BLACK);


	//Tasks
	xTaskCreatePinnedToCore(vTask_PWM_Direcao,      "PWM_Direção",           4096, NULL, 5, NULL, 1);
	xTaskCreatePinnedToCore(vTask_PWM_Tracao,       "Task PWM_Tração",       4096, NULL, 4, NULL, 1);
	xTaskCreatePinnedToCore(vTask_Sensor_Distancia, "Task Sensor_Distancia", 4096, NULL, 3, NULL, 1);
	xTaskCreatePinnedToCore(vTask_PWM_Buzzer,       "Task PWM_Buzzer",       2048, NULL, 3, NULL, 1);
	xTaskCreatePinnedToCore(vTask_Luminosidade_ADC, "Task ADC",              2048, NULL, 2, NULL, 1);
	xTaskCreatePinnedToCore(vTask_Display,          "Task Display",          4096, NULL, 1, NULL, 1);
	xTaskCreatePinnedToCore(vTask_Bluetooth,        "Task Bluetooth",        4096, NULL, 1, NULL, 0);     //Bluetooth core 0


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

    Serial.print("vTask_PWM_Direcao iniciada");


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
	int sentido_de_tracao = 0;
	int carga = 0;
	Controlo msg;

	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	ledcAttach(PWM_trac , freq, resolution);
	ledcWrite(PWM_trac , 0);

    Serial.print("vTask_PWM_Tracao iniciada");


	while(1){

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
	vTaskDelay(30 / portTICK_PERIOD_MS);
  }
}


void vTask_Sensor_Distancia(void *pvParameters) {
	int distance = 0, distance_filtered=0, distance_tmp=0;

	// Segurança caso o sensor não inicialize
	while (!sensorInitialized) {
	     vTaskDelay(10 / portTICK_PERIOD_MS);
	}


	Serial.print("vTask_Sensor_Distancia iniciada");


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
	Serial.println("task distancia = ");
	Serial.println(distance_filtered);
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

    Serial.print("vTask_PWM_Buzzer iniciada");


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
	Serial.println("task buzzer = ");
	Serial.println(distancia);
	//


	vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}


void vTask_Luminosidade_ADC(void *pvParameters) {
	int escuro = 200;

	pinMode(LED_rear , OUTPUT);
	pinMode(LED_front, OUTPUT);
	digitalWrite(IN1_buzzer, LOW);
	digitalWrite(IN2_buzzer, LOW);

	Serial.print("vTask_Luminosidade_ADC iniciada");


	while(1){
	int LDR_value = analogRead(LDR);

	if (LDR_value < escuro)	{
	  digitalWrite (LED_front, HIGH);
	  digitalWrite (LED_rear, HIGH);
	  Serial.println("LED is ON - Its dark");      //debug
	  } else {
	  digitalWrite (LED_front, LOW);
	  digitalWrite (LED_rear, LOW);
	  Serial.println("LED is OFF - Its bright");   //debug
	  }

	vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}


void vTask_Display(void *pvParameters) {
	int dist = 0;       //distancia
	int prevDist = -1;  //distancia anterior
	int duty = 0;       //dutycycle
	int prevduty = -1;  //dutycycle anterior
	Controlo prevctrl = {999,999,999};  //struct de controlo do motor anterior
	Controlo ctrl;                      //struct de controlo do motor
	bool prevHighBeam = false;
	char dutyText[10];
	char distText[10];
	int distUpdateCounter = 0;


	// desenhar elementos fixos no início
	if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(200))) {
	drawOutlineRect(leftX, leftY, leftW, leftH+1, COLOR_OUTLINE);         //barra dutycycle
	drawOutlineRect(centerX, centerY, leftW, leftH+1, COLOR_OUTLINE);   //barra proximidade
	drawChassis();
	tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(1);
	tft.setCursor(15, leftY+leftH+5);
	tft.println("DC");
	tft.setCursor(43, leftY+leftH+5);
	tft.println("Prox");
	drawHighBeams(false);


	xSemaphoreGive(displayMutex);
	}

	Serial.print("vTask_Display iniciada");

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

        if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50))) {
          drawPercentBar(centerX, centerY, centerW, centerH, dist_pct, COLOR_CENTER_FILL);

          //Inserir valores dentro da barra
          sprintf(distText, "%dcm", dist / 10);  // convert mm → cm
          drawCenteredText(centerX, centerY, centerW, centerH, distText);


      xSemaphoreGive(displayMutex);
      }
	  prevDist = dist;
	  }

}


	   // ----------------------------------------------------------------
	   // ------------------- HIGH BEAMS ("máximos") ---------------------
	   // ----------------------------------------------------------------


	bool high = digitalRead(LED_front);
    if (high != prevHighBeam){
	if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50))) {
	  drawHighBeams(high);

    xSemaphoreGive(displayMutex);
    }
	prevHighBeam = high;
    }


	   // ----------------------------------------------------------------
	   // ------------------ BAR Esquerda : Duty-cycle tração ------------
	   // ----------------------------------------------------------------


	if (xQueuePeek(controloQueue, &ctrl, 10 / portTICK_PERIOD_MS)) {
		  duty = map(ctrl.carga, 175, 255, 0, 100);              //carga_atual*100% a dividir pela resolução
	  if (duty > 100 ) duty = 100;
	  if (duty < 0   ) duty = 0;

	  if (duty != prevduty){
	    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50))) {
		  drawPercentBar(leftX, leftY, leftW, leftH, duty, COLOR_LEFT_FILL);

		  //Inserir valores dentro da barra
		  sprintf(dutyText, "%d%%", duty);
		  drawCenteredText(leftX, leftY, leftW, leftH, dutyText);


		xSemaphoreGive(displayMutex);
	    }
	    prevduty = duty;
	    }


	    // ----------------------------------------------------------------
	    // ------------------- Setas de direções --------------------------
	    // ----------------------------------------------------------------


	   if (ctrl.sentido_de_tracao != prevctrl.sentido_de_tracao ||
	       ctrl.direcao           != prevctrl.direcao){

		   if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50))) {

		     eraseArrows();

		     if (ctrl.sentido_de_tracao > 0) drawArrowFront();
		     else if (ctrl.sentido_de_tracao < 0) drawArrowBack();
		     if (ctrl.direcao > 0) drawArrowRight();
		     else if (ctrl.direcao < 0) drawArrowLeft();


		 xSemaphoreGive(displayMutex);
		 }
	     prevctrl = ctrl;
	     }
    }
	  vTaskDelay(30 / portTICK_PERIOD_MS);
  }
}


void vTask_Bluetooth(void *pvParameters) {
	Controlo msg;
	int Xmove;
	int Ymove;
    int carga = 255;


	Serial.print("vTask_Bluetooth iniciada");


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
	        carga += 5;
	    } else if (GamePad.isStartPressed()) {
	        carga -= 5;
	    }
	    carga = constrain(carga, 175, 255);

	    // -------- Guardar valores --------

	    msg.direcao            = GamePad.getXaxisData();
	    msg.sentido_de_tracao  = Ymove;
	    msg.carga              = carga;

	    // Debug
	    Serial.print("[BT] X=");
	    Serial.print(msg.direcao);
	    Serial.print("  Y=");
	    Serial.println(msg.sentido_de_tracao);
	    Serial.print(" Carga=");
	    Serial.print(msg.carga);

	xQueueOverwrite(controloQueue, &msg);

	vTaskDelay(20 / portTICK_PERIOD_MS);
	}
}


void loop(){
	vTaskDelete(NULL);
}

