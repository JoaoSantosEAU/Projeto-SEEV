# Projeto-SEEV

/*
Nome João Santos - 2231131
Nome Afonso Fernandes - 2232108
IPLEIRIA - Instituto Politécnico de Leiria
ESTG - Escola Superior de Tecnologia e Gestão
LEAU - Licenciatura em Engenharia Automóvel
SEEV - Sistemas Elétricos e Eletrónicos de Veículos

TP1: Neste projeto pretende-se simular o controlo remoto de um veículo movido a motores DC apartir de um dispositivo bluetooth,
     como também simular o comportamento de um sensor de luminosidade e um sensor de proximidade que por sua vês gera um sinal 
     sonoro se detetar alta proximidade com um corpo. 
	 Link para o vídeo: https://youtu.be/Rq4K3AZypbY

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

