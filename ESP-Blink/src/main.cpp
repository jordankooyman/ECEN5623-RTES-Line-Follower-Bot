#include <Arduino.h>
#include "FreeRTOS.h"

#define G_LED_PIN (GPIO_NUM_16) // External LED - Green
#define Y_LED_PIN (GPIO_NUM_17) // External LED - Yellow
#define R_LED_PIN (GPIO_NUM_18) // External LED - Red
#define NEO_PIN (GPIO_NUM_48) // Neopixel Data Pin(?)

struct blink_params_t {
  gpio_num_t LED_Pin;
  TickType_t delay;
} blink_params_t;

struct blink_params_t green, yellow, red;

// put function declarations here:
void blink_led(void* arg);

void setup() {
  // put your setup code here, to run once:
  pinMode(G_LED_PIN, OUTPUT);
  pinMode(Y_LED_PIN, OUTPUT);
  pinMode(R_LED_PIN, OUTPUT);

  printf("Hello world!\n");
     
  green.LED_Pin = G_LED_PIN;
  green.delay = 1000 / portTICK_PERIOD_MS;
  yellow.LED_Pin = Y_LED_PIN;
  yellow.delay = 2000 / portTICK_PERIOD_MS;
  red.LED_Pin = R_LED_PIN;
  red.delay = 500 / portTICK_PERIOD_MS;

  // Start Blinking LEDs as independent Threads
  printf("Green LED On\n\r");
  xTaskCreate(blink_led, "Blink Green", 2048, (void*)&green, 2, NULL); 
      // Note: Will get stuck on task creation if StackDepth is too small
  printf("Yellow LED On\n\r");
  xTaskCreate(blink_led, "Blink Yellow", 2048, (void*)&yellow, 1, NULL);
  printf("Red LED On\n\r");
  xTaskCreate(blink_led, "Blink Red", 2048, (void*)&red, 3, NULL);
}

void loop() {
  // put your main code here, to run repeatedly:
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

 
void blink_led(void* arg)
{
  struct blink_params_t* Params;
  Params = (struct blink_params_t*)arg;
  printf("Blinking Pin %d with a delay of %d\n\r", Params->LED_Pin, (int)Params->delay);
  while (1) {
    if(gpio_set_level(Params->LED_Pin, 1) != ESP_OK)
      printf("GPIO Error\n\r");
    vTaskDelay(Params->delay);
    if(gpio_set_level(Params->LED_Pin, 0) != ESP_OK)
      printf("GPIO Error\n\r");
    vTaskDelay(Params->delay);
  }
}
 
 

     // Hard spin loop
 //        printf("LED On\n\r");
 //        if(gpio_set_level(LED_PIN, 1) != ESP_OK)
 //            printf("GPIO Error\n\r");
         
 //        printf("LED Off\n\r");
 //        if(gpio_set_level(LED_PIN, 0) != ESP_OK)
 //            printf("GPIO Error\n\r");
 //        vTaskDelay(1000 / portTICK_PERIOD_MS);
