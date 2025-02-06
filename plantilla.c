#include <unistd.h>
#include "sdkconfig.h"
#include "driver/gpio.h"
#include <stdio.h>
#include "esp_task_wdt.h"



void app_main(void) {
    esp_task_wdt_deinit();
    int contador = 0;
    printf("Cuenta: %d \n", contador);


    while (true) {
      
    }
}
