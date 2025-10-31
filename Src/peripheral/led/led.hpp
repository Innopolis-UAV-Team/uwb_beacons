#include "main.h"

#define BLUE_LED_Pin LED2_Pin
#define GREEN_LED_Pin LED3_Pin
#define RED_LED_Pin LED1_Pin

#define BLUE_LED_GPIO_Port LED2_GPIO_Port
#define GREEN_LED_GPIO_Port LED3_GPIO_Port
#define RED_LED_GPIO_Port LED1_GPIO_Port

enum LED_COLOR {
    BLUE,
    GREEN,
    RED,
    ALL
};

class LED {
 public:
    void init();
    void on(LED_COLOR color);
    void off(LED_COLOR color);

 private:
    void set_color(LED_COLOR color);
    void blink(LED_COLOR color, uint32_t delay_ms);
};
