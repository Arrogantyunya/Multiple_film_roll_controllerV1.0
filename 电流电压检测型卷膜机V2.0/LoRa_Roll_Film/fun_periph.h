#ifndef _FUN_PERIPH_H
#define _FUN_PERIPH_H

#include <Arduino.h>

/*一些外设的引脚*/
/*Some peripheral GPIO configuration*/
#define BUZZ_PIN            PC5

// #define LED1_PIN            PC0
// #define LED2_PIN            PC1
// #define LED3_PIN            PC2
// #define LED4_PIN            PC3

#define LED1_PIN            PC2
#define LED2_PIN            PC3
#define LED3_PIN            PC0
#define LED4_PIN            PC1

#define SW_FUN1             PC8 //Button1
#define SW_FUN2             PC7 //Button2

enum LED{
    RED1, RED2, GREEN1, GREEN2
};

/*LED GPIO switch*/
#define GREEN1_OFF                  (digitalWrite(LED1_PIN, LOW))
#define GREEN1_ON                   (digitalWrite(LED1_PIN, HIGH))
#define RED1_OFF                    (digitalWrite(LED2_PIN, LOW))
#define RED1_ON                     (digitalWrite(LED2_PIN, HIGH))
#define GREEN2_OFF                  (digitalWrite(LED3_PIN, LOW))
#define GREEN2_ON                   (digitalWrite(LED3_PIN, HIGH))
#define RED2_OFF                    (digitalWrite(LED4_PIN, LOW))
#define RED2_ON                     (digitalWrite(LED4_PIN, HIGH))

/*BUZZ GPIO switch*/
#define BUZZ_ON                     (digitalWrite(BUZZ_PIN, LOW))
#define BUZZ_OFF                    (digitalWrite(BUZZ_PIN, HIGH))

#define LED_NO_REGISTER             (Some_Peripheral.LED_Display(RED1, 5))
#define LED_RUNNING                 (Some_Peripheral.LED_Display(GREEN1, 10))
#define LED_RESET_ROUTE             (Some_Peripheral.LED_Display(GREEN2, 20))
#define LED_OPENING                 (Some_Peripheral.LED_Display(GREEN2, 10))
#define LED_FORCE_OPENING           (Some_Peripheral.LED_Display(GREEN2, 5))
#define LED_SELF_CHECK_ERROR        (Some_Peripheral.LED_Display(RED2, 5))
#define LED_SET_LORA_PARA_ERROR     (Some_Peripheral.LED_Display(RED2, 20))

class Some_Peripherals{
public:
    /*Configurate some functional pins*/
    void Peripheral_GPIO_Config(void);
    void LED_Display(LED which_led, unsigned char freq);
    void Stop_LED(void);
    void Start_LED(void);

    void Key_Buzz(unsigned int d); //800ms  1000 times.
    //void Alarm(unsigned int d);
};

void LED_Interrupt(void);

/*Create peripheral object*/
extern Some_Peripherals Some_Peripheral;

#endif