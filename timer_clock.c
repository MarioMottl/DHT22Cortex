/*
File : timer_clock.c
*/
#include "timer_clock.h"
#include "dht22.h" // comm_read()

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "misc.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ==================== LOCAL DEFINES ====================
// Convenient flag check
#define HAS_FLAG_SET(var, flag) ((var & flag) != 0)

// Editing flags
#define EDIT_FLAG_SELECT 0x80
#define EDIT_FLAG_ADJUST 0x40
#define EDIT_FLAG_HRS 0x08
#define EDIT_FLAG_MIN 0x04
#define EDIT_FLAG_SEC 0x02
#define EDIT_FLAG_MS  0x01

//Edit states
#define EDIT_STATE_IDLE 0x00
#define EDIT_STATE_SELECT_HRS 0x88
#define EDIT_STATE_SELECT_MIN 0x84
#define EDIT_STATE_SELECT_SEC 0x82
#define EDIT_STATE_SELECT_MS  0x81
#define EDIT_STATE_ADJUST_HRS 0x48
#define EDIT_STATE_ADJUST_MIN 0x44
#define EDIT_STATE_ADJUST_SEC 0x42
#define EDIT_STATE_ADJUST_MS  0x41
#define EDIT_STATE_SELECTING HAS_FLAG_SET(editing_flags, EDIT_FLAG_SELECT)
#define EDIT_STATE_ADJUSTING HAS_FLAG_SET(editing_flags, EDIT_FLAG_ADJUST)

// time between toggling in ms
#define blink_period 5

// ==================== local functions ====================
static void sysclock_config(void);
static void TIM4_config(void);
static void incremental_config(void);
static void tick2str_cond(char* buf, uint64_t tick, int8_t draw_edge, int8_t draw_hrs, int8_t draw_min, int8_t draw_sec, int8_t draw_ms);
static void lcd_put_tick_cond(uint64_t tick, int8_t draw_edge, int8_t draw_hrs, int8_t draw_min, int8_t draw_sec, int8_t draw_ms);
static int8_t new_position(int8_t cur_pos, uint8_t input);
static void rotary_button_pressed(void);
static void rotary_rotate_right(void);
static void rotary_rotate_left(void);

// ==================== local variables ====================
volatile static uint64_t tick = 0;
volatile static uint64_t editing_tick = 0;
volatile static uint8_t editing_flags = EDIT_STATE_IDLE; // select adjust * *  hrs min sec ms
volatile static int8_t blink_effect_off = 0;
volatile static int8_t rotary_position = 0; 

// ==================== function implementations ====================
uint64_t get_tick(void)
{
    return tick;
}
void set_tick(uint64_t new_tick)
{
    tick = new_tick;
}
void init_clock(void)
{
    sysclock_config();
    incremental_config();
    TIM4_config();
    return;
}
void lcd_put_tick(void)
{
    lcd_put_tick_cond(HAS_FLAG_SET(editing_flags, EDIT_FLAG_SELECT) ? editing_tick : tick, 
         HAS_FLAG_SET(editing_flags, EDIT_FLAG_SELECT), // draw border when selecting
        !HAS_FLAG_SET(editing_flags, EDIT_FLAG_HRS) || blink_effect_off, // draw hrs when : not editing hrs or else when blink effect is off
        !HAS_FLAG_SET(editing_flags, EDIT_FLAG_MIN) || blink_effect_off, // draw hrs when : not editing min or else when blink effect is off
        !HAS_FLAG_SET(editing_flags, EDIT_FLAG_SEC) || blink_effect_off,
        !HAS_FLAG_SET(editing_flags, EDIT_FLAG_MS ) || blink_effect_off );
}
int8_t is_editing(void)
{
    return editing_flags != EDIT_STATE_IDLE;
}
// ==================== Interrupt handler ====================
void TIM4_IRQHandler(void) // runs every 100ms
{
    //TIM4->SR &= ~0x01;
    TIM_ClearFlag(TIM4, TIM_FLAG_Update); // Pending bit löschen
    tick++;
    if((tick % blink_period) == 0) blink_effect_off = !blink_effect_off; // Blinkflag togglen
    if((tick % read_period) == 0) 
    {
			    SDA = 0;
					for (int i = 0;i<=640;i++){wait_10us();}
					SDA = 1;
					comm_read();
    }
}
void EXTI9_5_IRQHandler(void) // runs on exti ,7,8,9
{
    if(EXTI_GetITStatus(EXTI_Line7) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line7);
        //usart1_send_char('B');
        if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7) == 0) // Button pressed
            rotary_button_pressed();
    }
    if((EXTI_GetITStatus(EXTI_Line8) != RESET) || (EXTI_GetITStatus(EXTI_Line9) != RESET))
    {
        EXTI_ClearITPendingBit(EXTI_Line8);
        EXTI_ClearITPendingBit(EXTI_Line9);
        uint8_t input = (GPIO_ReadInputData(GPIOC) & 0x0300 /*0b 0000 0011 0000 0000*/) >> 8;
        rotary_position = new_position(rotary_position, input);
    }
}
// ==================== local function implementations ====================
static void sysclock_config(void)
{
    FLASH->ACR = 0x12; //Set Flash latency (2 waitstates)
    RCC->CR |= RCC_CR_HSEON; //HSE on
    while ((RCC->CR & RCC_CR_HSERDY) == 0); // Wait for HSERDY=1 (HSE is ready)
    RCC->CFGR |= RCC_CFGR_PLLMULL9; // 9 mal 8 = 72 MHz (SYSCLK)
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6; //ADCCLK = SYSCLK/6 (APB2 PRESCALER=1)
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; // PCLK1(APB1)=SYSCLK/2 (HCLK=SYSCLK)
    RCC->CFGR |= RCC_CFGR_PLLSRC; //PLL = SYSCLK,HCLK=SYSCLK,da AHB PRESCALER=1)
    RCC->CR |= RCC_CR_PLLON; //PLL on
    while ((RCC->CR & RCC_CR_PLLRDY) == 0); // Wait for PLLRDY=1 (PLL is ready)
    //RCC->CFGR |= RCC_CFGR_SW_HSE; //HSE = Systemclock
    RCC->CFGR |= RCC_CFGR_SW_PLL; //PLL = Systemclock
    while ((RCC->CFGR & RCC_CFGR_SWS_PLL) == 0);
    /* Wait till SYSCLK is stabilized (depending on selected clock) */
    while ((RCC->CFGR & RCC_CFGR_SWS) != ((RCC->CFGR<<2) & RCC_CFGR_SWS));
    // end of stm32_ClockSetup
    RCC->BDCR |=RCC_BDCR_LSEON; //32kHz für RTC siehe AN2821 Reference Manual 448ff/1072
}
static void TIM4_config(void)
{
    TIM_TimeBaseInitTypeDef timer;
    TIM_OCInitTypeDef outputcompare;
    NVIC_InitTypeDef nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_DeInit(TIM4);
    timer.TIM_ClockDivision = TIM_CKD_DIV1;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    /* T_INT = 13,8ns, Annahme: Presc = 135 ---> Auto Reload Wert = 52801
    (=0xCE41) --> 0.1s Update Event*/
    //Auto-Reload Wert = Maximaler Zaehlerstand des Upcounters
    timer.TIM_Period = 0xCE41;
    timer.TIM_Prescaler = 135;
    timer.TIM_RepetitionCounter = 0; // Repetition deaktivieren

    TIM_TimeBaseInit(TIM4, &timer);

    /* initialize Output Compare Channels */
    outputcompare.TIM_OCMode = TIM_OCMode_PWM1;
    outputcompare.TIM_OutputState = TIM_OutputState_Enable;
    outputcompare.TIM_Pulse = (timer.TIM_Period)/2;
    /* Compare Value*/
    outputcompare.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC3Init(TIM4, &outputcompare); /* save initialisation */

    TIM_ITConfig(TIM4, TIM_DIER_UIE, ENABLE); // Update Interrupt enable

    nvic.NVIC_IRQChannel = TIM4_IRQn;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 4;

    NVIC_Init(&nvic);
    TIM_Cmd(TIM4, ENABLE); // Counter-enable bit setzten
}
static void incremental_config(void)
{ // C7: Button; C8,C9: rotate switches
    GPIO_InitTypeDef gpio;
    EXTI_InitTypeDef exti;
    NVIC_InitTypeDef nvic;

    // enable GPIOC
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    // Init GPIO Struct
    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode = GPIO_Mode_IPU;

    // Set PC7 as input
    gpio.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOC, &gpio);
    // Set PC8 as input
    gpio.GPIO_Pin = GPIO_Pin_8;
    GPIO_Init(GPIOC, &gpio);
    // Set PC9 as input
    gpio.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOC, &gpio);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    EXTI_DeInit();

    // init exti line 7
    EXTI_StructInit(&exti);
    exti.EXTI_Line = EXTI_Line7;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Falling;
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource7);
    EXTI_ClearITPendingBit(EXTI_Line7);

    // init exti line 8
    exti.EXTI_Line = EXTI_Line8;
    exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_Init(&exti);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource8);
    EXTI_ClearITPendingBit(EXTI_Line8);

    // init exti line 9
    exti.EXTI_Line = EXTI_Line9;
    EXTI_Init(&exti);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource9);
    EXTI_ClearITPendingBit(EXTI_Line9);

    // init nvic
    nvic.NVIC_IRQChannel = EXTI9_5_IRQn;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 5;
    NVIC_Init(&nvic);
}
static void tick2str_cond(char* buf, uint64_t tick, int8_t draw_edge, int8_t draw_hrs, int8_t draw_min, int8_t draw_sec, int8_t draw_ms)
{
    int ms, sec, min, hrs;

    ms  = ((tick) % 10);
    sec = ((tick / (10)) % 60);
    min = ((tick / (10 * 60)) % 60);
    hrs = ((tick / (10 * 60 * 60)) % 24);

    //          " - hh:mm:ss:z - "
    strcpy(buf, " -   :  :  :  - ");
    if(draw_hrs != 0)
    {
        buf[ 3] = (hrs / 10) + '0';
        buf[ 4] = (hrs % 10) + '0';
    }
    if(draw_min != 0)
    {
        buf[ 6] = (min / 10) + '0';
        buf[ 7] = (min % 10) + '0';
    }
    if(draw_sec != 0)
    {
        buf[ 9] = (sec / 10) + '0';
        buf[10] = (sec % 10) + '0';
    }
    if(draw_ms  != 0)
    {
        buf[12] = ms + '0';
    }
    if(draw_edge != 0)
    {
        buf[ 0] = '=';
        buf[ 1] = '>';
        buf[14] = '<';
        buf[15] = '=';
    }
}
static void lcd_put_tick_cond(uint64_t tick, int8_t draw_edge, int8_t draw_hrs, int8_t draw_min, int8_t draw_sec, int8_t draw_ms)
{
    char buf[17];
    tick2str_cond(buf, is_editing() ? editing_tick : tick, draw_edge, draw_hrs, draw_min, draw_sec, draw_ms);
    lcd_set_cursor(0,0);
    lcd_put_string(buf);
}
static int8_t new_position(int8_t cur_pos, uint8_t input)
{
    static const int8_t data[4][7] = {
        {-4,-2, 0, 0, 0, 2, 4}, // 00
        {-3,-1,-1,-1, 1, 3, 3}, // 01
        {-3,-3,-1, 1, 1, 1, 3}, // 10
        {-2,-2,-2, 0, 2, 2, 2}  // 11
    };
    int8_t retval = data[input%4][(cur_pos+3)%7];
    if(retval == 4) // Right turn
    {
        rotary_rotate_right();
        return 0;
    }
    if(retval == -4) // Left turn
    {
        rotary_rotate_left();
        return 0;
    }
    return retval;
}
static void rotary_button_pressed(void)
{
    if(editing_flags == EDIT_STATE_IDLE)
    {
        editing_flags = EDIT_STATE_SELECT_HRS;
        editing_tick = tick;
    }
    else if(HAS_FLAG_SET(editing_flags, EDIT_FLAG_SELECT))
    {
        editing_flags &= ~EDIT_FLAG_SELECT;
        editing_flags |= EDIT_FLAG_ADJUST;
    }
    else if(HAS_FLAG_SET(editing_flags, EDIT_FLAG_ADJUST))
    {
        editing_flags = EDIT_STATE_IDLE;
        tick = editing_tick;
    }
}
static void rotary_rotate_right(void)
{
    if(EDIT_STATE_SELECTING)
    {
        switch(editing_flags)
        {
        case EDIT_STATE_SELECT_HRS:
            editing_flags = EDIT_STATE_SELECT_MIN;
            break;
        case EDIT_STATE_SELECT_MIN:
            editing_flags = EDIT_STATE_SELECT_SEC;
            break;
        case EDIT_STATE_SELECT_SEC:
            editing_flags = EDIT_STATE_SELECT_MS;
            break;
        //case EDIT_STATE_SELECT_MS:
    	default:
            editing_flags = EDIT_STATE_SELECT_HRS;
            break;
        }
    }
    if(EDIT_STATE_ADJUSTING)
    {
        switch(editing_flags)
        {
        case EDIT_STATE_ADJUST_HRS:
            if(((editing_tick/(10*60*60)) % 24) == 23)
                editing_tick -= 10*60*60*24;
            editing_tick += 10*60*60;
            break;
        case EDIT_STATE_ADJUST_MIN:
            if(((editing_tick/(10*60)) % 60) == 59)
                editing_tick -= 10*60*60;
            editing_tick += 10*60;
            break;
        case EDIT_STATE_ADJUST_SEC:
            if(((editing_tick/10) % 60) == 59)
                editing_tick -= 10*60;
            editing_tick += 10;
            break;
        case EDIT_STATE_ADJUST_MS:
            if((editing_tick % 10) == 9)
                editing_tick -= 10;
            editing_tick += 1;
            break;
    	default:
            break;
        }
    }
}
static void rotary_rotate_left(void)
{
    if(EDIT_STATE_SELECTING)
    {
        switch(editing_flags)
        {
        case EDIT_STATE_SELECT_HRS:
            editing_flags = EDIT_STATE_SELECT_MS;
            break;
        case EDIT_STATE_SELECT_MIN:
            editing_flags = EDIT_STATE_SELECT_HRS;
            break;
        case EDIT_STATE_SELECT_SEC:
            editing_flags = EDIT_STATE_SELECT_MIN;
            break;
        //case EDIT_STATE_SELECT_MS:
    	default:
            editing_flags = EDIT_STATE_SELECT_SEC;
            break;
        }
    }
    if(EDIT_STATE_ADJUSTING)
    {
        switch(editing_flags)
        {
        case EDIT_STATE_ADJUST_HRS:
            if(((editing_tick/(10*60*60)) % 24) == 0)
                editing_tick += 10*60*60*24;
            editing_tick -= 10*60*60;
            break;
        case EDIT_STATE_ADJUST_MIN:
            if(((editing_tick/(10*60)) % 60) == 0)
                editing_tick += 10*60*60;
            editing_tick -= 10*60;
            break;
        case EDIT_STATE_ADJUST_SEC:
            if(((editing_tick/10) % 60) == 0)
                editing_tick += 10*60;
            editing_tick -= 10;
            break;
        case EDIT_STATE_ADJUST_MS:
            if((editing_tick % 10) == 0)
                editing_tick += 10;
            editing_tick -= 1;
            break;
    	default:
            break;
        }
    }
}
