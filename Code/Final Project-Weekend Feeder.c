#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "clock.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"
#include "wait.h"
#include "eeprom.h"

#define MAX_CHARS 80
#define MAX_FIELDS 6
#define GREEN_LED_MASK 8    // PF3
#define AUDIO_MASK 32       // PE5
#define MOTOR_MASK 16       // PC4
#define PUMP_MASK 32        // PC5
#define SENSOR_MASK 4       // PB2
#define BLUE_LED_MASK 4     // PF2
#define PC7_MASK 128        // C0-
#define PC6_MASK 64         // C0+
#define PD1_MASK 2          // GPO
#define PC4_MASK 16         // PC4
#define PC5_MASK 32         // PC5

// Pin bitbands
#define MOTOR   (*((volatile uint32_t*)(0x42000000 + (0x400063FC - 0x40000000)*32 + 4*4)))      //  PC4
#define AUDIO   (*((volatile uint32_t*)(0x42000000 + (0x400243FC - 0x40000000)*32 + 5*4)))      //  PE5
#define PUMP    (*((volatile uint32_t*)(0x42000000 + (0x400063FC - 0x40000000)*32 + 5*4)))      //  PC5
#define BLUE_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))   //  PF2
#define FET_DRAIN   (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 1*4)))   //  PD1
#define SENSOR      (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 2*4)))   //  PB2

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------
uint32_t Ticks = 0;
uint16_t waterLvl;
int nextEventIndex;
int modeSet;
int alert;
int prevTicks = 0;
uint32_t volume;
int block = 1;
int index = 6;
int prevCC = -1;
char str[100];

typedef struct _USER_DATA
{
    char buffer[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
} USER_DATA;



// Initialize Hardware
void initHw()
{
    initUart0();
    setUart0BaudRate(115200, 40e6);

    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    //Enable Hibernation Module
    SYSCTL_RCGCHIB_R |= SYSCTL_RCGCHIB_R0;

    // Enable gpio clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5 | SYSCTL_RCGCGPIO_R4 | SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R3 |
                            SYSCTL_RCGCGPIO_R1;

    //Enable Comparator 0
    SYSCTL_RCGCACMP_R   |= SYSCTL_RCGCACMP_R0;

    // Enable wide timer clocks
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1 | SYSCTL_RCGCWTIMER_R0;

    // Enable time clk
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R4 | SYSCTL_RCGCTIMER_R3 | SYSCTL_RCGCTIMER_R2 | SYSCTL_RCGCTIMER_R1
                            | SYSCTL_RCGCTIMER_R0;

    _delay_cycles(3);

    // Enable EEPROM Module
    initEeprom();

    // Config Hibernation
    while(HIB_CTL_WRC & ~HIB_CTL_R);
    HIB_CTL_R   |= HIB_CTL_CLK32EN | HIB_CTL_RTCEN;         // sets clock to  32.768-kHz Hibernation oscillator
                                                            // and enable the RTC to begin counting
    while(HIB_CTL_WRC & ~HIB_CTL_R);
    HIB_IM_R    |= HIB_IM_RTCALT0;                          // set HIB interupt mask
    // Set the required RTC match interrupt mask in the RTCALT0
    while(HIB_CTL_WRC & ~HIB_CTL_R);
    HIB_RTCLD_R = 0;                                        // inti the counter to 0
    while(HIB_CTL_WRC & ~HIB_CTL_R);
    NVIC_EN1_R = 1 << (INT_HIBERNATE-16-32);

    // Configure LED and pushbutton pins
    GPIO_PORTE_DIR_R |= AUDIO_MASK;
    GPIO_PORTC_DIR_R |= MOTOR_MASK | PUMP_MASK;
    GPIO_PORTF_DIR_R &= ~GREEN_LED_MASK;  // bit 3 is input, other pins are outputs
    GPIO_PORTF_DEN_R |= GREEN_LED_MASK;   // enable LEDs and
    GPIO_PORTE_DEN_R |= AUDIO_MASK;
    GPIO_PORTC_DEN_R |= MOTOR_MASK | PUMP_MASK;

    // Configure LED pin
    GPIO_PORTF_DIR_R    |= BLUE_LED_MASK;
    GPIO_PORTF_DEN_R    |= BLUE_LED_MASK;
    GPIO_PORTD_DIR_R    |= PD1_MASK;
    GPIO_PORTD_DEN_R    |= PD1_MASK;
    GPIO_PORTB_DIR_R &= ~SENSOR_MASK;   // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTB_DEN_R |= SENSOR_MASK;    // enable LEDs and

    //wide timer 1A
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    WTIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    WTIMER1_TAILR_R = 400000000;                      // set load value to 40e7 for 100 mHz (10 secs) interrupt rate
    WTIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
    NVIC_EN3_R = 1 << (INT_WTIMER1A-16-96);           // turn-on interrupt 112 (WTIMER1A) in NVIC
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer

    // C0
    GPIO_PORTC_DIR_R    &= ~PC7_MASK;
    GPIO_PORTC_AFSEL_R  &= ~PC7_MASK;
    GPIO_PORTC_AMSEL_R  |= PC7_MASK;
    GPIO_PORTC_DEN_R    &= ~PC7_MASK;

    COMP_ACREFCTL_R |= COMP_ACREFCTL_EN;
    COMP_ACREFCTL_R &= ~COMP_ACREFCTL_RNG;
    COMP_ACREFCTL_R |= COMP_ACREFCTL_VREF_M;

    COMP_ACCTL0_R |= COMP_ACCTL0_ASRCP_REF;
    COMP_ACCTL0_R |= COMP_ACCTL0_CINV;
    COMP_ACCTL0_R |= COMP_ACCTL0_ISEN_RISE;

    WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    WTIMER0_TAMR_R = TIMER_TAMR_TAMR_1_SHOT | TIMER_TAMR_TACDIR;

    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;
    _delay_cycles(3);
    GPIO_PORTC_DEN_R |= PC4_MASK;
    GPIO_PORTC_AFSEL_R |= PC4_MASK;
    GPIO_PORTC_DEN_R |= PC5_MASK;
    GPIO_PORTC_AFSEL_R |= PC5_MASK;
    GPIO_PORTC_PCTL_R &= ~(GPIO_PCTL_PC4_M | GPIO_PCTL_PC5_M);
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC4_M0PWM6 | GPIO_PCTL_PC5_M0PWM7;

    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;                // reset PWM0 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    _delay_cycles(3);
    PWM0_3_CTL_R = 0;                                // turn-off PWM0 generator 3 (drives outs 6 and 7)
    PWM0_3_GENA_R = PWM_0_GENA_ACTCMPAD_ONE | PWM_0_GENA_ACTLOAD_ZERO;
                                                         // output 6 on PWM1, gen 3a, cmpa
    PWM0_3_GENB_R = PWM_0_GENB_ACTCMPBD_ONE | PWM_0_GENB_ACTLOAD_ZERO;
                                                         // output 7 on PWM1, gen 3b, cmpb

    PWM0_3_LOAD_R = 1024;                            // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz

    PWM0_3_CMPA_R = 0;                               // motor c4 off (0=always low, 1023=always high)
    PWM0_3_CMPB_R = 0;                               // pump off

    PWM0_3_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM0 generator 3
    PWM0_ENABLE_R = PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
                                                         // enable outputs
}

// getsUart0 gets input from PuTTY which only can be alphabet, 'ENTER', 'BACKSPACE', and numerical
void getsUart0(USER_DATA *data)
{
    int count = 0;
    char c;

    while(true)
    {
        c = getcUart0();
        if((c == 8 || c==127) && count>0)
        {
            count--;
        }
        else if(c == 13)
        {
            data->buffer[count] = 0;
            return;
        }
        else if(c >= 32)
        {
            data->buffer[count] = c;
            count++;
            if(count == MAX_CHARS)
            {
                data->buffer[count] = 0;
                return;
            }
        }
    }
}

//  puts null anywhere except numerical or alphabet, sets field type and count and position
void parseFields(USER_DATA *data)
{
    int bufferIndex = 0, fieldIndex = 0;
    bool flag = 0;
    data->fieldCount = 0;
    while(data->buffer[bufferIndex])
    {
        if(data->fieldCount == MAX_FIELDS)
        {
            return;
        }
        if(((data->buffer[bufferIndex] >= 65) && (data->buffer[bufferIndex] <= 90)) ||
                ((data->buffer[bufferIndex] >= 97) && (data->buffer[bufferIndex] <= 122)))
        {
            if(!flag)
            {
                (data->fieldCount)++;
                data->fieldPosition[fieldIndex] = bufferIndex;
                data->fieldType[fieldIndex] = 97; // 'a'
                flag = 1;
                fieldIndex++;
            }
        }
        else if(data->buffer[bufferIndex] >= 48 && data->buffer[bufferIndex] <= 57)
        {
            if(!flag)
            {
                (data->fieldCount)++;
                data->fieldPosition[fieldIndex] = bufferIndex;
                data->fieldType[fieldIndex] = 'n'; // 110
                flag = 1;
                fieldIndex++;
            }
        }
        else
        {
            flag = 0;
            data->buffer[bufferIndex] = 0;
        }

        bufferIndex++;
    }
}

// checks if asked fieldNumber is alpha, if so returns it address
char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{
    if((fieldNumber < data->fieldCount ) && (data->fieldType[fieldNumber] == 'a'))
    {
        return &(data->buffer[data->fieldPosition[fieldNumber]]);
    }
    return 0;
}

// checks if asked fieldNumber is numerical, if so converts the value from string to integer and returns it
uint32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
    if((fieldNumber < data->fieldCount) && (data->fieldType[fieldNumber] == 'n'))
    {
        uint32_t returnNum = 0;
        char* number = &data->buffer[data->fieldPosition[fieldNumber]];
        int i = 0;
        while(number[i])
        {
            int num = (number[i]-48);
            if(i)
            {
                returnNum *= 10;
            }
            returnNum += num;
            i++;
        }
        return returnNum;
    }
    return 0;
}

// checks command validation from data (1st argument) with passed string, also checks valid arguments
bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments)
{
    if(data->fieldCount >= minArguments+1) // plus 1 because fieldCount-1 for command
    {
        int i = 0;
        char* buffStr = &(data->buffer[data->fieldPosition[0]]);
        while(strCommand[i])
        {
            if(buffStr[i] != strCommand[i])
            {
                return false;
            }
            i++;
        }
        return true;
    }
    return false;
}

// 10 seconds period interrupt that turns on GPO (discharges pet dish capacitance) and turns on another
//    timer which measures clock time and stops on analog comparator interrupt (happens when pet
//      dish capacitance charges up to 2.469 Volts)
void wideTimer1Isr()
{
    BLUE_LED ^= 1;
    FET_DRAIN = 1;
    waitMicrosecond(100);
    FET_DRAIN = 0;

    WTIMER0_TAV_R = 0;
    WTIMER0_CTL_R |= TIMER_CTL_TAEN;                  // turn-on widetimer0A

    COMP_ACINTEN_R |= COMP_ACINTEN_IN0;
    NVIC_EN0_R = 1 << (INT_COMP0-16);

    WTIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag for wt1a
}

// turns off Pump
void timer3Isr(){
    PWM0_3_CMPB_R = 0;
    TIMER3_ICR_R = TIMER_ICR_TATOCINT;
}

// configures an interrupt that fill pet dish with pump for 8 seconds
//  refill is only called in auto mode (refill under certain level)
void refill(){
    TIMER3_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER3_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER3_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;          // configure as one shot timer

    TIMER3_TAILR_R = (40000000*8);                   // set load value to 40e6 for 1 Hz interrupt rate
    PWM0_3_CMPB_R = 1023;

    TIMER3_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
    TIMER3_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    NVIC_EN1_R = 1 << (INT_TIMER3A-16-32);           // turn-on interrupt 37 (TIMER1A) in NVIC
}

// 2 seconds periodic interrupt initiated when mode is switched to motion
//  on capture by motion sensor water is 'freshed'
void timer0Isr(){
    // Configure Timer 1 as the time base
    if(SENSOR && !modeSet)                       // motion refill
    {
        TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
        TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
        TIMER2_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;           // config as one shot timer
        TIMER2_TAILR_R = 40000000 ;                  // set load value to 40e6 for 1 Hz interrupt rate
        PWM0_3_CMPB_R = 1023;
        TIMER2_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
        TIMER2_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
        NVIC_EN0_R = 1 << (INT_TIMER2A-16);              // turn-on interrupt 37 (TIMER1A) in NVIC
    }
    TIMER0_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
}

// Analog comparator interrupt that gets clock time of pet dish capacitance charging
//      with many experiments ranges are created of 50 mL sensitivity
//An equation was derived but not accurate due to clock time not being linear in proportion to water quantity
// Interrupt also sets alert on water not being filled with minimum requirement
void wideTimer0Isr()
{
    Ticks = WTIMER0_TAV_R;              // read counter input
    WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;   // turn-off event counter
    COMP_ACMIS_R = COMP_ACMIS_IN0;  // clear interrupt flag

/*    if(Ticks < 3000){
        waterLvl = 0;
    }
    else{
        waterLvl = 50 * ((int) ((float) ((Ticks - 3040)/1.372/50)));
    }*/

    if(Ticks < 3045){
        waterLvl = 0;
    }
    else if(Ticks > 3031 && Ticks < 3145){
        waterLvl = 50;
    }
    else if(Ticks > 3145 && Ticks < 3240){
        waterLvl = 100;
    }
    else if(Ticks > 3240 && Ticks < 3285){
        waterLvl = 150;
    }
    else if(Ticks > 3285 && Ticks < 3380){
        waterLvl = 200;
    }
    else if(Ticks > 3380 && Ticks < 3430){
        waterLvl = 250;
    }
    else if(Ticks > 3430 && Ticks < 3480){
        waterLvl = 300;
    }
    else if(Ticks > 3480 && Ticks < 3540){
        waterLvl = 350;
    }
    else if(Ticks > 3540 && Ticks < 3610){
        waterLvl = 400;
    }
    else if(Ticks > 3610 && Ticks < 3670){
        waterLvl = 450;
    }
    else if(Ticks > 3670 ){
        waterLvl = 500;
    }

    if(modeSet && (waterLvl < volume)) {
        refill();
    }

    if((prevTicks > Ticks-25) && alert && (waterLvl < volume)){
        int k = 0;
        while(k < 5400){
            AUDIO = 1;
            waitMicrosecond(185);
            AUDIO = 0;
            waitMicrosecond(185);
            k++;
        }
    }
    prevTicks = Ticks;
}

// Hibernate interrupt occurs when RTC_M0 (set by next event which user puts in) equals RTC_CC (current time),
//  (time to put food in the dish) Auger is turned on based on user input in EEprom
void hibIsr(){
    uint32_t on = readEeprom(16*nextEventIndex+1);
    float dutyCyc = (float)readEeprom(16*nextEventIndex+2); // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                        // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;                  // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;                 // config as one shot timer
    TIMER1_TAILR_R = 40000000*on;                           // set load value to 40e6 for 1 Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                        // turn-on interrupts for timeout in timer module

    PWM0_3_CMPA_R = (uint32_t)(1023.0*(float)(dutyCyc/100));
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                         // turn-on timer
    NVIC_EN0_R = 1 << (INT_TIMER1A-16);                     // turn-on interrupt 37 (TIMER1A) in NVIC

    HIB_IC_R = HIB_RIS_RTCALT0;
}

// setNextEvent finds the next closest time compared to RTCM0 from the EEPROM
//      If closest time not found, resets to smallest time (earliest hour)
void setNextEvent(){
    while(HIB_CTL_WRC & ~HIB_CTL_R);
    uint32_t days = (HIB_RTCC_R/3600)/24;
    while(HIB_CTL_WRC & ~HIB_CTL_R);
    uint32_t cur = HIB_RTCC_R-(days*86400);       // seconds, 24*3600 is 86,400
    uint32_t nextEvent = 0;
    bool nextFound = 0;
    int i;

    for(i=0; i<10; i++){
        if(readEeprom(16*i)<10){
            uint32_t sec = readEeprom(16*i+3)*3600+readEeprom(16*i+4)*60;
            if(sec > cur){
                if(nextEvent == 0 || sec < nextEvent)
                {
                    nextEvent = sec;
                    nextFound = 1;
                    nextEventIndex = i;
                    while(HIB_CTL_WRC & ~HIB_CTL_R);
                    HIB_RTCM0_R = nextEvent+(days*86400);
                }
            }
        }
    }

   if(!nextFound)
    {
        cur = 0;
        for(i=0; i<10; i++){
            if(readEeprom(16*i)<10){
                uint32_t sec = readEeprom(16*i+3)*3600+readEeprom(16*i+4)*60;
                if(sec > cur){
                    if(nextEvent == 0 || sec < nextEvent)
                    {
                        nextEvent = sec;
                        nextEventIndex = i;
                    }
                }
            }
        }
        while(HIB_CTL_WRC & ~HIB_CTL_R);
        HIB_RTCM0_R = nextEvent+(days*86400);
    }
    return;
}

// turns off Pump
void timer2Isr(){
    PWM0_3_CMPB_R = 0;
    TIMER2_ICR_R = TIMER_ICR_TATOCINT;
}

// turns off Auger
void timer1Isr(){
    PWM0_3_CMPA_R = 0;
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
    setNextEvent();
}

// initiates 2 second periodic interrupt for checking motion
void init2secMotion(){
    TIMER0_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER0_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER0_TAILR_R = 40000000*2;                     // set load value to 40e6 for 1 Hz interrupt rate
    TIMER0_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
    TIMER0_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    NVIC_EN0_R = 1 << (INT_TIMER0A-16);              // turn-on interrupt 19 (TIMER0A) in NVIC
}

// stores pets logs (notes down time when pet visits the dish) in EEprom
//  does not store time of same minute
void timer4ISR(){
    if(SENSOR){
        while(HIB_CTL_WRC & ~HIB_CTL_R);
        uint32_t CC = HIB_RTCC_R;
        if(CC/60 != prevCC){
            if(index != 12){
                writeEeprom(16*1+index, CC);
                ++index;
            }
            else if(index == 12){
                index = 6;
                writeEeprom(16*1+index, CC);
                ++index;
            }
        }
        prevCC = CC/60;
    }
    TIMER4_ICR_R = TIMER_ICR_TATOCINT;
}

// Period 3 second interrupt to check pet visit to the dish
void init3seclog()
{
    TIMER4_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER4_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER4_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER4_TAILR_R = 40000000*3;                     // set load value to 40e6 for 1 Hz interrupt rate
    TIMER4_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
    TIMER4_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    NVIC_EN2_R = 1 << (INT_TIMER4A-16-64);              // turn-on interrupt 96 (TIMER4A) in NVIC
}
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------
// Controls all input command and initialization
int main(void)
{
    uint32_t x;
    // Initialize hardware
    initHw();
    initUart0();
    USER_DATA data;
    init2secMotion();
    init3seclog();
    alert = readEeprom(7);
    modeSet = readEeprom(6);
    volume = readEeprom(5);
    while(true)
    {
        bool valid = false;
        getsUart0(&data);       //  Get the string from the user
        parseFields(&data);     //  Takes the

        if(isCommand(&data, "time", 2))
        {
            uint32_t hour = getFieldInteger(&data, 1);
            uint32_t minute = getFieldInteger(&data, 2);
            uint32_t seconds = hour*3600 + minute*60;
            HIB_RTCLD_R = seconds;
            setNextEvent();
            valid = true;
        }
        else if(isCommand(&data, "time", 0))
        {
            x = HIB_RTCC_R;
            int hour = x/3600;
            int minutes = (x%3600)/60;
            int sec = x-hour*3600-minutes*60;
            snprintf(str, sizeof(str),"RTC time: %02d:%02d:%02d\n", hour, minutes, sec);
            putsUart0(str);

            valid = true;
        }
        else if(isCommand(&data, "feed", 5))
        {
            int i, j;

            uint16_t block = getFieldInteger(&data, 1);        // gets the index for new event
            //  sets eeprom[0] to index, and data to others
            writeEeprom(16*block+0, block);
            writeEeprom(16*block+1, getFieldInteger(&data, 2));
            writeEeprom(16*block+2, getFieldInteger(&data, 3));
            writeEeprom(16*block+3, getFieldInteger(&data, 4));
            writeEeprom(16*block+4, getFieldInteger(&data, 5));

            for (i = 0; i < 10; i++) {
                for(j = 0; j<5; j++){
                    uint32_t values = readEeprom(16*i+j);
                    snprintf(str, sizeof(str),"%d\t", values);
                    putsUart0(str);
                }
                putsUart0("\n");
            }
            setNextEvent();                             // updates feeding time
            valid = true;
        }
        else if(isCommand(&data, "feed", 2))
        {
            int i = getFieldInteger(&data, 1), j;           // get the index of feeding (eeprom block) to delete
           // prints to be deleted index
            for (j = 0; j < 5; j++) {
                uint32_t values = readEeprom(16*i+j);
                snprintf(str, sizeof(str),"%d\t", values);
                putsUart0(str);
            }
            putsUart0("\n");

            uint16_t block = getFieldInteger(&data, 1);
            // places 11 for unavaliable index and 0 for data
            writeEeprom(16*block+0, 11);
            writeEeprom(16*block+1, 0);
            writeEeprom(16*block+2, 0);
            writeEeprom(16*block+3, 0);
            writeEeprom(16*block+4, 0);
            setNextEvent();                             // recalibrates next feeding time

            valid = true;
        }
        else if(isCommand(&data, "water", 1))
        {
            volume = getFieldInteger(&data, 1);
            writeEeprom(5, volume);
            valid = 1;
        }
        else if(isCommand(&data, "water", 0))
        {
            snprintf(str, sizeof(str),"Refill level: %d\tWater level: %d mL\tTicks: %d\n", volume, waterLvl, Ticks);
            putsUart0(str);
            valid = 1;
        }
        else if(isCommand(&data, "fill", 1))
        {
            char* cmd = getFieldString(&data, 1);
            if(cmd[0] == 'a')
            {
                modeSet = 1;
            }
            else if(cmd[0] == 'm')
            {
                modeSet = 0;
            }
            writeEeprom(6, modeSet);
            valid = 1;
        }
        else if(isCommand(&data, "set", 2))
        {
            int32_t add = getFieldInteger(&data, 1);
            int32_t dataS = getFieldInteger(&data, 2);
            valid = true;
        }
        // alert ON|OFF → alert ON or alert OFF are the expected commands
        else if (isCommand(&data,"alert", 1))
        {
            char* str = getFieldString(&data, 1);
            if(str[1] == 'N' || str[1] == 'n')
            {
                alert = 1;
            }
            else if(str[1]  == 'F' || str[1] == 'f')
            {
                alert = 0;
            }
            writeEeprom(7, alert);
            valid = true;
            // process the string with your custom strcmp instruction, then do something
        }
        else if(isCommand(&data,"logs", 0)){
            int j;
                for(j = 6; j<12; j++){
                    uint32_t y = readEeprom(16*1+j);
                    int hour = y/3600;
                    int minutes = (y%3600)/60;
                    snprintf(str, sizeof(str),"%d:%d", hour, minutes);
                    putsUart0(str);
                    putsUart0("\n");
                }
                valid =1;
        }
        else if(isCommand(&data,"next", 0)){
            int i,j;

            setNextEvent();
            for (i = 0; i < 10; i++){
                for(j = 0; j<5; j++){
                    uint32_t values = readEeprom(16*i+j);
                    snprintf(str, sizeof(str),"%d\t", values);
                    putsUart0(str);
                }
                putsUart0("\n");
            }
            while(HIB_CTL_WRC & ~HIB_CTL_R);
            uint32_t c = HIB_RTCM0_R;
            snprintf(str, sizeof(str),"%d\n", c);
            putsUart0(str);
            valid = 1;
        }

        // Look for invalid command
        if (!valid)
        {
            putsUart0("Invalid command\n");
        }
    }
}
