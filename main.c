//EE5314 PROJECT(WAVEFORM GENERATOR)
//JEFFRIN STEPHEN 1001716714
//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "tm4c123gh6pm.h"

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))

//DAC pins
#define CS       (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 5*4)))
#define LDAC     (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4)))

#define GREEN_LED_MASK 8
#define RED_LED_MASK 2
#define delay4Cycles() __asm(" NOP\n NOP\n NOP\n NOP")

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

uint8_t count=0;
#define MAXCHAR    80
#define MAX_CHARS    80
#define MAX_FIELDS 15
#define PI 3.14159265

char c;
char str[MAX_CHARS+1];
uint8_t pos[MAX_FIELDS];
uint8_t argCount;
uint16_t LUT[3][4096];           //LUT
uint32_t phi[2]={0,0};           //phase
uint32_t D_phi[2];               //frequency change
uint32_t tmpD_phi;
uint16_t Data;
uint16_t Data0[2]={14346,47097};
uint8_t out;
uint8_t cout;
uint8_t outsq;
int x;

uint16_t Cycles[2];
uint16_t tmp_Cycles[2];

bool AC_Mode = false;
bool Play_wave = false;
bool Cyc_Mode = false;


// PortE masks
#define AIN0_MASK 8
#define AIN1_MASK 4

//DAC masks
#define CS_MASK 32
#define LDAC_MASK 32


// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);
    //Enable clock for Timer
        SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;

    //Enable clock for SSI
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A,B,E and F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOB;

    // Configure LED pins
    GPIO_PORTF_DIR_R = GREEN_LED_MASK | RED_LED_MASK;  // bits 1 and 3 are outputs
    GPIO_PORTF_DR2R_R = GREEN_LED_MASK | RED_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = GREEN_LED_MASK | RED_LED_MASK;  // enable LEDs

    // Configure UART0 pins
    GPIO_PORTA_DIR_R |= 2;                           // enable output on UART0 TX pin: default, added for clarity
    GPIO_PORTA_DEN_R |= 3;                           // enable digital on UART0 pins: default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // use peripheral to drive PA0, PA1: default, added for clarity
    GPIO_PORTA_PCTL_R &= 0xFFFFFF00;                 // set fields for PA0 and PA1 to zero
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
                                                     // select UART0 to drive pins PA0 and PA1: default, added for clarity

    // Configure UART0 to 115200 baud, 8N1 format
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other UARTs in same status
    delay4Cycles();                                  // wait 4 clock cycles
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                     // enable TX, RX, and module

    //DAC CONFIGURATION
    GPIO_PORTB_DIR_R |= CS_MASK;       // make bits 1 and 6 outputs
    GPIO_PORTB_DR2R_R |= CS_MASK;      // set drive strength to 2mA
    GPIO_PORTB_DEN_R |= CS_MASK;       // enable bits 1 and 6 for digital
    GPIO_PORTA_DIR_R |= LDAC_MASK;       // make bits 1 and 6 outputs
    GPIO_PORTA_DR2R_R |= LDAC_MASK;      // set drive strength to 2mA
    GPIO_PORTA_DEN_R |= LDAC_MASK;       // enable bits 1 and 6 for digital

    // Configure SSI2 pins for SPI configuration
            GPIO_PORTB_DIR_R |= 176;                        // make bits 4 and 7 outputs
            GPIO_PORTB_DR2R_R |= 176;                       // set drive strength to 2mA
            GPIO_PORTB_AFSEL_R |= 176;                      // select alternative functions for MOSI, SCLK pins
            GPIO_PORTB_PCTL_R = GPIO_PCTL_PB7_SSI2TX | GPIO_PCTL_PB4_SSI2CLK | GPIO_PCTL_PB5_SSI2FSS; // map alt fns to SSI2
            GPIO_PORTB_DEN_R |= 176;                        // enable digital operation on TX, CLK pins
            GPIO_PORTB_PUR_R |= 16;                        // must be enabled when SPO=1

   // Configure the SSI2 as a SPI master, mode 3, 8bit operation, 1 MHz bit rate

                SSI2_CR1_R &= ~SSI_CR1_SSE;                        // turn off SSI1 to allow re-configuration
                SSI2_CR1_R = 0;                                    // select master mode
                SSI2_CC_R = 0;                                     // select system clock as the clock source
                SSI2_CPSR_R = 10;                                  // set bit rate to 4 MHz (if SR=0 in CR0)
                SSI2_CR0_R = SSI_CR0_SPH | SSI_CR0_SPO | SSI_CR0_FRF_MOTO | SSI_CR0_DSS_16; // set SR=0, mode 0 (SPH=0, SPO=0), 8-bit
                SSI2_CR1_R |= SSI_CR1_SSE;                         // turn on SSI1


}
void initTimer()
{
    // Configure Timer 1 as the time base
                    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
                    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;
                    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
                    TIMER1_TAILR_R = 400;
                    TIMER1_IMR_R = TIMER_IMR_TATOIM;
                    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);
                    TIMER1_CTL_R |= TIMER_CTL_TAEN;
}


// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                        // get character from fifo
}
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #3");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

//Step 2
void getString(char str[], uint8_t maxchar)
{ while(count<maxchar)
  {

   c=getcUart0();

   if(c==8||c==127)
   {
       if(count>0)
    {
       count-- ;
    }
       else
       continue;
   }


   else if(c==10||c==13)
   {
       str[count]=0;
       break;
   }

   else if(c>=32)
   {
       str[count++]=c;
   }

   if(count==MAXCHAR)
   {str[count]=0;
      break;
     }
   else
       continue;
  }
  count=0;
    }

//Step3
void parseString(char str[], uint8_t pos[], uint8_t maxfields, uint8_t *pArgCount)
{   uint8_t i=0;
    uint8_t j=0;

     *pArgCount=0;
    while(i<maxfields)
    {
     c=str[i];
    if(((str[i+1]>=48 && str[i+1]<=57) || (str[i+1]>=65 && str[i+1]<=90) || (str[i+1]>=97 && str[i+1]<=122) || (str[i+1]==46) || (str[i+1]==45))&&(!((c>=48 && c<=57) || (c>=65 && c<=90) || (c>=97 && c<=122) || (c==46) || (c==45))))
              {
                  pos[j]=i+1;
                  *pArgCount=j+1;
                  j++;
              }
    else if((i==0)&&((str[i]>=48 && str[i]<=57) || (str[i]>=65 && str[i]<=90) || (str[i]>=97 && str[i]<=122) || (str[i]==46) || (str[i]==45)))
    {
        pos[j]=i;
        *pArgCount=j+1;
        j++;

    }
     i++;
    }
    i=0;
    while(i<maxfields)
    {

        if(!((str[i]>=48 && str[i]<=57) || (str[i]>=65 && str[i]<=90) || (str[i]>=97 && str[i]<=122) || (str[i]==46) || (str[i]==45)))
         str[i]='\0';
        i++;

    }


}

//Step4
char *getArgString(uint8_t argNo)
{
  if(argNo<argCount)
      return &str[pos[argNo]];
  else
      return NULL;
}

uint32_t getValueArgInt(uint8_t argNo)
{
  uint32_t value;
  value = atoi(getArgString(argNo));
  return value;
}

float getValueArgFloat(uint8_t argNo)
{
  float value;
  value = atof(getArgString(argNo));
  return value;
}

//Step5
bool isCommand(char strCMD[], uint8_t minArg)
{   x = strcmp(strCMD, getArgString(0));
    if((!x) && (minArg<argCount))
    {

        return true;
    }
    else
        return false;
}

//Step6
float readAdc0Ss3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
}

//Step7
void setDC(uint8_t out, float volt)
{ float vout, raw1, raw2;
 raw1= volt*(-388.458) +2057.785;
 raw2= volt*(-390.339) +2040.976;
 AC_Mode=false;
    if(out==0)
  { vout=12288+raw1;
    CS = 0;                        // assert chip select
        LDAC=1;
        SSI2_DR_R = vout;                  // write data
        while (SSI2_SR_R & SSI_SR_BSY);    // wait for transmission to stop
        CS = 1;                        // de-assert chip select
        LDAC = 0;
        __asm(" NOP\n NOP\n NOP\n NOP");
        LDAC = 1;
  }
  else
  { vout=45056+raw2;
      CS = 0;                        // assert chip select
          LDAC=1;
          SSI2_DR_R = vout;                  // write data
          while (SSI2_SR_R & SSI_SR_BSY);    // wait for transmission to stop
          CS = 1;                        // de-assert chip select
          LDAC = 0;
          __asm(" NOP\n NOP\n NOP\n NOP");
          LDAC = 1;
    }
}

float readAnalogFeedback()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    while (ADC0_SSFSTAT3_R & ADC_SSFSTAT3_EMPTY);
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
}

//Step8
//void timer1Isr()
//{
  //  LDAC = 0;
    // __asm(" NOP\n NOP\n NOP\n NOP");
    //LDAC = 1;
    //TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
//}

void timer1Isr()
{

    GREEN_LED ^= 1;
    LDAC = 0;
     __asm(" NOP\n NOP\n NOP\n NOP ");
    LDAC = 1;
    if(Play_wave)
    {if(AC_Mode)// status
    {if(out==0)
        out=1;
    else
        out=0;
    phi[!out] = phi[!out] + D_phi[!out];
    Data=LUT[!out][phi[!out]>>20];
    SSI2_DR_R = Data;
    if(Cyc_Mode)
    {tmp_Cycles[cout]--;
    if(tmp_Cycles[cout]==0)
    {
     Play_wave=false;
     tmp_Cycles[cout]=Cycles[cout];
    }
    }
    }
    }
    else
    {if(out==0)
        out=1;
    else
        out=0;
    SSI2_DR_R = Data0[!out];
    }

   TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
}



//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{   uint8_t IN;
    uint32_t i;
    float volt, amp[2], freq[2], ofs, frequency_1, frequency_2, increment, input_A, input_B, gain,s, R2, correction_factor;                          //waveform variables
    float raw, instantVOLT, instantVOLT_A, instantVOLT_B;
    uint16_t duty_cycle;
    char stren[100], strs[100];
    // Initialize hardware
    initHw();
    uint8_t N;                       //code and discard
    //sTEP1
      GREEN_LED = 1;
      waitMicrosecond(500000);
      GREEN_LED = 0;

      for(i=0;i<4096;i++)
                 {LUT[0][i]=2057.785 + 12288;
                 }
      for(i=0;i<4096;i++)
                 {LUT[1][i]=2040.976 + 45056;
                 }

     while(1)
     {
      //sTEP2
     putsUart0("\r\n");
     putsUart0("Enter Command:");
     getString(str, MAXCHAR);
     //sTEP3 & sTEP4
     parseString(str, pos, MAX_FIELDS, &argCount);

     //sTEP5
     if(isCommand("reset",0))
     { NVIC_APINT_R = 0x05FA0004;
     }

     //sTEP6
     if(isCommand("VOLTAGE",1))
          {SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;
          IN=getValueArgInt(1);
          if(IN==0)
              {GPIO_PORTE_AFSEL_R |= AIN0_MASK;                 // select alternative functions for AN0 (PE3)
              GPIO_PORTE_DEN_R &= ~AIN0_MASK;                  // turn off digital operation on pin PE3
              GPIO_PORTE_AMSEL_R |= AIN0_MASK;                 // turn on analog operation on pin PE3

              // Configure ADC
              ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
              ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
              ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
              ADC0_SSMUX3_R = 0;                               // set first sample to AIN0
              ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
              ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation

              }
          else
          {
              GPIO_PORTE_AFSEL_R |= AIN1_MASK;                 // select alternative functions for AN0 (PE2)
              GPIO_PORTE_DEN_R &= ~AIN1_MASK;                  // turn off digital operation on pin PE2
              GPIO_PORTE_AMSEL_R |= AIN1_MASK;                 // turn on analog operation on pin PE2

              // Configure ADC
              ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
              ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
              ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
              ADC0_SSMUX3_R = 1;                               // set first sample to AIN1
              ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
              ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation
          }
          while(true)
          { raw = readAdc0Ss3();
            instantVOLT = (((raw+0.5) / 4096.0) * 3.31);
            sprintf(stren, "Raw ADC:          %4f\r\n", raw);
            putsUart0(stren);
            sprintf(stren, "Unfiltered (C):   %4f\r\n", instantVOLT);
            putsUart0(stren);
            waitMicrosecond(1000000);

          }
          }
     //sTEP7
          if(isCommand("DC",2))
          {TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
           out=getValueArgInt(1);
           volt=getValueArgFloat(2);
           AC_Mode=false;
           setDC(out, volt);
          }

     //sTEP8
          if(isCommand("Sine",4))
          {AC_Mode=true;
           outsq=getValueArgInt(1);
           freq[outsq]=getValueArgFloat(2);
           amp[outsq]=getValueArgFloat(3);
           ofs=getValueArgFloat(4);
           D_phi[outsq]=(freq[out]*pow(2,32)*2)/100000;
           if(outsq==0)
           {for(i=0;i<4096;i++)
           {LUT[0][i]=((ofs +(sin((2*PI*i)/(4096)))*amp[0])*(-388.458) +2057.785) + 12288;
           }
           }
           else
           {for(i=0;i<4096;i++)
           {LUT[1][i]=((ofs +(sin((2*PI*i)/(4096)))*amp[1])*(-390.339) +2040.976) + 45056;
           }
           }
          }

          //sTEP11
          if(isCommand("sqr",4))
         {AC_Mode=true;
          outsq=getValueArgInt(1);
          freq[outsq]=getValueArgFloat(2);
          amp[outsq]=getValueArgFloat(3);
          ofs=getValueArgFloat(4);
          D_phi[outsq]=(freq[outsq]*pow(2,32)*2)/100000;
          if(outsq==0)
          {for(i=0;i<4096;i++)
          {   if(i<2048)
              LUT[0][i]=((ofs + amp[0])*(-388.458)) +2057.785 + 12288;
              else
              LUT[0][i]=((ofs)*(-388.458)) +2057.785 + 12288;
              }
              }
              else
             {for(i=0;i<4096;i++)
              if(i<2048)
              LUT[1][i]=((ofs + amp[1])*(-390.339)) +2040.976 + 45056;
              else
              LUT[1][i]=((ofs)*(-390.339)) +2040.976 + 45056;
              }
              }

          //sTEP12
          if(isCommand("swt",4))
                    {AC_Mode=true;
                     outsq=getValueArgInt(1);
                     freq[outsq]=getValueArgFloat(2);
                     amp[outsq]=getValueArgFloat(3);
                     ofs=getValueArgFloat(4);
                     D_phi[outsq]=(freq[outsq]*pow(2,32)*2)/100000;
                     if(outsq==0)
                     {for(i=0;i<4096;i++)
                     {LUT[0][i]=((ofs + (amp[0]/2) +(((-2)*(amp[0]/2))/PI)*atan((1/tan((PI*i)/4096))))*(-388.458) +2057.785) + 12288;
                     }
                     }
                     else
                     {for(i=0;i<4096;i++)
                     {LUT[1][i]=((ofs + (amp[1]/2) +(((-2)*(amp[1]/2))/PI)*atan((1/tan((PI*i)/4096))))*(-390.339) +2040.976) + 45056;
                     }
                     }
                    }

          //sTEP13
          if(isCommand("tri",4))
                              {AC_Mode=true;
                               outsq=getValueArgInt(1);
                               freq[outsq]=getValueArgFloat(2);
                               amp[outsq]=getValueArgFloat(3);
                               ofs=getValueArgFloat(4);
                               D_phi[outsq]=(freq[outsq]*pow(2,32)*2)/100000;
                               if(outsq==0)
                               {for(i=0;i<4096;i++)
                               {LUT[0][i]=((ofs + (amp[0]/2) +(((2)*(amp[0]/2))/PI)*asin((sin((2*PI*i)/4096))))*(-388.458) +2057.785) + 12288;
                               }
                               }
                               else
                               {for(i=0;i<4096;i++)
                               {LUT[1][i]=((ofs + (amp[1]/2) +(((2)*(amp[1]/2))/PI)*asin((sin((2*PI*i)/4096))))*(-390.339) +2040.976) + 45056;
                               }
                               }
                              }

          //sTEP16
          if(isCommand("sqr",5))
                   {AC_Mode=true;
                    outsq=getValueArgInt(1);
                    freq[outsq]=getValueArgFloat(2);
                    amp[outsq]=getValueArgFloat(3);
                    ofs=getValueArgFloat(4);
                    D_phi[outsq]=(freq[outsq]*pow(2,32)*2)/100000;
                    duty_cycle=(4096*getValueArgInt(5))/100;
                    if(outsq==0)
                    {for(i=0;i<4096;i++)
                    {   if(i<duty_cycle)
                        LUT[0][i]=((ofs + amp[0])*(-388.458)) +2057.785 + 12288;
                        else
                        LUT[0][i]=((ofs)*(-388.458)) +2057.785 + 12288;
                        }
                        }
                        else
                       {for(i=0;i<4096;i++)
                        if(i<duty_cycle)
                        LUT[1][i]=((ofs + amp[1])*(-390.339)) +2040.976 + 45056;
                        else
                        LUT[1][i]=((ofs)*(-390.339)) +2040.976 + 45056;
                        }
                        }

          //sTEP9
          if(isCommand("run",0))
                         { Play_wave = true;
                           initTimer();
                         }
          if(isCommand("stop",0))
             { Play_wave = false;
             }

          //sTEP10
          if(isCommand("cycles",2))
           {
              Cyc_Mode = true;
              N=getValueArgInt(2);
              cout=getValueArgInt(1);
              Cycles[cout]=(100000/freq[cout]) * N;
              tmp_Cycles[cout]=Cycles[cout];

            }

          if(isCommand("cyclesCnt",0))
                           {
                             Cyc_Mode = false;
                             Play_wave = true;
                           }

          //sTEP15
          if(isCommand("differential",1))
               {if(ofs==0)
               {
                strcpy(stren,getArgString(1));
                if(!strcmp(stren,"on"))
                {
                 tmpD_phi=D_phi[1];
                 D_phi[1]=D_phi[0];
                 for(i=0;i<4096;i++)
                 {LUT[2][i]=LUT[1][i];
                 }
                 for(i=0;i<4096;i++)
                 {LUT[1][i]=45056 + 2057.785 + 2049.976 + 12288 - LUT[0][i];
                 }
                 }
                else if(!strcmp(stren,"off"))
                {
                D_phi[1]=tmpD_phi;
                for(i=0;i<4096;i++)
                {LUT[1][i]=LUT[2][i];
                }
                }
               }
               else
               {
                   putsUart0("\r\nCannot be supported for offset other than 0V");
                   putsUart0("\r\n");
               }
               }

          //sTEP14
          if(isCommand("hilbert",1))
               { strcpy(stren,getArgString(1));
                if(!strcmp(stren,"on"))
                {
                 tmpD_phi=D_phi[1];
                 D_phi[1]=D_phi[0];
                 for(i=0;i<4096;i++)
                 {LUT[2][i]=LUT[1][i];
                 }
                 for(i=0;i<4096;i++)
                 {

                   LUT[1][i]=((ofs + (-cos((2*PI*i)/(4096)))*amp[0])*(-390.339) +2040.976) + 45056;

                  }
                 }

                else if(!strcmp(stren,"off"))
                {
                D_phi[1]=tmpD_phi;
                for(i=0;i<4096;i++)
                {LUT[1][i]=LUT[2][i];
                }
                }
               }

          //sTEP19
          if(isCommand("gain",2))
          {frequency_1=getValueArgFloat(1);
           frequency_2=getValueArgFloat(2);
           raw=frequency_2-frequency_1;
           increment=raw/20;
           SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;
           putsUart0("\r\n");
           putsUart0(" frequency      gain");
           putsUart0("\r\n");
           for(s=frequency_1;s<=frequency_2;s+=increment)
           {Play_wave = false;
           D_phi[0]=(s*pow(2,32)*2)/100000;
            for(i=0;i<4096;i++)
            {LUT[0][i]=((ofs +(sin((2*PI*i)/(4096)))*4)*(-388.458) +2057.785) + 12288;
            }

            Play_wave = true;
           // Configure AIN1 as an analog input
           GPIO_PORTE_AFSEL_R |= AIN1_MASK;                 // select alternative functions for AN1 (PE2)
           GPIO_PORTE_DEN_R &= ~AIN1_MASK;                  // turn off digital operation on pin PE2
           GPIO_PORTE_AMSEL_R |= AIN1_MASK;                 // turn on analog operation on pin PE2

           // Configure ADC
           ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
           ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
           ADC0_PC_R = ADC_PC_SR_1M;                        // select 1Msps rate
           ADC0_SAC_R = ADC_SAC_AVG_64X;                    // 64-sample HW averaging
           ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
           ADC0_SSMUX3_R = 1;                               // set first sample to AIN1
           ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
           ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation
           waitMicrosecond(500000);
           input_B=readAnalogFeedback();
           instantVOLT_B = (((input_B+0.5) / 4096.0) * 3.31);
           // Configure AIN0 as an analog input
           GPIO_PORTE_AFSEL_R |= AIN0_MASK;                 // select alternative functions for AN0 (PE3)
           GPIO_PORTE_DEN_R &= ~AIN0_MASK;                  // turn off digital operation on pin PE3
           GPIO_PORTE_AMSEL_R |= AIN0_MASK;                 // turn on analog operation on pin PE3

           // Configure ADC
           ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
           ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
           ADC0_PC_R = ADC_PC_SR_1M;                        // select 1Msps rate
           ADC0_SAC_R = ADC_SAC_AVG_64X;                    // 64-sample HW averaging
           ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
           ADC0_SSMUX3_R = 0;                               // set first sample to AIN0
           ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
           ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation
           waitMicrosecond(500000);
           input_A=readAnalogFeedback();
           instantVOLT_A = (((input_A+0.5) / 4096.0) * 3.31);
           gain = instantVOLT_B/instantVOLT_A;
           sprintf(stren, "%4f", gain);
           sprintf(strs, " %4f", s);
           putsUart0(strs);
           putsUart0("    ");
           putsUart0(stren);
           putsUart0("\r\n");
           waitMicrosecond(500000);
           }
          }

          //sTEP18
          if(isCommand("ALC",2))
          {outsq=getValueArgFloat(1);
          SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;
          strcpy(stren,getArgString(2));
          if(!strcmp("on",stren))
          {if(outsq==0)
          {
              GPIO_PORTE_AFSEL_R |= AIN0_MASK;                 // select alternative functions for AN0 (PE3)
              GPIO_PORTE_DEN_R &= ~AIN0_MASK;                  // turn off digital operation on pin PE3
              GPIO_PORTE_AMSEL_R |= AIN0_MASK;                 // turn on analog operation on pin PE3

              // Configure ADC
              ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
              ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
              ADC0_PC_R = ADC_PC_SR_1M;                        // select 1Msps rate
              ADC0_SAC_R = ADC_SAC_AVG_64X;                    // 64-sample HW averaging
              ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
              ADC0_SSMUX3_R = 0;                               // set first sample to AIN0
              ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
              ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation
              waitMicrosecond(500000);
              input_A=readAnalogFeedback();
              instantVOLT_A = (((input_A+0.5) / 4096.0) * 3.31);
              R2=((50*instantVOLT_A)/(amp[outsq]-instantVOLT_A))*2.06;
              correction_factor=(R2/(50+R2));
              for(i=0;i<4096;i++)
              {LUT[2][i]=LUT[0][i];
              }
              for(i=0;i<4096;i++)
              {LUT[0][i]=(LUT[0][i] - 2057.785 - 12288)/correction_factor + 2057.785 + 12288;
              }
          }
          else
          {
              GPIO_PORTE_AFSEL_R |= AIN1_MASK;                 // select alternative functions for AN1 (PE2)
              GPIO_PORTE_DEN_R &= ~AIN1_MASK;                  // turn off digital operation on pin PE2
              GPIO_PORTE_AMSEL_R |= AIN1_MASK;                 // turn on analog operation on pin PE2

              // Configure ADC
              ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
              ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
              ADC0_PC_R = ADC_PC_SR_1M;                        // select 1Msps rate
              ADC0_SAC_R = ADC_SAC_AVG_64X;                    // 64-sample HW averaging
              ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
              ADC0_SSMUX3_R = 1;                               // set first sample to AIN0
              ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
              ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation
              waitMicrosecond(500000);
              input_B=readAnalogFeedback();
              instantVOLT_B = (((input_B+0.5) / 4096.0) * 3.31);
              R2=((50*instantVOLT_B)/(amp[outsq]-instantVOLT_B))*2.06;
              correction_factor=(R2/(50+R2));
              for(i=0;i<4096;i++)
              {LUT[2][i]=LUT[1][i];
              }
              for(i=0;i<4096;i++)
              {LUT[1][i]=(LUT[1][i] - 2040.976 - 45056)/correction_factor + 2040.976 + 45056;
              }
          }
          }
          else if(!strcmp("off",stren))
          {if(outsq==0)
          {
              for(i=0;i<4096;i++)
              {LUT[0][i]=LUT[2][i];
              }

          }
          else
          {
              for(i=0;i<4096;i++)
              {LUT[1][i]=LUT[2][i];
              }

          }

          }
          else
              putsUart0("invalid alc command");

          }

          if((!isCommand("reset",0))&&(!isCommand("VOLTAGE",1))&&(!isCommand("DC",2))&&(!isCommand("Sine",4))&&(!isCommand("sqr",4))&&(!isCommand("swt",4))&&(!isCommand("tri",4))&&(!isCommand("sqr",5))&&(!isCommand("run",0))&&(!isCommand("stop",0))&&(!isCommand("cycles",2))&&(!isCommand("cyclesCnt",0))&&(!isCommand("differential",1))&&(!isCommand("hilbert",1))&&(!isCommand("gain",2))&&(!isCommand("ALC",2)))
          {   putsUart0("\r\n");
              putsUart0("invalid command");
          }

     }
}
