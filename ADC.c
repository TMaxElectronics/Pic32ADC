#include <xc.h>
#include <stdint.h>

#include "ADC.h"
#include "FreeRTOSConfig.h"

static void ADC_powerUp();
static void ADC_powerDown();

void ADC_init(ADC_OUTPUT_FORMAT_t format, uint32_t conversionTime_us){
    //set basic module options
    AD1CON1 = 0b1000000011100000 | ((format & 0x7) << 8);
    AD1CON2 = 0;
    
    uint32_t autoSampleScaler = 0;
    uint32_t clockScaler = (((configPERIPHERAL_CLOCK_HZ / 1000) * conversionTime_us) / 24000) - 1;
    if(clockScaler > 0xff) clockScaler = 0xff;
    AD1CON3 = ((autoSampleScaler & 0x1f) << 8) | (clockScaler & 0xff);
    AD1CHS = 0;
    
    //reduce power consumption as long as no conversion is running
    ADC_powerDown();
}

uint32_t ADC_read(uint32_t channel){
    //TODO add a semaphore to make sure that no conversions are started in parallel! (especially for conversions with more than one sample!)
    
    ADC_powerUp();
    
    if(AD1CHSbits.CH0SA != channel){
        //switch channel
        AD1CHSbits.CH0SA = channel;
        //wait one tick after input channel change
        vTaskDelay(1);
    }
    
    AD1CON1CLR = _AD1CON1_DONE_MASK;
    AD1CON1SET = _AD1CON1_SAMP_MASK;
    while(!(AD1CON1 & _AD1CON1_DONE_MASK));
    
    ADC_powerDown();
    
    return ADC1BUF0;
}

static void ADC_powerDown(){
    //is a conversion currently running? If so wait until its done
    while(!(AD1CON1 & _AD1CON1_DONE_MASK));
    
    //power down module
    AD1CON1bits.ON = 0;
}

static void ADC_powerUp(){
    AD1CON1bits.ON = 1;
}