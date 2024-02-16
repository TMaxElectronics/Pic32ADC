#include <xc.h>
#include <stdint.h>

#include "ADC.h"
#include "FreeRTOSConfig.h"


void ADC_init(ADC_OUTPUT_FORMAT_t format, uint32_t conversionTime_us){
    //set basic module options
    AD1CON1 = 0b1000000011100000 | ((format & 0x7) << 8);
    AD1CON2 = 0;
    
    uint32_t autoSampleScaler = 0;
    uint32_t clockScaler = (((configPERIPHERAL_CLOCK_HZ / 1000) * conversionTime_us) / 24000) - 1;
    if(clockScaler > 0xff) clockScaler = 0xff;
    AD1CON3 = ((autoSampleScaler & 0x1f) << 8) | (clockScaler & 0xff);
    AD1CHS = 0;
}

uint32_t ADC_read(uint32_t channel){
    if(AD1CHSbits.CH0SA != channel){
        //switch channel
        AD1CHSbits.CH0SA = channel;
        //wait one tick after input channel change
        vTaskDelay(1);
    }
    
    AD1CON1CLR = _AD1CON1_DONE_MASK;
    AD1CON1SET = _AD1CON1_SAMP_MASK;
    while(!(AD1CON1 & _AD1CON1_DONE_MASK));
    
    return ADC1BUF0;
}