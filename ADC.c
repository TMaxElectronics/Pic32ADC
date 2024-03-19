#include <xc.h>
#include <stdint.h>
#include <limits.h>
#include <sys/attribs.h>

#include "ADC.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "System.h"

static void ADC_powerUp();
static void ADC_powerDown();
static DMA_RINGBUFFERHANDLE_t * ADC_currentBuffer = NULL;

static struct ADC_setup{
    uint32_t ASAM;
    uint32_t sampleRate;
    
    uint32_t scanningEnabled;
};

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
    //perform a read so the done flag is set... TODO implement this properly
    ADC_read(0);
}

uint32_t ADC_getActiveChannelCount(){
    uint32_t channelsActive = 1;
    //is scanning enabled?
    if(AD1CON2bits.CSCNA){
        //yes => check which channels are actually active
        
        //every bit counts as one input with the starting value being 0 TODO: actually understand and explain lol
        channelsActive --;
        
        //scan through the scan input select register
        for(uint32_t i = 0; i < 16; i++){
            if(AD1CSSL & SYS_BITMASK[i]) channelsActive ++;
        }
    }
    
    // is alternate input selection on? If so then every second sample comes from the "B" channel
    if(AD1CON2bits.ALTS){
        channelsActive *= 2;
    }
    
    return channelsActive;
}

#define ADC_MAX_CLOCKRATE_NORMAL 5000000
#define ADC_MAX_CLOCKRATE_EXTREF 15384615
#define ADC_isAutoSampling() AD1CON1bits.ASAM
#define ADC_isScanning() AD1CON2bits.CSCNA
#define ADC_isAlternating() AD1CON2bits.ALTS
#define ADC_isSampling() !AD1CON1bits.DONE

uint32_t ADC_setupAutoSampling(uint32_t enabled, uint32_t sampleRate_Hz){
    //setup automatic sampling at a given sampling rate
    
    //calculate the required divider ratios
    uint32_t totalDividerRatio = configPERIPHERAL_CLOCK_HZ / (2*sampleRate_Hz);
    
    //check if we can even go that slow (maximum divider we can do is 31 * 512)
    if(totalDividerRatio > 256 * (12+0b11111)){
        //hmm the sample rate is so slow we can't divide the clock enough. Return with an error
        return pdFAIL;
    }else{
        //calculate aquisition time (totalDividerRatio = ADCS * (12 + SAMC))
        uint32_t clockDivider = 0;
        uint32_t bestSAMC = 0;

        //step through and find the best match
        int32_t bestError = INT_MAX;
        for(uint32_t currentSAMC = 1; currentSAMC <= 0b11111; currentSAMC++){
            //calculate the adc clock divider resulting from the current postscaler
            int32_t currClockDivider = totalDividerRatio / (12+currentSAMC);
            
            if(currClockDivider == 0) continue;

            //calculate the resulting ACTUAL sample rate, including any potential rounding errors and the resulting total sample rate error
            int32_t effectiveSampleRate_Hz = configPERIPHERAL_CLOCK_HZ / (2 * currClockDivider * (12 + currentSAMC));
            int32_t error = abs(effectiveSampleRate_Hz - sampleRate_Hz);

            //is the error for the current divider lower than the best one so far?
            if(error < bestError){
                //yes => save the current ratios
                bestSAMC = currentSAMC;
                clockDivider = currClockDivider;
                bestError = error;
            }
        }

        //now apply the divider ratios
        AD1CON3bits.SAMC = bestSAMC;
        AD1CON3bits.ADCS = clockDivider - 1;
    }
}

void ADC_selectScanChannels(uint32_t channelsSelected){
    uint32_t resumeSampling = 0;
    
    //check if the adc is currently auto-sampling
    if(ADC_isAutoSampling() && ADC_isScanning()){
        //yes, since we can't change any settings (as the buffer size is not dynamic) we just return
        return;
    }
    
    AD1CSSL = channelsSelected & 0xffff;
}

void ADC_setMuxAConfig(uint32_t posInput, uint32_t negInput){
    AD1CHSbits.CH0NA = negInput;
    AD1CHSbits.CH0SA = posInput;
}

void ADC_setMuxBConfig(uint32_t posInput, uint32_t negInput){
    AD1CHSbits.CH0NB = negInput;
    AD1CHSbits.CH0SB = posInput;
}

void ADC_setInputScan(uint32_t scanningEnabled, uint32_t alternatingEnabled){
    uint32_t resumeSampling = 0;
    
    //check if we will change the scanning setting
    if(ADC_isAutoSampling() && (ADC_isScanning() != scanningEnabled)){
        //yes, since we can't change any settings (as the buffer size is not dynamic) we just return
        return;
    }
    
    //check if we will change the mux alternating setting
    if(ADC_isAutoSampling() && (ADC_isAlternating() != alternatingEnabled)){
        //yes, we need to disable asam so we don't misalign the buffer position and the channels
        ADC_stopAutoSampling();
        resumeSampling = 1;
    }
    
    AD1CON2bits.ALTS = alternatingEnabled;
    AD1CON2bits.CSCNA = scanningEnabled;
}

void ADC_stopAutoSampling(){
    if(ADC_isAutoSampling()){
        //set conversion stop bit
        AD1CON1SET = _AD1CON1_CLRASAM_MASK;
        
        //wait for the conversion to finish. Do it in a non-blocking way if freeRTOS is available
#if __has_include("FreeRTOS.h")
        while(AD1CON1 & _AD1CON1_CLRASAM_MASK) vTaskDelay(1);
#else
        while(AD1CON1 & _AD1CON1_CLRASAM_MASK);
#endif
        //de-initialize the ringbuffer (this will flush all data!)
        DMA_freeRingBuffer(ADC_currentBuffer);
        ADC_currentBuffer = NULL;
    }
}

DMA_RINGBUFFERHANDLE_t * ADC_startAutoSampling(uint32_t sampleCount){
    //are we sampling already?
    if(!ADC_isAutoSampling()){
        //no => start asam
        
        //first power up the ADC so it has time to get ready
        ADC_powerUp();
        
        //first make sure that the interrupt sequence is reset and the sample size set appropriately
        //reset sequence (TODO evaluate if this actually does what I expect it to do)
        AD1CON2bits.SMPI = 0;
        
        //and then reset it to the correct value
        uint32_t channelsActive = ADC_getActiveChannelCount();
        if(channelsActive > 16){ 
            //TODO handle this case 
            channelsActive = 16;
        }
        AD1CON2bits.SMPI = channelsActive - 1;
        
        //setup the dma ringbuffer to take data out of the ADC
        ADC_currentBuffer = DMA_createRingBuffer(sampleCount * sizeof(ADC_ADCBUF_t) * channelsActive, sizeof(ADC_ADCBUF_t) * channelsActive, &ADC1BUF0, _ADC_IRQ, 2, RINGBUFFER_DIRECTION_RX);
        
        //enable debug isr
        IPC5bits.AD1IP = 3;
        IEC0bits.AD1IE = 0;
        
        //start the conversion
        AD1CON1bits.ASAM = 1;
    }
    return ADC_currentBuffer;
}

void ADC_setVrefSource(ADC_RefSource_t pRef, ADC_RefSource_t nRef){
    uint32_t reference = ((pRef == ADC_REF_EXTREF) ? 0b1 : 0) | ((nRef == ADC_REF_EXTREF) ? 0b10 : 0);
    AD1CON2bits.VCFG = reference & 0b11;
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

static void __ISR(_ADC_VECTOR) adcISR(){
    IFS0CLR = _IFS0_AD1IF_MASK;
    LATBINV = _LATB_LATB6_MASK;
}