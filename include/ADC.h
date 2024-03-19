#ifndef ADC_INC
#define ADC_INC

#include <xc.h>
#include "DMA.h"
#include "DMAutils.h"

typedef enum {ADC_REF_SUPPLY, ADC_REF_EXTREF} ADC_RefSource_t;
typedef enum {ADC_FORMAT_UINT16 = 0b000, ADC_FORMAT_INT16 = 0b001, ADC_FORMAT_FRAC_UINT16 = 0b010, ADC_FORMAT_FRAC_INT16 = 0b011, ADC_FORMAT_UINT32 = 0b100, ADC_FORMAT_INT32 = 0b101, ADC_FORMAT_FRAC_UINT32 = 0b110, ADC_FORMAT_FRAC_INT32 = 0b111} ADC_OUTPUT_FORMAT_t;

typedef struct{
    uint32_t sample;
    uint32_t microchipIsAStupidPieceOfShit;
    uint32_t likeSerioulsyWhoImplementedThisStupidADCRegisterMap;
    uint32_t iReallyWonderIfTheyEverUsedTheirOwnHardware;
} ADC_ADCBUF_t;

void ADC_init(ADC_OUTPUT_FORMAT_t format, uint32_t conversionTime_us);
uint32_t ADC_read(uint32_t channel);

uint32_t ADC_getActiveChannelCount();
uint32_t ADC_setupAutoSampling(uint32_t enabled, uint32_t sampleRate_Hz);
void ADC_selectScanChannels(uint32_t channelsSelected);
void ADC_setMuxAConfig(uint32_t posInput, uint32_t negInput);
void ADC_setMuxBConfig(uint32_t posInput, uint32_t negInput);
void ADC_setInputScan(uint32_t scanningEnabled, uint32_t alternatingEnabled);
void ADC_stopAutoSampling();
DMA_RINGBUFFERHANDLE_t * ADC_startAutoSampling(uint32_t sampleCount);
void ADC_setVrefSource(ADC_RefSource_t pRef, ADC_RefSource_t nRef);

#endif