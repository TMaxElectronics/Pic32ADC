#ifndef ADC_INC
#define ADC_INC

#include <xc.h>

typedef enum {ADC_FORMAT_UINT16 = 0b000, ADC_FORMAT_INT16 = 0b001, ADC_FORMAT_FRAC_UINT16 = 0b010, ADC_FORMAT_FRAC_INT16 = 0b011, ADC_FORMAT_UINT32 = 0b100, ADC_FORMAT_INT32 = 0b101, ADC_FORMAT_FRAC_UINT32 = 0b110, ADC_FORMAT_FRAC_INT32 = 0b111} ADC_OUTPUT_FORMAT_t;

void ADC_init(ADC_OUTPUT_FORMAT_t format, uint32_t conversionTime_us);
uint32_t ADC_read(uint32_t channel);

#endif