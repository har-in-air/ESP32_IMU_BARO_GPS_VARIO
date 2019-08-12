#include "common.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "myadc.h"

#define TAG "adc"


#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   4          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_7;     //GPIO35 ( ADC1 )
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;

static void check_efuse();
static void print_char_val_type(esp_adc_cal_value_t val_type);

static void check_efuse(){
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        ESP_LOGD(TAG,"eFuse Two Point: Supported\n");
        } 
    else {
        ESP_LOGD(TAG,"eFuse Two Point: NOT supported\n");
        }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        ESP_LOGD(TAG,"eFuse Vref: Supported\n");
        } 
    else {
        ESP_LOGD(TAG,"eFuse Vref: NOT supported\n");
        }
    }

static void print_char_val_type(esp_adc_cal_value_t val_type){
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        ESP_LOGD(TAG,"Characterized using Two Point Value\n");
        } 
    else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        ESP_LOGD(TAG,"Characterized using eFuse Vref\n");
        } 
    else {
        ESP_LOGD(TAG,"Characterized using Default Vref\n");
        }
    }

void adc_init(){
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel, atten);

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
    }
    
uint32_t adc_sampleMV() {
    uint32_t adc_reading = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        adc_reading += adc1_get_raw((adc1_channel_t)channel);
        }
    adc_reading /= NO_OF_SAMPLES;
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    ESP_LOGD(TAG,"Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
    return voltage;
    }


