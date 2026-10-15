#include "ntc_adc.h"
#include <math.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

#define NTC_ADC_UNIT            ADC_UNIT_1
#define NTC_ADC_CHANNEL         ADC_CHANNEL_6   // Chân D34 (GPIO34) trên Schematic
#define NTC_ADC_ATTEN           ADC_ATTEN_DB_12
#define NTC_VCC_MV              3300.0f
#define NTC_R3_PULLDOWN_OHM     10000.0f        // Điện trở R3 = 10k nối GND trên Schematic
#define NTC_R25_OHM             10000.0f        // NTC 10k tại 25 C
#define NTC_BETA                3950.0f         // NTC B3950 10K ghi trên Schematic
#define NTC_T25_KELVIN          298.15f

// Ngưỡng phát hiện đứt dây (< 100mV) hoặc chập NTC (> 3200mV)
#define NTC_FAULT_MIN_MV        100
#define NTC_FAULT_MAX_MV        3200

static const char *TAG = "NTC_ADC";
static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_cali_handle_t s_cali_handle = NULL;
static bool s_cali_enabled = false;

esp_err_t ntc_adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = NTC_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &s_adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = NTC_ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, NTC_ADC_CHANNEL, &chan_cfg));

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = NTC_ADC_UNIT,
        .atten = NTC_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_line_fitting(&cali_cfg, &s_cali_handle) == ESP_OK) {
        s_cali_enabled = true;
    }
#endif

    ESP_LOGI(TAG, "Khởi tạo ADC đọc NTC B3950 10K tại GPIO34 (NTC nối 3.3V, R3=10k nối GND).");
    return ESP_OK;
}

esp_err_t ntc_adc_read_temp(float *temp_hot_c)
{
    if (!temp_hot_c || !s_adc_handle) return ESP_ERR_INVALID_ARG;

    int raw = 0;
    esp_err_t err = adc_oneshot_read(s_adc_handle, NTC_ADC_CHANNEL, &raw);
    if (err != ESP_OK) return err;

    int voltage_mv = 0;
    if (s_cali_enabled) {
        adc_cali_raw_to_voltage(s_cali_handle, raw, &voltage_mv);
    } else {
        voltage_mv = (raw * 3300) / 4095;
    }

    // Kiểm tra đứt dây NTC (điện áp kéo về ~0V bởi R3) hoặc chập NTC (điện áp vọt lên ~3.3V)
    if (voltage_mv < NTC_FAULT_MIN_MV || voltage_mv > NTC_FAULT_MAX_MV) {
        ESP_LOGE(TAG, "Lỗi phần cứng NTC trên chân GPIO34 (V=%d mV, Raw=%d)!", voltage_mv, raw);
        return ESP_ERR_INVALID_STATE;
    }

    // Công thức chuẩn theo Schematic: Header 2P (NTC) nối 3.3V, R3 (10k) nối GND
    float r_ntc = NTC_R3_PULLDOWN_OHM * ((NTC_VCC_MV - (float)voltage_mv) / (float)voltage_mv);
    float inv_t = (1.0f / NTC_T25_KELVIN) + (1.0f / NTC_BETA) * logf(r_ntc / NTC_R25_OHM);
    *temp_hot_c = (1.0f / inv_t) - 273.15f;

    return ESP_OK;
}