

#include <stdint.h> 
#include <stdio.h> //saadc
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_hrs.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "boards.h"//k
#include "sensorsim.h"
//#include "softdevice_handler.h"//k
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "app_timer.h"
#include "bsp.h"//k
#include "nrf_delay.h"//k
#include "bsp_btn_ble.h"
#include "peer_manager.h"
#include "fds.h"
//#include "fstorage.h"//k
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "ble_conn_state.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"
#include "app_util_platform.h" //k
#include "nrf_drv_power.h" //k
#include "nrf_drv_lpcomp.h"//k

#include "nrf_drv_pwm.h"

// #define ENABLE_LOG

#define DEVICE_NAME                         "04222022_spo2_100hz_3"                            /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                   "NordicSemiconductor"                   /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                    500 //org 300, 2056 = 1285ms                                     
                                           //org:300 /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION                    30000                                 /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define APP_BLE_CONN_CFG_TAG                1                                       /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO               3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */

//#define BATTERY_LEVEL_MEAS_INTERVAL         APP_TIMER_TICKS(2000)                   /**< Battery level measurement interval (ticks). */
//#define MIN_BATTERY_LEVEL                   81                                      /**< Minimum simulated battery level. */
//#define MAX_BATTERY_LEVEL                   100                                     /**< Maximum simulated 7battery level. */
//#define BATTERY_LEVEL_INCREMENT             1                                       /**< Increment between each simulated battery level measurement. */

#define APP_TIMER_TICKS_US(US)              ((uint32_t)ROUNDED_DIV((US)*(uint64_t)APP_TIMER_CLOCK_FREQ, 1000000*(APP_TIMER_CONFIG_RTC_FREQUENCY+1)))
#define HEART_RATE_MEAS_INTERVAL0           APP_TIMER_TICKS_US(10000)                   /**< Heart rate measurement interval (ticks). 1ticks=1ms*/
// #define HEART_RATE_MEAS_INTERVAL1           APP_TIMER_TICKS(12000)                   /**< Heart rate measurement interval (ticks). 1ticks=1ms*/
#define MIN_HEART_RATE                      140                                     /**< Minimum heart rate as returned by the simulated measurement function. */
#define MAX_HEART_RATE                      300                                     /**< Maximum heart rate as returned by the simulated measurement function. */
#define HEART_RATE_INCREMENT                10                                      /**< Value by which the heart rate is incremented/decremented for each call to the simulated measurement function. */

#define DISCHARGE_INTERVAL                  APP_TIMER_TICKS(5000)

//#define RR_INTERVAL_INTERVAL                APP_TIMER_TICKS(300)                    /**< RR interval interval (ticks). */
//#define MIN_RR_INTERVAL                     100                                     /**< Minimum RR interval as returned by the simulated measurement function. */
//#define MAX_RR_INTERVAL                     500                                     /**< Maximum RR interval as returned by the simulated measurement function. */
//#define RR_INTERVAL_INCREMENT               1                                       /**< Value by which the RR interval is incremented/decremented for each call to the simulated measurement function. */

//#define SENSOR_CONTACT_DETECTED_INTERVAL    APP_TIMER_TICKS(5000)                   /**< Sensor Contact Detected toggle interval (ticks). */

#define MIN_CONN_INTERVAL                   MSEC_TO_UNITS(10, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                   MSEC_TO_UNITS(30, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                       0            //3                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    MSEC_TO_UNITS(6000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY      APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY       APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT        3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                      1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                      0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                  0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                       0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE              7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE              16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                           0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define SAMPLES_IN_BUFFER 4
//#define MON_EN 20 
//#define HEAT_EN 19
//#define DSCB_EN 22

//#define LED_ON 12
//#define LEDA_ON   BSP_BOARD_LED_2
//#define LEDB_ON   BSP_BOARD_LED_3 
//#define LED1   BSP_BOARD_LED_0 

#define BUTTON_PREV_ID           0                           /**< Button used to switch the state. */
#define BUTTON_NEXT_ID           1                           /**< Button used to switch the state. */

//#define LPCOMP_ENABLED  1
//#define LPCOMP_CONFIG_REFERENCE 3
// <0=> Supply 1/8 , <1=> Supply 2/8 , <2=> Supply 3/8 , <3=> Supply 4/8 
// <4=> Supply 5/8 , <5=> Supply 6/8 , <6=> Supply 7/8 , <8=> Supply 1/16 
// <9=> Supply 3/16 , <10=> Supply 5/16 , <11=> Supply 7/16 , <12=> Supply 9/16    
// <13=> Supply 11/16  ,<14=> Supply 13/16  , <15=> Supply 15/16 , <7=> External Ref 0 
// <65543=> External Ref 1 

//#define LPCOMP_CONFIG_DETECTION 1
// <0=> Crossing, <1=> Up, <2=> Down 

BLE_HRS_DEF(m_hrs);                                                 /**< Heart rate service instance. */
BLE_BAS_DEF(m_bas);                                                 /**< Structure used to identify the battery service. */
NRF_BLE_GATT_DEF(m_gatt);                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                 /**< Advertising module instance. */
//APP_TIMER_DEF(m_battery_timer_id);                                  /**< Battery timer. */
APP_TIMER_DEF(m_heart_rate_timer_id);                               /**< Heart rate measurement timer. */

static uint16_t m_conn_handle         = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

//static int lpcomp_flag = 0;
bool erase_bonds ;

//AC Voltage
int DSCA_flag=0;
int DSCB_flag=0; 
int test_i=0;


static uint8_t ble_counter = 0;
static uint8_t avg_counter = 0;
static uint8_t avg_ble_counter = 0;
static uint32_t temp_avg[SAMPLES_IN_BUFFER];
static double temp_buf[SAMPLES_IN_BUFFER];
//static uint8_t avg_ble_data[36]; //org:20
static uint8_t avg_ble_data[48];

#define MAX_ADC_VAL 4095
#define UPPER_THR 90 // in %
#define LOWER_THR 10 // in %
static uint8_t saadc_gain[SAMPLES_IN_BUFFER];
static double saadc_gain_scale[8] = {1.0/6.0, 1.0/5.0, 1.0/4.0, 1.0/3.0, 1.0/2.0, 1.0, 2.0, 4.0};

static nrf_saadc_value_t     m_buffer_pool[3][SAMPLES_IN_BUFFER];

static ble_uuid_t m_adv_uuids[] =                                   /**< Universally unique service identifiers. */
{
    {BLE_UUID_HEART_RATE_SERVICE,           BLE_UUID_TYPE_BLE},
    {BLE_UUID_BATTERY_SERVICE,              BLE_UUID_TYPE_BLE},
    {BLE_UUID_DEVICE_INFORMATION_SERVICE,   BLE_UUID_TYPE_BLE}
};


///////////////////////////////// updated, 2022.02.03, do /////////////////////////////////
static uint16_t cur_brightness[2] = {0,0};
static uint16_t m_top = 10000;
#define pwm_seq_len                         100
#define ir_led                              22
#define red_led                             23
static nrfx_pwm_t m_pwm0 = NRFX_PWM_INSTANCE(0);                            
static nrf_pwm_values_grouped_t sequence_values[pwm_seq_len];               // sequence containing the intensity
static double UPPER_LED_THR = 0.8;
static double LOWER_LED_THR = 0.6;
static double MIDDLE_LED_THR = 0.7;
#define LIGHT_INTENSITY_COUNT               1000                              // Controls light intensity update duration
static uint16_t led_counter = 0;
static uint32_t temp_avg_light[2];
#define LIGHT_INTENSITY_STEP                10                              // varies from 1 ~ 10 with 10 as maximum value
#define LIGHT_INTENSITY_BASE                71                              // Base GPIO voltage in % (ex. currently base GPIO value is 91% of 2.9XXXV, which is 2.7V
///////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////// updated, 2022.02.03, do /////////////////////////////////
static void pwm_common_init(void) // pwm initiation
{
  //for(int i = 0 ; i < pwm_seq_len ; i++){sequence_values[i].group_0 = 0;}
  //for(int i = 0 ; i < pwm_seq_len ; i++){sequence_values[i].group_1 = 0;}
  for(int i = 0 ; i < pwm_seq_len ; i++){sequence_values[i].group_0 = m_top*(100-LIGHT_INTENSITY_BASE)/100;}
  for(int i = 0 ; i < pwm_seq_len ; i++){sequence_values[i].group_1 = m_top*(100-LIGHT_INTENSITY_BASE)/100;} // m_top means the highest pwm level. Higher pwm level means lower output
  nrfx_pwm_config_t const config0 = 
  {
      .output_pins=
      {
          BSP_LED_0 | NRFX_PWM_PIN_INVERTED,                                  // 4 output pins that are going to be used. led_external & led_external_2
          ir_led,                                                       // are IR & red output pins
          BSP_LED_1 | NRFX_PWM_PIN_INVERTED,                                  // Board LEDs are just for indication!
          red_led                                                      // pins can be left unused by using NRFX_PWM_PIN_NOT_USED 
      },
      .irq_priority = APP_IRQ_PRIORITY_LOWEST,
      .base_clock = NRF_PWM_CLK_8MHz,                                         // clock used in PWM
      .count_mode = NRF_PWM_MODE_UP,
      .top_value = m_top,                                                     // selects the maximum pwm value
      .load_mode = NRF_PWM_LOAD_GROUPED,                                      // grouped mode means 4 control output pins are divided into two groups which acts in a same way
      .step_mode = NRF_PWM_STEP_AUTO
  };
  APP_ERROR_CHECK(nrfx_pwm_init(&m_pwm0, &config0, NULL));
}

static void pwm_play(void)
{
    nrf_pwm_sequence_t const seq0 = 
    {
      .values.p_grouped = sequence_values,
      .length         = NRF_PWM_VALUES_LENGTH(sequence_values),
      .repeats        = 0,
      .end_delay      = 0
    };

    (void)nrfx_pwm_simple_playback(&m_pwm0, &seq0, 1, NRFX_PWM_FLAG_LOOP);    // plays pwm in a loop

}

////////////////////////////////////////////////////////////////////////////////////////////////
////////////this function determines the brightness of IR & RED by modifying PWM seq////////////
////////////idx : indicates IR or LED, brightness : determines the brightness //////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
static void pwm_common_set(int idx, uint16_t brightness)
{
    switch(idx)
    {
    
        case 0:
            for(int i = 0 ; i < pwm_seq_len ; i++){
                sequence_values[i].group_0 = (100-LIGHT_INTENSITY_BASE) * m_top/100 - m_top * (100-LIGHT_INTENSITY_BASE) * brightness/10000;
                //sequence_values[i].group_0 = 5;
            }
            break;

        case 1:
            for(int i = 0 ; i < pwm_seq_len ; i++){
                sequence_values[i].group_1 = (100-LIGHT_INTENSITY_BASE) * m_top/100 - m_top * (100-LIGHT_INTENSITY_BASE) * brightness/10000;
            }
            break;

        default:
            NRF_LOG_INFO("Should not reach here..");
            break;
    }

}
///////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    #ifdef ENABLE_LOG
    NRF_LOG_INFO("Erase bonds!");
    #endif

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


void advertising_start(bool erase_bonds)
{
    //nrf_drv_gpiote_out_clear(HEAT_EN);
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        ret_code_t err_code;

        err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }
    //nrf_drv_gpiote_out_clear(HEAT_EN);
}


void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        // ret_code_t err_code;
        
        nrf_drv_saadc_uninit(); 
        //nrf_drv_gpiote_out_clear(HEAT_EN);

        uint16_t temp_ch;
        uint16_t temp_light;
        uint16_t temp_avg2;        
        int i, j=0;
        double beta=0.05;//;0.2; //0.01
        
        for(i=0;i<SAMPLES_IN_BUFFER;i++)
        {
            temp_ch=(uint16_t)(p_event->data.done.p_buffer[(i%2)+2]);
            if(temp_ch>0x0FFF){
                temp_ch=0;
            }
            temp_avg[i]+=temp_ch;
            if(i>1)
            {
                temp_avg_light[i-2] += temp_ch;
            }
        }
        avg_counter++;
        led_counter++;

        if(led_counter == LIGHT_INTENSITY_COUNT)
        {
            for(i = 2; i<SAMPLES_IN_BUFFER; i++)
            {
                temp_light = temp_avg_light[i-2] / led_counter;

                if(temp_light <= (int) MAX_ADC_VAL * LOWER_LED_THR)
                {
                    if(cur_brightness[i-2] < 100)
                    {
                        cur_brightness[i-2] = cur_brightness[i-2] + (MAX_ADC_VAL * MIDDLE_LED_THR - temp_light) * 10 * LIGHT_INTENSITY_STEP / MAX_ADC_VAL;    // adaptive adaptation
                        //cur_brightness[i-2] = cur_brightness[i-2] + LIGHT_INTENSITY_STEP;                                                               // constant adaptation
                        if(cur_brightness[i-2] > 100) {cur_brightness[i-2] = 100;}
                        pwm_common_set(i-2, cur_brightness[i-2]);
                    }
                    else{NRF_LOG_INFO("Already maximum brightness!");}
                }

                if(temp_light >= (int) MAX_ADC_VAL * UPPER_LED_THR)
                {
                    if(cur_brightness[i-2] > 0)
                    {
                        cur_brightness[i-2] = cur_brightness[i-2] - (temp_light - MIDDLE_LED_THR * MAX_ADC_VAL) * 10 * LIGHT_INTENSITY_STEP / MAX_ADC_VAL;      // adaptive adaptation
                        //cur_brightness[i-2] = cur_brightness[i-2] - LIGHT_INTENSITY_STEP;                                                                 // constant adaptation
                        if(cur_brightness[i-2] < 0){cur_brightness[i-2] = 0;}
                        pwm_common_set(i-2, cur_brightness[i-2]);
                    }
                    else{NRF_LOG_INFO("Already minimum brightness!");}
                }
                temp_avg_light[i-2] = 0;
            }
            led_counter = 0;
        }

        //s4 -> avg_counter=100 -> 2ch* 8 data per 4s 
        //s8 -> avg_counter=200 -> 2ch* 8 data per 8s 
        if(avg_counter==1){//recent 10  //20-> 10 data per 1s, org 25-> new 2ch * 8 data per 1s   // 50 2ch* 8 data per 2s    
            for(j=0;j<SAMPLES_IN_BUFFER;j++)
            {
                //temp_avg[j]=(uint32_t)(temp_avg[j]/avg_counter);
               
                //*************************************************************//
                //avg_ble_data[avg_ble_counter++] = (uint8_t)((temp_avg[j] & 0xFF));
                //avg_ble_data[avg_ble_counter++] = (uint8_t)((temp_avg[j] >> 8));                
                //*************************************************************//

                //****************************MEV****************************//
                
                if (ble_counter<SAMPLES_IN_BUFFER){
                    temp_buf[j]=(double)(temp_avg[j]/avg_counter);
                    ble_counter+=1;
                }else{//digital low pass filtering
                   temp_buf[j]=(1.0-beta)*temp_buf[j]+beta*(double)(temp_avg[j]/avg_counter);
                }

                temp_avg2=(uint16_t)(temp_buf[j]/saadc_gain_scale[saadc_gain[j]]);
                //temp_avg2 = (uint16_t) avg_ble_counter;
                
                //temp_avg2=(uint16_t)((test_i++)+(j)*5000);

                if(temp_buf[j] > (double)(MAX_ADC_VAL * UPPER_THR)/100.0){
                    if(saadc_gain[j] > 0){
                        //saadc_gain[j]--;                       
                    }
                } else if(temp_buf[j] < (double)(MAX_ADC_VAL * LOWER_THR)/100.0){
                    if(saadc_gain[j] < 7){
                        //saadc_gain[j]++;
                    }
                }
                //saadc_gain[2]=0;
                avg_ble_data[avg_ble_counter] = (uint8_t)((temp_avg2 & 0xFF));
                                //avg_ble_data[avg_ble_counter] = (uint8_t)(avg_ble_counter);//((temp_avg2 & 0xFF));
                avg_ble_counter++;
                avg_ble_data[avg_ble_counter] = (uint8_t)((temp_avg2 >> 8));
                avg_ble_counter++;
                //*************************************************************//

                temp_avg[j]=0;                
            } 
            avg_counter=0;
            
            if(avg_ble_counter == 32){ //org:20    // Number of data per ch = 32/2/ch; 
                ble_hrs_heart_rate_measurement_send32(&m_hrs, avg_ble_data);
                avg_ble_counter = 0;
            }   
        }   
    }
}


void saadc_init(void)
{
    ret_code_t err_code;

    nrf_drv_saadc_config_t saadc_config = NRF_DRV_SAADC_DEFAULT_CONFIG;
    saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;

    nrf_saadc_channel_config_t channel_0_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2); //AIN5, 29(5) - 2, 3, 4, 5, 28, 29, 30, 31
        switch(saadc_gain[0]){
            case 0 : channel_0_config.gain = NRF_SAADC_GAIN1_6; break;
            case 1 : channel_0_config.gain = NRF_SAADC_GAIN1_5; break;
            case 2 : channel_0_config.gain = NRF_SAADC_GAIN1_4; break;
            case 3 : channel_0_config.gain = NRF_SAADC_GAIN1_3; break;
            case 4 : channel_0_config.gain = NRF_SAADC_GAIN1_2; break;
            case 5 : channel_0_config.gain = NRF_SAADC_GAIN1;   break;
            case 6 : channel_0_config.gain = NRF_SAADC_GAIN2;   break;
            case 7 : channel_0_config.gain = NRF_SAADC_GAIN4;   break;
            default: channel_0_config.gain = NRF_SAADC_GAIN1;   break;
        }
      //channel_0_config.gain = NRF_SAADC_GAIN1;
      channel_0_config.reference = NRF_SAADC_REFERENCE_INTERNAL;
        channel_0_config.acq_time = NRF_SAADC_ACQTIME_40US;

    nrf_saadc_channel_config_t channel_1_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3); //AIN1, 2(0) - 2, 3, 4, 5, 28, 29, 30, 31
        switch(saadc_gain[1]){
                case 0 : channel_1_config.gain = NRF_SAADC_GAIN1_6; break;
                case 1 : channel_1_config.gain = NRF_SAADC_GAIN1_5; break;
                case 2 : channel_1_config.gain = NRF_SAADC_GAIN1_4; break;
                case 3 : channel_1_config.gain = NRF_SAADC_GAIN1_3; break;
                case 4 : channel_1_config.gain = NRF_SAADC_GAIN1_2; break;
                case 5 : channel_1_config.gain = NRF_SAADC_GAIN1;   break;
                case 6 : channel_1_config.gain = NRF_SAADC_GAIN2;   break;
                case 7 : channel_1_config.gain = NRF_SAADC_GAIN4;   break;
                default: channel_1_config.gain = NRF_SAADC_GAIN1;   break;
        }
      //channel_1_config.gain = NRF_SAADC_GAIN1;
      channel_1_config.reference = NRF_SAADC_REFERENCE_INTERNAL;
        channel_1_config.acq_time = NRF_SAADC_ACQTIME_40US;

     nrf_saadc_channel_config_t channel_2_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN4); //AIN1, 2(0) - 2, 3, 4, 5, 28, 29, 30, 31
        switch(saadc_gain[2]){
                case 0 : channel_2_config.gain = NRF_SAADC_GAIN1_6; break;
                case 1 : channel_2_config.gain = NRF_SAADC_GAIN1_5; break;
                case 2 : channel_2_config.gain = NRF_SAADC_GAIN1_4; break;
                case 3 : channel_2_config.gain = NRF_SAADC_GAIN1_3; break;
                case 4 : channel_2_config.gain = NRF_SAADC_GAIN1_2; break;
                case 5 : channel_2_config.gain = NRF_SAADC_GAIN1;   break;
                case 6 : channel_2_config.gain = NRF_SAADC_GAIN2;   break;
                case 7 : channel_2_config.gain = NRF_SAADC_GAIN4;   break;
                default: channel_2_config.gain = NRF_SAADC_GAIN1;   break;
        }
      //channel_1_config.gain = NRF_SAADC_GAIN1;
      channel_2_config.reference = NRF_SAADC_REFERENCE_INTERNAL;
        channel_2_config.acq_time = NRF_SAADC_ACQTIME_10US;


    nrf_saadc_channel_config_t channel_3_config=
      NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN5);
      switch(saadc_gain[3]){
              case 0 : channel_3_config.gain = NRF_SAADC_GAIN1_6; break;
              case 1 : channel_3_config.gain = NRF_SAADC_GAIN1_5; break;
              case 2 : channel_3_config.gain = NRF_SAADC_GAIN1_4; break;
              case 3 : channel_3_config.gain = NRF_SAADC_GAIN1_3; break;
              case 4 : channel_3_config.gain = NRF_SAADC_GAIN1_2; break;
              case 5 : channel_3_config.gain = NRF_SAADC_GAIN1; break;
              case 6 : channel_3_config.gain = NRF_SAADC_GAIN2; break;
              case 7 : channel_3_config.gain = NRF_SAADC_GAIN4; break;
              default: channel_3_config.gain = NRF_SAADC_GAIN1; break;
      }
      channel_3_config.reference = NRF_SAADC_REFERENCE_INTERNAL;
      channel_3_config.acq_time = NRF_SAADC_ACQTIME_10US;


    err_code = nrf_drv_saadc_channel_init(0, &channel_0_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(1, &channel_1_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(2, &channel_2_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(3, &channel_3_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_int_enable_check(0x0);
}

static void saadc_timer_handler(void * p_context)
{
   UNUSED_PARAMETER(p_context);

   ret_code_t err_code;
    //bsp_board_led_off(SLEEP_LED);
    //nrf_drv_gpiote_out_set(HEAT_EN);
    //nrf_delay_ms(5);
    saadc_init();
   err_code = nrf_drv_saadc_sample();
   APP_ERROR_CHECK(err_code);
}


static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
   err_code = app_timer_create(&m_heart_rate_timer_id,
                        APP_TIMER_MODE_REPEATED,
                        saadc_timer_handler); // kwon
                                //(void *)saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief GATT module event handler.
 */
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("GATT ATT MTU on connection 0x%x changed to %d.",
                     p_evt->conn_handle,
                     p_evt->params.att_mtu_effective);
    }

    ble_hrs_on_gatt_evt(&m_hrs, p_evt);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */

static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */

static void services_init(void)
{
    ret_code_t         err_code;
    ble_hrs_init_t     hrs_init;
    //ble_bas_init_t     bas_init;
    ble_dis_init_t     dis_init;
    nrf_ble_qwr_init_t qwr_init = {0};
    uint8_t            body_sensor_location;

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Heart Rate Service.
    body_sensor_location = BLE_HRS_BODY_SENSOR_LOCATION_FINGER;

    memset(&hrs_init, 0, sizeof(hrs_init));

    hrs_init.evt_handler                 = NULL;
    hrs_init.is_sensor_contact_supported = true;
    hrs_init.p_body_sensor_location      = &body_sensor_location;

    // Here the sec level for the Heart Rate Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hrs_init.hrs_hrm_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_hrm_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_hrm_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hrs_init.hrs_bsl_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_bsl_attr_md.write_perm);

    err_code = ble_hrs_init(&m_hrs, &hrs_init);
    APP_ERROR_CHECK(err_code);

    /*
    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);
    */
    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}

static void application_timers_start(void)
{
    ret_code_t err_code;

    // Start application timers.
    
    err_code = app_timer_start(m_heart_rate_timer_id, HEART_RATE_MEAS_INTERVAL0, NULL);
    APP_ERROR_CHECK(err_code);
    
    //err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    //APP_ERROR_CHECK(err_code);

    //err_code = app_timer_start(m_rr_interval_timer_id, RR_INTERVAL_INTERVAL, NULL);
    //APP_ERROR_CHECK(err_code);

    //err_code = app_timer_start(m_sensor_contact_timer_id, SENSOR_CONTACT_DETECTED_INTERVAL, NULL);
    //APP_ERROR_CHECK(err_code);
    
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    /*
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }*/
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = m_hrs.hrm_handles.cccd_handle;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    //err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    //APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

/*1001
static void lpcomp_event_handler(nrf_lpcomp_event_t event)
{
    
    //uint32_t     err_code;
    if (event == NRF_LPCOMP_EVENT_UP)
    {
        advertising_start(erase_bonds);

    }
    
    //lpcomp_uninit();
    //while (NRF_LPCOMP->EVENTS_READY == 1);
    //NRF_LPCOMP->EVENTS_READY = ;    
}

static void lpcomp_init(void)
{
   uint32_t                err_code;

   nrf_drv_lpcomp_config_t config = NRF_DRV_LPCOMP_DEFAULT_CONFIG;

   config.hal.detection = 1;
    // <0=> Crossing, <1=> Up, <2=> Down 

   config.hal.reference = 8;
    // <0=> Supply 1/8 , <1=> Supply 2/8 , <2=> Supply 3/8 , <3=> Supply 4/8 
    // <4=> Supply 5/8 , <5=> Supply 6/8 , <6=> Supply 7/8 , <8=> Supply 1/16 
    // <9=> Supply 3/16 , <10=> Supply 5/16 , <11=> Supply 7/16 , <12=> Supply 9/16    
    // <13=> Supply 11/16  ,<14=> Supply 13/16  , <15=> Supply 15/16 , <7=> External Ref 0 
    // <65543=> External Ref 1 

   config.input = NRF_LPCOMP_INPUT_4;
   // initialize LPCOMP driver, from this point LPCOMP will be active and provided
   // event handler will be executed when defined action is detected

   err_code = nrf_drv_lpcomp_init(&config, lpcomp_event_handler);
   APP_ERROR_CHECK(err_code);
   nrf_drv_lpcomp_enable();
}*/

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    //ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
        #ifdef ENABLE_LOG
            NRF_LOG_INFO("Fast advertising.");
        #endif
            //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            //APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            /*1001    
            lpcomp_init();//lpcomp
            while (NRF_LPCOMP->EVENTS_READY == 0);
            
            lpcomp_flag=1;//lpcomp
            sleep_mode_enter();
            */
            advertising_start(erase_bonds);
            break;

        default:
            break;
    }
}
/*1001
static void lpcomp_uninit(void)
{
   nrf_drv_lpcomp_uninit();   
   nrf_drv_lpcomp_disable();
}*/

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        #ifdef ENABLE_LOG
            NRF_LOG_INFO("Connected.");
        #endif
            //err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            //APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            //saadc_init();
            pwm_play();
            application_timers_start(); // added by kwon
            
            //High Current for the Heat
            /*
            nrf_gpio_cfg_output(HEAT_EN);
            
            nrf_gpio_cfg(HEAT_EN,
            NRF_GPIO_PIN_DIR_OUTPUT,
            NRF_GPIO_PIN_INPUT_DISCONNECT,
            NRF_GPIO_PIN_NOPULL,
            NRF_GPIO_PIN_H0H1, 
            NRF_GPIO_PIN_NOSENSE);
            
            nrf_drv_gpiote_out_set(HEAT_EN); 
            */
            // enable the heat   
            //nrf_drv_gpiote_out_clear(HEAT_EN); 
            //discharge();                // added by kwon
            //uint32_t err_code1 = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, -40); 
           //APP_ERROR_CHECK(err_code1);   
            /*1001
            if(lpcomp_flag == 1)
            {
                lpcomp_flag = 0;
                lpcomp_uninit();
            }*/           
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected, reason %d.",
                          p_ble_evt->evt.gap_evt.params.disconnected.reason);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
           // nrf_drv_gpiote_out_clear(HEAT_EN); 
            advertising_start(erase_bonds);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */

void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;
        /*
        case BSP_EVENT_KEY_0:
            err_code = app_timer_start(m_heart_rate_timer_id, HEART_RATE_MEAS_INTERVAL0, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BSP_EVENT_KEY_1:
            err_code = app_timer_start(m_heart_rate_timer_id, HEART_RATE_MEAS_INTERVAL1, NULL);
            APP_ERROR_CHECK(err_code);
            break;
        */
        default:
            break;
    }
}


/**@brief Function for the Peer Manager initialization.
 */
/*
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = fds_register(fds_evt_handler);
    APP_ERROR_CHECK(err_code);
}*/


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */

static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


//@brief Function for initializing the nrf log module.
/*
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}
*/


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    //if (NRF_LOG_PROCESS() == false)
    //{
        nrf_pwr_mgmt_run();
    //}
}

/*
static void gpio_init(void)
{
    ret_code_t err_code=NRFX_SUCCESS;

    if(!nrf_drv_gpiote_is_init())
    {
                err_code = nrf_drv_gpiote_init();
    }
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
    err_code = nrf_drv_gpiote_out_init(HEAT_EN, &out_config);
    APP_ERROR_CHECK(err_code);
    
      //Set Pin for EN1

    //nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    //err_code = nrf_drv_gpiote_out_init(DSC_EN, &out_config);
    //APP_ERROR_CHECK(err_code);

    //err_code = nrf_drv_gpiote_out_init(DSCB_EN, &out_config);
    //APP_ERROR_CHECK(err_code);
    
}

*/
int main(void)
{
    int i;

    //bool erase_bonds;
    NRF_POWER->DCDCEN = 1;
    test_i=0;
    //sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);

    for(i=0;i<SAMPLES_IN_BUFFER;i++){
        saadc_gain[i] = 5;   //1.0/6.0, 1.0/5.0, 1.0/4.0, 1.0/3.0, 1.0/2.0, 1.0, 2.0, 4.0
        
    }
    


    

    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    
    ble_stack_init();
    gap_params_init();
    gatt_init();
    
    
    advertising_init();
    services_init();
    conn_params_init();

    bsp_board_init(BSP_INIT_LEDS);
    pwm_common_init();
    
    advertising_start(erase_bonds);
     
    for (;;)
    {        
        idle_state_handle();
    }    
}
