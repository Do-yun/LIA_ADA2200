

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
#include "nrf_drv_spi.h"


#include "arm_const_structs.h"

#include "gen_rr.h"

#include "ble_nus.h"

#define DEVICE_NAME                         "ADA2200"                            /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                   "NordicSemiconductor"                   /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                    500 //org 300, 2056 = 1285ms                                     
                                           //org:300 /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION                    30000                                 /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define APP_BLE_CONN_CFG_TAG                1                                       /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO               3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define DISCHARGE_INTERVAL                  APP_TIMER_TICKS(5000)

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

#define SAMPLES_IN_BUFFER 1
#define SPI_BUFFSIZE        8
#define SPI_INSTANCE        0

#define HEART_RATE_MEAS_INTERVAL0       APP_TIMER_TICKS(10)

#define SPI_SETTING_REG_ADDRESS         0x00
#define DEMOD_CTRL_REG_ADDRESS          0x2A

#define SET_READ_SINGLE_CMD(x) (x| 0x8000)

uint8_t spi_tx_buf[SPI_BUFFSIZE] = {1,1,1,1,1,1,1,1};
uint8_t spi_rx_buf[SPI_BUFFSIZE] = {1,1,1,1,1,1,1,1};
static volatile bool spi_xfer_done;

static const nrf_drv_spi_t m_spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);

#define BUTTON_PREV_ID           0                           /**< Button used to switch the state. */
#define BUTTON_NEXT_ID           1                           /**< Button used to switch the state. */

#define NUS_SERVICE_UUID_TYPE       BLE_UUID_TYPE_VENDOR_BEGIN

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

// updated 2022.08.10
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);

//AC Voltage
int DSCA_flag=0;
int DSCB_flag=0; 
int test_i=0;
int m_adc_evt_counter = 0;


static uint8_t avg_ble_counter = 0;
static uint8_t avg_ble_data[33] = {2};
static double temp_buf;
//static uint8_t avg_ble_data[36]; //org:20

static nrf_ppi_channel_t ppi_channel;

#define MAX_ADC_VAL 4095

static uint8_t saadc_gain[SAMPLES_IN_BUFFER];
static double saadc_gain_scale[8] = {1.0/6.0, 1.0/5.0, 1.0/4.0, 1.0/3.0, 1.0/2.0, 1.0, 2.0, 4.0};

static nrf_saadc_value_t     m_buffer_pool[3][SAMPLES_IN_BUFFER];

static ble_uuid_t m_adv_uuids[] =                                   /**< Universally unique service identifiers. */
{
    {BLE_UUID_HEART_RATE_SERVICE,           BLE_UUID_TYPE_BLE},
    {BLE_UUID_BATTERY_SERVICE,              BLE_UUID_TYPE_BLE},
    {BLE_UUID_DEVICE_INFORMATION_SERVICE,   BLE_UUID_TYPE_BLE}
};


////////////////////////////// pwm parameters //////////////////////////////
static uint16_t m_top = 50;
#define pwm_seq_len                         100
#define CLKIN                               12
#define SYNCO                               16
static nrfx_pwm_t m_pwm0 = NRFX_PWM_INSTANCE(0);                            
static nrf_pwm_values_common_t sequence_values[pwm_seq_len];               // sequence containing the intensity
static uint16_t output_count = 0;
static int16_t output_value = 0;
static uint16_t total_output_value = 0;
static int16_t output_value_90 =0;


///////////////////////////////// updated, 2022.02.03, do /////////////////////////////////
static void pwm_common_init(void) // pwm initiation
{
    for(int i = 0 ; i < pwm_seq_len ; i++)
    {
        sequence_values[i] = m_top/2;
    }
  
    nrfx_pwm_config_t const config0 = 
    {
        .output_pins=
        {
          CLKIN,                                  // 4 output pins that are going to be used. led_external & led_external_2
          NRFX_PWM_PIN_NOT_USED,                                                       // for ADA2200 clock source
          NRFX_PWM_PIN_NOT_USED,                                  // Board LEDs are just for indication!
          NRFX_PWM_PIN_NOT_USED                                                      // pins can be left unused by using NRFX_PWM_PIN_NOT_USED 
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock = NRF_PWM_CLK_16MHz,                                         // clock used in PWM
        .count_mode = NRF_PWM_MODE_UP,
        .top_value = m_top,                                                     // selects the maximum pwm value
        .load_mode = NRF_PWM_LOAD_COMMON,                                      // grouped mode means 4 control output pins are divided into two groups which acts in a same way
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
////////////////////////////////////////////////////////////////////////////

static void saadc_timer_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    ret_code_t err_code;
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
static void application_timers_start(void)
{
    ret_code_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_heart_rate_timer_id, HEART_RATE_MEAS_INTERVAL0, NULL);
    APP_ERROR_CHECK(err_code);
    
}
////////////////////////////// spi methods //////////////////////////////
void spi_event_handler(nrf_drv_spi_evt_t const *p_event, void *p_context)
{
    spi_xfer_done = true;
    if(spi_rx_buf[0] != 0)
    {
        NRF_LOG_INFO("received");
        //NRF_LOG_HEXDUMP_INFO(spi_rx_buf, strlen((const char *)spi_rx_buf));
    }
}

void spi_init(void)
{
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

    spi_config.ss_pin = 18;
    spi_config.miso_pin = 7;
    spi_config.mosi_pin = 8;
    spi_config.sck_pin = 14;
    spi_config.frequency = NRF_DRV_SPI_FREQ_4M;
    APP_ERROR_CHECK(nrf_drv_spi_init(&m_spi, &spi_config, spi_event_handler, NULL));
}


uint8_t ADA2200_read_reg(int reg)
{
    
    spi_tx_buf[0] = (uint8_t) ((SET_READ_SINGLE_CMD(reg))>>8);
    spi_tx_buf[1] = (uint8_t) ((SET_READ_SINGLE_CMD(reg))&0xFF);
    
    NRF_LOG_INFO("spi_tx_buf[0] : %x", spi_tx_buf[0]);
    NRF_LOG_INFO("spi_tx_buf[1] : %x", spi_tx_buf[1]);
    spi_xfer_done = false;
    // why do we send 2, here?
    // It is because SPI is a ring-shaped shift register. One byte is for Read address input
    // Next byte is for Output
    NRF_LOG_INFO("spi_rx_buf[0] : %x", spi_rx_buf[0]);
    NRF_LOG_INFO("spi_rx_buf[1] : %x", spi_rx_buf[1]);
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&m_spi, spi_tx_buf, 3, spi_rx_buf, 3));

    while(spi_xfer_done == false){}
    // This is why we have to receive 1th value of rx buffer, not 0th!
    NRF_LOG_INFO("spi_rx_buf[0] : %x", spi_rx_buf[0]);
    NRF_LOG_INFO("spi_rx_buf[1] : %x", spi_rx_buf[1]);
    NRF_LOG_INFO("spi_rx_buf[2] : %x", spi_rx_buf[2]);
    return spi_rx_buf[2];
}

void ADA2200_write_reg(int reg, int val)
{
    //spi_tx_buf[0] = (uint8_t) (0x11); // address
    
    spi_tx_buf[0] = (uint8_t) (reg >> 8);
    spi_tx_buf[1] = (uint8_t) (reg & 0xFF);
    spi_tx_buf[2] = (uint8_t) (val & 0xFF);
    
    NRF_LOG_INFO("spi_tx_buf[0] : %d", spi_tx_buf[0]);
    NRF_LOG_INFO("spi_tx_buf[1] : %d", spi_tx_buf[1]);
    NRF_LOG_INFO("spi_tx_buf[2] : %d", spi_tx_buf[2]);

    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&m_spi, spi_tx_buf,3, spi_rx_buf,0));
    while(spi_xfer_done == false){}
}
//////////////////////////////////////////////////////////////////////


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

////////////////////////////// gpiote setting //////////////////////////////
void interrupt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    // empty handler
}

/*
void gpio_init()
{
    // initialize gpio event
    ret_code_t err_code;
    err_code = nrf_drv_gpiote_init(); // initialize gpiote module
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    in_config.pull = NRF_GPIO_PIN_PULLDOWN;

    err_code = nrf_drv_gpiote_in_init(SYNCO, &in_config, interrupt_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(SYNCO, true);
}
*/
//////////////////////////////////////////////////////////////////////


////////////////////////////// saadc setting //////////////////////////////
void saadc_callback(nrf_drv_saadc_evt_t const *p_event)
{
    if(p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;
        uint16_t temp_value = 0;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        int i;
        //NRF_LOG_INFO("ADC event number : %d", (int) m_adc_evt_counter);
        
        temp_value = (uint16_t) (p_event->data.done.p_buffer[0]);
        //NRF_LOG_INFO("output value : %d", temp_value);
        output_value += temp_value;
        
        
        m_adc_evt_counter++;
        output_count++;
        if(output_count == 8)
        {
            output_value = output_value /8; // divide by 8 to calculate cyclemean
            //output_value_90 = output_value_90 >> 3;
            //NRF_LOG_INFO("output_value_averaged : %d", output_value);
            output_value = 0;
            
            output_count = 0;
        }
        NRF_LOG_FLUSH();

        avg_ble_data[avg_ble_counter+1] = (uint8_t) (temp_value & 0xFF);
        avg_ble_counter++;
        avg_ble_data[avg_ble_counter+1] = (uint8_t) (temp_value >> 8);
        avg_ble_counter++;
        NRF_LOG_INFO("avg_ble_counter : %d", avg_ble_counter);
        NRF_LOG_INFO("temp_value : %d", temp_value);
        if(avg_ble_counter == 32)
        {
            NRF_LOG_INFO("hello");
            ble_hrs_heart_rate_measurement_send33(&m_hrs, avg_ble_data);
            avg_ble_counter = 0;
        }
    }
}

static void saadc_init(void)
{
    ret_code_t err_code;

    nrf_drv_saadc_config_t saadc_config = NRF_DRV_SAADC_DEFAULT_CONFIG;
    saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;

    nrf_saadc_channel_config_t channel_0_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN4); //AIN5, 29(5) - 2, 3, 4, 5, 28, 29, 30, 31 // this is IR channel
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
    channel_0_config.acq_time = NRF_SAADC_ACQTIME_20US;

    /*
    nrf_saadc_channel_config_t channel_1_config =
    NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1); //AIN5, 29(5) - 2, 3, 4, 5, 28, 29, 30, 31 // this is IR channel
    switch(saadc_gain[1]){
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
    channel_1_config.reference = NRF_SAADC_REFERENCE_INTERNAL;
    channel_1_config.acq_time = NRF_SAADC_ACQTIME_40US;
    */
    err_code = nrf_drv_saadc_channel_init(0, &channel_0_config);
    APP_ERROR_CHECK(err_code);

    /*
    err_code = nrf_drv_saadc_channel_init(1, &channel_1_config);
    APP_ERROR_CHECK(err_code);
    */
    
    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_int_enable_check(0x0);
}


static void ppi_init(void)
{
    ret_code_t err_code = NRF_SUCCESS;
    uint32_t saadc_task_addr;
    uint32_t synco_evt_addr;
    saadc_init();

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_alloc(&ppi_channel);
    APP_ERROR_CHECK(err_code);

    synco_evt_addr = nrf_drv_gpiote_in_event_addr_get(SYNCO);
    saadc_task_addr = nrf_drv_saadc_sample_task_get();
    //saadc_task_addr = nrf_drv_gpiote_out_task_addr_get(17);


    err_code = nrf_drv_ppi_channel_assign(ppi_channel, synco_evt_addr, saadc_task_addr);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_ppi_channel_enable(ppi_channel);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("ppi_init");
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
    ble_nus_init_t     nus_init;
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

    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
    /*
    // Initialize Nordic UART Service
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
    */
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
            break;

        case BLE_ADV_EVT_IDLE:
            advertising_start(erase_bonds);
            break;

        default:
            break;
    }
}

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
            //nrf_delay_ms(6000);
            application_timers_start();
                    
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected, reason %d.",
                          p_ble_evt->evt.gap_evt.params.disconnected.reason);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
           // nrf_drv_gpiote_out_clear(HEAT_EN); 
            // advertising_start(erase_bonds);
            nrfx_pwm_stop(&m_pwm0, false);
            app_timer_stop(m_heart_rate_timer_id);
            //nrf_drv_saadc_uninit(); 
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
        default:
            break;
    }
}


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



//@brief Function for initializing the nrf log module.

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


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

int main(void)
{
    int i;
    ret_code_t err_code;
    uint8_t spi_data, spi_data1 = 0;
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    //bool erase_bonds;
    NRF_POWER->DCDCEN = 1;
    test_i=0;
    //sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
    
    for(i=0;i<SAMPLES_IN_BUFFER;i++){
        saadc_gain[i] = 2;   //1.0/6.0, 1.0/5.0, 1.0/4.0, 1.0/3.0, 1.0/2.0, 1.0, 2.0, 4.0
        
    }
    /*
    uint16_t a = 2;
    uint16_t b = 2;
    uint16_t c = (uint16_t) (sqrt((double) a * (double) a + (double) b * (double) b)*100);
    */

    log_init();
    timers_init();
    
    power_management_init();
    
    ble_stack_init();
    gap_params_init();
    gatt_init();

    advertising_init();
    services_init();
    conn_params_init();
    saadc_init();
    //bsp_board_init(BSP_INIT_LEDS);
    pwm_common_init();
    pwm_play();
    spi_init();    
    advertising_start(erase_bonds);
    NRF_LOG_INFO("initialization all complete");

    //nrf_delay_ms(5000);
    //spi_data = ADA2200_read_reg(0x2A);
    //ADA2200_write_reg(0x00, 0x00); // enable SDO pin
    //ADA2200_write_reg(0x2A, 0x18); // shows SDO value, also delays 90
    //spi_data = ADA2200_read_reg(0x00);

    
    
    //NRF_LOG_INFO("read register");
    //NRF_LOG_INFO("spi_data is : %x", spi_data);
    //NRF_LOG_INFO("spi_data : %x", spi_data);
    //NRF_LOG_INFO("spi_data1 : %d", spi_data1);
    //nrf_delay_ms(5000);
    
    if(spi_data == 0x18)
    {
        NRF_LOG_INFO("hello there");
        //nrf_gpio_pin_set(11);
    }
    // wait for a sec
    //nrf_delay_ms(2000);
    
    // then read 0x29 reg, and see if the default value is read
    // if normally function, turn on LED 1
    /*
    if(ADA2200_read_reg(0x29) == 0x2D)
    {
        nrf_gpio_cfg(17, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
        nrf_gpio_pin_clear(17);
    }
    */
    //nrf_delay_ms(1000);
    NRF_LOG_INFO("int casting of 0x thingy : %d", (int) 0X10);

    
    for (;;)
    {        
        idle_state_handle();
        //NRF_LOG_INFO("still playing");
        //ADA2200_read_reg(0x29);
    }    
}
