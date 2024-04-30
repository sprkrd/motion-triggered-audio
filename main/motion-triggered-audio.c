#include "audio_file.h"

#include <inttypes.h>
#include <stdio.h>

#include "driver/dac_continuous.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"
#include "wavfile.h"

#define ESP_INTR_FLAG_DEFAULT 0
#define NUM_OF_DAC_DESCRIPTORS 4
#define DMA_DAC_BUF_SIZE_KB 2

#define MOUNT_POINT "/sd"
#define TAG "motion_triggered_audio"

static uint8_t wav_read_buffer[DMA_DAC_BUF_SIZE_KB*1024/2];
static QueueHandle_t motion_sensor_evt_queue = NULL;
static QueueHandle_t dac_que = NULL;
sdmmc_card_t *card;

static void IRAM_ATTR motion_sensor_handler(void* data)
{
    int gpio_pin = (int)data;
    int level = gpio_get_level(gpio_pin);
    xQueueSendFromISR(motion_sensor_evt_queue, &level, NULL);
}

static bool IRAM_ATTR  dac_on_convert_done_callback(dac_continuous_handle_t handle, const dac_event_data_t *event, void *user_data)
{
    BaseType_t need_awoke = 0;
    BaseType_t tmp;
    /* When the queue is full, drop the oldest item */
    if (xQueueIsQueueFullFromISR(dac_que)) {
        dac_event_data_t dummy;
        xQueueReceiveFromISR(dac_que, &dummy, &tmp);
        need_awoke |= tmp;
    }
    /* Send the event from callback */
    xQueueSendFromISR(dac_que, event, &tmp);
    need_awoke |= tmp;
    return need_awoke;
}

esp_err_t setup_dac(uint32_t freq_hz, bool stereo, dac_continuous_handle_t* handle)
{
    dac_continuous_config_t dac_conf =
    {
        .chan_mask = DAC_CHANNEL_MASK_ALL,
        .desc_num = NUM_OF_DAC_DESCRIPTORS,
        .buf_size = DMA_DAC_BUF_SIZE_KB*1024,
        .freq_hz = freq_hz,
        .offset = 0,
        .clk_src = DAC_DIGI_CLK_SRC_APLL,
        .chan_mode = stereo? DAC_CHANNEL_MODE_ALTER : DAC_CHANNEL_MODE_SIMUL,
    };
    esp_err_t err = dac_continuous_new_channels(&dac_conf, handle);
    if (err != ESP_OK)
    {
        *handle = NULL;
        return err;
    }

    dac_event_callbacks_t cbs = {
        .on_convert_done = dac_on_convert_done_callback,
        .on_stop = NULL,
    };
    err = dac_continuous_register_event_callback(*handle, &cbs, NULL);
    if (err != ESP_OK)
    {
        dac_continuous_del_channels(*handle);
        *handle = NULL;
        return err;
    }

    err = dac_continuous_enable(*handle);
    if (err != ESP_OK)
    {
        dac_continuous_del_channels(*handle);
        *handle = NULL;
        return err;
    }

    err = dac_continuous_start_async_writing(*handle);
    if (err != ESP_OK)
    {
        dac_continuous_disable(*handle);
        dac_continuous_del_channels(*handle);
        *handle = NULL;
    }

    return ESP_OK;
}

esp_err_t shutdown_dac(dac_continuous_handle_t handle)
{
    esp_err_t err = ESP_OK;
    esp_err_t tmp;

    if (handle != NULL)
    {
        //err = dac_continuous_stop_async_writing(handle);

        tmp = dac_continuous_disable(handle);
        err = err==ESP_OK? tmp : err;

        tmp = dac_continuous_del_channels(handle);
        err = err==ESP_OK? tmp : err;
    }

    if (dac_que != NULL)
    {
        xQueueReset(dac_que);
    }

    return err;
}

esp_err_t play_audio(const char* path)
{
    WavFileHandle wav = open_wavfile(path);

    if (wav == NULL)
    {
        return ESP_FAIL;
    }

    bool stereo = wav->num_channels == 2;

    dac_continuous_handle_t handle = NULL;
    ESP_RETURN_ON_ERROR(setup_dac(wav->sample_rate, stereo, &handle), TAG, "Error setting up DAC");

    dac_event_data_t evt_data;

    esp_err_t err = ESP_OK;
    esp_err_t tmp;

    size_t bytes_written = 0;
    while (bytes_written < wav->nsamples && err==ESP_OK)
    {
        int bytes = load_samples(wav, wav_read_buffer, sizeof(wav_read_buffer));
        if (bytes < 0)
        {
            err = ESP_FAIL;
            break;
        }
        xQueueReceive(dac_que, &evt_data, portMAX_DELAY);
        size_t loaded_bytes = 0;
        err = dac_continuous_write_asynchronously(handle, evt_data.buf, evt_data.buf_size,
                                                  wav_read_buffer, bytes, &loaded_bytes);
        bytes_written += loaded_bytes;
    }

    unsigned char* last_buf = evt_data.buf;
    while(1) {
        xQueueReceive(dac_que, &evt_data, portMAX_DELAY);
        if (evt_data.buf == last_buf) {
            break;
        }
    }

    tmp = shutdown_dac(handle);

    err = err==ESP_OK? tmp : err;

    close_wavfile(wav);

    return err;
}

esp_err_t setup()
{sdmmc_card_t *card;
    // setup motion sensor gpio
    gpio_config_t io_conf =
    {
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE,
        .pin_bit_mask = 1ULL<<CONFIG_GPIO_MOTION_SENSOR,
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "Error configuring GPIO");

    motion_sensor_evt_queue = xQueueCreate(10, sizeof(int));

    ESP_RETURN_ON_ERROR(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT), TAG, "Error while configuring interruption");
    ESP_RETURN_ON_ERROR(
        gpio_isr_handler_add(CONFIG_GPIO_MOTION_SENSOR, motion_sensor_handler, (void*)CONFIG_GPIO_MOTION_SENSOR),
	TAG, "Error while configuring interruption"
    );

    // setup DAC queue
    dac_que = xQueueCreate(NUM_OF_DAC_DESCRIPTORS, sizeof(dac_event_data_t));

    // setup SD card

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = 7500;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = CONFIG_SD_PIN_MOSI,
        .miso_io_num = CONFIG_SD_PIN_MISO,
        .sclk_io_num = CONFIG_SD_PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA),
                        TAG, "Error while initializing SPI bus");

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = CONFIG_SD_PIN_CS;
    slot_config.host_id = host.slot;

    ESP_RETURN_ON_ERROR(esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card),
                        TAG, "Error while mounting");

    ESP_LOGI(TAG, "SD card mounted in " MOUNT_POINT "\n");
    sdmmc_card_print_info(stdout, card);

    return ESP_OK;
}

static inline void wrap_inc(int* cnt, int limit)
{
    *cnt += 1;
    if (*cnt == limit)
    {
        *cnt = 0;
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(setup());

    struct Playlist playlist;
    build_playlist(MOUNT_POINT, &playlist);

    int curr_sound = 0;
    int level = 0;

    while (1)
    {
        if (xQueueReceive(motion_sensor_evt_queue, &level, pdMS_TO_TICKS(2000)))
        {
            if (level)
            {
                printf("Motion sensor has detected movement.\n");
                ESP_ERROR_CHECK(play_audio(playlist.sounds[curr_sound]));
                wrap_inc(&curr_sound, playlist.size);
            }
            else
            {
                printf("Motion sensor has stopped detecting movement.\n");
            }
        }
        else if (level)
        {
            ESP_LOGI(TAG, "Movement has not stopped, playing sound again");
            ESP_ERROR_CHECK(play_audio(playlist.sounds[curr_sound]));
            wrap_inc(&curr_sound, playlist.size);
        }
    }
}
