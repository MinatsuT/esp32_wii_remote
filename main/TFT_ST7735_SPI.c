#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_system.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "tft.h"
#include "tftspi.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/fcntl.h>
#include <time.h>

#define SPI_BUS TFT_HSPI_HOST

void tft_st7735_spi_init() {
    // test_sd_card();
    // ========  PREPARE DISPLAY INITIALIZATION  =========
    esp_err_t ret;

    // === SET GLOBAL VARIABLES ==========================
    // tft_disp_type = DEFAULT_DISP_TYPE;
    // tft_disp_type = DISP_TYPE_ILI9341;
    // tft_disp_type = DISP_TYPE_ILI9488;
    tft_disp_type = DISP_TYPE_ST7735B;
    //tft_disp_type = DISP_TYPE_ST7735R;
    // ===================================================

    // ===================================================
    // === Set display resolution if NOT using default ===
    // === DEFAULT_TFT_DISPLAY_WIDTH &                 ===
    // === DEFAULT_TFT_DISPLAY_HEIGHT                  ===
    //_width = DEFAULT_TFT_DISPLAY_WIDTH;  // smaller dimension
    //_height = DEFAULT_TFT_DISPLAY_HEIGHT; // larger dimension
    _width = 128;  // smaller dimension
    _height = 160; // larger dimension
    // ===================================================

    // ===================================================
    // ==== Set maximum spi clock for display read    ====
    //      operations, function 'find_rd_speed()'    ====
    //      can be used after display initialization  ====
    //max_rdclock = 8000000;
    max_rdclock = 8000000;
    // ===================================================

    // ====================================================================
    // === Pins MUST be initialized before SPI interface initialization ===
    // ====================================================================
    TFT_PinsInit();

    // ====  CONFIGURE SPI DEVICES(s)  ====================================================================================

    spi_lobo_device_handle_t spi;

    spi_lobo_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO, // set SPI MISO pin
        .mosi_io_num = PIN_NUM_MOSI, // set SPI MOSI pin
        .sclk_io_num = PIN_NUM_CLK,  // set SPI CLK pin
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 6 * 1024,
    };
    spi_lobo_device_interface_config_t devcfg = {
        .clock_speed_hz = 8000000,         // Initial clock out at 8 MHz
        .mode = 0,                         // SPI mode 0
        .spics_io_num = -1,                // we will use external CS pin
        .spics_ext_io_num = PIN_NUM_CS,    // external CS pin
        .flags = LB_SPI_DEVICE_HALFDUPLEX, // ALWAYS SET  to HALF DUPLEX MODE!! for display spi
    };

    // ====================================================================================================================

    vTaskDelay(500 / portTICK_RATE_MS);
    printf("==============================\r\n");
    printf("Pins used: miso=%d, mosi=%d, sck=%d, cs=%d\r\n", PIN_NUM_MISO, PIN_NUM_MOSI, PIN_NUM_CLK, PIN_NUM_CS);
    printf("==============================\r\n\r\n");

    // ==================================================================
    // ==== Initialize the SPI bus and attach the LCD to the SPI bus ====

    ret = spi_lobo_bus_add_device(SPI_BUS, &buscfg, &devcfg, &spi);
    assert(ret == ESP_OK);
    printf("SPI: display device added to spi bus (%d)\r\n", SPI_BUS);
    disp_spi = spi;

    // ==== Test select/deselect ====
    ret = spi_lobo_device_select(spi, 1);
    assert(ret == ESP_OK);
    ret = spi_lobo_device_deselect(spi);
    assert(ret == ESP_OK);

    printf("SPI: attached display device, speed=%u\r\n", spi_lobo_get_speed(spi));
    printf("SPI: bus uses native pins: %s\r\n", spi_lobo_uses_native_pins(spi) ? "true" : "false");

    // ================================
    // ==== Initialize the Display ====

    printf("SPI: display init...\r\n");
    TFT_display_init();
    printf("OK\r\n");

    // ---- Detect maximum read speed ----
    max_rdclock = find_rd_speed();
    printf("SPI: Max rd speed = %u\r\n", max_rdclock);

    // ==== Set SPI clock used for display operations ====
    spi_lobo_set_speed(spi, DEFAULT_SPI_CLOCK);
    printf("SPI: Changed speed to %u\r\n", spi_lobo_get_speed(spi));

    font_rotate = 0;
    text_wrap = 0;
    font_transparent = 0;
    font_forceFixed = 0;
    gray_scale = 0;
    TFT_setGammaCurve(DEFAULT_GAMMA_CURVE);
    TFT_setRotation(PORTRAIT);
    TFT_setFont(DEFAULT_FONT, NULL);
    TFT_resetclipwin();
}
