/*
 * Application main
 */
#include "esp32_wiiremote.h"

#include "TFT_ST7735_SPI.h"

/***************************************************************************
 * Definitions & variables
 ***************************************************************************/
#define W _width
#define H _height
static uint8_t connected = 0;
static uint8_t disp_rot = 0;

/***************************************************************************
 * Prototypes
 ***************************************************************************/
void redraw(void);
void waitFrame(void);

/***************************************************************************
 * Application routines
 ***************************************************************************/
static int16_t x = 0;
static int16_t y = 0;

// Application setup
void setup() {
    tft_st7735_spi_init();
    x = W / 2;
    y = H / 2;
    redraw();
}

// Application main loop (once it returns, called again with new button states)
static uint8_t countEnable = 1;
static uint8_t cnt = 0;
static uint8_t led;
void loop(uint16_t btn, uint16_t pressed, uint16_t released) {
    /// LED control
    if (pressed & BTN_A) {
        countEnable ^= 1;
    }

    if (pressed & BTN_MINUS) {
        cnt = 0xff;
    }

    if (countEnable) {
        cnt++;
    }

    uint8_t c = cnt >> 4;
    led = (c & 0b0001) << 3;
    led |= (c & 0b0010) << 1;
    led |= (c & 0b0100) >> 1;
    led |= (c & 0b1000) >> 3;
    wii_setLed(led);

    // Drawing
    x += !!(btn & BTN_RIGHT) - !!(btn & BTN_LEFT);
    y += !!(btn & BTN_DOWN) - !!(btn & BTN_UP);

    if (pressed & BTN_PLUS) {
        x = W / 2;
        y = H / 2;
        TFT_setRotation(disp_rot); // Clear
    }

    if (pressed & BTN_B) {
        disp_rot = (disp_rot + 1) % 4;
        TFT_setRotation(disp_rot);
        switch (disp_rot) {
        case 0:
            printf("PORTRAIT");
            break;
        case 1:
            printf("LANDSCAPE");
            break;
        case 2:
            printf("PORTRAIT FLIP");
            break;
        case 3:
            printf("LANDSCAPE FLIP");
            break;
        }
        printf("\n");
    }

    redraw();

    waitFrame(); // wait next frame (60fps)
}

// Display redraw routine
static color_t col;
static TickType_t startTime = 0;
static TickType_t now;
static int32_t elapsedTimeMS;
static float fps;
static int32_t drawCount = -1;
static char fpsBuf[20];
static char *ConnectWiiRemote = "Connect Wii Remote";
void redraw() {
    col.r = 0;
    col.g = 128;
    col.b = 255;
    TFT_drawCircle(x, y, 10, col);
    TFT_setFont(DEFAULT_FONT, NULL);
    _fg = TFT_WHITE;
    TFT_print("Wii Remote Test", 0, 0);

    drawCount++;
    now = xTaskGetTickCount();
    if (startTime == 0) {
        startTime = now;
    }
    elapsedTimeMS = (now - startTime) * portTICK_PERIOD_MS;
    if (elapsedTimeMS) {
        fps = drawCount * 1000.0 / elapsedTimeMS;
        fps = (fps > 99.9) ? 99.9 : fps;
        sprintf(fpsBuf, "%4.1f", fps);
        TFT_setFont(FONT_7SEG, NULL);
        set_7seg_font_atrib(6, 1, 1, TFT_GREEN);
        TFT_print(fpsBuf, 0, 16);

        TFT_setFont(SMALL_FONT, NULL);
        TFT_print("FPS", 15 * 4, 24);
    }
#if 0
    if (!connected) {
        TFT_setFont(DEF_SMALL_FONT, NULL);
        _fg = TFT_YELLOW;
        TFT_print(ConnectWiiRemote, (W - TFT_getStringWidth(ConnectWiiRemote)) / 2, H / 2 + 12);
    }
#endif
}

// Wii Remote event handlers
void wii_connected() {
    connected = 1;
    printf("Wii Remote connected.\n");
}
void wii_disconnected() {
    connected = 0;
    printf("Wii Remote disconnected.\n");
}

// Utilities
static TickType_t xLastWakeTime = 0;
static TickType_t xNow;
static const TickType_t xFrequency = 1000 / 60 / portTICK_PERIOD_MS; // 60 fps
void waitFrame() {
    // Always wait at least one frame
    xNow = xTaskGetTickCount();
    if (xLastWakeTime + xFrequency < xNow) {                            // It is already delayed over one frame.
        xLastWakeTime = ((TickType_t)(xNow / xFrequency)) * xFrequency; // Reset the last time.
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
}
