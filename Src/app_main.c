// #include "stm32469i_discovery.h"
#include "main.h"
#include "stm32469i_discovery_lcd.h"

#define WVGA_RES_X 800
#define WVGA_RES_Y 480

#define LCD_SCREEN_WIDTH WVGA_RES_X
#define LCD_SCREEN_HEIGHT WVGA_RES_Y

#define ARGB8888_BYTE_PER_PIXEL 4
#define RGB888_BYTE_PER_PIXEL 3

uint8_t lcd_status = LCD_OK;

void app_main(void)
{
    BSP_LED_Init(LED1);
    BSP_LED_Init(LED2);
    BSP_LED_Init(LED3);
    BSP_LED_Init(LED4);

    lcd_status = BSP_LCD_InitEx(LCD_ORIENTATION_LANDSCAPE);

    BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER_BACKGROUND,
                             LCD_FB_START_ADDRESS);

    BSP_LCD_Clear(LCD_COLOR_BLACK);
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_SetBackColor(LCD_COLOR_BLACK);

    BSP_LCD_DrawLine(0, 0, 799, 479);
    BSP_LCD_DrawLine(0, 479, 799, 0);

    BSP_LCD_SetFont(&Font24);
    BSP_LCD_DisplayStringAt(
        0, BSP_LCD_GetYSize() / 2,
        (uint8_t *)"It's dark here. You may be eaten by a grue.", CENTER_MODE);
}
