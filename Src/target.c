// /*! ----------------------------------------------------------------------------
//  * @file    target.c
//  * @brief   target generator script for the RTK system
//  *
//  * @attention
//  * All rights reserved.
//  *
//  * @author Ehsan Shaghaei
//  */

// #define ANCHOR
// #define ANCHOR_ID 0x02
// // #define ROUTER
// #define ANCHOR_IDS         \
//   {                        \
//     0x01, 0x02, 0x03, 0x04 \
//   }
// #define DEBUG

// #include <target.h>
// #include <port.h>
// #include <usart.h>

// #ifdef DEBUG
// #include <stdio.h>
// // #include <ILI9341_Driver.h>
// #endif  // DEBUG

// #if (defined(ANCHOR) && defined(ANCHOR_ID))
// #include "anchor.c"
// #endif  // ANCHOR && ANCHOR_ID

// #if (defined(ROUTER) && defined(ANCHOR_IDS))
// #include "rover.c"
// #endif  // ROUTER

// void target() {
// #ifdef DEBUG
//   clear();
// #endif  // DEBUG
//   init_debug();
//   setup_DW1000RSTnIRQ(1);
//   clear();
//   dw_main();
//   // logs("hello");
//   // log_check();

//   // dw_log_check();
// }

// #ifdef DEBUG

// void init_debug(void) {
//   // usb_init();
//   usart_init();
//   // SSD1306_Init();
//   // SSD1306_DrawBitmap(0, 0, logo, 128, 64, 1);
//   // SSD1306_UpdateScreen(); // update screen
//   // ILI9341_Init();
//   // ILI9341_Set_Rotation(3);
// }

// void clear(void) {
//   // SSD1306_Clear();
//   // SSD1306_UpdateScreen();
// }

// // extern FontDef_t Font_7x10;

// void logs(char *s) {
//   usart_send(s, strlen(s));
//   // usart_send((uint8_t *)s, strlen(s));
//   // SSD1306_GotoXY(x, y);
//   // SSD1306_Puts(s, &Font_7x10, 1);
//   // SSD1306_UpdateScreen();  // update screen
// }

// #endif // DEBUG
