/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bsp/board_api.h"
#include "tusb.h"
#include "pico/stdlib.h"

#include "pico/cyw43_arch.h"
#include "usb_descriptors.h"


#include "hardware/adc.h"


#define BUTTON_PIN_A 15        // change to whichever GPIO you wired
#define BUTTON_PIN_B 14        // change to whichever GPIO you wired
#define BUTTON_PIN_X 13        // change to whichever GPIO you wired
#define BUTTON_PIN_Y 12        // change to whichever GPIO you wired

//encoder pins
#define BUTTON_PIN_0 0        // change to whichever GPIO you wired
#define BUTTON_PIN_1 1        // change to whichever GPIO you wired
#define BUTTON_PIN_2 2        // change to whichever GPIO you wired
#define BUTTON_PIN_GS 3        // change to whichever GPIO you wired

static inline bool button_pressed(void)
{
    // Active-low: returns true when the line is pulled low
    return !gpio_get(BUTTON_PIN_A);
}

int binaryToDecimal(int n) {
    int dec = 0;

    // Initializing base value to 1, i.e 2^0
    int base = 1;
    
    // Extracting each digits of binary number
    // and adding corresponding exponent of 2
    while (n) {
        int last_digit = n % 10;
        n = n / 10;

        // Multiplying the last digit with the base value
        // and adding it to the decimal value
        dec += last_digit * base;

        // Updating the base value by multiplying it by 2
        base = base * 2;
    }

    return dec;
}

#define LEVER_TIME_PRESS 100
static bool lever_up = false;
static int lever_timer = 0;

static inline uint32_t get_gamepad_buttons(void)
{
    // Active-low: returns true when the line is pulled low
    uint32_t b = 0;
    bool lever_current = lever_up;


    if (!gpio_get(BUTTON_PIN_A)) b |= GAMEPAD_BUTTON_A;  // 1 << 0
    if (!gpio_get(BUTTON_PIN_B)){
      lever_up = true;
    } else {
      lever_up = false;
    } // 1 << 1 //lever

    if (lever_up != lever_current){
      lever_timer = LEVER_TIME_PRESS;
    }

    if (lever_timer > 0)
    {
      lever_timer--;
      b |= GAMEPAD_BUTTON_1;
    }
    

    if (!gpio_get(BUTTON_PIN_X)) b |= GAMEPAD_BUTTON_X;  // 1 << 2
    if (!gpio_get(BUTTON_PIN_Y)) b |= GAMEPAD_BUTTON_Y;  // 1 << 3
    //encoder
    
    int A2A1A0 = 0;

    
    if (!gpio_get(BUTTON_PIN_GS) && gpio_get(BUTTON_PIN_0) && gpio_get(BUTTON_PIN_1) && gpio_get(BUTTON_PIN_2)){
      if (!lever_up){
        b |= GAMEPAD_BUTTON_5;  // 1 << 2
      } else {
        b |= GAMEPAD_BUTTON_13;
      }
    } else {
      if (!gpio_get(BUTTON_PIN_0)) A2A1A0 += 1;  // 1 << 0
      if (!gpio_get(BUTTON_PIN_1)) A2A1A0 += 10;  // 1 << 1
      if (!gpio_get(BUTTON_PIN_2)) A2A1A0 += 100;  // 1 << 2

      int decodedNum = binaryToDecimal(A2A1A0) + 5;

      if (!lever_up){
        switch (decodedNum)
        {
        case 5:
          break;
        case 6:
          b |= GAMEPAD_BUTTON_6;
          break;
        case 7:
          b |= GAMEPAD_BUTTON_7;
          break;
        case 8:
          b |= GAMEPAD_BUTTON_8;
          break;
        case 9:
          b |= GAMEPAD_BUTTON_9;
          break;
        case 10:
          b |= GAMEPAD_BUTTON_10;
          break;
        case 11:
          b |= GAMEPAD_BUTTON_11;
          break;
        case 12:
          b |= GAMEPAD_BUTTON_12;
          break;
        default:
          b |= GAMEPAD_BUTTON_31;
          break;
        }
      }

      if (lever_up){
        switch (decodedNum)
        {
        case 5:
          break;
        case 6:
          b |= GAMEPAD_BUTTON_14;
          break;
        case 7:
          b |= GAMEPAD_BUTTON_15;
          break;
        case 8:
          b |= GAMEPAD_BUTTON_16;
          break;
        case 9:
          b |= GAMEPAD_BUTTON_17;
          break;
        case 10:
          b |= GAMEPAD_BUTTON_18;
          break;
        case 11:
          b |= GAMEPAD_BUTTON_19;
          break;
        case 12:
          b |= GAMEPAD_BUTTON_20;
          break;
        default:
          b |= GAMEPAD_BUTTON_31;
          break;
        } 
      }
    }
    return b;
}
// ADC channel connected to potentiometer wiper (GP26 / ADC0)
#define POT_ADC_CHANNEL 0

// uint16_t raw = adc_read();                     // 0-4095
// float voltage = raw * 3.3f / (1 << 12);        // Convert to volts
// printf("Potentiometer voltage: %.2f V\n", voltage);

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void led_blinking_task(void);
void hid_task(void);

/* ===== 2. Helper: read & scale any ADC channel ===== */
static inline int8_t adc_to_axis(uint8_t channel)
{
    adc_select_input(channel);
    int32_t raw = (int32_t)adc_read();     // signed

    const int32_t in_min  = 430;           // joystick low calibration
    const int32_t in_max  = 3390;          // joystick high calibration
    const int32_t out_min = 0;
    const int32_t out_max = 4095;

    // Map raw -> 0..4095 using signed math
    int32_t newValue = (raw - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

    // Clamp to valid ADC range BEFORE using it
    if (newValue < out_min) newValue = out_min;
    if (newValue > out_max) newValue = out_max;

    int32_t centered = newValue - 2048;    // roughly ±2048
    int32_t scaled   = centered / 16;      // roughly ±128

    // Clamp to int8-friendly joystick range (common is -127..127)
    if (scaled > 127)  scaled = 127;
    if (scaled < -127) scaled = -127;

    return (int8_t)scaled;
}

/*------------- MAIN -------------*/
int main(void)
{
  
  board_init();
  stdio_init_all();

  // NEW: configure the GPIO as input with pull-up
  gpio_init(BUTTON_PIN_A);
  gpio_pull_up(BUTTON_PIN_A);
  gpio_set_dir(BUTTON_PIN_A, GPIO_IN);

  gpio_init(BUTTON_PIN_B);
  gpio_pull_up(BUTTON_PIN_B);
  gpio_set_dir(BUTTON_PIN_B, GPIO_IN);

  gpio_init(BUTTON_PIN_X);
  gpio_pull_up(BUTTON_PIN_X);
  gpio_set_dir(BUTTON_PIN_X, GPIO_IN);

  gpio_init(BUTTON_PIN_Y);
  gpio_pull_up(BUTTON_PIN_Y);
  gpio_set_dir(BUTTON_PIN_Y, GPIO_IN);

  gpio_init(BUTTON_PIN_0);
  gpio_pull_up(BUTTON_PIN_0);
  gpio_set_dir(BUTTON_PIN_0, GPIO_IN);

  gpio_init(BUTTON_PIN_1);
  gpio_pull_up(BUTTON_PIN_1);
  gpio_set_dir(BUTTON_PIN_1, GPIO_IN);

  gpio_init(BUTTON_PIN_2);
  gpio_pull_up(BUTTON_PIN_2);
  gpio_set_dir(BUTTON_PIN_2, GPIO_IN);

  gpio_init(BUTTON_PIN_GS);
  gpio_pull_up(BUTTON_PIN_GS);
  gpio_set_dir(BUTTON_PIN_GS, GPIO_IN);

  // Initialize ADC
  adc_init();
  adc_gpio_init(26);  // GP26 for ADC0
  adc_gpio_init(27); 

  // Power up the wireless chip – this also lets us drive WL_GPIO0.
  if (cyw43_arch_init()) {
      return -1;            // init failed
  }

  // init device stack on configured roothub port
  tud_init(BOARD_TUD_RHPORT);

  if (board_init_after_tusb) {
    board_init_after_tusb();
  }

  while (1)
  {
    tud_task(); // tinyusb device task
    led_blinking_task();

    hid_task();
  }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}



//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

static void send_hid_report(uint8_t report_id, uint32_t btn)
{
  // skip if hid is not ready yet
  if ( !tud_hid_ready() ) return;

  switch(report_id)
  {
    case REPORT_ID_KEYBOARD:
    {
      // use to avoid send multiple consecutive zero report for keyboard
      static bool has_keyboard_key = false;

      if ( btn )
      {
        uint8_t keycode[6] = { 0 };
        keycode[0] = HID_KEY_B;

        tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, keycode);
        has_keyboard_key = true;
      }else
      {
        // send empty key report if previously has key pressed
        if (has_keyboard_key) tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, NULL);
        has_keyboard_key = false;
      }
    }
    break;

    case REPORT_ID_MOUSE:
    {
      int8_t const delta = 5;

      // no button, right + down, no scroll, no pan
      tud_hid_mouse_report(REPORT_ID_MOUSE, 0x00, delta, delta, 0, 0);
    }
    break;

    case REPORT_ID_CONSUMER_CONTROL:
    {
      // use to avoid send multiple consecutive zero report
      static bool has_consumer_key = false;

      if ( btn )
      {
        // volume down
        uint16_t volume_down = HID_USAGE_CONSUMER_VOLUME_DECREMENT;
        tud_hid_report(REPORT_ID_CONSUMER_CONTROL, &volume_down, 2);
        has_consumer_key = true;
      }else
      {
        // send empty key report (release key) if previously has key pressed
        uint16_t empty_key = 0;
        if (has_consumer_key) tud_hid_report(REPORT_ID_CONSUMER_CONTROL, &empty_key, 2);
        has_consumer_key = false;
      }
    }
    break;

    case REPORT_ID_GAMEPAD:
    {
      // use to avoid send multiple consecutive zero report for keyboard
      static bool has_gamepad_key = false;

      hid_gamepad_report_t report =
      {
        .x   = 0, .y = 0, .z = 0, .rz = 0, .rx = 0, .ry = 0,
        .hat = 0, .buttons = 0
      };

      // uint16_t rawadc = adc_read(); // 0-4095
      // int8_t xaxis = (rawadc-2048)/16; //converts 0-4095 to -128 - 127
      // report.x = xaxis;

      report.y = adc_to_axis(0);
      report.x = adc_to_axis(1) * -1;

      report.buttons = get_gamepad_buttons();
      //report.buttons = 0x80000000u;
      // if ( btn )
      // {
      //   report.hat = GAMEPAD_HAT_UP;
      //   report.buttons = gamepad_buttons;
      //   // tud_hid_report(REPORT_ID_GAMEPAD, &report, sizeof(report));

      //   has_gamepad_key = true;
      // }else
      // {
      //   report.hat = GAMEPAD_HAT_CENTERED;
      //   report.buttons = 0;
      //   // if (has_gamepad_key) tud_hid_report(REPORT_ID_GAMEPAD, &report, sizeof(report));
      //   has_gamepad_key = false;
      // }
      tud_hid_report(REPORT_ID_GAMEPAD, &report, sizeof(report));
    }
    break;

    default: break;
  }
}

// Every 10ms, we will sent 1 report for each HID profile (keyboard, mouse etc ..)
// tud_hid_report_complete_cb() is used to send the next report after previous one is complete
void hid_task(void)
{
  // Poll every 10ms
  const uint32_t interval_ms = 1;
  static uint32_t start_ms = 0;

  if ( board_millis() - start_ms < interval_ms) return; // not enough time
  start_ms += interval_ms;

  uint32_t const btn = button_pressed();

  uint32_t const gamepad_buttons = get_gamepad_buttons();

  // Remote wakeup
  if ( tud_suspended() && btn )
  {
    // Wake up host if we are in suspend mode
    // and REMOTE_WAKEUP feature is enabled by host
    tud_remote_wakeup();
  }else
  {
    // Send the 1st of report chain, the rest will be sent by tud_hid_report_complete_cb()
    send_hid_report(REPORT_ID_GAMEPAD, btn);
  }
}

// Invoked when sent REPORT successfully to host
// Application can use this to send the next report
// Note: For composite reports, report[0] is report ID
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, uint16_t len)
{
  (void) instance;
  (void) len;

  uint8_t next_report_id = report[0] + 1u;

  if (next_report_id < REPORT_ID_COUNT)
  {
    send_hid_report(next_report_id, board_button_read());
  }
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
  // TODO not Implemented
  (void) instance;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
  (void) instance;

  if (report_type == HID_REPORT_TYPE_OUTPUT)
  {
    // Set keyboard LED e.g Capslock, Numlock etc...
    if (report_id == REPORT_ID_KEYBOARD)
    {
      // bufsize should be (at least) 1
      if ( bufsize < 1 ) return;

      uint8_t const kbd_leds = buffer[0];

      if (kbd_leds & KEYBOARD_LED_CAPSLOCK)
      {
        // Capslock On: disable blink, turn led on
        blink_interval_ms = 0;
        board_led_write(true);
      }else
      {
        // Caplocks Off: back to normal blink
        board_led_write(false);
        blink_interval_ms = BLINK_MOUNTED;
      }
    }
  }
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // blink is disabled
  if (!blink_interval_ms) return;

  // Blink every interval ms
  if ( board_millis() - start_ms < blink_interval_ms) return; // not enough time
  start_ms += blink_interval_ms;

  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
  led_state = 1 - led_state; // toggle
}
