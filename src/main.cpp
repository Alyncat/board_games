#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <lvgl.h>
#include <FastLED.h>
//#include <R200.h>
#define NUM_LEDS 1
#define DATA_PIN 19
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];
#define GFX_BL (-1)
#include "R200.h"

#define RX_PIN 18  // Define RX pin
#define TX_PIN 17  // Define TX pin

HardwareSerial mySerial(2);
// GLOBALS
unsigned long lastResetTime = 0;
R200 rfid;

Arduino_DataBus *bus = new Arduino_SWSPI(
    GFX_NOT_DEFINED /* DC */, 1 /* CS */,
    12 /* SCK */, 11 /* MOSI */, GFX_NOT_DEFINED /* MISO */);

auto rgbpanel = new Arduino_ESP32RGBPanel(
    45 /* DE */, 4 /* VSYNC */, 5 /* HSYNC */, 21 /* PCLK */,
    39 /* R0 */, 40 /* R1 */, 41 /* R2 */, 42 /* R3 */, 2 /* R4 */,
    0 /* G0 */, 9 /* G1 */, 14 /* G2 */, 47 /* G3 */, 48 /* G4 */, 3 /* G5 */,
    6 /* B0 */, 7 /* B1 */, 15 /* B2 */, 16 /* B3 */, 8 /* B4 */,
    1 /* hsync_polarity */, 10 /* hsync_front_porch */, 8 /* hsync_pulse_width */, 50 /* hsync_back_porch */,
    1 /* vsync_polarity */, 10 /* vsync_front_porch */, 8 /* vsync_pulse_width */, 20 /* vsync_back_porch */);

/**
 * Arduino_RGB_Display
 */
auto gfx = new Arduino_RGB_Display(
    480 /* width */, 480 /* height */, rgbpanel, 0 /* rotation */, true /* auto_flush */,
    bus, GFX_NOT_DEFINED /* RST */, st7701_type1_init_operations, sizeof(st7701_type1_init_operations));

#include "touch.h"

/* Change to your screen resolution */
static uint32_t screenWidth;
static uint32_t screenHeight;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_draw_buf;
static lv_disp_drv_t disp_drv;

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, static_cast<int16_t>(w), static_cast<int16_t>(h));
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

  lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  if (touch_has_signal())
  {
    if (touch_touched())
    {
      data->state = LV_INDEV_STATE_PR;

      /*Set the coordinates*/
      data->point.x = static_cast<lv_coord_t>(touch_last_x);
      data->point.y = static_cast<lv_coord_t>(touch_last_y);
    }
    else if (touch_released())
    {
      data->state = LV_INDEV_STATE_REL;
    }
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }
}


/*
 * Even App
 */
static void event_handler(lv_event_t * e) {
  const lv_event_code_t code = lv_event_get_code(e);
  const lv_obj_t * obj = lv_event_get_target(e);

  if (code == LV_EVENT_VALUE_CHANGED) {
    if (lv_obj_has_state(obj, LV_STATE_CHECKED)) {
      leds[0] = CRGB::Green;
    } else {
      leds[0] = CRGB::Blue;
    }
    FastLED.show();
  }
}

static void event_handler_relay(lv_event_t * e) {

  const lv_event_code_t code = lv_event_get_code(e);
  const lv_obj_t * obj = lv_event_get_target(e);

  if (code == LV_EVENT_VALUE_CHANGED) {
    if (lv_obj_has_state(obj, LV_STATE_CHECKED)) {
      digitalWrite(20,HIGH);
    } else {
      digitalWrite(20,LOW);
    }
  }

}

static void event_handler_read(lv_event_t * e) {
  while (true) {
    rfid.loop();

    // Periodically re-send the read command
    if(millis() - lastResetTime > 1000){
      //  digitalWrite(LED_BUILTIN, HIGH);
      rfid.poll();
      rfid.dumpUIDToSerial();
      //rfid.getModuleInfo();
      //  digitalWrite(LED_BUILTIN, LOW);
      lastResetTime = millis();
    }
    break;
  }
}

/*
 * App
 */
void App() {
  lv_obj_t * btn2 = lv_btn_create(lv_scr_act());
  lv_obj_add_event_cb(btn2, event_handler, LV_EVENT_ALL, nullptr);
  lv_obj_align(btn2, LV_ALIGN_CENTER, 0, 40);
  lv_obj_add_flag(btn2, LV_OBJ_FLAG_CHECKABLE);
  lv_obj_set_height(btn2, LV_SIZE_CONTENT);

  lv_obj_t *label = lv_label_create(btn2);
  lv_label_set_text(label, "Toggle RGB");
  lv_obj_center(label);

  lv_obj_t* btn = lv_btn_create(lv_scr_act());
  lv_obj_add_event_cb(btn, event_handler_relay, LV_EVENT_ALL, nullptr);
  lv_obj_align(btn, LV_ALIGN_CENTER, 0, -40);
  lv_obj_add_flag(btn, LV_OBJ_FLAG_CHECKABLE);
  lv_obj_set_height(btn, LV_SIZE_CONTENT);

  lv_obj_t *label2 = lv_label_create(btn);
  lv_label_set_text(label2, "Toggle Relay");
  lv_obj_center(label2);


  lv_obj_t* btn3 = lv_btn_create(lv_scr_act());
  lv_obj_add_event_cb(btn3, event_handler_read, LV_EVENT_ALL, nullptr);
  lv_obj_align(btn3, LV_ALIGN_CENTER, 0, -100);
  lv_obj_set_height(btn3, LV_SIZE_CONTENT);

  lv_obj_t *label3 = lv_label_create(btn3);
  lv_label_set_text(label3, "Read EPC TAG");
  lv_obj_center(label3);
}




void setup()
{
  Serial.begin(115200);
  CFastLED::addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(25);
  leds[0] = CRGB::Red;
  FastLED.show();
  pinMode(20, OUTPUT);
  mySerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  rfid.begin(&mySerial, 115200, RX_PIN, TX_PIN);
  Serial.println("UART2 Initialized");
  rfid.dumpModuleInfo();
  // while (!Serial);
  Serial.println("LVGL Widgets Demo");

  // Init touch device
  touch_init();

  // Init Display
  gfx->begin();
  gfx->fillScreen(BLACK);

#ifdef GFX_BL
  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, HIGH);
#endif

  lv_init();

  screenWidth = gfx->width();
  screenHeight = gfx->height();
#ifdef ESP32
  disp_draw_buf = static_cast<lv_color_t *>(
    heap_caps_malloc(sizeof(lv_color_t) * screenWidth * 10, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
#else
  disp_draw_buf = (lv_color_t *)malloc(sizeof(lv_color_t) * screenWidth * 10);
#endif
  if (!disp_draw_buf)
  {
    Serial.println("LVGL disp_draw_buf allocate failed!");
  }
  else
  {
    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, nullptr, screenWidth * 10);
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = static_cast<lv_coord_t>(screenWidth);
    disp_drv.ver_res = static_cast<lv_coord_t>(screenHeight);
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    App();

    Serial.println("Setup done");
  }
}

void loop()
{
  lv_timer_handler(); /* let the GUI do its work */
  delay(5);
}