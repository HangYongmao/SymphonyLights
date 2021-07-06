#include <TuyaWifi.h>
#include <SoftwareSerial.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h>
#endif

#define PIXEL_PIN     3   // 灯带引脚
#define BUTTON_MIC    4   // 本地音乐
#define BUTTON_Scene  5   // 场景
#define PIXEL_COUNT   30  // 灯带数目
#define LED_WiFi      6   // WiFi指示灯
#define BUTTON_WiFi   7   // 重置WiFi按键

Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

boolean oldState = HIGH;
int mode=9;// Currently-active animation mode


TuyaWifi my_device;

unsigned char led_state = 0;
/* Connect network button pin */

#define DPID_SWITCH_LED               20  // 开关
#define DPID_WORK_MODE                21  // 模式
#define DPID_BRIGHT_VALUE             22  // 亮度值
#define DPID_TEMP_VALUE               23  // 冷暖值
#define DPID_COLOUR_DATA              24  // 彩光
#define DPID_SCENE_DATA               25  // 场景
#define DPID_COUNTDOWN                26  // 倒计时剩余时间
#define DPID_MUSIC_DATA               27  // 音乐灯
#define DPID_CONTROL_DATA             28  // 调节
#define DPID_SLEEP_MODE               31  // 入睡
#define DPID_WAKEUP_MODE              32  // 唤醒
#define DPID_POWER_MEMORY             33  // 断电记忆
#define DPID_DO_NOT_DISTURB           34  // 勿扰模式
#define DPID_MIC_MUSIC_DATA           42  // 麦克风音乐律动
#define DPID_DREAMLIGHT_SCENE_MODE    51  // 炫彩情景
#define DPID_DREAMLIGHTMIC_MUSIC_DATA 52  // 炫彩本地音乐律动
#define DPID_LIGHTPIXEL_NUMBER_SET    53  // 点数/长度设置

// Current device DP values
unsigned char dp_bool_value = 0;
long dp_value_value = 0;
unsigned char dp_enum_value = 0;
unsigned char dp_string_value[21] = {"0"};
uint16_t Hue=0; //HSV
uint8_t Sat=0;
uint8_t Val=0;
uint8_t scene_mode=0;
unsigned char hex[10] = {"0"};
//unsigned char dp_raw_value[8] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef};
//int dp_fault_value = 0x01;

/* Stores all DPs and their types. PS: array[][0]:dpid, array[][1]:dp type. 
 * dp type(TuyaDefs.h) : DP_TYPE_RAW, DP_TYPE_BOOL, DP_TYPE_VALUE, DP_TYPE_STRING, DP_TYPE_ENUM, DP_TYPE_BITMAP
*/
unsigned char dp_array[][2] = {
  {DPID_SWITCH_LED, DP_TYPE_BOOL},
  {DPID_WORK_MODE, DP_TYPE_ENUM},
  {DPID_BRIGHT_VALUE, DP_TYPE_VALUE},
  {DPID_TEMP_VALUE, DP_TYPE_VALUE},
  {DPID_COLOUR_DATA, DP_TYPE_STRING},
  {DPID_SCENE_DATA, DP_TYPE_STRING},
  {DPID_COUNTDOWN, DP_TYPE_VALUE},
  {DPID_MUSIC_DATA, DP_TYPE_STRING},
  {DPID_CONTROL_DATA, DP_TYPE_STRING},
  {DPID_SLEEP_MODE, DP_TYPE_RAW},
  {DPID_WAKEUP_MODE, DP_TYPE_RAW},
  {DPID_POWER_MEMORY, DP_TYPE_RAW},
  {DPID_DO_NOT_DISTURB, DP_TYPE_BOOL},
  {DPID_MIC_MUSIC_DATA, DP_TYPE_STRING},
  {DPID_DREAMLIGHT_SCENE_MODE, DP_TYPE_RAW},
  {DPID_DREAMLIGHTMIC_MUSIC_DATA, DP_TYPE_RAW},
  {DPID_LIGHTPIXEL_NUMBER_SET, DP_TYPE_VALUE},
};

unsigned char pid[] = {"e9emh0rvqmkuzutc"};// 涂鸦IoT平台创建的产品的PID
unsigned char mcu_ver[] = {"1.0.0"};

unsigned long last_time = 0;
SoftwareSerial DebugSerial(8,9);
void setup() {
  pinMode(BUTTON_Scene, INPUT_PULLUP);
  strip.begin(); // Initialize NeoPixel strip object (REQUIRED)
  strip.show();  // Initialize all pixels to 'off'

DebugSerial.begin(9600);

  Serial.begin(9600);
 //Initialize led port, turn off led.
  pinMode(LED_WiFi, OUTPUT);
  digitalWrite(LED_WiFi, LOW);
//Initialize networking keys.
  pinMode(BUTTON_WiFi, INPUT_PULLUP);

  //incoming all DPs and their types array, DP numbers
  //Enter the PID and MCU software version
  my_device.init(pid, mcu_ver);
  my_device.set_dp_cmd_total(dp_array, 17);
  //register DP download processing callback function
  my_device.dp_process_func_register(dp_process);
  //register upload all DP callback function
  my_device.dp_update_all_func_register(dp_update_all);
  
  last_time = millis();
}

void loop() {

  //Enter the connection network mode when Pin7 is pressed.
  if (digitalRead(BUTTON_WiFi) == LOW)
  {
    delay(80);
    if (digitalRead(BUTTON_WiFi) == LOW)
    {
      my_device.mcu_set_wifi_mode(SMART_CONFIG);
      digitalWrite(LED_WiFi, HIGH);
    }
  }
  my_device.uart_service();

  /* LED blinks when network is being connected */
  // 获取WiFi状态
  if ((my_device.mcu_get_wifi_work_state() != WIFI_LOW_POWER) && (my_device.mcu_get_wifi_work_state() != WIFI_CONN_CLOUD) && (my_device.mcu_get_wifi_work_state() != WIFI_SATE_UNKNOW))
  {
    if (millis()- last_time >= 500)
    {
      last_time = millis();
      if (led_state == LOW)
      {
        led_state = HIGH;
      }
      else
      {
        led_state = LOW;
      }
      digitalWrite(LED_WiFi, led_state);
    }
  }

  // Get current button state.
  boolean newState = digitalRead(BUTTON_Scene);

  // Check if state changed from high to low (button press).
  if((newState == LOW) && (oldState == HIGH))
  {
    // Short delay to debounce button.
    delay(20);
    // Check if button is still low after debounce.
    newState = digitalRead(BUTTON_Scene);
    if(newState == LOW)// Yes, still low
    {
      if(++mode > 10)
        mode = 0; // Advance to next mode, wrap around after #8
    }
  }
  // Set the last-read button state to the old state.
  oldState = newState;

  // Start the animation
  switch(mode)
  {
    case 0:
      colorWipe(strip.Color(  0,   0,   0), 50);    // Black/off
      break;
    case 1:
      colorWipe(strip.Color(255,   0,   0), 50);    // Red
      break;
    case 2:
      colorWipe(strip.Color(  0, 255,   0), 50);    // Green
      break;
    case 3:
      colorWipe(strip.Color(  0,   0, 255), 50);    // Blue
      break;
    case 4:
      theaterChase(strip.Color(127, 127, 127), 50); // White
      break;
    case 5:
      theaterChase(strip.Color(127,   0,   0), 50); // Red
      break;
    case 6:
      rainbowCycle(50);
      break;
    case 7:
      rainbow(10);                                  // 七彩
      break;
    case 8:
      theaterChaseRainbow(50);
      break;
    case 9:
      for (int i=0; i<strip.numPixels(); i++)
      {
          strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(analogRead(A0)*127)));
      }
      break;
  }
}

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

// Theater-marquee-style chasing lights. Pass in a color (32-bit value,
// a la strip.Color(r,g,b) as mentioned above), and a delay time (in ms)
// between frames.
void theaterChase(uint32_t color, int wait) {
  for(int a=0; a<10; a++) {  // Repeat 10 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in steps of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show(); // Update strip with new contents
      delay(wait);  // Pause for a moment
    }
  }
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait) {
  // Hue of first pixel runs 3 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 3*65536. Adding 256 to firstPixelHue each time
  // means we'll make 3*65536/256 = 768 passes through this outer loop:
  for(long firstPixelHue = 0; firstPixelHue < 3*65536; firstPixelHue += 256) {
    for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
      // Offset pixel hue by an amount to make one full revolution of the
      // color wheel (range of 65536) along the length of the strip
      // (strip.numPixels() steps):
      int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
      // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
      // optionally add saturation and value (brightness) (each 0 to 255).
      // Here we're using just the single-argument hue variant. The result
      // is passed through strip.gamma32() to provide 'truer' colors
      // before assigning to each pixel:
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
    }
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}

// Rainbow-enhanced theater marquee. Pass delay time (in ms) between frames.
void theaterChaseRainbow(int wait) {
  int firstPixelHue = 0;     // First pixel starts at red (hue 0)
  for(int a=0; a<30; a++) {  // Repeat 30 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in increments of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        // hue of pixel 'c' is offset by an amount to make one full
        // revolution of the color wheel (range 65536) along the length
        // of the strip (strip.numPixels() steps):
        int      hue   = firstPixelHue + c * 65536L / strip.numPixels();
        uint32_t color = strip.gamma32(strip.ColorHSV(hue)); // hue -> RGB
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show();                // Update strip with new contents
      delay(wait);                 // Pause for a moment
      firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

/**
 * @description: DP download callback function.
 * @param {unsigned char} dpid
 * @param {const unsigned char} value
 * @param {unsigned short} length
 * @return {unsigned char}
 */
unsigned char dp_process(unsigned char dpid,const unsigned char value[], unsigned short length)
{
  switch(dpid) {
    case DPID_SWITCH_LED:
      dp_bool_value = my_device.mcu_get_dp_download_data(dpid, value, length); /* Get the value of the down DP command */
      if (dp_bool_value) {
        //Turn on
        colorfill (strip.Color(  0, 255,  0)); //上一次状态
      } else {
        //Turn off
        colorfill (strip.Color(  0, 0,   0));
      }
      //Status changes should be reported.
      my_device.mcu_dp_update(dpid, value, length);
    break;
    
    case DPID_WORK_MODE:
    colorfill (strip.Color( 255, 255,  0));
    dp_enum_value  = my_device.mcu_get_dp_download_data(dpid, value, length); /* Get the value of the down DP command */
      switch(dp_enum_value){
        case 0: // white mode
          colorfill (strip.Color(  255, 255,  255));
        break;
        case 1: // colour mode
        
        break;
        case 2: // scene mode
          mode = 0;
        break;
       case 3: // music mode
          mode = 9;
        break;

      }
      //Status changes should be reported.
      my_device.mcu_dp_update(dpid, value, length);
    break;
    
    case DPID_COUNTDOWN:  //倒计时
      colorfill (strip.Color( 255, 0,  0));
      my_device.mcu_dp_update(dpid, value, length);
    break;

    case DPID_MUSIC_DATA: //音乐律动
        mode = 9;
        my_device.mcu_dp_update(dpid, value, length);
        colour_data_control(value, length);
    break;
    
    case DPID_DREAMLIGHT_SCENE_MODE: //炫彩情景
      my_device.mcu_dp_update(DPID_DREAMLIGHT_SCENE_MODE, value, length);
      scene_mode=value[1];
      
      switch(scene_mode)
      {
        case 0:
          //colorWipe(strip.Color(  0,   0,   0), 50);    // Black/off
          mode = 0;
          break;
        case 1:
          //colorWipe(strip.Color(255,   0,   0), 50);    // Red
          mode = 1;
          break;
        case 2:
          //colorWipe(strip.Color(  0, 255,   0), 50);    // Green
          mode = 2;
          break;
        case 3:
          //colorWipe(strip.Color(  0,   0, 255), 50);    // Blue
          mode = 3;
          break;
        case 4:
          //theaterChase(strip.Color(127, 127, 127), 50); // White
          mode = 4;
          break;
        case 5:
          //theaterChase(strip.Color(127,   0,   0), 50); // Red
          mode = 5;
          break;
        case 6:
          //theaterChase(strip.Color(  0,   0, 127), 50); // Blue
          mode = 6;
          break;
        case 7:
          //rainbow(10);
          mode = 7;
          break;
        case 8:
          //theaterChaseRainbow(50);
          mode = 8;
          break;
      }
      
      break;
    case DPID_LIGHTPIXEL_NUMBER_SET: //长度设置
      my_device.mcu_dp_update(dpid, value, length);
      break;
    case DPID_MIC_MUSIC_DATA: // 本地音乐
      mode = 9;
      break;
    default:break;
  }
  return SUCCESS;
}

/**
 * @description: Upload all DP status of the current device.
 * @param {*}
 * @return {*}
 */
void dp_update_all(void)
{
  my_device.mcu_dp_update(DPID_SWITCH_LED, led_state, 1);
}

//拓展
void colorfill(uint32_t color) {
 strip.fill(color,0,PIXEL_COUNT);
    strip.show();                          //  Update strip to match   
  
}

void colour_data_control( const unsigned char value[], u16 length)
{
  u8 string_data[13];
  u16 h, s, v;
  u8 r, g, b;
  u16 hue;
  u8 sat,val;

  u32 c=0;

  string_data[0] = value[0]; //渐变、直接输出
  string_data[1] = value[1];
  string_data[2] = value[2];
  string_data[3] = value[3];
  string_data[4] = value[4];
  string_data[5] = value[5];
  string_data[6] = value[6];
  string_data[7] = value[7];
  string_data[8] = value[8];
  string_data[9] = value[9];
  string_data[10] = value[10];
  string_data[11] = value[11];
  string_data[12] = value[12];


  h = __str2short(__asc2hex(string_data[1]), __asc2hex(string_data[2]), __asc2hex(string_data[3]), __asc2hex(string_data[4]));
  s = __str2short(__asc2hex(string_data[5]), __asc2hex(string_data[6]), __asc2hex(string_data[7]), __asc2hex(string_data[8]));
  v = __str2short(__asc2hex(string_data[9]), __asc2hex(string_data[10]), __asc2hex(string_data[11]), __asc2hex(string_data[12]));



  // if (v <= 10) {
  //     v = 0;
  // } else {
  //     v = color_val_lmt_get(v);
  // }

  //hsv2rgb((float)h, (float)s / 1000.0, (float)v / 1000.0, &r , &g, &b);

  // c= r<<16|g<<8|b;
  hue=h*182;
  sat=s/4;
  val=v/4;
  c = strip.gamma32(strip.ColorHSV(hue,sat,val)); // hue -> RGB
  DebugSerial.println(hue);
  DebugSerial.println(sat);
  DebugSerial.println(val);


  strip.fill(c,0,PIXEL_COUNT);

  strip.show(); // Update strip with new contents

  //tuya_light_gamma_adjust(r, g, b, &mcu_default_color.red, &mcu_default_color.green, &mcu_default_color.blue);

  //printf("r=%d,g=%d,b=%d\r\n", mcu_default_color.red, mcu_default_color.green, mcu_default_color.blue);
  //rgb_init(mcu_default_color.red, mcu_default_color.green, mcu_default_color.blue);
}

/**
 * @brief  str to short
 * @param[in] {a} Single Point
 * @param[in] {b} Single Point
 * @param[in] {c} Single Point
 * @param[in] {d} Single Point
 * @return Integrated value
 * @note   Null
 */
u32 __str2short(u32 a, u32 b, u32 c, u32 d)
{
    return (a << 12) | (b << 8) | (c << 4) | (d & 0xf);
}

/**
  * @brief ASCALL to Hex
  * @param[in] {asccode} 当前ASCALL值
  * @return Corresponding value
  * @retval None
  */
u8 __asc2hex(u8 asccode)
{
    u8 ret;
    
    if ('0' <= asccode && asccode <= '9')
        ret = asccode - '0';
    else if ('a' <= asccode && asccode <= 'f')
        ret = asccode - 'a' + 10;
    else if ('A' <= asccode && asccode <= 'F')
        ret = asccode - 'A' + 10;
    else
        ret = 0;
    
    return ret;
}

/**
  * @brief Normalized
  * @param[in] {dp_val} dp value
  * @return result
  * @retval None
  */
u16 color_val_lmt_get(u16 dp_val)
{
    u16 max = 255 * 100 / 100;
    u16 min = 255 * 1 / 100;
    
    return ((dp_val - 10) * (max - min) / (1000 - 10) + min);
}

/**
  * @brief hsv to rgb
  * @param[in] {h} tone
  * @param[in] {s} saturation
  * @param[in] {v} Lightness
  * @param[out] {color_r} red
  * @param[out] {color_g} green
  * @param[out] {color_b} blue
  * @retval None
  */
void hsv2rgb(float h, float s, float v, u8 *color_r, u8 *color_g, u8 *color_b)
{
  float h60, f;
  u32 h60f, hi;

  h60 = h / 60.0;
  h60f = h / 60;

  hi = ( signed int)h60f % 6;
  f = h60 - h60f;

  float p, q, t;

  p = v * (1 - s);
  q = v * (1 - f * s);
  t = v * (1 - (1 - f) * s);

  float r, g, b;

  r = g = b = 0;
  if (hi == 0) {
      r = v;          g = t;        b = p;
  } else if (hi == 1) {
      r = q;          g = v;        b = p;
  } else if (hi == 2) {
      r = p;          g = v;        b = t;
  } else if (hi == 3) {
      r = p;          g = q;        b = v;
  } else if (hi == 4) {
      r = t;          g = p;        b = v;
  } else if (hi == 5) {
      r = v;          g = p;        b = q;
  }

  DebugSerial.println(r);
  DebugSerial.println(g);
  DebugSerial.println(b);
  r = (r * (float)255);
  g = (g * (float)255);
  b = (b * (float)255);

  *color_r = r;
  *color_g = g;
  *color_b = b;
  
  // r *= 100;
  // g *= 100;
  // b *= 100;

  // *color_r = (r + 50) / 100;
  // *color_g = (g + 50) / 100;
  // *color_b = (b + 50) / 100;
}
