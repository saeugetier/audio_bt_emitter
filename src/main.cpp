#include "AudioKitHAL.h"
#include "AudioTools.h"
#include "AudioLibs/AudioA2DP.h"

AudioKit kit;
BluetoothA2DPSource a2dp_source;
const int16_t BYTES_PER_FRAME = 4;

bool pairing = false;
uint32_t connection_timeout = 0;
uint32_t pairing_timeout = 0;
uint32_t button_hold_time = 0;
uint32_t led_cycle = 0;

constexpr int PLAY_BUTTON = 12;
constexpr int MENU_BUTTON = 13;
constexpr int MIDDLE_BUTTON = 15;

constexpr int LED_PIN = 19;
constexpr uint8_t LED_OFF = LOW;
constexpr uint8_t LED_ON = HIGH;

enum BtTransmitterState
{
  UNCONNECTED,
  CONNECTED,
  PAIRING
};

BtTransmitterState btTransmitterState = UNCONNECTED;

// callback used by A2DP to provide the sound data - usually len is 128 2 channel int16 frames
int32_t get_sound_data(Frame *data, int32_t frameCount)
{
  return kit.read((uint8_t *)data, frameCount * BYTES_PER_FRAME) / BYTES_PER_FRAME;
}

bool isValid(const char *ssid, esp_bd_addr_t address, int rssi)
{
  // Serial.print("available SSID: ");
  // Serial.println(ssid);
  Serial.print("============== Found new device: ");
  Serial.print(ssid);
  Serial.println("==============");
  bool result = pairing;
  return result;
}

void setup()
{
  Serial.begin(115200);
  // open in read mode

  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  // GPIO13, GPIO15 as wake up input
  esp_sleep_enable_ext1_wakeup(0xA000, ESP_EXT1_WAKEUP_ALL_LOW);

  // input pins for buttons (active low)
  // play button
  pinMode(PLAY_BUTTON, INPUT_PULLUP);
  // menu button
  pinMode(MENU_BUTTON, INPUT_PULLUP);
  // middle button
  pinMode(MIDDLE_BUTTON, INPUT_PULLUP);

  // output for LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF);

  rtc_gpio_pulldown_dis(GPIO_NUM_13);
  rtc_gpio_pullup_en(GPIO_NUM_13);

  rtc_gpio_pulldown_dis(GPIO_NUM_15);
  rtc_gpio_pullup_en(GPIO_NUM_15);

  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

  // double check if digital pins 13 and 15 are still low after wakeup. Otherwise ǵo to sleep again.
  if((digitalRead(MENU_BUTTON) || digitalRead(MIDDLE_BUTTON)) && wakeup_reason == ESP_SLEEP_WAKEUP_EXT1)
  {
    Serial.println("============== Go To Deep Sleep ============");
    esp_deep_sleep_start();
  }
  // go to connection mode on first boot up

  auto cfg = kit.defaultConfig(AudioInput);
  cfg.adc_input = AUDIO_HAL_ADC_INPUT_LINE1; // line 1 is aux in
  kit.begin(cfg);

  a2dp_source.set_ssid_callback(isValid);
  a2dp_source.set_auto_reconnect(true);
  a2dp_source.start(get_sound_data);

  a2dp_source.set_volume(255);
}

void loop()
{
  // handle state of transmitter
  if(a2dp_source.is_connected() && btTransmitterState != PAIRING)
  {
    // device is connected
    connection_timeout = 0;
    pairing = false;
    btTransmitterState = CONNECTED;
  }
  else
  {
    if(btTransmitterState == PAIRING)
    {
      pairing_timeout++;
      
      // tasks is executed every 10ms --> 5 minute timeout
      if(pairing_timeout > 100*60*5)
      {
        // pairing ended, try to reconnect
        pairing = false;
        btTransmitterState = UNCONNECTED;
        //a2dp_source.set_auto_reconnect(true);
      }
    }
    else // invalid state or unconnected
    {
      btTransmitterState = UNCONNECTED;

      //a2dp_source.set_auto_reconnect(true);

      connection_timeout++;

      // tasks is executed every 10ms --> 3 minute timeout
      if(connection_timeout > 100*60*3)
      {
        Serial.println("============== Shutdown ============");
        a2dp_source.end();
        kit.end();
        
        Serial.println("============== Go To Deep Sleep ============");
        esp_deep_sleep_start();
      }
    }
  }

  delay(2);

  // Led signalization
  // 1 blink connected, 2 blink unconnected, flashing pair
  if(led_cycle < 3)
  {
    digitalWrite(LED_PIN, LED_ON);
  }
  else if(led_cycle >= 20 && led_cycle < 23 && btTransmitterState == UNCONNECTED)
  {
    digitalWrite(LED_PIN, LED_ON);
  }
  else if((led_cycle % 6) < 3 && btTransmitterState == PAIRING)
  {
    digitalWrite(LED_PIN, LED_ON);
  }
  else
  {
    digitalWrite(LED_PIN, LED_OFF);
  }

  led_cycle++;
  // led cycle is 3 seconds
  if(led_cycle > 100*3)
  {
    led_cycle = 0;
  }

  delay(3);

  // Task Buttons

  bool play_pressed = !digitalRead(PLAY_BUTTON);
  bool menu_pressed = !digitalRead(MENU_BUTTON);
  bool middle_pressed = !digitalRead(MIDDLE_BUTTON);

  // menu + middle (wake up)
  // middle + play (pair)
  if(menu_pressed && middle_pressed && !play_pressed && (btTransmitterState == UNCONNECTED))
  {
    // hold for 1s
    if(button_hold_time > 100)
    {
      // set timeout for connection back to 0
      connection_timeout = 0;
    }
    else
    {
      button_hold_time++;
    }
  }
  else if(play_pressed && middle_pressed && !menu_pressed && (btTransmitterState != PAIRING))
  {
    if(button_hold_time > 100)
    {
      Serial.println("============== Start pairing ============");
      btTransmitterState = PAIRING;
      pairing = true;
      a2dp_source.disconnect();
    }
    else
    {
      button_hold_time++;
    }
  }
  else // no valid combination pressed --> reset button hold time
  {
    button_hold_time = 0;
  }

  delay(5);
} ////