#define FASTLED_ESP8266_RAW_PIN_ORDER

#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP8266WebServer.h>
#include <FastLED.h>
#include <WiFiManager.h>
#include <EEPROM.h>
#include "Arduino.h"

#define RECV_PIN 6
IRrecv IRrecv(RECV_PIN);
decode_results results;
#define MAX_TIME 150
long lastPressTime = 0;
bool ir_interrupted = 0;

#define NUM_LEDS 9 // 1 led with 3 colors thanks to CRGB

// Use Correction from fastLED library or not
#define USE_F_LED_CC true

// How many LED Colors are there including CW and/or WW
// fastLED only controls rgb, not w
#define LED_COLORS 3
#define NUM_LIGHTS 3

// FastLED settings, data and clock pin for spi communication
// Note that the protocol for SM16716 is the same for the SM16726
#define DATA_PIN_RGB 4
#define DATA_PIN_WWA 5
//#define CLOCK_PIN 4
#define COLOR_ORDER GRB
#define LED_TYPE WS2812B
#define CORRECTION TypicalSMD5050

int strip_rgb[NUM_LIGHTS][2] = {
    {0, 108},
    {109, 227},
    {228, 340}
};

int strip_wwa[NUM_LIGHTS][2] = {
    {0, 111},
    {112, 229},
    {230, 340}
};

//#define USE_STATIC_IP //! uncomment to enable Static IP Adress
#ifdef USE_STATIC_IP
IPAddress strip_ip ( 192,  168,   0,  95); // choose an unique IP Adress
IPAddress gateway_ip ( 192,  168,   0,   1); // Router IP
IPAddress subnet_mask(255, 255, 255,   0);
#endif

#define COLORMODE_RGB 0
#define COLORMODE_XY 1
#define COLORMODE_CT 2
#define COLORMODE_HUE 3

uint8_t rgb[NUM_LIGHTS][3], wwa[NUM_LIGHTS][3], color_mode[NUM_LIGHTS], scene;
bool light_state[NUM_LIGHTS], in_transition[NUM_LIGHTS];
int transitiontime, ct[NUM_LIGHTS], hue[NUM_LIGHTS], bri[NUM_LIGHTS], sat[NUM_LIGHTS];
float step_level_rgb[NUM_LIGHTS][3], step_level_wwa[NUM_LIGHTS][3], current_rgb[NUM_LIGHTS][3], current_wwa[NUM_LIGHTS][3], x[NUM_LIGHTS], y[NUM_LIGHTS];
byte mac[6];

ESP8266WebServer server(80);


// Set up array for use by FastLED
CRGB leds_rgb[NUM_LEDS];
CRGB leds_wwa[NUM_LEDS];

void convert_xy(uint8_t light)
{
  int optimal_bri = bri[light];
  if (optimal_bri < 5) {
    optimal_bri = 5;
  }
  float Y = y[light];
  float X = x[light];
  float Z = 1.0f - x[light] - y[light];

  // sRGB D65 conversion
  float r =  X * 3.2406f - Y * 1.5372f - Z * 0.4986f;
  float g = -X * 0.9689f + Y * 1.8758f + Z * 0.0415f;
  float b =  X * 0.0557f - Y * 0.2040f + Z * 1.0570f;

  // Apply gamma correction v.1 (better color accuracy), try this first!
  r = r <= 0.0031308f ? 12.92f * r : (1.0f + 0.055f) * pow(r, (1.0f / 2.4f)) - 0.055f;
  g = g <= 0.0031308f ? 12.92f * g : (1.0f + 0.055f) * pow(g, (1.0f / 2.4f)) - 0.055f;
  b = b <= 0.0031308f ? 12.92f * b : (1.0f + 0.055f) * pow(b, (1.0f / 2.4f)) - 0.055f;

  float maxv = 0;// calc the maximum value of r g and b
  if (r > maxv) maxv = r;
  if (g > maxv) maxv = g;
  if (b > maxv) maxv = b;

  if (maxv > 0) {// only if maximum value is greater than zero, otherwise there would be division by zero
    r /= maxv;   // scale to maximum so the brightest light is always 1.0
    g /= maxv;
    b /= maxv;
  }

  r = r < 0 ? 0 : r;
  g = g < 0 ? 0 : g;
  b = b < 0 ? 0 : b;

  float ri = r;
  float gi = g;
  float bi = b;
  ri = ri > 1.0f ? 1.0f : ri;
  gi = gi > 1.0f ? 1.0f : gi;
  bi = bi > 1.0f ? 1.0f : bi;
  ri = ri < 0.0f ? 0.0f : ri;
  gi = gi < 0.0f ? 0.0f : gi;
  bi = bi < 0.0f ? 0.0f : bi;

  rgb[light][0] = (int) (r * optimal_bri);
  rgb[light][1] = (int) (g * optimal_bri);
  rgb[light][2] = (int) (b * optimal_bri);

  wwa[light][0] = 0;
  wwa[light][1] = 0;
  wwa[light][2] = 0;
}

void convert_ct(uint8_t light) {
  uint8 percent_warm;
  uint8 percent_cold;
  uint8 percent_amber;

  if (ct[light] < 400) {
    percent_warm = ((ct[light] - 153) * 100) / 247;
    percent_cold = 100 - percent_warm;
    percent_amber = 0;
  } else {
    percent_cold = 0;
    percent_warm = 100;
    percent_amber = 100 - (500 - ct[light]);
  }

  wwa[light][1] = (bri[light] * percent_cold) / 100;
  wwa[light][2] =  (bri[light] * percent_warm) / 100;
  wwa[light][0] =  (bri[light] * percent_amber) / 100;

  rgb[light][0] = 0;
  rgb[light][1] = 0;
  rgb[light][2] = 0;
}

void encodeIR() {
  uint64_t irresult = results.value;
  for (uint8_t light = 0; light < NUM_LIGHTS; light++) {
    switch (irresult) {
      case 0x12345678: rgb[light][0] = 255; rgb[light][1] = 0; rgb[light][2] = 0;
    }
  }
}

void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}

void apply_scene(uint8_t new_scene, uint8_t light) {
  if ( new_scene == 0) {
    bri[light] = 144; ct[light] = 447; color_mode[light] = 2; convert_ct(light);
  } else if ( new_scene == 1) {
    bri[light] = 254; ct[light] = 346; color_mode[light] = 2; convert_ct(light);
  } else if ( new_scene == 2) {
    bri[light] = 254; ct[light] = 233; color_mode[light] = 2; convert_ct(light);
  }  else if ( new_scene == 3) {
    bri[light] = 254; ct[light] = 156; color_mode[light] = 2; convert_ct(light);
  }  else if ( new_scene == 4) {
    bri[light] = 77; ct[light] = 367; color_mode[light] = 2; convert_ct(light);
  }  else if ( new_scene == 5) {
    bri[light] = 254; ct[light] = 447; color_mode[light] = 2; convert_ct(light);
  }  else if ( new_scene == 6) {
    bri[light] = 1; x[light] = 0.561; y[light] = 0.4042; color_mode[light] = 1; convert_xy(light);
  }  else if ( new_scene == 7) {
    bri[light] = 203; x[light] = 0.380328; y[light] = 0.39986; color_mode[light] = 1; convert_xy(light);
  }  else if ( new_scene == 8) {
    bri[light] = 112; x[light] = 0.359168; y[light] = 0.28807; color_mode[light] = 1; convert_xy(light);
  }  else if ( new_scene == 9) {
    bri[light] = 142; x[light] = 0.267102; y[light] = 0.23755; color_mode[light] = 1; convert_xy(light);
  }  else if ( new_scene == 10) {
    bri[light] = 216; x[light] = 0.393209; y[light] = 0.29961; color_mode[light] = 1; convert_xy(light);
  }
}

//void process_lightdata(float transitiontime, uint8_t light) {
//  transitiontime *= 17 - (NUM_LEDS / 40); //every extra led add a small delay that need to be counted
//  if (!ir_interrupted) {
//    if (color_mode[light] == COLORMODE_XY && light_state[light] == true) {
//      convert_xy(light);
//    } else if (color_mode[light] == COLORMODE_CT && light_state[light] == true) {
//      convert_ct(light);
//    }
//  } else {
//    encodeIR();
//  }
//  transitiontime *= 16;
//  for (uint8_t color = 0; color < LED_COLORS; color++) {
//    if (light_state[light]) {
//      step_level_rgb[light][color] = ((float) rgb[light][color] - current_rgb[light][color]) / transitiontime;
//      step_level_wwa[light][color] = ((float) wwa[light][color] - current_wwa[light][color]) / transitiontime;
//    } else {
//      step_level_rgb[light][color] = current_rgb[light][color] / transitiontime;
//      step_level_wwa[light][color] = current_wwa[light][color] / transitiontime;
//    }
//  }
//}

// function to get white pwm value


void lightEngine() {
  for (uint8_t light = 0; light < NUM_LIGHTS; light++) {
    for (uint8_t color = 0; color < LED_COLORS; color++) {
      if (light_state[light]) {
        if (rgb[light][color] != current_rgb[light][color]) {
          in_transition[light] = true;
          current_rgb[light][color] += step_level_rgb[light][color];
          if ((step_level_rgb[light][color] > 0.0f && current_rgb[light][color] > rgb[light][color]) ||
              (step_level_rgb[light][color] < 0.0f && current_rgb[light][color] < rgb[light][color])) {
            current_rgb[light][color] = rgb[light][color];
          }
          for (int i = strip_rgb[light][0]; i < strip_rgb[light][1] + 1; i++) {
            leds_rgb[i] = CRGB((int) current_rgb[light][0], (int) current_rgb[light][1], (int) current_rgb[light][2]);
          }
        }
        if (wwa[light][color] != current_wwa[light][color]) {
          in_transition[light] = true;
          current_wwa[light][color] += step_level_wwa[light][color];
          if ((step_level_wwa[light][color] > 0.0f && current_wwa[light][color] > wwa[light][color]) ||
              (step_level_wwa[light][color] < 0.0f && current_wwa[light][color] < wwa[light][color])) {
            current_wwa[light][color] = wwa[light][color];
          }
          for (int i = strip_wwa[light][0]; i < strip_wwa[light][1] + 1; i++) {
            leds_wwa[i] = CRGB((int) current_wwa[light][0], (int) current_wwa[light][1], (int) current_wwa[light][2]);
          }
        }
        FastLED.show();
      } else {
        if (current_rgb[light][color] != 0) {
          in_transition[light] = true;
          current_rgb[light][color] -= step_level_rgb[light][color];
          if (current_rgb[light][color] < 0.0f) {
            current_rgb[light][color] = 0;
          }
          for (int i = strip_rgb[light][0]; i < strip_rgb[light][1] + 1; i++) {
            leds_rgb[i] = CRGB((int) current_rgb[light][0], (int) current_rgb[light][1], (int) current_rgb[light][2]);
          }
        }
        if (current_wwa[light][color] != 0) {
          in_transition[light] = true;
          current_wwa[light][color] -= step_level_wwa[light][color];
          if (current_wwa[light][color] < 0.0f) {
            current_wwa[light][color] = 0;
          }
          for (int i = strip_wwa[light][0]; i < strip_wwa[light][1] + 1; i++) {
            leds_wwa[i] = CRGB((int) current_wwa[light][0], (int) current_wwa[light][1], (int) current_wwa[light][2]);
          }
        }
        FastLED.show();
      }
    }
    if (in_transition[light]) {
      FastLED.delay(6);
      in_transition[light] = false;
    }
  }
}

void readIR() {
  if (IRrecv.decode(&results)) {
    if (results.value != 0xFFFFFFFF) {
      if (millis() - lastPressTime > MAX_TIME) {
        ir_interrupted = true;
      }
      lastPressTime = millis();
    }
    IRrecv.resume();
  }
}

void setup() {
  IRrecv.enableIRIn();
  

  FastLED.addLeds<LED_TYPE, DATA_PIN_RGB, COLOR_ORDER>(leds_rgb, NUM_LEDS).setCorrection( CORRECTION );
  FastLED.addLeds<LED_TYPE, DATA_PIN_WWA, COLOR_ORDER>(leds_wwa, NUM_LEDS).setCorrection( CORRECTION );

  EEPROM.begin(512);

#ifdef USE_STATIC_IP
  WiFi.config(strip_ip, gateway_ip, subnet_mask);
#endif

  for (uint8_t light = 0; light < NUM_LIGHTS; light++) {
    apply_scene(EEPROM.read(2), light);
    step_level_rgb[light][0] = rgb[light][0] / 150.0;
    step_level_rgb[light][1] = rgb[light][1] / 150.0;
    step_level_rgb[light][2] = rgb[light][2] / 150.0;
    step_level_wwa[light][0] = wwa[light][0] / 150.0;
    step_level_wwa[light][1] = wwa[light][1] / 150.0;
    step_level_wwa[light][2] = wwa[light][2] / 150.0;
  }

  if (EEPROM.read(1) == 1 || (EEPROM.read(1) == 0 && EEPROM.read(0) == 1)) {
    for (uint8_t light = 0; light < NUM_LIGHTS; light++) {
      light_state[light] = true;
    }
    for (uint8_t i = 0; i < 200; i++) {
      lightEngine();
    }
  }

  WiFiManager wifiManager;
  wifiManager.autoConnect("New Hue Light");

  WiFi.macAddress(mac);

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.begin();

  server.on("/set", []() {
    uint8_t light;
    float transitiontime = 4;
    for (uint8_t i = 0; i < server.args(); i++) {
      if (server.argName(i) == "light") {
        light = server.arg(i).toInt() - 1;
      }
      if (server.argName(i) == "on") {
        if (server.arg(i) == "True" || server.arg(i) == "true") {
          if (EEPROM.read(1) == 0 && EEPROM.read(0) != 1) {
            EEPROM.write(0, 1);
            EEPROM.commit();
          }
          light_state[light] = true;
        }
        else {
          if (EEPROM.read(1) == 0 && EEPROM.read(0) != 0) {
            EEPROM.write(0, 0);
            EEPROM.commit();
          }
          light_state[light] = false;
        }
      }
      else if (server.argName(i) == "r") {
        rgb[light][0] = server.arg(i).toInt();
        color_mode[light] = COLORMODE_RGB;
      }
      else if (server.argName(i) == "g") {
        rgb[light][1] = server.arg(i).toInt();
        color_mode[light] = COLORMODE_RGB;
      }
      else if (server.argName(i) == "b") {
        rgb[light][2] = server.arg(i).toInt();
        color_mode[light] = COLORMODE_RGB;
      }
//      else if (server.argName(i) == "w") {
//        rgb[3] = server.arg(i).toInt();
//        color_mode = 0;
//      }
      else if (server.argName(i) == "x") {
        x[light] = server.arg(i).toFloat();
        color_mode[light] = COLORMODE_XY;
      }
      else if (server.argName(i) == "y") {
        y[light] = server.arg(i).toFloat();
        color_mode[light] = COLORMODE_XY;
      }
      else if (server.argName(i) == "bri") {
        if (server.arg(i).toInt() != 0)
          bri[light] = server.arg(i).toInt();
      }
      else if (server.argName(i) == "bri_inc") {
        bri[light] += server.arg(i).toInt();
        if (bri[light] > 255) bri[light] = 255;
        else if (bri[light] < 0) bri[light] = 0;
      }
      else if (server.argName(i) == "ct") {
        ct[light] = server.arg(i).toInt();
        color_mode[light] = COLORMODE_CT;
      }
      else if (server.argName(i) == "sat") {
        sat[light] = server.arg(i).toInt();
        color_mode[light] = COLORMODE_HUE;
      }
      else if (server.argName(i) == "hue") {
        hue[light] = server.arg(i).toInt();
        color_mode[light] = COLORMODE_HUE;
      }
      else if (server.argName(i) == "alert" && server.arg(i) == "select") {
        if (light_state[light]) {
          if (color_mode[light] == COLORMODE_XY) {
            current_rgb[light][0] = 0;
            current_rgb[light][1] = 0;
            current_rgb[light][2] = 0;
          } else if (color_mode[light] == COLORMODE_CT) {
            current_wwa[light][0] = 0;
            current_wwa[light][1] = 0;
            current_wwa[light][2] = 0;
          }
        } else {
          if (color_mode[light] == COLORMODE_XY) {
            current_rgb[light][0] = 255;
            current_rgb[light][1] = 255;
            current_rgb[light][2] = 255;
          } else if (color_mode[light] == COLORMODE_CT) {
            current_wwa[light][0] = 255;
            current_wwa[light][1] = 255;
            current_wwa[light][2] = 255;
          }
        }
      }
      else if (server.argName(i) == "transitiontime") {
        transitiontime = server.arg(i).toInt();
      }
    }
    server.send(200, "text/plain", "OK, light: " + (String)light + "x: " + (String)x[light] + ", y:" + (String)y[light] + ", bri:" + (String)bri[light] + ", ct:" + ct[light] + ", colormode:" + color_mode[light] + ", state:" + light_state[light]);
    if (color_mode[light] == COLORMODE_XY && light_state[light] == true) {
      convert_xy(light);
    } else if (color_mode[light] == COLORMODE_CT && light_state[light] == true) {
      convert_ct(light);
    }
    //transitiontime *= 16;
    for (uint8_t color = 0; color < LED_COLORS; color++) {
      if (light_state[light]) {
        step_level_rgb[light][color] = (rgb[light][color] - current_rgb[light][color]) / transitiontime;
        step_level_wwa[light][color] = (wwa[light][color] - current_wwa[light][color]) / transitiontime;
      } else {
        step_level_rgb[light][color] = current_rgb[light][color] / transitiontime;
        step_level_wwa[light][color] = current_wwa[light][color] / transitiontime;
      }
    }
  });

  server.on("/get", []() {
    uint8_t light;
    if (server.hasArg("light"))
      light = server.arg("light").toInt() - 1;
    String colormode;
    String power_status;
    power_status = light_state[light] ? "true" : "false";
    if (color_mode[light] == 1)
      colormode = "xy";
    else if (color_mode[light] == 2)
      colormode = "ct";
    else if (color_mode[light] == 3)
      colormode = "hs";
    server.send(200, "text/plain", "{\"on\": " + power_status + ", \"bri\": " + (String)bri[light] + ", \"xy\": [" + (String)x[light] + ", " + (String)y[light] + "], \"ct\":" + (String)ct[light] + ", \"sat\": " + (String)sat[light] + ", \"hue\": " + (String)hue[light] + ", \"colormode\": \"" + colormode + "\"}");
  });


  server.on("/detect", []() {
    server.send(200, "text/plain", "{\"hue\": \"strip\",\"lights\": 3,\"name\": \"TV-Wand\",\"modelid\": \"LST002\",\"mac\": \"" + String(mac[5], HEX) + ":"  + String(mac[4], HEX) + ":" + String(mac[3], HEX) + ":" + String(mac[2], HEX) + ":" + String(mac[1], HEX) + ":" + String(mac[0], HEX) + "\"}");
  });

  server.on("/", []() {
    float transitiontime = 100;
    if (server.hasArg("startup")) {
      if (  EEPROM.read(1) != server.arg("startup").toInt()) {
        EEPROM.write(1, server.arg("startup").toInt());
        EEPROM.commit();
      }
    }

    for (int light = 0; light < NUM_LIGHTS; light++) {
      if (server.hasArg("scene")) {
        if (server.arg("bri") == "" && server.arg("hue") == "" && server.arg("ct") == "" && server.arg("sat") == "") {
          if (EEPROM.read(2) != server.arg("scene").toInt() && EEPROM.read(1) < 2) {
            EEPROM.write(2, server.arg("scene").toInt());
            EEPROM.commit();
          }
          apply_scene(server.arg("scene").toInt(), light);
        } else {
          if (server.arg("bri") != "") {
            bri[light] = server.arg("bri").toInt();
          }
          if (server.arg("hue") != "") {
            hue[light] = server.arg("hue").toInt();
          }
          if (server.arg("sat") != "") {
            sat[light] = server.arg("sat").toInt();
          }
          if (server.arg("ct") != "") {
            ct[light] = server.arg("ct").toInt();
          }
          if (server.arg("colormode") == "1" && light_state[light] == true) {
            convert_xy(light);
          } else if (server.arg("colormode") == "2" && light_state[light] == true) {
            convert_ct(light);
          }
          color_mode[light] = server.arg("colormode").toInt();
        }
      } else if (server.hasArg("on")) {
        if (server.arg("on") == "true") {
          light_state[light] = true;
          {
            if (EEPROM.read(1) == 0 && EEPROM.read(0) != 1) {
              EEPROM.write(0, 1);
            }
          }
        } else {
          light_state[light] = false;
          if (EEPROM.read(1) == 0 && EEPROM.read(0) != 0) {
            EEPROM.write(0, 0);
          }
        }
        EEPROM.commit();
      } else if (server.hasArg("alert")) {
        if (light_state[light]) {
          current_rgb[light][0] = 0;
          current_rgb[light][1] = 0;
          current_rgb[light][2] = 0;
        } else {
          current_rgb[light][0] = 255;
          current_rgb[light][1] = 255;
          current_rgb[light][2] = 255;
        }
      }
      for (uint8_t color = 0; color < LED_COLORS; color++) {
        if (light_state[light]) {
          step_level_rgb[light][color] = ((float) rgb[light][color] - current_rgb[light][color]) / transitiontime;
          step_level_wwa[light][color] = ((float) wwa[light][color] - current_wwa[light][color]) / transitiontime;
        } else {
          step_level_rgb[light][color] = current_rgb[light][color] / transitiontime;
          step_level_wwa[light][color] = current_wwa[light][color] / transitiontime;
        }
      }
    }
    if (server.hasArg("reset")) {
      ESP.reset();
    }

//    String http_content = "<!doctype html>";
//    http_content += "<html>";
//    http_content += "<head>";
//    http_content += "<meta charset=\"utf-8\">";
//    http_content += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
//    http_content += "<title>Light Setup</title>";
//    http_content += "<link rel=\"stylesheet\" href=\"https://unpkg.com/purecss@0.6.2/build/pure-min.css\">";
//    http_content += "</head>";
//    http_content += "<body>";
//    http_content += "<fieldset>";
//    http_content += "<h3>Light Setup</h3>";
//    http_content += "<form class=\"pure-form pure-form-aligned\" action=\"/\" method=\"post\">";
//    http_content += "<div class=\"pure-control-group\">";
//    http_content += "<label for=\"power\"><strong>Power</strong></label>";
//    http_content += "<a class=\"pure-button"; if (light_state) http_content += "  pure-button-primary"; http_content += "\" href=\"/?on=true\">ON</a>";
//    http_content += "<a class=\"pure-button"; if (!light_state) http_content += "  pure-button-primary"; http_content += "\" href=\"/?on=false\">OFF</a>";
//    http_content += "</div>";
//    http_content += "<div class=\"pure-control-group\">";
//    http_content += "<label for=\"startup\">Startup</label>";
//    http_content += "<select onchange=\"this.form.submit()\" id=\"startup\" name=\"startup\">";
//    http_content += "<option "; if (EEPROM.read(1) == 0) http_content += "selected=\"selected\""; http_content += " value=\"0\">Last state</option>";
//    http_content += "<option "; if (EEPROM.read(1) == 1) http_content += "selected=\"selected\""; http_content += " value=\"1\">On</option>";
//    http_content += "<option "; if (EEPROM.read(1) == 2) http_content += "selected=\"selected\""; http_content += " value=\"2\">Off</option>";
//    http_content += "</select>";
//    http_content += "</div>";
//    http_content += "<div class=\"pure-control-group\">";
//    http_content += "<label for=\"scene\">Scene</label>";
//    http_content += "<select onchange = \"this.form.submit()\" id=\"scene\" name=\"scene\">";
//    http_content += "<option "; if (EEPROM.read(2) == 0) http_content += "selected=\"selected\""; http_content += " value=\"0\">Relax</option>";
//    http_content += "<option "; if (EEPROM.read(2) == 1) http_content += "selected=\"selected\""; http_content += " value=\"1\">Read</option>";
//    http_content += "<option "; if (EEPROM.read(2) == 2) http_content += "selected=\"selected\""; http_content += " value=\"2\">Concentrate</option>";
//    http_content += "<option "; if (EEPROM.read(2) == 3) http_content += "selected=\"selected\""; http_content += " value=\"3\">Energize</option>";
//    http_content += "<option "; if (EEPROM.read(2) == 4) http_content += "selected=\"selected\""; http_content += " value=\"4\">Bright</option>";
//    http_content += "<option "; if (EEPROM.read(2) == 5) http_content += "selected=\"selected\""; http_content += " value=\"5\">Dimmed</option>";
//    http_content += "<option "; if (EEPROM.read(2) == 6) http_content += "selected=\"selected\""; http_content += " value=\"6\">Nightlight</option>";
//    http_content += "<option "; if (EEPROM.read(2) == 7) http_content += "selected=\"selected\""; http_content += " value=\"7\">Savanna sunset</option>";
//    http_content += "<option "; if (EEPROM.read(2) == 8) http_content += "selected=\"selected\""; http_content += " value=\"8\">Tropical twilight</option>";
//    http_content += "<option "; if (EEPROM.read(2) == 9) http_content += "selected=\"selected\""; http_content += " value=\"9\">Arctic aurora</option>";
//    http_content += "<option "; if (EEPROM.read(2) == 10) http_content += "selected=\"selected\""; http_content += " value=\"10\">Spring blossom</option>";
//    http_content += "<option "; if (EEPROM.read(2) == 11) http_content += "selected=\"selected\""; http_content += " value=\"11\">Rainbow</option>";
//    http_content += "</select>";
//    http_content += "</div>";
//    http_content += "<br>";
//    http_content += "<div class=\"pure-control-group\">";
//    http_content += "<label for=\"state\"><strong>State</strong></label>";
//    http_content += "</div>";
//    http_content += "<div class=\"pure-control-group\">";
//    http_content += "<label for=\"bri\">Bri</label>";
//    http_content += "<input id=\"bri\" name=\"bri\" type=\"text\" placeholder=\"" + (String)bri + "\">";
//    http_content += "</div>";
//    http_content += "<div class=\"pure-control-group\">";
//    http_content += "<label for=\"hue\">Hue</label>";
//    http_content += "<input id=\"hue\" name=\"hue\" type=\"text\" placeholder=\"" + (String)hue + "\">";
//    http_content += "</div>";
//    http_content += "<div class=\"pure-control-group\">";
//    http_content += "<label for=\"sat\">Sat</label>";
//    http_content += "<input id=\"sat\" name=\"sat\" type=\"text\" placeholder=\"" + (String)sat + "\">";
//    http_content += "</div>";
//    http_content += "<div class=\"pure-control-group\">";
//    http_content += "<label for=\"ct\">CT</label>";
//    http_content += "<input id=\"ct\" name=\"ct\" type=\"text\" placeholder=\"" + (String)ct + "\">";
//    http_content += "</div>";
//    http_content += "<div class=\"pure-control-group\">";
//    http_content += "<label for=\"colormode\">Color</label>";
//    http_content += "<select id=\"colormode\" name=\"colormode\">";
//    http_content += "<option "; if (color_mode == 1) http_content += "selected=\"selected\""; http_content += " value=\"1\">xy</option>";
//    http_content += "<option "; if (color_mode == 2) http_content += "selected=\"selected\""; http_content += " value=\"2\">ct</option>";
//    http_content += "<option "; if (color_mode == 3) http_content += "selected=\"selected\""; http_content += " value=\"3\">hue</option>";
//    http_content += "</select>";
//    http_content += "</div>";
//    http_content += "<div class=\"pure-controls\">";
//    http_content += "<span class=\"pure-form-message\"><a href=\"/?alert=1\">alert</a> or <a href=\"/?reset=1\">reset</a></span>";
//    http_content += "<label for=\"cb\" class=\"pure-checkbox\">";
//    http_content += "</label>";
//    http_content += "<button type=\"submit\" class=\"pure-button pure-button-primary\">Save</button>";
//    http_content += "</div>";
//    http_content += "</fieldset>";
//    http_content += "</form>";
//    http_content += "</body>";
//    http_content += "</html>";
//
//
//    server.send(200, "text/html", http_content);

  });

  server.onNotFound(handleNotFound);

  server.begin();
}

void loop() {
  ArduinoOTA.handle();
  server.handleClient();
  lightEngine();
}
