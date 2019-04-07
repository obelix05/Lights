#define FASTLED_ESP8266_RAW_PIN_ORDER

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP8266WebServer.h>
#include <FastLED.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include "Arduino.h"

#define NUM_LEDS 9

#define NUM_LIGHTS 3

// Use Correction from fastLED library or not
#define USE_F_LED_CC true

// How many LED Colors are there including CW and/or WW
// fastLED only controls rgb, not w
#define LED_COLORS 3 


// FastLED settings, data and clock pin for spi communication
// Note that the protocol for SM16716 is the same for the SM16726
#define DATA_PIN_RGB 4
#define DATA_PIN_WWA 5
#define COLOR_ORDER GRB
#define LED_TYPE WS2812B
#define CORRECTION TypicalSMD5050

uint8_t rgb[NUM_LIGHTS][3], wwa[NUM_LIGHTS][3], color_mode, scene;
bool light_state[NUM_LIGHTS], in_transition;
int transitiontime, ct[NUM_LIGHTS], hue, bri[NUM_LIGHTS], sat;
//int strip[NUM_LIGHTS][2] = {
//   {0, 108},
//   {109, 228},
//   {229, 340}
//};
int strip[NUM_LIGHTS][2] = {
   {0, 2},
   {3, 5},
   {6, 8}
};
float step_level[NUM_LIGHTS][3], current_rgb[NUM_LEDS][3], current_wwa[NUM_LEDS][3], x, y;
byte mac[6];

ESP8266WebServer server(80);

// InfoLight doesn't seem to work...
CRGB red = CRGB(255, 0, 0);
CRGB green = CRGB(0, 255, 0);
CRGB white = CRGB(255, 255, 255);
CRGB black = CRGB(0, 0, 0);

// Set up array for use by FastLED
CRGB leds[NUM_LEDS], leds2[NUM_LEDS];

void convert_hue(uint8_t light)
{
  double      hh, p, q, t, ff, s, v;
  long        i;
  
  s = sat / 255.0;
  v = bri[light] / 255.0;

  if (s <= 0.0) {      // < is bogus, just shuts up warnings
    rgb[light][0] = v;
    rgb[light][1] = v;
    rgb[light][2] = v;
    return;
  }
  hh = hue;
  if (hh >= 65535.0) hh = 0.0;
  hh /= 11850, 0;
  i = (long)hh;
  ff = hh - i;
  p = v * (1.0 - s);
  q = v * (1.0 - (s * ff));
  t = v * (1.0 - (s * (1.0 - ff)));

  switch (i) {
    case 0:
      rgb[light][0] = v * 255.0;
      rgb[light][1] = t * 255.0;
      rgb[light][2] = p * 255.0;
      break;
    case 1:
      rgb[light][0] = q * 255.0;
      rgb[light][1] = v * 255.0;
      rgb[light][2] = p * 255.0;
      break;
    case 2:
      rgb[light][0] = p * 255.0;
      rgb[light][1] = v * 255.0;
      rgb[light][2] = t * 255.0;
      break;
    case 3:
      rgb[light][0] = p * 255.0;
      rgb[light][1] = q * 255.0;
      rgb[light][2] = v * 255.0;
      break;
    case 4:
      rgb[light][0] = t * 255.0;
      rgb[light][1] = p * 255.0;
      rgb[light][2] = v * 255.0;
      break;
    case 5:
    default:
      rgb[light][0] = v * 255.0;
      rgb[light][1] = p * 255.0;
      rgb[light][2] = q * 255.0;
      break;
  }

}

void convert_xy(uint8_t light)
{
  int optimal_bri = int( 10 + bri[light] / 1.04);
  
  float Y = y;
  float X = x;
  float Z = 1.0f - x - y;

  // sRGB D65 conversion
  float r =  X * 3.2406f - Y * 1.5372f - Z * 0.4986f;
  float g = -X * 0.9689f + Y * 1.8758f + Z * 0.0415f;
  float b =  X * 0.0557f - Y * 0.2040f + Z * 1.0570f;

//  // Apply gamma correction v.2
//  // Altering exponents at end can create different gamma curves
//  r = r <= 0.04045f ? r / 12.92f : pow((r + 0.055f) / (1.0f + 0.055f), 2.4f);
//  g = g <= 0.04045f ? g / 12.92f : pow((g + 0.055f) / (1.0f + 0.055f), 2.4f);
//  b = b <= 0.04045f ? b / 12.92f : pow((b + 0.055f) / (1.0f + 0.055f), 2.4f);

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
  float w = 0.0f;

  // Getting luminance from value to add white leds
  // https://stackoverflow.com/questions/40312216/converting-rgb-to-rgbw
  w = 0.0f;

  rgb[light][0] = (int) (r * optimal_bri); rgb[light][1] = (int) (g * optimal_bri); rgb[light][2] = (int) (b * optimal_bri);
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

//  int optimal_bri = int( 10 + bri / 1.04);
//  int hectemp = 10000 / ct;
//  int r, g, b;
//  if (hectemp <= 66) {
//    r = 255;
//    g = 99.4708025861 * log(hectemp) - 161.1195681661;
//    b = hectemp <= 19 ? 0 : (138.5177312231 * log(hectemp - 10) - 305.0447927307);
//  } else {
//    r = 329.698727446 * pow(hectemp - 60, -0.1332047592);
//    g = 288.1221695283 * pow(hectemp - 60, -0.0755148492);
//    b = 255;
//  }
//  float w = 0;
//  uint8 percent_warm = ((ct - 150) * 100) / 350;
//
//  w = ((optimal_bri * (100 - percent_warm)) / 100) / 3;
//
//  w = w > 255 ? 255 : w;
//  r = r > 255 ? 255 : r;
//  g = g > 255 ? 255 : g;
//  b = b > 255 ? 255 : b;
//  rgb[0] = r * (bri / 255.0f); rgb[1] = g * (bri / 255.0f); rgb[2] = b * (bri / 255.0f); 
////  rgb[3] = w;
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

void infoLight(CRGB color) {
  // Flash the strip in the selected color. White = booted, green = WLAN connected, red = WLAN could not connect
  leds[0]= color;
  FastLED.show();
  FastLED.delay(10);
  leds[0] = CRGB::Black;
  FastLED.show();
}


void apply_scene(uint8_t new_scene, uint8_t light) {
  if ( new_scene == 0) {
    bri[light] = 144; ct[light] = 447; color_mode = 2; convert_ct(light);
  } else if ( new_scene == 1) {
    bri[light] = 254; ct[light] = 346; color_mode = 2; convert_ct(light);
  } else if ( new_scene == 2) {
    bri[light] = 254; ct[light] = 233; color_mode = 2; convert_ct(light);
  }  else if ( new_scene == 3) {
    bri[light] = 254; ct[light] = 156; color_mode = 2; convert_ct(light);
  }  else if ( new_scene == 4) {
    bri[light] = 77; ct[light] = 367; color_mode = 2; convert_ct(light);
  }  else if ( new_scene == 5) {
    bri[light] = 254; ct[light] = 447; color_mode = 2; convert_ct(light);
  }  else if ( new_scene == 6) {
    bri[light] = 1; x = 0.561; y = 0.4042; color_mode = 1; convert_xy(light);
  }  else if ( new_scene == 7) {
    bri[light] = 203; x = 0.380328; y = 0.39986; color_mode = 1; convert_xy(light);
  }  else if ( new_scene == 8) {
    bri[light] = 112; x = 0.359168; y = 0.28807; color_mode = 1; convert_xy(light);
  }  else if ( new_scene == 9) {
    bri[light] = 142; x = 0.267102; y = 0.23755; color_mode = 1; convert_xy(light);
  }  else if ( new_scene == 10) {
    bri[light] = 216; x = 0.393209; y = 0.29961; color_mode = 1; convert_xy(light);
  }
}

void process_lightdata(uint8_t light, float transitiontime) {
  transitiontime *= 17 - (NUM_LEDS / 40); //every extra led add a small delay that need to be counted
  if (color_mode == 1 && light_state[light] == true) {
    convert_xy(light);
  } else if (color_mode == 2 && light_state[light] == true) {
    convert_ct(light);
  } else if (color_mode == 3 && light_state[light] == true) {
    convert_hue(light);
  }
  transitiontime *= 16;
  for (uint8_t color = 0; color < LED_COLORS; color++) {
    if (light_state[light]) {
      step_level[light][color] = ((float)rgb[light][color] - current_rgb[light][color]) / transitiontime;
    } else {
      step_level[light][color] = current_rgb[light][color] / transitiontime;
    }
  }
}

// function to get white pwm value


void lightEngine() {
  for (int i = 0; i < NUM_LIGHTS; i++) {
    for (uint8_t color = 0; color < LED_COLORS; color++) {
      if (light_state[i]) {
        if (rgb[i][color] != current_rgb[i][color] ) {
          in_transition = true;
          current_rgb[i][color] += step_level[i][color];
          if ((step_level[i][color] > 0.0f && current_rgb[i][color] > rgb[i][color]) || (step_level[i][color] < 0.0f && current_rgb[i][color] < rgb[i][color])) {
            current_rgb[i][color] = rgb[i][color];
          }
  //        leds[0]=CRGB((int)current_rgb[0], (int)current_rgb[1], (int)current_rgb[2]);
//          fill_solid(leds + strip[i][0], strip[i][1] - strip[i][0], CRGB((int)current_rgb[0], (int)current_rgb[1], (int)current_rgb[2]));
          fill_solid(leds2, NUM_LEDS, CRGB((int)current_rgb[0], (int)current_rgb[1], (int)current_rgb[2]));
          FastLED.show();
        }
      } else {
        if (current_rgb[color] != 0) {
          in_transition = true;
          current_rgb[i][color] -= step_level[i][color];
          if (current_rgb[i][color] < 0.0f) {
            current_rgb[i][color] = 0;
          }
  //        leds[0]=CRGB((int)current_rgb[0], (int)current_rgb[1], (int)current_rgb[2]);
//          fill_solid(leds + strip[i][0], strip[i][1] - strip[i][0], CRGB((int)current_rgb[0], (int)current_rgb[1], (int)current_rgb[2]));
          fill_solid(leds2, NUM_LEDS, CRGB((int)current_rgb[0], (int)current_rgb[1], (int)current_rgb[2]));
          FastLED.show();
        }
      }
    }
  }
  if (in_transition) {
    FastLED.delay(1);
    in_transition = false;
  }
}

void setup() {
  if(USE_F_LED_CC == true) {
    FastLED.addLeds<LED_TYPE, DATA_PIN_RGB, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( CORRECTION );
    FastLED.addLeds<LED_TYPE, DATA_PIN_WWA, COLOR_ORDER>(leds2, NUM_LEDS).setCorrection( CORRECTION );
  } else {
    FastLED.addLeds<LED_TYPE, DATA_PIN_RGB, COLOR_ORDER>(leds, NUM_LEDS); 
    FastLED.addLeds<LED_TYPE, DATA_PIN_WWA, COLOR_ORDER>(leds2, NUM_LEDS); 
  }
  EEPROM.begin(512);
  
#ifdef USE_STATIC_IP
  WiFi.config(strip_ip, gateway_ip, subnet_mask);
#endif

  for (uint8_t light = 0; light < NUM_LIGHTS; light++) {
    apply_scene(EEPROM.read(2), light);
    for (uint8_t j = 0; j < 3; j++) {
      step_level[light][j] = rgb[light][0] / 150.0; step_level[light][1] = rgb[light][1] / 150.0; step_level[light][2] = rgb[light][2] / 150.0;
    }
  }

  if (EEPROM.read(1) == 1 || (EEPROM.read(1) == 0 && EEPROM.read(0) == 1)) {
    for (int i = 0; i < NUM_LIGHTS; i++) {
      light_state[i] = true;
    }
    for (uint8_t j = 0; j < 200; j++) {
      lightEngine();
    }
  }

  WiFiManager wifiManager;
  wifiManager.autoConnect("New Hue Light");

  if (! light_state) {
    infoLight(white);
    while (WiFi.status() != WL_CONNECTED) {
      infoLight(red);
      delay(500);
    }
    // Show that we are connected
    infoLight(green);

  }

  WiFi.macAddress(mac);

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.begin();
  
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
//  digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH

  server.on("/switch", []() {
    server.send(200, "text/plain", "OK");
    for (int light = 0; light < NUM_LIGHTS; light++) {
      int button;
      for (uint8_t i = 0; i < server.args(); i++) {
        if (server.argName(i) == "button") {
          button = server.arg(i).toInt();
        }
      }
      if (button == 1000) {
        if (light_state == false) {
          light_state[light] = true;
          scene = 0;
        } else {
          apply_scene(scene, light);
          scene++;
          if (scene == 11) {
            scene = 0;
          }
        }
      } else if (button == 2000) {
        if (light_state == false) {
          bri[light] = 30;
          light_state[light] = true;
        } else {
          bri[light] += 30;
        }
        if (bri[light] > 255) bri[light] = 255;
        if (color_mode == 1) convert_xy(light);
        else if (color_mode == 2) convert_ct(light);
        else if (color_mode == 3) convert_hue(light);
      } else if (button == 3000 && light_state[light] == true) {
        bri[light] -= 30;
        if (bri[light] < 1) bri[light] = 1;
        else {
          if (color_mode == 1) convert_xy(light);
          else if (color_mode == 2) convert_ct(light);
          else if (color_mode == 3) convert_hue(light);
        }
      } else if (button == 4000) {
        light_state[light] = false;
      }
      for (uint8_t color = 0; color < LED_COLORS; color++) {
        if (light_state) {
          step_level[light][color] = (rgb[light][color] - current_rgb[light][color]) / 54;
        } else {
          step_level[light][color] = current_rgb[light][color] / 54;
        }
      }
    }
  });


  server.on("/set", []() {
    uint8_t light;
//    light_state = true;
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
        color_mode = 0;
      }
      else if (server.argName(i) == "g") {
        rgb[light][1] = server.arg(i).toInt();
        color_mode = 0;
      }
      else if (server.argName(i) == "b") {
        rgb[light][2] = server.arg(i).toInt();
        color_mode = 0;
      }
      else if (server.argName(i) == "w") {
//        rgb[light][3] = server.arg(i).toInt();
        color_mode = 0;
      }
      else if (server.argName(i) == "x") {
        x = server.arg(i).toFloat();
        color_mode = 1;
      }
      else if (server.argName(i) == "y") {
        y = server.arg(i).toFloat();
        color_mode = 1;
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
        color_mode = 2;
      }
      else if (server.argName(i) == "sat") {
        sat = server.arg(i).toInt();
        color_mode = 3;
      }
      else if (server.argName(i) == "hue") {
        hue = server.arg(i).toInt();
        color_mode = 3;
      }
      else if (server.argName(i) == "alert" && server.arg(i) == "select") {
        if (light_state[light]) {
          current_rgb[light][0] = 0; current_rgb[light][1] = 0; current_rgb[light][2] = 0;
        } else {
          current_rgb[light][0] = 255; current_rgb[light][1] = 255; current_rgb[light][2] = 255;
        }
      }
      else if (server.argName(i) == "transitiontime") {
        transitiontime = server.arg(i).toInt();
      }
    }
    server.send(200, "text/plain", "OK, x: " + (String)x + ", y:" + (String)y + ", bri:" + (String)bri[light] + ", ct:" + ct[light] + ", colormode:" + color_mode + ", state:" + light_state[light]);
    if (color_mode == 1 && light_state[light] == true) {
      convert_xy(light);
    } else if (color_mode == 2 && light_state[light] == true) {
      convert_ct(light);
    } else if (color_mode == 3 && light_state[light] == true) {
      convert_hue(light);
    }
    transitiontime *= 16;
    for (uint8_t color = 0; color < LED_COLORS; color++) {
      if (light_state[light]) {
        step_level[light][color] = (rgb[light][color] - current_rgb[light][color]) / transitiontime;
      } else {
        step_level[light][color] = current_rgb[light][color] / transitiontime;
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
    if (color_mode == 1)
      colormode = "xy";
    else if (color_mode == 2)
      colormode = "ct";
    else if (color_mode == 3)
      colormode = "hs";
    server.send(200, "text/plain", "{\"on\": " + power_status + ", \"bri\": " + (String)bri[light] + ", \"xy\": [" + (String)x + ", " + (String)y + "], \"ct\":" + (String)ct[light] + ", \"sat\": " + (String)sat + ", \"hue\": " + (String)hue + ", \"colormode\": \"" + colormode + "\"}");
  });  


//  server.on("/detect", []() {
//    server.send(200, "text/plain", "{\"hue\": \"strip\",\"lights\": " + (String)NUM_LIGHTS + " ,\"modelid\": \"LCT015\",\"mac\": \"" + String(mac[5], HEX) + ":"  + String(mac[4], HEX) + ":" + String(mac[3], HEX) + ":" + String(mac[2], HEX) + ":" + String(mac[1], HEX) + ":" + String(mac[0], HEX) + "\"}");
//  });

  server.on("/detect", []() {
    char macString[32] = {0};
    sprintf(macString, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    DynamicJsonBuffer newBuffer;
    JsonObject& root = newBuffer.createObject();
    root["hue"] = "strip";
    root["name"] = "Hue rgb strip";
    root["lights"] = 1;
    root["protocol"] = "native_multi";
    root["modelid"] = "LST002";
    root["type"] = "ws2812_strip";
    root["mac"] = String(macString);
    root["version"] = 2.0;
    String output;
    root.printTo(output);
    server.send(200, "text/plain", output);
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
          if (  EEPROM.read(2) != server.arg("scene").toInt() && EEPROM.read(1) < 2) {
            EEPROM.write(2, server.arg("scene").toInt());
            EEPROM.commit();
          }
          apply_scene(server.arg("scene").toInt(), light);
        } else {
          if (server.arg("bri") != "") {
            bri[light] = server.arg("bri").toInt();
          }
          if (server.arg("hue") != "") {
            hue = server.arg("hue").toInt();
          }
          if (server.arg("sat") != "") {
            sat = server.arg("sat").toInt();
          }
          if (server.arg("ct") != "") {
            ct[light] = server.arg("ct").toInt();
          }
          if (server.arg("colormode") == "1" && light_state[light] == true) {
            convert_xy(light);
          } else if (server.arg("colormode") == "2" && light_state[light] == true) {
            convert_ct(light);
          } else if (server.arg("colormode") == "3" && light_state[light] == true) {
            convert_hue(light);
          }
          color_mode = server.arg("colormode").toInt();
        }
      } else if (server.hasArg("on")) {
        if (server.arg("on") == "true") {
          light_state[light] = true; {
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
        if (light_state) {
          current_rgb[light][0] = 0; current_rgb[light][1] = 0; current_rgb[light][2] = 0;
        }
      }
      for (uint8_t color = 0; color < LED_COLORS; color++) {
        if (light_state) {
          step_level[light][color] = ((float)rgb[light][color] - current_rgb[light][color]) / transitiontime;
        } else {
          step_level[light][color] = current_rgb[light][color] / transitiontime;
        }
      }
    }
    if (server.hasArg("reset")) {
      ESP.reset();
    }




    String http_content = "<!doctype html>";
    http_content += "<html>";
    http_content += "<head>";
    http_content += "<meta charset=\"utf-8\">";
    http_content += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
    http_content += "<title>Light Setup</title>";
    http_content += "<link rel=\"stylesheet\" href=\"https://unpkg.com/purecss@0.6.2/build/pure-min.css\">";
    http_content += "</head>";
    http_content += "<body>";
    http_content += "<fieldset>";
    http_content += "<h3>Light Setup</h3>";
    http_content += "<form class=\"pure-form pure-form-aligned\" action=\"/\" method=\"post\">";
    http_content += "<div class=\"pure-control-group\">";
    http_content += "<label for=\"power\"><strong>Power</strong></label>";
    http_content += "<a class=\"pure-button"; if (light_state[0]) http_content += "  pure-button-primary"; http_content += "\" href=\"/?on=true\">ON</a>";
    http_content += "<a class=\"pure-button"; if (!light_state[0]) http_content += "  pure-button-primary"; http_content += "\" href=\"/?on=false\">OFF</a>";
    http_content += "</div>";
    http_content += "<div class=\"pure-control-group\">";
    http_content += "<label for=\"startup\">Startup</label>";
    http_content += "<select onchange=\"this.form.submit()\" id=\"startup\" name=\"startup\">";
    http_content += "<option "; if (EEPROM.read(1) == 0) http_content += "selected=\"selected\""; http_content += " value=\"0\">Last state</option>";
    http_content += "<option "; if (EEPROM.read(1) == 1) http_content += "selected=\"selected\""; http_content += " value=\"1\">On</option>";
    http_content += "<option "; if (EEPROM.read(1) == 2) http_content += "selected=\"selected\""; http_content += " value=\"2\">Off</option>";
    http_content += "</select>";
    http_content += "</div>";
    http_content += "<div class=\"pure-control-group\">";
    http_content += "<label for=\"scene\">Scene</label>";
    http_content += "<select onchange = \"this.form.submit()\" id=\"scene\" name=\"scene\">";
    http_content += "<option "; if (EEPROM.read(2) == 0) http_content += "selected=\"selected\""; http_content += " value=\"0\">Relax</option>";
    http_content += "<option "; if (EEPROM.read(2) == 1) http_content += "selected=\"selected\""; http_content += " value=\"1\">Read</option>";
    http_content += "<option "; if (EEPROM.read(2) == 2) http_content += "selected=\"selected\""; http_content += " value=\"2\">Concentrate</option>";
    http_content += "<option "; if (EEPROM.read(2) == 3) http_content += "selected=\"selected\""; http_content += " value=\"3\">Energize</option>";
    http_content += "<option "; if (EEPROM.read(2) == 4) http_content += "selected=\"selected\""; http_content += " value=\"4\">Bright</option>";
    http_content += "<option "; if (EEPROM.read(2) == 5) http_content += "selected=\"selected\""; http_content += " value=\"5\">Dimmed</option>";
    http_content += "<option "; if (EEPROM.read(2) == 6) http_content += "selected=\"selected\""; http_content += " value=\"6\">Nightlight</option>";
    http_content += "<option "; if (EEPROM.read(2) == 7) http_content += "selected=\"selected\""; http_content += " value=\"7\">Savanna sunset</option>";
    http_content += "<option "; if (EEPROM.read(2) == 8) http_content += "selected=\"selected\""; http_content += " value=\"8\">Tropical twilight</option>";
    http_content += "<option "; if (EEPROM.read(2) == 9) http_content += "selected=\"selected\""; http_content += " value=\"9\">Arctic aurora</option>";
    http_content += "<option "; if (EEPROM.read(2) == 10) http_content += "selected=\"selected\""; http_content += " value=\"10\">Spring blossom</option>";
    http_content += "</select>";
    http_content += "</div>";
    http_content += "<br>";
    http_content += "<div class=\"pure-control-group\">";
    http_content += "<label for=\"state\"><strong>State</strong></label>";
    http_content += "</div>";
    http_content += "<div class=\"pure-control-group\">";
    http_content += "<label for=\"bri\">Bri</label>";
    http_content += "<input id=\"bri\" name=\"bri\" type=\"text\" placeholder=\"" + (String)bri[0] + "\">";
    http_content += "</div>";
    http_content += "<div class=\"pure-control-group\">";
    http_content += "<label for=\"hue\">Hue</label>";
    http_content += "<input id=\"hue\" name=\"hue\" type=\"text\" placeholder=\"" + (String)hue + "\">";
    http_content += "</div>";
    http_content += "<div class=\"pure-control-group\">";
    http_content += "<label for=\"sat\">Sat</label>";
    http_content += "<input id=\"sat\" name=\"sat\" type=\"text\" placeholder=\"" + (String)sat + "\">";
    http_content += "</div>";
    http_content += "<div class=\"pure-control-group\">";
    http_content += "<label for=\"ct\">CT</label>";
    http_content += "<input id=\"ct\" name=\"ct\" type=\"text\" placeholder=\"" + (String)ct[0] + "\">";
    http_content += "</div>";
    http_content += "<div class=\"pure-control-group\">";
    http_content += "<label for=\"colormode\">Color</label>";
    http_content += "<select id=\"colormode\" name=\"colormode\">";
    http_content += "<option "; if (color_mode == 1) http_content += "selected=\"selected\""; http_content += " value=\"1\">xy</option>";
    http_content += "<option "; if (color_mode == 2) http_content += "selected=\"selected\""; http_content += " value=\"2\">ct</option>";
    http_content += "<option "; if (color_mode == 3) http_content += "selected=\"selected\""; http_content += " value=\"3\">hue</option>";
    http_content += "</select>";
    http_content += "</div>";
    http_content += "<div class=\"pure-controls\">";
    http_content += "<span class=\"pure-form-message\"><a href=\"/?alert=1\">alert</a> or <a href=\"/?reset=1\">reset</a></span>";
    http_content += "<label for=\"cb\" class=\"pure-checkbox\">";
    http_content += "</label>";
    http_content += "<button type=\"submit\" class=\"pure-button pure-button-primary\">Save</button>";
    http_content += "</div>";
    http_content += "</fieldset>";
    http_content += "</form>";
    http_content += "</body>";
    http_content += "</html>";


    server.send(200, "text/html", http_content);

  });

  server.on("/reset", []() {
    server.send(200, "text/html", "reset");
    delay(100);
    ESP.reset();
  });


  server.onNotFound(handleNotFound);

  server.begin();
}

void loop() {
  ArduinoOTA.handle();
  server.handleClient();
  lightEngine();
}
