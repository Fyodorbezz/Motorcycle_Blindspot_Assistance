
#include <Arduino.h>

#include <SPI.h>
#include <Adafruit_GFX.h>


#include <string.h>
#include <FastLED.h>
#include <ILI9341_Fast.h>

#define TFT_RST  8
#define TFT_DC   9
#define TFT_CS   10 

#define TFT_BACKLIGHT 3

#define BTN1_PIN 6
#define BTN2_PIN A4
#define BTN3_PIN 7
#define BTN4_PIN 5

#define LEDS1_PIN 2
#define LEDS2_PIN 4

#define LED_NUM 12

#define BLACK_565 0x0000       ///<   0,   0,   0
#define NAVY_565 0x000F        ///<   0,   0, 123
#define DARKGREEN_565 0x03E0   ///<   0, 125,   0
#define DARKCYAN_565 0x03EF    ///<   0, 125, 123
#define MAROON_565 0x7800      ///< 123,   0,   0
#define PURPLE_565 0x780F      ///< 123,   0, 123
#define DARK_PURPLE_565 RGBto565(50, 0, 50)
#define OLIVE_565 0x7BE0       ///< 123, 125,   0
#define LIGHTGRE_565Y 0xC618   ///< 198, 195, 198
#define DARKGREY_565 0x7BEF    ///< 123, 125, 123
#define GREY_565 RGBto565(50, 50, 50)
#define BLUE_565 0x001F        ///<   0,   0, 255
#define LIGHT_BLUE_565 RGBto565(0, 80, 255)
#define GREEN_565 0x07E0       ///<   0, 255,   0
#define CYAN_565 0x07FF        ///<   0, 255, 255
#define RED_565 0xF800         ///< 255,   0,   0
#define MAGENTA_565 0xF81F     ///< 255,   0, 255
#define YELLOW_565 0xFFE0      ///< 255, 255,   0
#define WHITE_565 0xFFFF       ///< 255, 255, 255
#define ORANGE_565 RGBto565(255, 70, 0)      ///< 255, 165,   0
#define GREENYELLOW_565 0xAFE5 ///< 173, 255,  41
#define PINK_565 0xFC18        ///< 255, 130, 198


#define ILI9341_BLACK BLACK_565
#define ILI9341_WHITE WHITE_565
#define ILI9341_YELLOW YELLOW_565
#define ILI9341_MAGENTA MAGENTA_565

CRGB leds1[LED_NUM];
CRGB leds2[LED_NUM];

ILI9341 tft = ILI9341(TFT_DC, TFT_RST, TFT_CS);
//Servo test;

struct Display_data{
  int8_t trash;
	int8_t zones_status[7] = {-1, -1, -1, -1, -1, -1, -1};
	uint8_t collisions[2] = {0, 0};
	uint8_t lidars_status[4];
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
  uint8_t power;
  uint8_t range;
  uint8_t sensetivity;
  uint8_t collision;
  uint8_t autoBreak;
  uint8_t brightness_1;
  uint8_t brightness_2;
};

struct Settings{
  uint8_t power;
  uint8_t range;
  uint8_t sensetivity;
  uint8_t collision;
  uint8_t autoBreak;
  uint8_t brightness_1;
  uint8_t brightness_2;
};

Display_data received_data;

Settings settings;

unsigned long long blink1_timer;
unsigned long long blink2_timer;

int blink_1_time = 500;
int blink_2_time = 500;

bool last_blik_state_1 = 0;
bool last_blik_state_2 = 0;

bool blink_1 = 0;
bool blink_2 = 0;

bool main_screen = 1;

int coordinates_x[4] = {126, 106, 126, 106};
int coordinates_y[4] = {120, 120, 190, 190};

void draw_motorcycle(){
  tft.fillTriangle(136, 210, 151, 160, 136, 110, LIGHT_BLUE_565);
  tft.fillTriangle(106, 210, 91, 160, 106, 110, LIGHT_BLUE_565);
  tft.fillRect(107, 110, 29, 101, LIGHT_BLUE_565);

  for(int i=0; i<4; i++){
    if (received_data.lidars_status[i] == 0){
      tft.fillRect(coordinates_x[3-i], coordinates_y[3-i], 10, 10, RED_565);
    }
    else if (received_data.lidars_status[i] == 1){
      tft.fillRect(coordinates_x[3-i], coordinates_y[3-i], 10, 10, ORANGE_565);
    }
    else if (received_data.lidars_status[i] == 2){
      tft.fillRect(coordinates_x[3-i], coordinates_y[3-i], 10, 10, GREEN_565);
    }
  }

  //tft.fillTriangle(140, 225, 160, 160, 140, 95, YELLOW_565);
  //tft.fillTriangle(100, 225, 80, 160, 100, 95, YELLOW_565);
  //tft.fillRect(101, 95, 39, 131, GREEN_565);
}

void draw_zone(int index, int color_index){
  uint16_t color;
  
  switch (color_index)
  {
  case -1:
    color = BLACK_565;
    break;
  
  case 0:
    color = GREY_565;
    break;
  
  case 1:
    color = YELLOW_565;
    break;

  case 2:
    color = ORANGE_565;
    break;
  
  case 3:
    color = RED_565;
    break;
  default:
    break;
  }

  switch (index)
  {
  case 0:
    tft.fillTriangle(218, 2, 163, 109, 218, 109, color);
    tft.fillRect(163, 110, 74, 20, color);
    tft.fillRect(218, 2, 19, 108, color);
  break;
  case 1:
    tft.fillTriangle(214, 2, 160, 107, 160, 2, color);
    tft.fillRect(122, 2, 38, 106, color);
  break;
  case 2:
    tft.fillTriangle(26, 2, 80, 107, 80, 2, color);
    tft.fillRect(81, 2, 38, 106, color);
  break;
  case 3:
    tft.fillTriangle(22, 2, 77, 109, 21, 109, color);
    tft.fillRect(2, 110, 76, 20, color);
    tft.fillRect(2, 2, 20, 108, color);
  break;
  case 4:
    tft.fillRect(2, 190, 76, 128, color);
    break;
  case 5:
    tft.fillRect(81, 213, 78, 105, color);
    break;
  case 6:
    tft.fillRect(162, 190, 75, 128, color);
    break;
  
  default:
    break;
  }
}

void led_zone(int index, int color_index){
  CRGB color;

  switch (color_index)
  {
  case -1:
    color = CRGB::Black;
    break;
  
  case 0:
    color = CRGB::Black;
    break;
  
  case 1:
    color = CRGB::Yellow;
    break;

  case 2:
    color = CRGB::Orange;
    break;
  
  case 3:
    color = CRGB::Red;
    break;
  default:
    break;
  }

  switch (index){
  case 0:
    for(int i=8; i<12; i++){
      leds2[i] = color;
    }
  break;
  case 1:
    for(int i=0; i<4; i++){
      leds2[i] = color;
    }
  break;
  case 2:
    for(int i=0; i<4; i++){
      leds1[i] = color;
    }
  break;
  case 3:
    for(int i=4; i<8; i++){
      leds1[i] = color;
    }
  break;
  case 4:
    for(int i=8; i<12; i++){
      leds1[i] = color;
    }
    break;
  case 6:
    for(int i=4; i<8; i++){
      leds2[i] = color;
    }
    break;
  
  default:
    break;
  }
}

void redraw_screeen(){
  //tft.fillScreen(DARK_PURPLE_565);
  if (main_screen){
    draw_motorcycle();
  }

  if(received_data.collisions[1] == 0){
    for(int i=0; i<4; i++){
      led_zone(i, received_data.zones_status[i]);
    }
    FastLED.show();
    if (main_screen){
      for(int i=0; i<4; i++){
        draw_zone(i, received_data.zones_status[i]);
      }
    }
  }

  if(received_data.collisions[0] == 0){
    if(received_data.collisions[1] == 0){
      for(int i=4; i<7; i++){
        led_zone(i, received_data.zones_status[i]);
      }
      FastLED.show();
    }

    if (main_screen){
      for(int i=4; i<7; i++){
        draw_zone(i, received_data.zones_status[i]);
      }
    }
  }
}

void redraw_low_screeen(){
  for(int i=4; i<7; i++){
    draw_zone(i, received_data.zones_status[i]);
  }

  tft.fillRect(78, 213, 3, 105, DARK_PURPLE_565);
  tft.fillRect(159, 213, 3, 105, DARK_PURPLE_565);
}

void rear_blink(bool state){
  if (state == 1){
    for(int i=4; i<8; i++){
      leds1[i+4] = CRGB::Red;
      leds2[i] = CRGB::Red;
    }
    FastLED.show();

    if (main_screen){
      tft.fillRect(2, 213, 235, 105, RED_565);
    }
  }
  if (state == 0){
    //for(int i=4; i<7; i++){
    //  led_zone(i, received_data.zones_status[i]);
    //}
    for(int i=4; i<8; i++){
      leds1[i+4] = CRGB::Black;
      leds2[i] = CRGB::Black;
    }
    FastLED.show();

    if (main_screen){
      redraw_low_screeen();
    }
  }
}

void redraw_upper_screeen(){
  for(int i=0; i<4; i++){
    draw_zone(i, received_data.zones_status[i]);
  }

  tft.fillRect(119, 2, 3, 106, DARK_PURPLE_565);
  tft.drawLine(22, 2, 77, 109, DARK_PURPLE_565);
  tft.drawLine(23, 2, 78, 109, DARK_PURPLE_565);
  tft.drawLine(24, 2, 79, 109, DARK_PURPLE_565);
  tft.drawLine(25, 2, 80, 109, DARK_PURPLE_565);

  tft.drawLine(215, 2, 160, 109, DARK_PURPLE_565);
  tft.drawLine(216, 2, 161, 109, DARK_PURPLE_565);
  tft.drawLine(217, 2, 162, 109, DARK_PURPLE_565);
  tft.drawLine(218, 2, 163, 109, DARK_PURPLE_565);
  
}

void forward_blink(bool state){
  if (state == 1){
    for(int i=0; i<12; i++){
      leds1[i] = CRGB::Red;
      leds2[i] = CRGB::Red;
    }
    FastLED.show();

    if (main_screen){
      tft.fillRect(2, 2, 235, 106, RED_565);
    }
  }
  if (state == 0){
    for(int i=0; i<12; i++){
      led_zone(i, received_data.zones_status[i]);
    }
    FastLED.show();

    if (main_screen){
      redraw_upper_screeen();
    }
  }

  
}

void blinking(){
  if(received_data.collisions[0] == 1 && last_blik_state_1 == 0){
    last_blik_state_1 = 1;
    blink_1_time = 500;
    blink1_timer = millis();
    blink_1 = 1;
    rear_blink(blink_1);
  }
  else if(received_data.collisions[0] == 0 && last_blik_state_1 == 1){
    last_blik_state_1 = 0;
    blink_1 = 0;
    rear_blink(blink_1);
  }

  if (millis() - blink1_timer > blink_1_time && received_data.collisions[0] == 1){
    if (blink_1 == 1){
      blink_1_time = 100;
    }
    else{
      blink_1_time = 500;
    }

    blink_1 = !blink_1;
    rear_blink(blink_1);
    blink1_timer = millis();
  }

  if(received_data.collisions[1] == 1 && last_blik_state_2 == 0){
    last_blik_state_2 = 1;
    blink_2_time = 500;
    blink2_timer = millis();
    blink_2 = 1;
    forward_blink(blink_2);
  }
  else if(received_data.collisions[1] == 0 && last_blik_state_2 == 1){
    last_blik_state_2 = 0;
    blink_2 = 0;
    forward_blink(blink_2);
  }

  if (millis() - blink2_timer > blink_2_time && received_data.collisions[1] == 1){
    if (blink_2 == 1){
      blink_2_time = 100;
    }
    else{
      blink_2_time = 500;
    }

    blink_2 = !blink_2;
    forward_blink(blink_2);
    blink2_timer = millis();
  }
}

void transition_to_main_screen(){
  main_screen = 1;
  tft.fillScreen(DARK_PURPLE_565);
}

//Normal menu prototype
void handleInput();
void updateMenu();
void adjustSetting();
void updateSettings();
void drawMenu();

//Mini menu prototype
void openMiniMenu();
void closeMiniMenu();
void drawMiniMenu(int settingType);
void handleMiniMenuInput(int settingType);

void testVariables();

void transition_to_menu_screen();
void menu_loop();
void setVariables();

void setup() {
  Serial.begin(115200);
  

  pinMode(TFT_BACKLIGHT, OUTPUT);

  tft.begin();
  tft.setRotation(2);
  tft.fillScreen(DARK_PURPLE_565);
  digitalWrite(TFT_BACKLIGHT, 1);

  redraw_screeen();

  FastLED.addLeds<WS2812, LEDS1_PIN, GRB>(leds1, LED_NUM);
  FastLED.addLeds<WS2812, LEDS2_PIN, GRB>(leds2, LED_NUM);
  FastLED.setBrightness(50);

 /*for (int i = 0, j=1; i < 4; i++, j++) {
    leds1[i].setRGB(j*(255/4), j*(255/4), 0);
    leds2[i].setRGB(j*(255/4), j*(255/4), 0);
  }

  for (int i = 4, j=1; i < 8; i++, j++) {
    leds1[i].setRGB(j*(255/4), 0, j*(255/4));
    leds2[i].setRGB(j*(255/4), 0, j*(255/4));
  }

  for (int i = 8, j=1; i < 12; i++, j++) {
    leds1[i].setRGB(j*(255/4), j*(255/4), 0);
    leds2[i].setRGB(j*(255/4), j*(255/4), 0);
  }*/

  FastLED.show();

  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);
  pinMode(BTN3_PIN, INPUT_PULLUP);
  pinMode(BTN4_PIN, INPUT_PULLUP);

  received_data.collisions[0] = 0;
  received_data.collisions[1] = 0;
}

unsigned long long buttons_sample_timer = 0;

void loop() {
  blinking();
  if (Serial.available() >= sizeof(Display_data)){

    Serial.readBytes((byte*)&received_data, sizeof(Display_data));

    if(main_screen){
      settings.sensetivity = received_data.sensetivity;
      settings.range = received_data.range;
      settings.collision = received_data.collision;
      settings.autoBreak = received_data.autoBreak;
      settings.power = received_data.power;
      settings.brightness_1 = received_data.brightness_1;
      settings.brightness_2 = received_data.brightness_2;
      setVariables();
    }

    while(Serial.available()){
      Serial.read();
    }
    redraw_screeen();
  }

  if (millis() - buttons_sample_timer > 100){
    if ((digitalRead(BTN1_PIN) == 0 || digitalRead(BTN2_PIN) == 0 || digitalRead(BTN3_PIN) == 0 || digitalRead(BTN4_PIN) == 0) && main_screen == 1){
      main_screen = 0;
      transition_to_menu_screen();
    }
    if (main_screen == 0){
      menu_loop();
    }
    buttons_sample_timer = millis();
    //Serial.write((byte*)&settings, sizeof(settings));
  }
  //Serial.write((byte*)&settings, sizeof(settings));
  delay(5);
}

#define BUTTON_UP A4
#define BUTTON_DOWN 6
#define BUTTON_BACK 5
#define BUTTON_SELECT 7

#define SCREEN_WIDTH  240
#define SCREEN_HEIGHT 320
#define fontsize 2
#define width 15

bool lastStart = false;
int lastSensitivity = 0;
int lastRange = 0;
int lastCollision = 1;
bool lastAutoBreak = false;
int lastBrightness = 0;
int lastLED = 0;
//variable for menu navigation

int currentSelection = 0;
const int numOptions = 8;
int sensitivity = 0;
bool EXIT = false;
bool Start = true;
int range = 0;    
int LED = 0;
int collision = 1;  
bool autoBreak = false;
int brightness = 0;
//mini menu stuff
bool inMiniMenu = false;
int miniMenuSelection = 0;
const int miniMenuCount = 3;
const int miniMenuCountLED = 5;

//screen brightness level
int brightnessLevels[] = {100, 175, 255};

void menu_loop(){
  handleInput();
  
}

void transition_to_menu_screen(){
  currentSelection = 1;
  Start = false;
  drawMenu();
  updateSettings();
  updateMenu();
}

//Create a menu screen
void drawMenu() {
  tft.fillScreen(ILI9341_BLACK);
  tft.drawRect(2, 2, SCREEN_WIDTH - 4, SCREEN_HEIGHT - 4, ILI9341_MAGENTA);
  tft.drawRect(4, 4, SCREEN_WIDTH - 8, SCREEN_HEIGHT - 8, ILI9341_MAGENTA);
  tft.fillRect(160,4,2,243, ILI9341_MAGENTA);
  tft.fillRect(4,245,SCREEN_WIDTH-8,2,ILI9341_MAGENTA);
  tft.setTextColor(ILI9341_BLACK);
  tft.setCursor(width,14);
  tft.setTextSize(fontsize);
  tft.setTextColor(ILI9341_WHITE);
  tft.println("EXIT");
}

//Take unput from button
void handleInput() {
  if (!inMiniMenu) {
    if (digitalRead(BUTTON_UP) == LOW) {
      currentSelection = (currentSelection - 1 + numOptions) % numOptions;
      updateMenu();
    }
    if (digitalRead(BUTTON_DOWN) == LOW) {
      currentSelection = (currentSelection + 1) % numOptions;
      updateMenu();
    }
    if (digitalRead(BUTTON_SELECT) == LOW) {
      adjustSetting();
      updateMenu();
    }
  } else { // Changed to else for clarity
    if (currentSelection != 7) {
      if (digitalRead(BUTTON_UP) == LOW) {
        miniMenuSelection = (miniMenuSelection - 1 + miniMenuCount) % miniMenuCount;
        drawMiniMenu(currentSelection);
      }
      if (digitalRead(BUTTON_DOWN) == LOW) {
        miniMenuSelection = (miniMenuSelection + 1) % miniMenuCount;
        drawMiniMenu(currentSelection);
      }
    } else if (currentSelection == 7) { // Added else if for clarity
      if (digitalRead(BUTTON_UP) == LOW) {
        miniMenuSelection = (miniMenuSelection - 1 + miniMenuCountLED) % miniMenuCountLED; // Use LED_MINI_MENU_COUNT
        drawMiniMenu(currentSelection);
      }
      if (digitalRead(BUTTON_DOWN) == LOW) {
        miniMenuSelection = (miniMenuSelection + 1) % miniMenuCountLED; // Use LED_MINI_MENU_COUNT
        drawMiniMenu(currentSelection);
      }
    }


    if (digitalRead(BUTTON_SELECT) == LOW) {
      handleMiniMenuInput(currentSelection);
    }
    if (digitalRead(BUTTON_BACK) == LOW) {
      closeMiniMenu();
    }
  }
}

void updateMenu() {
  // Highlight selected option
  for (int i = 0; i < numOptions; i++) {
    tft.setCursor(width, 14 + (i * 30));
    tft.setTextColor(i == currentSelection ? ILI9341_YELLOW : ILI9341_WHITE);
    switch (i) {
      case 0: tft.println("EXIT"); break;
      case 1:
        tft.println(String(Start ? "STOP" : "START"));
        break;
      case 2: tft.println("Sensitivity"); break;
      case 3: tft.println("Range"); break;
      case 4: tft.println("Collision"); break;
      case 5: tft.println("AutoBreak"); break;
      case 6: tft.println("Brightness"); break;
      case 7: tft.println("LED"); break;
    }
  }
}

void adjustSetting() {
  if (currentSelection == 1) {
    Start = !Start;
    updateSettings();
  } else if (digitalRead(BUTTON_SELECT) == LOW) {
    openMiniMenu();
  }
}

void openMiniMenu() {
  inMiniMenu = true;


  switch (currentSelection) {
    case 2:
      miniMenuSelection = sensitivity;
      break;
    case 3:
      miniMenuSelection = range;
      break;
    case 4:
      miniMenuSelection = collision - 1;
      break;
    case 5:
      miniMenuSelection = autoBreak ? 0 : 1;
      break;
    case 6:
      miniMenuSelection = brightness;
      break;
    case 7:
      miniMenuSelection = LED;
      break;
    default:
      miniMenuSelection = 0;
      break;
  }


  drawMiniMenu(currentSelection);
}

void closeMiniMenu() {
  inMiniMenu = false;
  updateSettings();
  updateMenu();
  testVariables();
}

void drawMiniMenu(int settingType) {
  tft.fillRect(10, 250, SCREEN_WIDTH - 20, 65, ILI9341_BLACK); // Clear the bottom area
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(fontsize);


  switch (settingType) {
    case 0: // EXIT
      tft.setCursor(40, SCREEN_HEIGHT - 50);
      tft.setTextColor(miniMenuSelection == 0 ? ILI9341_YELLOW : ILI9341_WHITE);
      tft.print("Exit");
      tft.setCursor(140, SCREEN_HEIGHT - 50);
      tft.setTextColor(miniMenuSelection == 1 ? ILI9341_YELLOW : ILI9341_WHITE);
      tft.print("Cancel");
      break;
    case 1: // Start/Stop
      tft.setCursor(40, SCREEN_HEIGHT - 50);
      tft.setTextColor(miniMenuSelection == 0 ? ILI9341_YELLOW : ILI9341_WHITE);
      tft.print("START");
      tft.setCursor(140, SCREEN_HEIGHT - 50);
      tft.setTextColor(miniMenuSelection == 1 ? ILI9341_YELLOW : ILI9341_WHITE);
      tft.print("STOP");
      break;


    case 5: // AutoBreak
      miniMenuSelection = constrain(miniMenuSelection, 0, 1)%2;
      tft.setCursor(50, SCREEN_HEIGHT - 50);
      tft.setTextColor(miniMenuSelection == 0 ? ILI9341_YELLOW : ILI9341_WHITE);
      tft.print("Yes");
      tft.setCursor(150, SCREEN_HEIGHT - 50);
      tft.setTextColor(miniMenuSelection == 1 ? ILI9341_YELLOW : ILI9341_WHITE);
      tft.print("No");
    break;
   
    case 7: // LED
      for (int i = 0; i < miniMenuCountLED; i++) {
      tft.setCursor(20 + i * 40, SCREEN_HEIGHT - 50);
      tft.setTextColor(i == miniMenuSelection ? ILI9341_YELLOW : ILI9341_WHITE);
      tft.print(i + 1);
      }
           
    break;
    default:
      for (int i = 0; i < miniMenuCount; i++) {
        tft.setCursor(50 + i * 50, SCREEN_HEIGHT - 50);
        tft.setTextColor(i == miniMenuSelection ? ILI9341_YELLOW : ILI9341_WHITE);
        tft.print(i+1);
      }
      break;
  }
}

void handleMiniMenuInput(int settingType) {
  if (digitalRead(BUTTON_SELECT) == LOW) {
    switch (settingType) {
      case 0: // Exit
        EXIT = miniMenuSelection == 0;
        break;
      case 1: // Start/Stop
        Start = miniMenuSelection == 0;
        break;
      case 2: // Sensitivity
        sensitivity = miniMenuSelection;
        break;
      case 3: // Range
        range = miniMenuSelection;
        break;
      case 4: // Collision
        collision = miniMenuSelection + 1; // Collision ranges from 1 to 3
        break;
      case 5: // AutoBreak
        autoBreak = miniMenuSelection == 0; // 0 is Yes, 1 is No
        break;
      case 6: // Brightness
        brightness = miniMenuSelection;
        analogWrite(TFT_BACKLIGHT, brightnessLevels[brightness]); // Adjust brightness
        break;
      case 7: // LED
        LED = miniMenuSelection; // Levels 1, 2, 3
        break;
    }
    closeMiniMenu(); // Close the mini-menu after selection
  }
}

void updateSettings() {
  tft.fillRect(width,40,100,20,ILI9341_BLACK);
  tft.fillRect(165, 6, SCREEN_WIDTH - 170, 235, ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(fontsize);

  tft.setCursor(width, 14);
  tft.println("EXIT");

  tft.setCursor(width, 44);
  tft.println(String(Start ? "STOP" : "START"));

  tft.setCursor(width, 74);
  tft.println("Sensitivity  " + String(sensitivity+1) + " lvl");

  tft.setCursor(width, 104);
  tft.println("Range        " + String(range+1) + " lvl");




  tft.setCursor(width, 134);
  tft.println("Collision    " + String(collision) + " sec");




  tft.setCursor(width, 164);
  tft.println("AutoBreak    " + String(autoBreak ? " Yes" : " No"));




  tft.setCursor(width, 194);
  tft.println("Brightness   " + String(brightness+1) + " lvl");
 
  tft.setCursor(width, 224);
  tft.println("LED          " + String(LED+1) + " lvl");


  tft.fillRect(10, 250, SCREEN_WIDTH-20, 65, ILI9341_BLACK);


  for (int i = 0; i < numOptions; i++) {
    tft.drawRect(4, 4 + (i * 30),SCREEN_WIDTH-8,2,ILI9341_MAGENTA);


  }
}

void testVariables() {
   bool updated = false;

   settings.power = Start;
   settings.sensetivity = sensitivity+1;
   settings.range = range+1;
   settings.collision = collision;
   settings.autoBreak = autoBreak;
   settings.brightness_1 = brightness;
   settings.brightness_2 = LED;

   Serial.write((byte*)&settings, sizeof(settings));

  if (Start != lastStart) {
    
    lastStart = Start;
    updated = true;
  }
 
  if (sensitivity != lastSensitivity) {
    lastSensitivity = sensitivity;
    updated = true;
  }
 
  if (range != lastRange) {
    lastRange = range;
    updated = true;
  }
 
  if (collision != lastCollision) {
    lastCollision = collision;
    updated = true;
  }
 
  if (autoBreak != lastAutoBreak) {
    lastAutoBreak = autoBreak;
    updated = true;
  }
 
  if (brightness != lastBrightness) {

    lastBrightness = brightness;
    updated = true;
  }
 
  if (LED != lastLED) {

    lastLED = LED;
    updated = true;
  }

  if (EXIT) {
    transition_to_main_screen();
    //Serial.println();
  }
}

void setVariables(){
  Start = settings.power;
  sensitivity = settings.sensetivity - 1;
  range = settings.range - 1;
  collision = settings.collision;
  autoBreak = settings.autoBreak;
  brightness = settings.brightness_1;
  LED = settings.brightness_2;
}