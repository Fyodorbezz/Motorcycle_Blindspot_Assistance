/*//Library
#include <Arduino.h>
#include<SPI.h>
#include<Adafruit_GFX.h>
#include<Adafruit_ILI9341.h>
//screen stuff
#define TFT_RST  8
#define TFT_DC   9
#define TFT_CS   10
#define TFT_BACKLIGHT 3
//button defenition
#define BUTTON_UP 6
#define BUTTON_DOWN A4
#define BUTTON_BACK 7
#define BUTTON_SELECT 5
//screen width/height
#define SCREEN_WIDTH  240
#define SCREEN_HEIGHT 320
//LED?
#define LEDS1_PIN 2
#define LEDS2_PIN 4
//fontsize
#define fontsize 2
//Margin
#define width 15

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS,TFT_DC,TFT_RST);
//testing stuff
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


void setup(){
  //Setup background
  Serial.begin(115200);
  //initial backlight
  pinMode(TFT_BACKLIGHT,OUTPUT);
  tft.begin();
  tft.setRotation(0);
  tft.fillScreen(ILI9341_BLACK);
  //button
  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);
  pinMode(BUTTON_SELECT, INPUT_PULLUP);
  pinMode(BUTTON_BACK, INPUT_PULLUP);
  analogWrite(TFT_BACKLIGHT, brightnessLevels[brightness]);
  currentSelection = 1;
  Start = false;
  drawMenu();
  updateSettings();
  updateMenu();


}


void loop(){
  handleInput();
  delay(100);
  testVariables();
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
  if (Start != lastStart) {
    Serial.print("Start: ");
    Serial.println(Start);
    lastStart = Start;
    updated = true;
  }
 
  if (sensitivity != lastSensitivity) {
    Serial.print("Sensitivity: ");
    Serial.println(sensitivity);
    lastSensitivity = sensitivity;
    updated = true;
  }
 
  if (range != lastRange) {
    Serial.print("Range: ");
    Serial.println(range);
    lastRange = range;
    updated = true;
  }
 
  if (collision != lastCollision) {
    Serial.print("Collision: ");
    Serial.println(collision);
    lastCollision = collision;
    updated = true;
  }
 
  if (autoBreak != lastAutoBreak) {
    Serial.print("AutoBreak: ");
    Serial.println(autoBreak ? "Yes" : "No");
    lastAutoBreak = autoBreak;
    updated = true;
  }
 
  if (brightness != lastBrightness) {
    Serial.print("Brightness: ");
    Serial.println(brightness + 1);
    lastBrightness = brightness;
    updated = true;
  }
 
  if (LED != lastLED) {
    Serial.print("LED: ");
    Serial.println(LED + 1);
    lastLED = LED;
    updated = true;
  }

  if (updated) {
    Serial.println();
  }
}
*/