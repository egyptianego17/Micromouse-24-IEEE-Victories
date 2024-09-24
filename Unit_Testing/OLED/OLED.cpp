#include"OLED_Interface.h"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void OLED_setup(){
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.setRotation(90);
  delay(10);
}
void OLED_drawTuffy(void) {
  display.clearDisplay();
  display.drawBitmap(0, 0, myBitmap, LOGO_WIDTH, LOGO_HEIGHT, 1);
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 48);
  display.print("Tuffy V1.0");
  display.display();
}
void OLED_displayData(int x , int y ,uint8_t textSize ,String Label,uint16_t Data){
  display.setTextSize(textSize);
  display.setCursor(x, y);
  display.setTextColor(SSD1306_WHITE);
  if (Label.length() > 0)
    display.print(Label);
  if (Data > 0){
    display.setTextColor(WHITE, BLACK);
    display.print(Data);
    display.print("cm");
    display.print("  ");
    }
  display.display();
}
