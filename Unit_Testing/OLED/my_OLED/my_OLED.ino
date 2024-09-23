
#include"my_OLED.h"

void drawTuffy(void) {
  display.clearDisplay();
  display.drawBitmap(0, 0, myBitmap, LOGO_WIDTH, LOGO_HEIGHT, 1);
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 48);
  display.print("Tuffy V1.0");
  display.display();
}
void displayData(int x , int y ,uint8_t textSize ,String Label,uint16_t Data){
  display.setTextSize(textSize);
  display.setCursor(x, y);
  display.setTextColor(SSD1306_WHITE);
  display.print(Label);
  display.print(Data);
  display.display();
}
