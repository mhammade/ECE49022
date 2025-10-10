#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);  // change to 0x3F if needed

void setup() {
  lcd.init();        // <-- required for LiquidCrystal_I2C
  lcd.backlight();
  LCD_Message("set", 1);
  LCD_Message("test1", 2);
}

void loop() {
  // your app logic...
    lcd.backlight();
    LCD_Message("set", 1);
    delay(500);
    LCD_Message("go", 1);
    delay(500);
}

void LCD_Message(const String& message, int row) {
  lcd.setCursor(0, row - 1);
  lcd.print("                "); // clear the line (16 spaces)
  lcd.setCursor(0, row - 1);
  lcd.print(message);
}
