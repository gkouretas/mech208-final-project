#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);

void setup()
{
    byte error;
    Wire.begin();
    Wire.beginTransmission(0x27);
    error = Wire.endTransmission();
    if(error == 0){                      // If your device is 0x27
      // LiquidCrystal_I2C lcd(0x27,20,4);
      lcd.init(); 
      lcd.backlight();
      lcd.setCursor(0,0);
      lcd.print("Device Addr = 0x27");
    }
    // else{                               // If your device is 0x3F
    //   LiquidCrystal_I2C lcd(0x3F,20,4);
    //   lcd.init(); 
    //   lcd.backlight();
    //   lcd.setCursor(0,0);
    //   lcd.print("Device Addr = 0x3F");
    // }
}

void loop(){
  for (int i=0; i<4; i++) {
    lcd.setCursor(0,i);
    lcd.print(i);
    for (int j=0; j<19; j++) {
      lcd.print(" "); // print 19 spaces
    }
  }
}