#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Set the LCD address to 0x27 or 0x3F for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 20, 4);  // Adjust the address (0x27) if necessary

// Define your custom SDA and SCL pins
#define SDA_PIN 21
#define SCL_PIN 19

void setup() {
  // Initialize I2C with the chosen pins
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize the LCD
  lcd.init();
  
  // Turn on the backlight
  lcd.backlight();

  // Print a message to the LCD
  lcd.setCursor(0, 0); // Set the cursor to column 0, line 1
  lcd.print("Hello, World!");
}

void loop() {
  // No need to repeat anything in the loop for a basic display
}
