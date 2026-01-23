#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>

#include "Constants.h"
#include "Types.h"
#include "Motor.h"
#include "Manager.h"
#include "SerialConsole.h"

// Function Declarations

// Constant Declarations
SerialConsole console;

// Setup
void setup() {
  console.init(Const::Serial::BAUD);
}

// Program Loop
void loop() {
  console.execute();
}