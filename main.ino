#include <IRSeeker.h>
#include <Wire.h>

IRSeeker t1;

void setup() { 
  Wire.begin(); // join i2c bus (address optional for master)
  t1.Read(true);
}

void loop(){
  t1.Read(false); 
}

