#include <TouchScreen.h>

TouchScreen ts = TouchScreen(A1, A0, A3, A2, 0 ); 

void setup() {
  Serial.begin(115200);
}

void loop() {
  PID(0, 0); 
}

void PID(double xSetPoint, double ySetPoint){
  TSPoint p = ts.getPoint(); 
  Serial.println((String) "X OUT = " + (p.x - xSetPoint) + "   Y OUT = " + (p.y - ySetPoint)); 
}