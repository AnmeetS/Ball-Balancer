#include <TouchScreen.h>

TouchScreen ts = TouchScreen(A1, A0, A3, A2, 0 ); //touch screen pins (XGND, YGND, X5V, Y5V)

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  PID(0, 0);  //(X setpoint, Y setpoint) -- must be looped
<<<<<<< Updated upstream

=======
>>>>>>> Stashed changes
}

void PID(double xSetPoint, double ySetPoint){
  TSPoint p = ts.getPoint();  //measure X and Y positions
  Serial.println((String) "X OUT = " + (p.x - xSetPoint) + "   Y OUT = " + (p.y - ySetPoint));  //print X and Y outputs
}