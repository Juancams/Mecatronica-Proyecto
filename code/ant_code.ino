#include <Servo.h>
#include <TimerOne.h>

int pos = 0;

int pinservo1 = 11;
int pinservo2 = 9;
int pinservo3 = 5;
int pinbutton = 2;

volatile bool first_time = true;

volatile bool move_ant = false;

Servo servo1;
Servo servo2;
Servo servo3;

void initialize()
{
    servo1.write(94);
    servo2.write(102);
    servo3.write(94);
    // wait 15 ms for servo to reach the position
    delay(1000); // Wait for 15 millisecond(s)
}

void callback_BUTTON()
{
  move_ant = !move_ant;
  first_time = true;
}

void setup()
{
  pinMode(pinservo1, OUTPUT);
  pinMode(pinservo2, OUTPUT);
  pinMode(pinservo3, OUTPUT);
  pinMode(pinbutton, INPUT);
  servo1.attach(pinservo1);
  servo2.attach(pinservo2);
  servo3.attach(pinservo3);
  attachInterrupt(digitalPinToInterrupt(pinbutton), callback_BUTTON, RISING);

  initialize();
  Serial.begin(9600);

}

void dancing()
{
  for (pos = 94; pos <= 124; pos++) {
    servo1.write(pos);
    servo3.write(pos);
    
    if ((pos <= 124) and (pos >= 102)){
      servo2.write(pos);
    }
    delay(15); // Wait for 15 millisecond(s)
  }
  
  for (pos = 124; pos >= 64; pos--) {
    servo1.write(pos);
    servo3.write(pos);
    
    if(pos <= 124 and pos >= 80){
      servo2.write(pos);
    }
    delay(15); // Wait for 15 millisecond(s)
  }
  
  for (pos = 64; pos <= 102; pos++) {
    if (pos <= 94) {
      servo1.write(pos);
      servo3.write(pos);
    }
    
    if (pos >= 80){
      servo2.write(pos);
    }
    delay(15); // Wait for 15 millisecond(s)
  }
  
}

void walking_forwards()
{ 
  if (first_time) {
    // mover delante
    for (pos = 94; pos <= 124; pos++) {
      servo1.write(pos);
      servo3.write(pos);
      delay(10); // Wait for 15 millisecond(s)
    }

    // mover medio
    for (pos = 102; pos <= 122; pos++) {
      servo2.write(pos);
      delay(10); // Wait for 15 millisecond(s)
    }
    first_time = false;
  }
  
  // mover al otro angulo
  
  for (pos = 124; pos >= 64; pos--) {
    servo1.write(pos);
    servo3.write(pos);
    delay(10); // Wait for 15 millisecond(s)
  }

  // mover medio
  for (pos = 122; pos >= 82; pos--) {
    servo2.write(pos);
    delay(10); // Wait for 15 millisecond(s)
  }

  // volver a inicial
  for (pos = 64; pos <= 124; pos++) {
    servo1.write(pos);
    servo3.write(pos);
    delay(10); // Wait for 15 millisecond(s)
  }

  for (pos = 82; pos <= 122; pos++) {
    servo2.write(pos);
    delay(10); // Wait for 15 millisecond(s)
  } 
}
void loop()
{
  if (move_ant == true) {
    walking_forwards();
  }
  else
  {
    initialize();
  }
}
