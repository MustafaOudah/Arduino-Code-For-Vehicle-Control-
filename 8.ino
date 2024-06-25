void sweep()                          // Can be used to simulate Metal Detecting or to rotate Ping Sensor
{ 
 Speed.attach(53); 
 int val = turn_Speed;            // reads the value of the potentiometer (value between 0 and 1023)
 m.write(val);                  // sets the servo position according to the scaled value
 delay(15);
 } 

