
  void pingToggle()                                         // Turns Collision avoidance on/ off
 {
 
  if (pingOn == true) {
    pingOn = false;
    Serial1.print("Collision Avoidance OFF");
  }
    else if (pingOn == false) {
    pingOn = true;
    Serial1.print("Collision Avoidance ON");
  }
  
 }
 
//********************************************************************************************************************************


void SENSOR() 
{
 
 if (pingOn == true)
 {
  
 long duration1, distance1;
  digitalWrite(trigPin1, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH);
  distance1 = (duration1/2) / 29.1;

   if (distance1 >= 500 || distance1 <= 0){
    Serial.println("Left Sensor Out of range");
  }
  else {
    Serial.print ( "Left Sensor  ");
    Serial.print ( distance1);
    Serial.println("cm");
  }

 long duration2, distance2;
  digitalWrite(trigPin2, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH);
  distance2= (duration2/2) / 29.1;

   if (distance2 >= 300 || distance2 <= 0){
    Serial.println("Mid Sensor Out of range");
  }
  else {
    Serial.print("Mid Sensor  ");
    Serial.print(distance2);
    Serial.println("cm");
  }

  long duration3, distance3;
  digitalWrite(trigPin3, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin3, LOW);
  duration3 = pulseIn(echoPin3, HIGH);
  distance3= (duration3/2) / 29.1;

   if (distance3 >= 500 || distance3 <= 0){
    Serial.println("Right Sensor Out of range");
  }
  else {
    Serial.print("Right Sensor  ");
    Serial.print(distance3);
    Serial.println("cm");
  }

  long duration4, distance4;
  digitalWrite(trigPin4, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin4, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin4, LOW);
  duration4 = pulseIn(echoPin4, HIGH);
  distance4= (duration4/2) / 29.1;

   if (distance4 >= 300 || distance4 <= 0){
    Serial.print("Back Sensor Out of range ");
   
  }
  else {
    Serial.print("Back Sensor  ");
    Serial.print(distance4);
    Serial.println("cm");
  }
 Serial.println("############################");
//delay (10);


//****************************************************************************************************************************
if (( distance2 <50 ) &&( distance2 > 10 )&&( distance4 <50 ) &&( distance4 > 10 ))
{
  Serial1.println("Dangerous Area 🚧");
  StopCar();
  //delay (10);
}

else if (( distance2 < 100 ) && ( distance2 > 10 )&& (distance4 >=100))
{
   Serial1.println("Backward");
   Reverse();
   //delay(10);
}   

else
{

if ( distance2 >=130 ) 
{
  Serial1.println("Safe Area 😌😘");
  Forward();
 // delay(10); 
}
else if (( distance2 <130 ) &&( distance2 > 100 ) && (distance3 >=100) && (distance1 < distance3))
{
  Serial1.println("Turn right :===>");
  SensorTurnRight();
  SensorTurnLeft();
  //delay(10);
}

else if (( distance2 <130 ) && ( distance2 > 100 )&& (distance1 >=100) && (distance1 > distance3))
{
  Serial1.println("Turn left <===:");
  SensorTurnLeft();
  SensorTurnRight();
  //delay(10);
}

}

 }
  //Compass_Forward();
  bluetooth();
   delay(1);   
  
  
 }
 
 

