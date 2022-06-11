 // Define Variables
float V;
float Rx;// Variables to convert voltage to resistance
float tempC,tempF;// Variables to convert resistance to temp
float R0 = 100.0; // resistance at 0 degree
float alpha = 0.00385; // constant
int Vin = A0; // Vin is Analog Pin A0
// p control
int setpoint = 12;
float Kp =100; // Proportional gain tunning parameter
float error = 0; //  e(t) =  setpoint - process variable
float Delay = 0; // for bit banging pwm
float gain;
// bts7960 motor driver pins definition
int R_IS=1; int R_EN=2; int R_PWM=3; int L_IS=4; int L_EN=5; int L_PWM=6;
float kalman_old=0; float cov_old=0; //kalman filter inital values
void setup() {
  Serial.begin(9600); // Set Baudrate at 9600
  pinMode(Vin,INPUT); // Make Vin Input
  pinMode(R_IS,OUTPUT); pinMode(R_EN,OUTPUT); pinMode(R_PWM,OUTPUT); pinMode(L_IS,OUTPUT); pinMode(L_EN,OUTPUT); pinMode(L_PWM,OUTPUT);
  // Making pins as output
  digitalWrite(R_IS,LOW); digitalWrite(L_IS,LOW); digitalWrite(R_EN,HIGH); digitalWrite(L_EN,HIGH);
}

void loop() {
  takeReadingTemperature();
  Serial.println(kalman_filter(tempC));
  //Serial.println(tempC);
  delay(20);
  error= (setpoint - tempF)/((setpoint + tempF)/2);
  Delay= (error)*1000+500+gain;
  //analogWrite(L_PWM,255); analogWrite(R_PWM,0); // peltier control
  if(setpoint<kalman_filter(tempC)){
 // if(setpoint<tempC){
     analogWrite(R_PWM,255);
     analogWrite(L_PWM,0);
  }
  else{
    analogWrite(R_PWM,0);
    analogWrite(L_PWM,0);
  }
}

void takeReadingTemperature(){
  // Bits to Voltage
  V = (analogRead(Vin)/1023.0)*5.0; // (bits/2^n-1)*Vmax 
  // Voltage to resistance
  Rx = (330*V)/(5-V); // Voltage divider
  // Resistance to Temperature
  tempC= (Rx/R0-1.0)/alpha-5.85; // from Rx = R0(1+alpha*X)
  tempF= tempC*1.8 +32; // celcius to fahrenheight
  //Serial.println(tempC);
}
float kalman_filter (float input)
{ 
  float  kalman_new = kalman_old; // eski değer alınır
  float cov_new = cov_old + 0.2; //yeni kovaryans değeri belirlenir. Q=0.50 alınmıştır
  
  float kalman_gain = cov_new / (cov_new + 0.9); //kalman kazancı hesaplanır. R=0.9 alınmıştır
  float kalman_calculated = kalman_new + (kalman_gain * (input - kalman_new)); //kalman değeri hesaplanır
  
  cov_new = (1 - kalman_gain) * cov_old; //yeni kovaryans değeri hesaplanır
  cov_old = cov_new; //yeni değerler bir sonraki döngüde kullanılmak üzere kaydedilir
  
  kalman_old = kalman_calculated;
 
  return kalman_calculated; //hesaplanan kalman değeri çıktı olarak verilir
}
float kalman_filter2 (float input)
{ 
  float samples[10];
  for(int i=0;i<10;i++){
  float  kalman_new = kalman_old; // saving old value
  float cov_new = cov_old + 0.50; // determination of the new covariance value. Q=0.50 
  
  float kalman_gain = cov_new / (cov_new + 0.90); // calculating kalman gain. R=0.9 
  float kalman_calculated = kalman_new + (kalman_gain * (input - kalman_new)); // calculating kalman value
  
  cov_new = (1 - kalman_gain) * cov_old; // calculating new covariance value
  cov_old = cov_new; // new values are saved for using next calculations
  kalman_old = kalman_calculated;
  //samples[i]=kalman_calculated;
  }
  float result=0;
  for(int i=0;i<10;i++){
    result+= samples[i];
  }
  if( samples[9]!=0 ){
    return result/10; //calculated kalman filter as output
  }
  }
