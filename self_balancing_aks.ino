#include <Wire.h>
//--- Accelerometer Register Addresses
#define Power_Register 0x2D
#define X_Axis_Register_DATAX0 0x32 // Hexadecimal address for the DATAX0 internal register.
#define X_Axis_Register_DATAX1 0x33 // Hexadecimal address for the DATAX1 internal register.
#define Y_Axis_Register_DATAY0 0x34 
#define Y_Axis_Register_DATAY1 0x35
#define Z_Axis_Register_DATAZ0 0x36
#define Z_Axis_Register_DATAZ1 0x37
//--- Gyro Register Addresses
#define Gyro_gX0 0x28  
#define Gyro_gX1 0x29
#define Gyro_gY0 0x2A
#define Gyro_gY1 0x2B
#define Gyro_gZ0 0x2C  
#define Gyro_gZ1 0x2D
#define rm1 4
#define rm0 5
#define lm1 7
#define lm0 6
#define kp 20
#define ki 0.0
#define kd 15



int X0,X1,X_out;
int Y0,Y1,Y_out;
int Z1,Z0,Z_out;
float Xa,Ya,Za;
int gX0, gX1, gX_out;
int gY0, gY1, gY_out;
int gZ0, gZ1, gZ_out;
float Xg,Yg,Zg;
float angleX,angleY,angleZ,angleXc,angleYc,angleZc;
int ADXAddress = 0x53;  //Accelerometer Device address 
int Gyro = 0x69; //Gyro Device address
unsigned long start, finished, elapsed;
float dt=0.015;
float p,i,d,lerror,errorgyro,mult,error,pidvalue;
void readaccel(){
 Wire.write(X_Axis_Register_DATAX0);
 Wire.write(X_Axis_Register_DATAX1);  
 Wire.endTransmission(); // Ends the transmission and transmits the data from the two registers
 Wire.requestFrom(ADXAddress,2); // Request the transmitted two bytes from the two registers
 if(Wire.available()<=2) {  // 
   X0 = Wire.read(); // Reads the data from the register
   X1 = Wire.read();
   /* Converting the raw data of the X-Axis into X-Axis Acceleration */ 
   X1=X1<<8;
   X_out =X0+X1;
   Xa=X_out/256.0; // Xa = output value from -1 to +1, Gravity acceleration acting on the X-Axis
 }
 // Y-Axis
 Wire.beginTransmission(ADXAddress); 
 Wire.write(Y_Axis_Register_DATAY0);
 Wire.write(Y_Axis_Register_DATAY1);  
 Wire.endTransmission(); 
 Wire.requestFrom(ADXAddress,2);
 if(Wire.available()<=2) { 
   Y0 = Wire.read();
   Y1 = Wire.read();
   Y1=Y1<<8;
   Y_out =Y0+Y1;
   Ya=Y_out/256.0;
 }
 // Z-Axis
 Wire.beginTransmission(ADXAddress); 
 Wire.write(Z_Axis_Register_DATAZ0);
 Wire.write(Z_Axis_Register_DATAZ1);  
 Wire.endTransmission(); 
 Wire.requestFrom(ADXAddress,2);
 if(Wire.available()<=2) { 
   Z0 = Wire.read();
   Z1 = Wire.read();
   Z1=Z1<<8;
   Z_out =Z0+Z1;
   Za=Z_out/256.0;
  }
}  

void readgyros(){
   start=millis();
 //---- X-Axis
 Wire.beginTransmission(Gyro); // transmit to device
 Wire.write(Gyro_gX0);
 Wire.endTransmission();
 Wire.requestFrom(Gyro,1); 
 if(Wire.available()<=1)   
 {
   gX0 = Wire.read();
 }
 Wire.beginTransmission(Gyro); // transmit to device
 Wire.write(Gyro_gX1);
 Wire.endTransmission();
 Wire.requestFrom(Gyro,1); 
 if(Wire.available()<=1)   
 {
   gX1 = Wire.read();
 }
 //---- Y-Axis
 Wire.beginTransmission(Gyro); // transmit to device
 Wire.write(Gyro_gY0);
 Wire.endTransmission();
 Wire.requestFrom(Gyro,1); 
 if(Wire.available()<=1)   
 {
   gY0 = Wire.read();
 }
 Wire.beginTransmission(Gyro); // transmit to device
 Wire.write(Gyro_gY1);
 Wire.endTransmission();
 Wire.requestFrom(Gyro,1); 
 if(Wire.available()<=1)   
 {
   gY1 = Wire.read();
 }
 //---- Z-Axis
 Wire.beginTransmission(Gyro); // transmit to device
 Wire.write(Gyro_gZ0);
 Wire.endTransmission();
 Wire.requestFrom(Gyro,1); 
 if(Wire.available()<=1)   
 {
   gZ0 = Wire.read();
 }
 Wire.beginTransmission(Gyro); // transmit to device
 Wire.write(Gyro_gZ1);
 Wire.endTransmission();
 Wire.requestFrom(Gyro,1); 
 if(Wire.available()<=1)   
 {
   gZ1 = Wire.read();
 }
 //---------- X - Axis
 // Raw Data
 gX1=gX1<<8;
 gX_out =gX0+gX1;
 // From the datasheet: 70 mdps/digit

 //---------- Y - Axis
 gY1=gY1<<8;
 gY_out =gY0+gY1;

 //---------- Z - Axis
 gZ1=gZ1<<8;
 gZ_out =gZ0+gZ1;

 }

void finderror(){
  errorgyro=90-(X_out); 
  mult=errorgyro/**gX_out*/;
  error = mult;
}

void pidcontrol(){
p = error;
d = (lerror-error);
i += (error);
lerror = error;
pidvalue = (kp*p)+(kd*d)+(ki*i);
pidvalue = pidvalue<0?-pidvalue:pidvalue; 

if (X_out>90){
  analogWrite(rm1,int (pidvalue));
  analogWrite(lm1,int (pidvalue));
  analogWrite(rm0,0);
  analogWrite(lm0,0);
  Serial.print("forward");}
  else {
  analogWrite(rm0,int(pidvalue));
  analogWrite(lm0,int(pidvalue));
  analogWrite(rm1,0);
  analogWrite(lm1,0);
  Serial.print("backward");  }
}

void setup() {
Wire.begin(); // Initiate the Wire library    
Serial.begin(9600);    

Wire.beginTransmission(ADXAddress);
Wire.write(Power_Register); // Power_CTL Register
// Enable measurement
Wire.write(8); // Bit D3 High for measuring enable (0000 1000)
Wire.endTransmission();
Wire.beginTransmission(Gyro);
Wire.write(0x20); // CTRL_REG1 - Power Mode
Wire.write(15);   // Normal mode: 15d - 00001111b   
Wire.endTransmission();
Wire.beginTransmission(Gyro);
Wire.write(0x23); // CTRL_REG4 - Sensitivity, Scale Selection
Wire.write(48);   // 2000dps: 48d - 00110000b
Wire.endTransmission();

}
void loop() {
 // X-axis
 Wire.beginTransmission(ADXAddress); // Begin transmission to the Sensor 
 //Ask the particular registers for data
readaccel();
readgyros();
 X_out=map(X_out,-260,250,0,185);
finderror();

 Serial.print("angle error= ");
 Serial.print(error);
Serial.print("    ");
/* Serial.print("gX= ");
 Serial.print(gX_out);*/
 
Serial.print(d, "    ");

 //delay(50);

 pidcontrol();
 Serial.println(pidvalue);
 
}
