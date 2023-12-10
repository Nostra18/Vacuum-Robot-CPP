#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <math.h>
#include <SimpleTimer.h>

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
SimpleTimer timer;              // Timer pour échantillonnage
SimpleTimer timerEncoder;       // timer pour les moteurs (encodeurs)
SimpleTimer timerDistance;
//--------------------------MODE AUTOMATIQUE-----------------------------
bool automatique = true;  // Si a false alors le mode manuel est utilisE
int pause = 0;
//-------------------------Module bluetooth-----------------------------------
char junk;
String inputString="";
//----------------------------------Les broches pour le moteur
int ENA = 10;
int ENB = 9;
int i = 0;
#define INA A0
#define INB A1
#define INC A2
#define IND A3
// ---------------------------------Mesure de la distance--------------------------------
#define trigAvant 12
#define echoAvant 13
#define trigEst 2
#define echoEst 3
#define trigWest 4
#define echoWest 5
float front_distance = 0;
float west_distance = 0;
float est_distance = 0;
// -----------------------------------Ventilateur--------------------------------------- 
const int ventilo = 7;
//----------------------------------Pour le MPU6050-----------------------------------------
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
//-----------------------------------PID-------------------------------------
const int frequence_asservissement = 200;  // Fréquence de correction(PID) en HZ 
const int frequence_odometrie = 50;
float kp = 20;//100;           // Coefficient proportionnel
float ki = 0.00;           // Coefficient intégrateur
float kd = 12;  
float targetAngle = 0;         // Coefficient dérivateur
float erreur_precedente = targetAngle;
float somme_erreur = 0;   // Somme des erreurs pour l'intégrateur
int cmd = 0;  // La commande 
//------------------------------------ODOMETRIE----------------------------------
const int vitesse = 52;
int PWM_d;
int PWM_g;
float angle, angle_radian; // Le lacet --yaw (depend de la disposition du MPU6050)
const int maxPWM = 100; // valeur max(PWM) a appliquer au moteur
const int minPWM = 70; // valeur min(PWM) a appliquer au moteur
float x=0, y=0, theta=0; // les variables d'etat du robot
float old_x = 0;
float old_y = 0;
int  compteur = 2; // 
float currentTime =0, previousTime=0;
//------------------------------------NAVIGATION-----------------------------------
int tour_avant = 0;
int tour_gauche = 0;
int tour_droite = 0;
int delta_tour = 5; // une valeur juste pour initialisation
float goal; 
char last_turn = 'l'; // Left and Right
bool is_obstacle = false;
//-----------------------------------INITIALISATION-------------------------------
void setup() {
  // put your setup code here, to run once:
  // Code pour configuration et calibrage du MPU6050 ligne 60 - 104
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
  #endif
  Serial.begin(9600);
  while (!Serial);
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
  //----------------------MOTEUR---------------------------
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(INC, OUTPUT);
  pinMode(IND, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  //-----------------------DISTANCE-------------------------
  pinMode(trigAvant, OUTPUT);//----avant 
  pinMode(echoAvant, INPUT);
  pinMode(trigEst, OUTPUT);//-----est
  pinMode(echoEst, INPUT);
  pinMode(trigWest, OUTPUT);//----ouest
  pinMode(echoWest, INPUT);
  pinMode(ventilo , OUTPUT);
  //-------------------------TIMERS---------------------------
  timer.setInterval(1000/frequence_asservissement, asservissement); // apres chaque periode 1000/freq. on appelle la fonction asservissement
  timerEncoder.setInterval(1000/frequence_odometrie, odometrie); // frequence Odometrie
  
}

//------------------------------------------BOUCLE PRINCIPAL-----------------------------
void loop() {
  // put your main code here, to run repeatedly:
  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //Serial.print("ypr:\t"); //ypr
            //Serial.println(ypr[0] * 180/M_PI);
            /*Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);*/
    #endif
  }
  angle_radian = ypr[0];
  angle = angle_radian * 180/M_PI; // ici l'angle en degre
  digitalWrite(ventilo, HIGH);

//-----------------------Bluetooth-------------------
    if(Serial.available()){
  while(Serial.available())
    {
      char inChar = (char)Serial.read(); //read the input
      inputString += inChar;        //make a string of the characters coming on serial
    }
    Serial.println(inputString);
    while (Serial.available() > 0)  
    { junk = Serial.read() ; }      // clear the serial buffer
    if(inputString == "0"){         //in case of 'a' turn the LED on
      automatique = true;
    }else if(inputString == "1"){   //incase of 'b' turn the LED off
      automatique = false;
    }
  }

  if (inputString=="P"){
    pause += 1;
    if (pause==2){
      pause = 0;
    }
  }

  if(automatique) {
    timer.run();
  //-----------------Distance-----------------------
  if (pause) {
    stop_motors();
  }
  else {
  front_distance = distance_avant();
  if ((front_distance < 25) && (compteur == 2)){
    timer.disable(0);
    //timerEncoder.disable(0);
    Serial.print("Petit\t");Serial.println(front_distance);
    stop_motors();
    compteur = 0;
    currentTime = millis();
  }
  if ((millis()-currentTime > 2500) && (compteur == 0)){
    compteur = 2;
    est_distance = distance_est();
    west_distance = distance_west();
    if (est_distance > 40) {
      turn_est();
    }
    else if (west_distance > 40) {
      turn_west();
    }

    else {
      while((west_distance < 40) && (est_distance < 40))
      backward();
    }
    
    timer.enable(0);
  }
  
  
  //if (front_distance )
  //-----Run des timers-------------------
  //timer.run();
  }
  }

  else{   
    if (pause) {
      stop_motors();
    }
    // Commands from the mobile application to the robot. 
    // G: Gauche(left), D:Droite (right), A:avancer(move forward), R:reculer(move back)
    else {
    stop_motors();
      if(inputString == "G") {
      run_motor_d(100);
      sens_oppose_g(100);
      delay(500);
    } 
    else if (inputString == "D") {
      run_motor_g(100);
      sens_oppose_d(100);
      delay(350);
    } 
    else if (inputString == "A") {
      run_motor_d(85);
      run_motor_g(85);
      delay(350);
    } 
    else if (inputString == "R") {
      sens_oppose_g(75);
      sens_oppose_d(75);
      delay(500);
    } 
      
    }
  }
  inputString = "";
}
//----------------------------------------FIN BOUCLE PRINCIPAL-------------------------------

// Retour en arriere (move back)

void backward(){
  sens_oppose_g(75);
  sens_oppose_d(75);
  delay(500);
  stop_motors();
  delay(1000);

}

//---------------------------------Tourner a l'est-------------------------
// Turn east
void turn_est(){
  run_motor_g(100);
  sens_oppose_d(100);
  delay(550);
  stop_motors();
  delay(1000);
  

  // INITIALISATION & OFFSET
  devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
       Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}
//---------------------------------Turn west---------------------------
void turn_west(){
  run_motor_g(100);
  sens_oppose_d(100);
  
  delay(550);
  stop_motors();
  delay(1000);

  // INITIALISATION & OFFSET
  devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
       Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}
// -----------------------ASSERVISSEMENT------------------------
void asservissement (){
  int mySpeed = 85;
  float erreur;
  
  erreur = angle-targetAngle;
  
  somme_erreur += erreur; // Pour integrale
  float delta_erreur = erreur-erreur_precedente; // Pour derivation
  erreur_precedente = erreur;
  cmd = kp*erreur + ki*somme_erreur + kd*delta_erreur;

  PWM_d = mySpeed + cmd; // Due au fait que sur le robot la roue gauche est a droite
  PWM_g = mySpeed - cmd; 
  
  if (PWM_d > maxPWM)
  {
    PWM_d = maxPWM;
  }
  else if (PWM_d < minPWM)
  {
    PWM_d = minPWM;
  }
  if (PWM_g > maxPWM)
  {
    PWM_g = maxPWM;
  }
  else if (PWM_g < minPWM)
  {
    PWM_g = minPWM;
  }
  Serial.print("Gauche:\t"); Serial.println(PWM_g);
  Serial.print("Droite:\t"); Serial.println(PWM_d);
  run_motor_g (PWM_g);
  run_motor_d (PWM_d); // rightSpeedVal
}
// -----------------------ODOMETRIE--------------------------
void odometrie () {

  float deltaTime = currentTime-previousTime;  
  x = old_x + vitesse * cos(angle_radian) * (double)(1.0/frequence_odometrie);                                                                         // posición del punto X actual
  y = old_y + vitesse * sin(angle_radian) * (double)(1.0/frequence_odometrie);    
  old_x = x;
  old_y = y;
}
//----------------------------west motor----------------------
void run_motor_d (int speed) {
  digitalWrite(INA, 0);
  digitalWrite(INB, 1);
  analogWrite(ENA, speed);
}

void sens_oppose_d(int speed) {
  digitalWrite(INA, 1);
  digitalWrite(INB, 0);
  analogWrite(ENA, speed);
}

void stop_motor_d() {
  digitalWrite(INA, 0);
  digitalWrite(INB, 0);
  analogWrite(ENA, 0);
}
//-------------------------------East motor----------------------
void run_motor_g (int speed) {
    digitalWrite(INC, 1);
    digitalWrite(IND, 0);
    analogWrite(ENB, speed);
}

void sens_oppose_g(int speed) {
  digitalWrite(INC, 0);
  digitalWrite(IND, 1);
  analogWrite(ENB, speed);
}

void stop_motor_g() {
  digitalWrite(INC, 0);
  digitalWrite(IND, 0);
  analogWrite(ENB, 0);
}

void stop_motors(){
  analogWrite(ENB, 0);
  delay(4);
  analogWrite(ENA, 0);
  digitalWrite(INA, 0);
  digitalWrite(INB, 0);
  digitalWrite(INC, 0);
  digitalWrite(IND, 0);
}
//-------------------------------------Reading YAW angle----------------------
float read_yaw() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
          #ifdef OUTPUT_READABLE_YAWPITCHROLL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          #endif
        }
  return ypr[0] * 180/M_PI;
}

float distance_avant (){
  float duration, distance ;
  digitalWrite(trigAvant, LOW);
  delayMicroseconds(2);
  digitalWrite(trigAvant, HIGH);
  delayMicroseconds(8);
  digitalWrite(trigAvant, LOW);
  duration = pulseIn(echoAvant, HIGH);
  distance = duration/58.2;// conversion du temps en cm
  delay(5);
  return distance;
}

float distance_est (){
  float duration, distance ;
  digitalWrite(trigEst, LOW);
  delayMicroseconds(2);
  digitalWrite(trigEst, HIGH);
  delayMicroseconds(8);
  digitalWrite(trigEst, LOW);
  duration = pulseIn(echoEst, HIGH);
  distance = duration/58.2;// conversion du temps en cm
  delay(5);
  return distance;
}

float distance_west (){
  float duration, distance ;
  digitalWrite(trigWest, LOW);
  delayMicroseconds(2);
  digitalWrite(trigWest, HIGH);
  delayMicroseconds(8);
  digitalWrite(trigWest, LOW);
  duration = pulseIn(echoWest, HIGH);
  distance = duration/58.2;// conversion du temps en cm
  delay(5);
  return distance;
}

