#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "SimpleTimer.h"
#include "Servo.h"
#include <NewPing.h>
Servo drapeau;
Servo pincePin;
NewPing sonar(12,14,200);


#define PWM_droite       D2  //14
#define sens_droite             D4  //12
#define PWM_gauche     D1   //13
#define sens_gauche           D3 //15
#define MAX_COMMAND 1023
#define TRIGGER_PIN  12
#define ECHO_PIN     11
#define MAX_DISTANCE 100

#define FPS_CIBLE 20

SimpleTimer timer;

/* 
 Command Reminder INDEX : 
 |    MOUVEMENT         |     Parametre
 -------------------------------------------  
 0 - NO VALUE           | 0
 1 - Turn Right         | 70 - 100
 2 - Turn Left          | 70 - 100
 3 - avancer Right      | 0 - 100
 4 - avancer Left       | 0 - 100
 5 - getDistance        | 0
 7 - State Fin          | 0
 */





//UTILE EN FAIT
int lectureVS;
int mouvement;
float parametre;
int diffpuiss;
int pos = 0;    // variable to store the servo position
int distance=1200;
int etat_pince=1;
int postur;
int delayTime;

#define WIFI_SSID " "
#define WIFI_PASS " "
#define UDP_PORT 4210

int directionMotorRight;
int directionMotorLeft;

IPAddress ipMaster;
int portMaster;

// UDP
WiFiUDP UDP;
int packet;

void setup() {
  //Blynk.begin(outh, ssid, pass);
  Serial.begin(9600);
  pinMode(PWM_droite, OUTPUT);
  pinMode(PWM_gauche, OUTPUT); 
  pinMode(sens_droite, OUTPUT);
  pinMode(sens_gauche, OUTPUT);
  pincePin.attach(5);
  
  digitalWrite(sens_droite, directionMotorRight);
  digitalWrite(sens_gauche, directionMotorLeft);
  drapeau.attach(12);
  drapeau.write(0);
  
  // Begin WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  // Connecting to WiFi...
  Serial.print("Connecting to ");
  Serial.print(WIFI_SSID);
  // Loop continuously while WiFi is not connected
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    Serial.print(".");
  }
  
  // Connected to WiFi
  Serial.println();
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());
 
  // Begin listening to UDP port
  UDP.begin(UDP_PORT);
  Serial.print("Listening on UDP port ");
  Serial.println(UDP_PORT);
    
}

void loop() {
  postur=0;
  parametre = 0;
  mouvement = 0;
  char packet[2]; 
  bool packetSize = false, prevPacketSize= false;
  while(1) {
      packetSize = (UDP.parsePacket()>0);
      if (packetSize) {
        prevPacketSize = true;
        UDP.read(packet, 39);
        
      }
      else if(prevPacketSize && !packetSize) {
        prevPacketSize = false;
        parametre = (int)packet[1];
        mouvement = (int) packet[0];  
        break;
      }
   }
  
  if(mouvement == 1 || mouvement == 2) tourner();
  if(mouvement == 3) avancer_correction_droite();
  if(mouvement == 4) avancer_correction_gauche();
  if(mouvement == 5) {
   digitalWrite(sens_droite, directionMotorRight);
   digitalWrite(sens_gauche, directionMotorLeft); 
   analogWrite(PWM_droite,(int)MAX_COMMAND*0.8); 
   analogWrite(PWM_gauche,(int)MAX_COMMAND*0.8);
    delay(150);
  }
  if (mouvement==7 ) {
    drapeau.write(180); //FIN
    digitalWrite(PWM_droite,LOW); 
    digitalWrite(PWM_gauche, LOW);
  } 
  if(lectureVS == 0) {
      delayTime = 0;
      digitalWrite(PWM_droite,LOW); 
      digitalWrite(PWM_gauche, LOW);
  }
  pince(1);
  


  
}

void avancer_correction_gauche() {
  parametre = (parametre)/100;
  directionMotorRight = 0;
  directionMotorLeft = 1;
  digitalWrite(sens_droite, directionMotorRight);
  digitalWrite(sens_gauche, directionMotorLeft); 
  diffpuiss=parametre*MAX_COMMAND;
  analogWrite(PWM_gauche,MAX_COMMAND); 
  analogWrite(PWM_droite,diffpuiss);

  Serial.print("moteur droite :");
  Serial.println(diffpuiss);
  Serial.print("moteur gauche :");
  Serial.println(MAX_COMMAND);
    
  delay(50);
}

void avancer_correction_droite() {
  parametre = parametre/100;
  directionMotorRight = 0;
  directionMotorLeft = 1;
  digitalWrite(sens_droite, directionMotorRight);
  digitalWrite(sens_gauche, directionMotorLeft); 
  diffpuiss=parametre*MAX_COMMAND;
  analogWrite(PWM_droite,MAX_COMMAND); 
  analogWrite(PWM_gauche,diffpuiss);

  Serial.print("moteur droite :");
  Serial.println(MAX_COMMAND);
  Serial.print("moteur gauche :");
  Serial.println(diffpuiss);
  
  delay(50);
}

void tourner() {
  if(mouvement == 1) { //DROITE
    directionMotorRight = 1;
    directionMotorLeft = 1;
    digitalWrite(sens_droite, HIGH);
    digitalWrite(sens_gauche, HIGH); 

    Serial.print("direction : ");
    Serial.println("droite");
  }
  else if(mouvement == 2){ //Gauche
    directionMotorRight = 0;
    directionMotorLeft = 0;
    digitalWrite(sens_droite, LOW);
    digitalWrite(sens_gauche, LOW); 
    Serial.print("direction : ");
    Serial.println("gauche");
  }


  Serial.print("puissance :");
  Serial.println(parametre);
  
  analogWrite(PWM_droite,MAX_COMMAND); 
  analogWrite(PWM_gauche,MAX_COMMAND);
  delay(60);
  digitalWrite(PWM_droite,0); 
  digitalWrite(PWM_gauche, 0);
  delay(parametre+10);
}

void pince(int leve){
  distance = sonar.ping_cm();
  //Serial.println(distance);
   if (distance <= 0.1  && etat_pince==1)// si pince haute et proche de cube on descend la pince
  {
     for (pos = 0; pos <= 90; pos += 1) 
       { 
        pincePin.write(pos);   
        delay(10);        
       }
        etat_pince=0;
  }
  if (leve==2)// if (on est arrivé)
  {
  }
}
