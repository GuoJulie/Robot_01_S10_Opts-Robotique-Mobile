#include <Servo.h>

#define SERVO_GAUCHE_PIN 12
#define SERVO_DROIT_PIN 13
#define T1 1300
#define T2 1500
#define T3 1700


Servo Servo_droit;
Servo Servo_gauche;

void setup() {
  


  pinMode (5, INPUT);
  pinMode (7, INPUT);
  
  Servo_droit.attach(SERVO_GAUCHE_PIN);
  Servo_gauche.attach(SERVO_DROIT_PIN);

  avancerRobot(1000);
  surPlaceDroite(1000);
  surPlaceGauche(1000);
  reculerRobot(1000);
  
  stopRobot();
}

void loop() {
  

}



void surPlaceDroite(int duree_deplacement){
  Servo_gauche.writeMicroseconds (T3);
  Servo_droit.writeMicroseconds (T3);
  if(duree_deplacement >= 0){
    delay(duree_deplacement);
  }
}

void surPlaceGauche(int duree_deplacement){
  Servo_droit.writeMicroseconds (T1);
  Servo_gauche.writeMicroseconds (T1);
  if(duree_deplacement >= 0){
    delay(duree_deplacement);
  }
}




void avancerRobot(int duree_deplacement){
  Servo_droit.writeMicroseconds (T3);
  Servo_gauche.writeMicroseconds (T1);
  if(duree_deplacement >= 0){
    delay(duree_deplacement);
  }
}

void reculerRobot(int duree_deplacement){
  Servo_droit.writeMicroseconds (T1);
  Servo_gauche.writeMicroseconds (T3);
  if(duree_deplacement >= 0){
    delay(duree_deplacement);
  }
}



void stopRobot(){
  Servo_droit.detach();
  Servo_gauche.detach();
}
