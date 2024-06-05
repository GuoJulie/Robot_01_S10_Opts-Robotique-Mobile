#include <Servo.h>

#define SERVO_GAUCHE_PIN 12
#define SERVO_DROIT_PIN 13
#define IMP_MAX 1700 //Impulsion maximale recevable par les servos moteurs
#define IMP_ARRET 1500 //Impulsion pour que les servos ne tourne pas
#define IMP_MIN 1300 //Impulsion minimale recevable par les servos moteurs


Servo Servo_droit;
Servo Servo_gauche;

void setup() {
  


  pinMode (5, INPUT);
  pinMode (7, INPUT);
  
  Servo_droit.attach(SERVO_GAUCHE_PIN);
  Servo_gauche.attach(SERVO_DROIT_PIN);

  avancerRobot(1000, 50);
  reculerRobot(2000, 20);
  surPlaceDroite(2000,35);
  surPlaceGauche (2000,50);

  
  stopRobot();
}

void loop() {
  

}


int getImpulsion(double vitesse){
  
  if(vitesse == 0) return 1500; //Evite les erreurs d'arrondi pour la vitesse 0
  
  double x = vitesse;
  int impulsion;
  double w;
  int const nbDeg = 10;
  
  //1498 + -2,7x + 0,0172x^2 + 6,11E-03x^3 + -7,09E-05x^4 + -1,04E-05x^5 + 9,25E-08x^6 + 6,59E-09x^7 + -4,7E-11x^8 + -1,42E-12x^9 + 8,17E-15x^10
  //impulsion = 1498 - 2.7*x + 0.0172*pow(x,2) + 6.11*pow(10,-3)*pow(x,3) - 7.09*pow(10,-5)*pow(x,4) - 1.04*pow(10,-5)*pow(x,5) + 9.25*pow(10,-8)*pow(x,6) + 6.59*pow(10,-9)*pow(x,7) - 4.7*pow(10,-11)*pow(x,8) - 1.42*pow(10,-12)*pow(x,9) + 8.17*pow(10,-15)*pow(x,10);

  //Initialisation du tableau des coefficients
  double coef[nbDeg+1] = {0.00000000000000817, -0.00000000000142, -0.000000000047, 0.00000000659, 0.0000000925, -0.0000104, -0.0000709, 0.00611, 0.0172, - 2.7, 1498};

  w = coef[0]; //Initialisation de w avec le coef de plus grand degré
  for(int i = 1; i < nbDeg+1; i++){
    w = w * x + coef[i];   //Récriture de la variable en suivant la méthode de Horner
  }

  impulsion = w;
  
  //Verification pour ne pas dépasser la capacité maximal des servos moteurs.
  if(impulsion > IMP_MAX) return IMP_MAX;
  if(impulsion < IMP_MIN) return IMP_MIN;
  return impulsion;
}





void surPlaceDroite(int duree_deplacement, int vitesse){
  Servo_gauche.writeMicroseconds (getImpulsion(-vitesse));
  Servo_droit.writeMicroseconds (getImpulsion(-vitesse));
  if(duree_deplacement >= 0){
    delay(duree_deplacement);
  }
}

void surPlaceGauche(int duree_deplacement, int vitesse){
  Servo_droit.writeMicroseconds (getImpulsion(vitesse));
  Servo_gauche.writeMicroseconds (getImpulsion(vitesse));
  if(duree_deplacement >= 0){
    delay(duree_deplacement);
  }
}




void avancerRobot(int duree_deplacement,int vitesse){
  Servo_droit.writeMicroseconds (getImpulsion(-vitesse));
  Servo_gauche.writeMicroseconds (getImpulsion(vitesse));
  if(duree_deplacement >= 0){
    delay(duree_deplacement);
  }
}

void reculerRobot(int duree_deplacement,int vitesse){
  avancerRobot(duree_deplacement, -vitesse);
}



void stopRobot(){
  Servo_droit.detach();
  Servo_gauche.detach();
}
