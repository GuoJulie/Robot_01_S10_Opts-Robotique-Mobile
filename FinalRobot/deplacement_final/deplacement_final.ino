#include <Servo.h>

#define SERVO_GAUCHE_PIN 12
#define SERVO_DROIT_PIN 13

#define M_GAUCHE 5
#define M_DROITE 7
#define PH_GAUCHE 8
#define PH_DROITE 6
  
#define PHOTO_TRANSISTOR 3

#define L 5.6 //Demi largeur du robot
#define VITESSE_MAX 50
#define IMP_MAX 1700 //Impulsion maximale recevable par les servos moteurs
#define IMP_ARRET 1500 //Impulsion pour que les servos ne tourne pas
#define IMP_MIN 1300 //Impulsion minimale recevable par les servos moteurs

Servo Servo_droit;
Servo Servo_gauche;
unsigned long lum_gauche;
unsigned long lum_droite;
unsigned long lum_diff;





void setup() {
  Serial.begin(9600);
  
  pinMode(M_GAUCHE, INPUT);
  pinMode(M_DROITE, INPUT);
  

  Servo_droit.attach(SERVO_GAUCHE_PIN);
  Servo_gauche.attach(SERVO_DROIT_PIN);

  initDirection();


  

}

void loop() {
  //Récupération des valeurs fournis par les phototransistors
  lum_gauche = rcTime(PH_GAUCHE);
  lum_droite = rcTime(PH_DROITE);
  lum_diff = lum_gauche - lum_droite;
    
   
   //Control du cote le plus lumineux et avancer en fonction
   if(lum_diff < ((lum_gauche + lum_droite)/2)*0.5){
    avancerRobot(60);
  }else if(lum_gauche < lum_droite){
    gauche(60,20,VITESSE_MAX); //Temps,Rayon de courbure,Vitesse
  }else{
    droite(60,20,VITESSE_MAX); //Temps,Rayon de courbure,Vitesse
  }
  
  controlMoustache();

}

void initDirection(){

  long minLumValue = 999999999;
  int nbRota = 0;

  int timeFullTour = 2200; //valeur de temps pour un tour
  int nbEtape = 8; //Nombre d'etape d'analyse voulu pendant le tour
  

  for(int i = 0 ; i < nbEtape; i++){
    
    long g = rcTime(PH_GAUCHE); //Lecture de la luminosite du capteur gauche
    long d = rcTime(PH_DROITE); //Lecture de la luminosite du capteur droit
    if((g+d)/2 < minLumValue){ 
      minLumValue = (g+d)/2;
      nbRota = i; //Retient la direction la plus eclairé
    }
    
    surPlaceDroite(timeFullTour/nbEtape,VITESSE_MAX); //Tourne de nb_etape
    
  } 
  Servo_droit.writeMicroseconds (IMP_ARRET);
  Servo_gauche.writeMicroseconds (IMP_ARRET);
  delay(500); 
  surPlaceDroite((timeFullTour/nbEtape)*(nbRota),VITESSE_MAX); //Se place dans la direction la plus eclairé
  
}

//Effectue le processus de control des deux monstaches et replace le robot en fonction
void controlMoustache(){  
  //Calcul aléatoire de temps pour les futurs déplacements
  int randomBackTime = rand()%(1000 - 500) + 500;
  int randomTurnTime = rand()%(500 - 200) + 200;
  
  if(digitalRead(M_GAUCHE) == LOW && digitalRead(M_DROITE) == LOW){ //DEMI-TOUR
    reculerRobot(randomBackTime);
    surPlaceDroite(1100,VITESSE_MAX);
  }else if(digitalRead(M_GAUCHE) == LOW){ //Choc à gauche deplacement vers la droite
    reculerRobot(randomBackTime);
    surPlaceDroite(randomTurnTime, VITESSE_MAX);
  }else if(digitalRead(M_DROITE) == LOW){ //Choc à droite deplacement vers la gauche
    reculerRobot(randomBackTime);
    surPlaceGauche(randomTurnTime, VITESSE_MAX);
  }
  
}


//Obtention du niveau de luminosite
long rcTime(int pin){
  unsigned long int time;

  //Charge du condensateur
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delay(1);

  pinMode(pin, INPUT);
  digitalWrite(pin, LOW);

  //Decharge du condensateur et mesure du temps
  time = micros();
  while(digitalRead(pin));
  time = micros() - time;

  return time;
}

//Retourne l'impulsion à envoyer au servo moteur en fonction de la vitesse désirée
int getImpulsion(double vitesse){
  
  if(vitesse == 0) return 1500; //Evite les erreurs d'arrondi pour la vitesse 0
  
  double x = vitesse;
  int impulsion;
  double w;
  int const nbDeg = 10;
  
  //1498 + -2,7x + 0,0172x^2 + 6,11E-03x^3 + -7,09E-05x^4 + -1,04E-05x^5 + 9,25E-08x^6 + 6,59E-09x^7 + -4,7E-11x^8 + -1,42E-12x^9 + 8,17E-15x^10
  //impulsion = 1498 - 2.7*x + 0.0172*pow(x,2) + 6.11*pow(10,-3)*pow(x,3) - 7.09*pow(10,-5)*pow(x,4) - 1.04*pow(10,-5)*pow(x,5) + 9.25*pow(10,-8)*pow(x,6) + 6.59*pow(10,-9)*pow(x,7) - 4.7*pow(10,-11)*pow(x,8) - 1.42*pow(10,-12)*pow(x,9) + 8.17*pow(10,-15)*pow(x,10);



  
  //double coef[nbDeg+1] = {8.17*pow(10,-15), -1.42*pow(10,-12), - 4.7*pow(10,-11), 6.59*pow(10,-9), 9.25*pow(10,-8), -0.0000104, -0.0000709, 0.00611, 0.0172, - 2.7, 1498};
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


//Calcul la vitesse à fournir à une roue en fonction du rayon de courbure voulu et d'une vitesse pour l'autre roue
double vitesseSelonRC(double P, int vitesseMax){
  if(P < 0) return -vitesseMax; //UTILE ???
  double vitesseMin;  
  vitesseMin = (vitesseMax*(P/L-1))/(P/L+1); //L (constante) est la demi largeur du robot
  return vitesseMin;
}



void gauche(int duree_deplacement, int RC, int vitesse){
  double v = vitesseSelonRC(RC,vitesse);
  int imp = getImpulsion(-v);
  Servo_gauche.writeMicroseconds(imp);
  Servo_droit.writeMicroseconds(getImpulsion(vitesse));
  if(duree_deplacement >= 0){
    delay(duree_deplacement);
  }
}


void droite(int duree_deplacement, int RC, int vitesse){
  double v = vitesseSelonRC(RC,vitesse);
  int imp = getImpulsion(v);
  Servo_droit.writeMicroseconds(imp);
  Servo_gauche.writeMicroseconds(getImpulsion(-vitesse));
  if(duree_deplacement >= 0){
    delay(duree_deplacement);
  }
}




void surPlaceDroite(int duree_deplacement,int vitesse){
  droite(duree_deplacement, 0, vitesse);
}

void surPlaceGauche(int duree_deplacement,int vitesse){
  gauche(duree_deplacement, 0, vitesse);
}




void avancerRobot(int duree_deplacement){
  mouvementLineaireSelonVitesse(duree_deplacement, VITESSE_MAX);
}

void mouvementLineaireSelonVitesse(int duree_deplacement, int vitesse){
  Servo_droit.writeMicroseconds (getImpulsion(vitesse));
  Servo_gauche.writeMicroseconds (getImpulsion(-vitesse));
  if(duree_deplacement >= 0){
    delay(duree_deplacement);
  }
}

void reculerRobot(int duree_deplacement){
  mouvementLineaireSelonVitesse(duree_deplacement, -VITESSE_MAX);
}



void stopRobot(){
  Servo_droit.detach();
  Servo_gauche.detach();
}
