#include <Wire.h>
#include <Encoder.h>
#include <Adafruit_MPL3115A2.h>
#define DIR 30
#define EN  11

Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2(); //Objet capteur

double theta; //Position angulaire du moteur
double thetaref = 0.; //Commande de position
double commande; //Commmande de vitesse envoyée au moteur

Encoder myEnc(3, 2); //Objet encodeur

//Ces variables de temps nous serviront pour le PID
unsigned long tim; 
unsigned long periode;
unsigned long duree = 0;

boolean dotransmit; //Permet d'autoriser ou non la transmission de données

static double Kp = 2.; // gain proportionnel du PID
static double Ki = 65; // gain intégral du PID
static double Kd = 0.05; // gain dérivé du PID

double error; //Erreur par rapport à la commande
double lastError = 0.;

void setup() {
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");
  pinMode(DIR, OUTPUT); // Pin configuration
  digitalWrite(DIR, LOW);
  pinMode(EN, OUTPUT);
  digitalWrite(EN, LOW);
  cli(); // Désactive l'interruption globale
  bitClear (TCCR2A, WGM20); // WGM20 = 0
  bitClear (TCCR2A, WGM21); // WGM21 = 0 
  TCCR2B = 0b00000110; // Clock / 256 soit 16 µs et WGM22 = 0
  TIMSK2 = 0b00000001; // Interruption locale autorisée par TOIE2
  sei(); // Active l'interruption globale

  dotransmit = 0;
}

long oldPosition  = -999;

int varCompteur = 0; // La variable compteur
 
// Routine d'interruption
ISR(TIMER2_OVF_vect) 
{
  TCNT2 = 256 - 250; // 250 x 16 µs = 4 ms
  // 500 * 4 ms = 2000 ms -> 1 mesure toutes les 2 secondes
  if (varCompteur++ > 400) 
  { 
    varCompteur = 0;
    
    // Autorisation mesure température
    dotransmit = 1; 
  }
}

boolean backtracking = false; //Permet de revenir à la position initiale
//Utile uniquement pour la démonstration

void loop() {
  //Permet de démarrer la trasmission et de vérifier qu'elle est possible
  if (!baro.begin()) {
    Serial.println("Couldn't find sensor");
    return;
  }

  
  long newPosition = myEnc.read(); //La nouvelle position du moteur

  oldPosition = newPosition;

  theta = (double)newPosition/(53*6*2)*360; //Traduction des impulsions en position angulaire
  Serial.print(theta); Serial.println(" theta"); 
  if (dotransmit){
    float tempC = baro.getTemperature();
    Serial.print(tempC); Serial.println(" tempC");
    if (!backtracking) thetaref = 6*tempC; //Génération de la position voulue
    Serial.print(thetaref); Serial.println(" thetaref");
   
    dotransmit = 0;
  }
  periode = millis()-tim;
  tim = millis();
  duree = duree + periode;
  if (duree > 5*1000) {
    backtracking = true;
    thetaref = 0.;
  }
  commande = PID(theta,thetaref); //Génération de la commande avec le PID
  //Permet d'arrêter le moteur lorsqu'on est assez proche de la position voulue
  if (abs(theta - thetaref) < 0.1 || abs(commande) < 40) {
    commande = 0;
  }
  commandeMoteur(commande); //Envoie de la commande au moteur
}

double PID(double theta, double thetaref){
  //Simple PID générant la commande
  error = thetaref-theta;

  double intError = error*periode/1000;
  double derError = (error - lastError)/periode*1000;

  lastError = error;
  return Kp*error+Ki*intError+Kd*derError; 
}

void commandeMoteur(double commande){
  if (commande >= 0){
    digitalWrite(DIR, LOW);
    analogWrite(EN, min(commande,50));
  } else {
    digitalWrite(DIR,HIGH);
    analogWrite(EN, max(-50, commande));
  }
}
