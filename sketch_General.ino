// Programme General            /*bp 3,2,19,20,21*/
const int pin1 = 10;
const int pin2 = 11;
const int pin3 = 12;
const int M_A = 13;                 // Bouton marche/arrêt
const int BAU = 9;                  // Bouton arrêt d'urgence 
const int CapteurR1=3;              // Depart
int CapteurR1Etat =0; 
const int CapteurR2=19;
int CapteurR2Etat =0;
int Power_Focusk1=4;         //la broche 4 d'arduino est liée au IN1 de shield moteur 
int Power_Focusk2=5;         //la broche 5 d'arduino est liée au IN2 de shield moteur
int Electrovanne=6;
int Shield_Motor_sens1=7;    //pin 7 Arduino vers broche A+ du  L293D
int Shield_Motor_sens2=8;    // pin 8 Arduino vers broche A- du L293D
int Act_Gachette=20;
int compt1=0;                // pour compter nombre des cycles '25'
int n=0;                     //nombre de fois on a la repetition de cette operation 
int compt2=0;                // pour cycle de refroidissement  
int x;


enum TestState { IDLE, ENDURANCE, CAPABILITE, CALIBRATION };
TestState currentState = IDLE;

void setup() {
  pinMode(pin1, INPUT);
  pinMode(pin2, INPUT);
  pinMode(pin3, INPUT);
  pinMode(M_A, INPUT_PULLUP);           // Utilisation de la résistance de tirage interne
  pinMode(BAU, INPUT_PULLUP);           // Utilisation de la résistance de tirage interne
  pinMode(CapteurR1, INPUT_PULLUP);
  pinMode(CapteurR2, INPUT_PULLUP);
  pinMode(Power_Focusk1, OUTPUT);
  pinMode(Power_Focusk2, OUTPUT);
  pinMode(Electrovanne, OUTPUT);
  pinMode(Shield_Motor_sens1, OUTPUT);
  pinMode(Shield_Motor_sens2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(BAU),handleEmergencyStop, FALLING);
  digitalWrite(Power_Focusk1, LOW);
  digitalWrite(Electrovanne, LOW);
  digitalWrite(Power_Focusk2, LOW);
  digitalWrite(Shield_Motor_sens1, LOW); 
  digitalWrite(Shield_Motor_sens2, LOW); 
  pinMode(Act_Gachette, OUTPUT);
  Serial.begin(9600);                      //Affichage
}

void Sens_Moteur(){
           //le moteur tourne dans le sens normal
         digitalWrite(Shield_Motor_sens1, HIGH);         // Activation de la broche A+ du L293D
         digitalWrite(Shield_Motor_sens2, LOW);          // Désactivation de la broche A- du L293D
         delay(3500);  
         digitalWrite(Power_Focusk1, LOW);
         digitalWrite(Power_Focusk2, LOW); 
 
         // Le moteur est à l'arret 
         digitalWrite(Shield_Motor_sens1, LOW);         // Désactivation de la broche A+ du L293D
         digitalWrite(Shield_Motor_sens2, LOW);         // Désactivation de la broche A- du L293D
         delay(2000);                                   // attendre pendant 2 secondes 
        
        while(digitalRead(M_A)==LOW){}                                                    
         // le moteur tourne dans le sens inverse 
         digitalWrite(Shield_Motor_sens1, LOW);        // Désactivation de la broche A+ du L293D
         digitalWrite(Shield_Motor_sens2, HIGH);       // Activation de la broche A- du L293D
         delay(3500);                                  // attendre pendant 3.5 secondes

         //Le moteur est à l'arret
         digitalWrite(Shield_Motor_sens1, LOW);       // Désactivation de la broche A+ du L293D
         digitalWrite(Shield_Motor_sens2, LOW);       // Désactivation de la broche A- du L293D 
         delay(13000);
}

void loop() {
  TestState state = readSelector();
  
  if (state != currentState) {
    currentState = state;
    Serial.print("State changed to: ");
    Serial.println(currentState);
  }

  switch (currentState) {
    case ENDURANCE:
      runEndurance();
      break;
    case CAPABILITE:
      runCapabilite();
      break;
    case CALIBRATION:
      runCalibration();
      break;
    case IDLE:
    default:
      // Do nothing
      break;
  }
}

TestState readSelector() {
  if (digitalRead(pin1) == HIGH) {
    return ENDURANCE;
  } else if (digitalRead(pin2) == HIGH) {
    return CAPABILITE;
  } else if (digitalRead(pin3) == HIGH) {
    return CALIBRATION;
  } else {
    return IDLE;
  }
}

void runEndurance() {
  Serial.println("Running Endurance Test...");
  while (readSelector() == ENDURANCE) {
    CycleEndurance();             
  }
  Serial.println("Endurance Test Interrupted or Completed");
}

void runCapabilite() {
  Serial.println("Running Capabilite Test...");
  while (readSelector() == CAPABILITE) {
    CycleCapabilite(); 
              
  }
  Serial.println("Capabilite Test Interrupted or Completed");
}

void runCalibration() {
  Serial.println("Running Calibration Test...");
  while (readSelector() == CALIBRATION) {
    CycleCalibration();       
  }
  Serial.println("Calibration Test Interrupted or Completed");
}
void handleEmergencyStop() {
    // Arrêt d'urgence détecté, arrêter le test en cours
    Serial.println("Emergency Stop Detected!");
    currentState = IDLE; // Mettre l'état actuel à IDLE pour arrêter le test en cours
    while (digitalRead(BAU) == LOW) {} // Attendre que le bouton d'arrêt d'urgence soit relâché
  
}

void CycleEndurance(){
  CapteurR1Etat =digitalRead(CapteurR1);
    while(digitalRead(M_A)==LOW){} 
    while(CapteurR1Etat==LOW){}         
  if( n<=360 /*nbrDeRepetitionDeCetteOpera 9000cycles=25*360*/ ){           
      while(compt1<26 ){
         while(digitalRead(M_A)==LOW){}                 
         digitalWrite(Power_Focusk1, HIGH);              //Traction       
         delay(7000); 
         digitalWrite(Power_Focusk1, LOW);             
        while(digitalRead(M_A)==LOW){}                                         
         digitalWrite(Electrovanne, HIGH);               //Libération de la pression (ouverture de l'électrovanne).        
         delay(500);                                     
         digitalWrite(Electrovanne, LOW); 
        while(digitalRead(M_A)==LOW){} 
         digitalWrite(Power_Focusk2, HIGH);               //Vissage (fermeture de k2 puis de k1) 
         delay(100);
         digitalWrite(Power_Focusk1, HIGH);
         delay(500);  
         Sens_Moteur();
        compt1++; 
        Serial.print("Le cycle Numéro :");  
        Serial.println(compt1);                         // Affichage du compteur de 25 cycle
    }
    compt1=0;
      // Refroidissement du systeme 
        while(compt2<=9600/*cycleDeRef 20min*60*8*/ && digitalRead(M_A)==HIGH){
         Serial.print("Cycle de Refroidissement:");
         Serial.println(compt2); 
         delay(125);
         compt2++;
 
     } 
   compt2=0; 
   n++;

 }
 else {}
}

void CycleCapabilite(){
  CapteurR1Etat =digitalRead(CapteurR1);
  CapteurR2Etat =digitalRead(CapteurR2);
while(digitalRead(M_A)==LOW){}
while(CapteurR1Etat==LOW){}          
        delay(x);                                   //temps d'appuyer sur la visseuse et de prendre les mesures     traction  
while(digitalRead(M_A)==LOW){}  
while(CapteurR2Etat==LOW){}                                                    
        digitalWrite(Electrovanne, HIGH);           //Libération de la pression (ouverture de l'électrovanne).        
        delay(500);                                 //Attente
        digitalWrite(Electrovanne, LOW); 
while(digitalRead(M_A)==LOW){} 
        delay(x);                                   //temps d'appuyer sur la visseuse et de prendre les mesures   vissage 
        delay(500);  
        Sens_Moteur();
        }

void CycleCalibration(){
    CapteurR1Etat =digitalRead(CapteurR1);
  CapteurR2Etat =digitalRead(CapteurR2);
      while(digitalRead(M_A)==LOW){} 
      while(CapteurR1Etat==LOW){ /*tant que R1 low ne rien faire*/}                                     
         digitalWrite(Power_Focusk1, HIGH);                //Traction       
         delay(7000); 
         digitalWrite(Power_Focusk1, LOW);             
      while(digitalRead(M_A)==LOW){}
      while(CapteurR2Etat==LOW){}                                              
         digitalWrite(Electrovanne, HIGH);              //Libération de la pression (ouverture de l'électrovanne).        
         delay(500);                                    //Attente
         digitalWrite(Electrovanne, LOW); 
      while(digitalRead(M_A)==LOW){}
         digitalWrite(Power_Focusk2, HIGH);            //Vissage (fermeture de k2 puis de k1) 
         delay(100);
         digitalWrite(Power_Focusk1, HIGH);
         delay(500); 
         Sens_Moteur();
}        