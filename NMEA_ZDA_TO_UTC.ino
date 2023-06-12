// GNZDA_to_UTC, Transforme le message NMEA "$GNZDA,Time,Date" en message "$UTC,Date,Timee" pour simuler ce message non standard NMEA d'Applanix.
//
// Auteur : Bruno Dionne 2023-06-04
//
// Suggestion d'interconnexion *****
//
// U-Blox ZED-F9R <--uart1--> Artemis Thing PLus   ------------------> DataLogger.
// U-Blox ZED-F9R <--QWIIC--> Artemis Thing PLus   ------------------> DataLogger.
// U-Blox ZED-F9R <- uart2--> Serial to Bluetooh   ------------------> Ordinateur, Tablette ou Mobile
//
// TX UART1       ----> RX Serial1, TX Serial1  ------------------> RX DataLogger.
//
// TX/RX UART2    <---> Module Bluetooth 4.0 SPP     <---> SPP Téléphone/Tablette.
// i2c QWIIC      <---> i2c QWIIC Artemis thing plus <---> Librairie "SparkFun_u-blox_GNSS_v3"
//
// Artemis thing plus particulatités ***** 
//
// Serial   est le port série UART via le port USB-C.
// Serial1  est l'autre port série UART via les broches TX et RX.
// Module Bluetooth BLE intégré pour envoyer ou recevoir des données de surveillance, des commandes de contrôle ou autres à une tablette ou un téléphone intelligent.
// Ne pas connecter ni la broche 3.3V ni la broche GND du connecteur QWIIC i2c au ZED-F9R ou à n'importe quoi connecté au ZED-F9R. Cela cause des délais inexplicables dans le traitmenet des données UART.
//
// Fonctionnement général :
//     Lire les messages NMEA du GPS qui arrivent en bloc juste après le signal PPS associé.
//     On note le temps d'arrivée du premier caractère '$' reçus pour estimé au mieux le déalais avec le signal PPS.
//     Chaque caratère reçu est immédiatement renvoyé sans traitement vers le data loggeur. 
//     Le message NMEA en cours de réception est stocké en mémoire temporairement pour traitement futur.
//     Le dernier message reçu est $GNZDA, avec l'heure et la date UTC.
//     On note le temps d'arrivée du dernier caractère reçu pour estime le temps de transit du bloc de message NMEA.
//     À partir du message $GNZDA stocké en mémoire, on contruit le nouveau message $UTC,Date,Heure au format Applanix.
//     On incrémente l'heure UTC du temps de transit du bloc de messages et délais PPS/Bloc messages NMEA, pour estimé au plus proche (± 0.5 milliseconde) l'heure UTC réelle de la transmission du message $UTC
//        Il est possible dans le code d'ajouter un offset supplémentaire à l'heure UTC, pour tenir compte du temps de traitement du message UTC par le Data Loggeur.
//
//     Un bloc de message NMEA se traite en 45 millisecondes en moyenne, il est donc possible de supporter le refresh maximal de 10 Hz du ZED-F9R.
//     Pour un total de 450 millesecondes de traitement entre chaque PPS.
//     Ce qui laisse un marge de manoeuvre pour ajouter d'autres fonctions personnalisées.
//
// Développement furtur :
//     Synchroniser l'horloge en temp réel RTC du Artemis thing plus avec le temps UTC aux fins de validations et vérification (GPS time anti spoofing).
//     Ajouter une fonction au bouton usager du Artemis Thing Plus pour des fonctionnalités supplémentaires. 
//     Ajouter une interruption pour capturer des alarmes de l'horloge en temps réeal RTC du Artémis Thing PLus pour des fonctionnalités supplémentaires.
//     Ajouter des sondes externes pour supporter plus de type de messages NMEA.
//     
//
// ***** Include  *****
//
//
#include <Arduino.h>          // Librairies, variables et constantes standards Arduino.
#include <SPI.h>              // used for SPI communication.
#include <Wire.h>             // Used for serial communication over USB //Needed for I2C to GNSS.
//#include "RTC.h"            // Real time clock. The RTC library included with the Arduino_Apollo3 core. Pas utilisé pour le moment.
#include "WDT.h"              // Watchdog library
//#include <ArduinoBLE.h>     // Module intégré Bluetooth BLE du Artemis thing plus. Pas utiliser pour le moment.
//
//
// ***** U-blox Sparkfun librairie en option *****
//
//
//#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
//SFE_UBLOX_GNSS myGNSS;
//
//#define myWire Wire1 // Connect using the Wire1 port. Change this if required
//#define gnssAddress 0x42 // The default I2C address for u-blox modules is 0x42. Change this if required
//
//
// ***** Define *****
//
//
// ***** Constant *****
//
//
// Habituellement les SBC Arduino définissent 'LED_BUILTIN'
// Ce symbole est associé à la broche pin_size_t qui est la LED embarquée.
//
const pin_size_t ledPinNumber = LED_BUILTIN;  //LED par son numéro de broche
//
// Mbed boards use a PinName called LED1 to indicate the built-in LED
//
const PinName ledPinName = LED1;  // By name
//
const int ppsIntPin        =  5;   // Numéro de la broche pour l'Interruption  de détection du signal PPS
const int spiChipSelectPin =  9;   // Numéro de la broche pour Chip select du port SPI
const int buttonIntPin     = 10;   // Numéro de la broche pour le "user button" 
const int resetPin         =  6;   // Numéro de la broche reliée à la broche RESET du GPS U-BLox ZED-F9R et ou du Artemis thing plus au choix.
const int transmitReadyPin =  7;   // Numéro de broche reliée à la broche TXR du GPS U-BLox ZED-F9R
//
//
// ***** Variable  *****
//
//
// ***** Variable pour les interruption ISR
//
volatile bool alarmISR_         = false; // Cette variable est assignée à true si l'horloge interne du Apollo génère une interruption ex: "alarme".
volatile bool buttonISR_        = false; // Cette variable est assigné à true si on appui sur le "user button".
volatile bool ppsISR_           = false; // Cette variable est assigné à true si le signal PPS en provenance du GPS est détectée.
volatile bool TransmitReadyISR_ = false; // Cette variable est assigné à true si le signal Transmit Ready en provenance du GPS U-BLox ZED-F9R est détectée.
//
volatile bool watchdogFlag = false; // indicateur Watchdog Timer ISR flag
volatile int watchdogInterrupt = 0; // Compteur Watchdog interrupt counter
//
// ***** autres variables 
//
// ***** Object  *****
//
//
///*********************************************************/
//
//
// ***** Souroutines *****
//
//
// Routines de gestion des interruptions
//
//
void ButtonISR()                     // Usage futur de détection du bouton. Ex: Déclencher un traitement si le bouton est poussé par un usager.
{
  buttonISR_ = true;
}
//
//
void TransmitReadyISR()
{
  TransmitReadyISR_ = true; 
}
//
volatile unsigned long eventPPS = 0;
void ppsISR()                       // Noter le temps d'arrivée du signal PPS.
{
  eventPPS = millis();
  ppsISR_ = true;
}
//
//
extern "C" void am_rtc_isr(void)     // Usage futur de l'horloge en temp réel du Artemis thing plus. Ex: déclenchement d'un traitement par une alarme de l'horloge  ± 4 microsecondes.
{ 
  // Effacer RTC alarm interrupt
  am_hal_rtc_int_clear(AM_HAL_RTC_INT_ALM);
  alarmISR_ = true;  
}
//
//
// ***** Appelé une fois quand le MCU est alimenté ou reset *****
//
//
void setup() { 
  // La pin ResetPin #? est reliée à la pin RESET du Artemis thing plus ou du GPS.
  // When the application starts up, all pins get pulled LOW. 
  // This would therefore disable Arduino from every running
  // BUT, the trick is: in setup() function, the FIRST thing that happens is we write HIGH to the pin resetPIN
  digitalWrite(resetPin, HIGH );                                        //Le plus rapidement possible mettre à HIGH la pin reliée à la pin RESET
  //delay(200);                                                         // Enlever le délais de 200 mS il empêche le Artemis thing plus de recevoir le code compilé par Arduino IDE
  pinMode(resetPin, OUTPUT);                                            // Ensuite en mode OUTPUT
  digitalWrite(transmitReadyPin, HIGH );                                        //Le plus rapidement possible mettre à HIGH la pin reliée à la pin RESET
  //delay(200);                                                         // Enlever le délais de 200 mS il empêche le Artemis thing plus de recevoir le code compilé par Arduino IDE
  pinMode(transmitReadyPin, OUTPUT);                                            // Ensuite en mode OUTPUT
  //
  Serial.begin(115200);                                                 // Start USB serial communication
  while (!Serial);                                                      // Wait for serial connection to open (only necessary on some boards)  //En commentaire au cas ou le port USB du Artemis thing plus ne serait pas branché à un ordi.
  Serial.print( "INFO, Serial USB=OK" );                                // Confirmation de la disponibilité du port Série USB  "Serial"  
  Serial1.begin(115200);                                                // Port série UART du Artemis thing plus.  "Serial1"
  while (!Serial1);                                                     // Wait for serial connection to open (only necessary on some boards).
  Serial.print( ", Serial1 UART=OK" );                                  // Confirmation de la disponibilité du port Série UART "Serial1"
  // 
  pinMode( ppsIntPin, INPUT );                                          //set the IRQ pin as an input pin. do not use INPUT_PULLUP - the ZED-F9R will pull the pin.
  pinMode(LED_BUILTIN, OUTPUT);                                         // Gestion de la LED bleue à OFF.  
  attachInterrupt(digitalPinToInterrupt(ppsIntPin), ppsISR, RISING);    // Le ZED-F9R will pull the interrupt pin HIGH when a PPS event is triggered.
  Serial.println(", PPS Interrupt=OK" );                                // Confirmer le mise en place de l'interruption
  attachInterrupt(digitalPinToInterrupt(transmitReadyPin), TransmitReadyISR, RISING);
  Serial.println(", TransmitReady Interrupt=OK" );                                // Confirmer le mise en place de l'interruption
  wdt.stop();
  delay(500);                                                           // Attendre que le ZED-F9R se stabilise.    
}
//
//
//******************************************************************************************
// !!!!! Après "setup()", "loop()" est une boucle infinie tant que le MCU est alimenté !!!!!
//******************************************************************************************
//
//
void loop() { // La boucle loop() est trop lente pour des SBC de moins de 16 MHz. Mais le Artemis Thing Plus roule à 48 MHz
  const byte numChars = 255;                      // Size du Buffer de caractères temporaire pour un message NMEA  Débutant par '$' et se terminant par une fin de ligne. 
  static char receivedChars[numChars];            // Buffer de caractères temporaire pour un message NMEA  Débutant par '$' et se terminant par une fin de ligne. 
  byte inByte =0;                                 // Caratère courant
  static byte indexMsg = 0;                       // Index du tableau des caractères pour le message courant
  static bool debutMessage = false;               // Détection du caractère '$'
  static bool finMessage = false;                 // Détection du caractère de fin de ligne \n - 10 
  static unsigned long debutCohortePPS = 0;       // Timestamp début des messages pour une seconde donnée PPS, en millisecondes écoulée depuis le dernier powerON ou reset du SBC.
  static unsigned long finCohortePPS = 0;         // Timestamp fin   des messages pour une seconde donnée PPS, en millisecondes écoulée depuis le dernier powerON ou reset du SBC.
    unsigned long DureeTransitCohorte = 0;        // Durée totale en miliseconde du traitement des messages suivant tout suite un PPS.
  unsigned long DelaisFixPosPPS = 0;              // Delais avant l'arrivée des messages NMEA pour un PPS données ou délais de "Position fix calculation".
  static unsigned long DeltaUTC = 0;              // Correrction du temps UTC de GNZDA pour ajouter le délais PPS-NMEA, le délais de transmission RS-232 et l'imprécision du timer millis()
  static unsigned long overflowDeltaUTC = 0;      // Compter le nombre de dépassement anormalement long de DeltaUTC  (Bug hardware/software introuvable pour le moment)
  static unsigned long lastEventPPS = 0;          // Utiliser pour calculer le le déaps entre deux PPS
//
//  
  if (ppsISR_ == true){                                       // Le ZED-F9R rapporte un event PPS
    ppsISR_ = false;                                          // indiqué que les traitements PPS sont terminés pour cette boucle
    digitalWrite(LED_BUILTIN, HIGH);                          // Indiquer l'arrivé de la PPS par le LED bleue
    delay(100);                                               // Délais pour qu'elle soit visible
    digitalWrite(LED_BUILTIN, LOW);                           // Éteindre la LED bleue
    }

if (TransmitReadyISR_ == true){                                       // Le ZED-F9R rapporte un event PPS
    TransmitReadyISR_ = false;                                          // indiqué que les traitements PPS sont terminés pour cette boucle
    digitalWrite(LED_BUILTIN, HIGH);                          // Indiquer l'arrivé de la PPS par le LED bleue
    delay(1000);                                               // Délais pour qu'elle soit visible
    digitalWrite(LED_BUILTIN, LOW);                           // Éteindre la LED bleue
    }


  //
  // Ne pas allumer ou éteindre la LED bleue durant la réception des carractères cela fait bugger le délais de traitement de 45 mS à > 1000 mS    
  //
  if ( Serial1.available()  ) {                               // Y-a-t'il des caractères d'arrivés ?
    inByte = Serial1.read();                                  // Lire un caractère comme un byte en provenance du GPS ( Serial ligne RX )
    Serial1.write( inByte );                                  // Écrire ce même caractère vers le data logger         ( Serial ligne TX )
    if( inByte == '$' ) {                                     // Est-ce un début de message '$' ?
      //digitalWrite(LED_BUILTIN, LOW); // Visual indication of system reset trigger
      if(debutCohortePPS== 0 ) {debutCohortePPS = millis();}  // Noter le début de traitmement d'un cohorte de message NMEA (i.e. même PPS timestamp )
      debutMessage = true;                                    // Alors début de message alors ajuster le flag de début
      finMessage = false;                                     // C'est un début de message alors annule le flag de fin de message
      receivedChars[0] = inByte;                              // Enregistre la caractère courant dans le buffer. Le début de message commence toujours à l'indice zéro du tableau      
      indexMsg = 1;                                           // Prochain caractère à l'index un
      } else if ( inByte == 10 || indexMsg > 82 ) {           // Sinon fin message LF ou maximum longueur de 82 ? 
          receivedChars[indexMsg] = 10;                       // Enregistre la caractère courant dans le buffer, Force à LF au cas ou le message > 82 caractères
          indexMsg++;                                         // incrémente l'index tableau
          receivedChars[indexMsg] = 0;                        // Ajoute un caractère NULL, pour faciliter traitement avec fonctions de type NULL terminated string.
          finCohortePPS = millis();                           // Horodatage de la fin du bloc de message NMEA
          if(debutMessage){                                   // Si une fin de messages arrive avant un début de message l'ignorée
            finMessage = true;}                               // Fin de message détectée
          }                                                   // Ne pas incrémenter indexMsg, ce caratère NULL ne fait pas partie de la chaîne et ne doit pas être transmis.
      else {
        if ( debutMessage  &&  !finMessage ){                 // Si un message est en cours de détection, accumuler les carractères dans le tableau pour traitement ultérieur
          receivedChars[indexMsg] = inByte;                   // Enregistre la caractère courant dans le buffer
          indexMsg++;                                         // incrémente l'index tableau  
          }
        }
    //  
    // Un caractère est reçu, raité et envoyé.
    //   Vous avez maintenant entre 2,6 et 5,6 millisecondes pour faire des traitements connexes sans risque de perdre des caractères en réception s'il reste encore des messages de la même cohorte PPS à venir. (32 ou 64 caractères à 155200 bauds)
    //   Vous avez maintenant 955 millisecondes pour faire des traitements connexes sans risque de perdre des caractères en réception, si tous les messages de la même cohorte PPS sont tous arrivés.
    //
    if ( debutMessage == true  &&  finMessage == true) {  // Un message complet a été reçu
      //finCohortePPS = millis();
      if (receivedChars[0] == '$' && receivedChars[1] == 'G' && receivedChars[2] == 'N' && receivedChars[3] == 'Z' && receivedChars[4] == 'D' && receivedChars[5] == 'A' && receivedChars[6] == ',' ) {
      /*char * pch;                                         // Pointeur sur un tableau de caractères avec un NULL comme délimiteur de fin de chaîne de caractères alias null terminated string.
      pch = strstr ( receivedChars, "$GNZDA," );          // Est-ce que la chaîne de caractères "$GNZDA," est présente dans le message reçu courant. 
      if ( pch != NULL )*/                                 // Si oui alors GO! construction du message $UTC à partir d'un message $GNZDA. Le message $GNZDA est le dernier message d'une cohorte PPS pour u-blox, donc ~955 millisecondes de lousse.
        Serial1.write( "$UTC,");                          // $UTC,  Nom du message     
        Serial1.write( &receivedChars[23], 4);            // $GNZDA Année
        Serial1.write( &receivedChars[20], 2);            // $GNZDA Mois
        Serial1.write( &receivedChars[17], 3);            // $GNZDA Jour + la virgule
        Serial1.write( &receivedChars[7], 7);             // $GNZDA HHMMSS + le point
        //finCohortePPS = millis();                         // Timestamp de la fin du traitement
        
        if ( finCohortePPS >= debutCohortePPS) {                                  // Si le message $GNZDA est reçu plus d'une fois par seconde (1 Hz), il faut modifier le code d'ajout du DureeTransitCohorte à l'heure GNZDA. 
          DureeTransitCohorte =  (unsigned long) (finCohortePPS-debutCohortePPS); // Temps total de traitement avec une protection contre le rollover 
          } else { DureeTransitCohorte =  (unsigned long) (debutCohortePPS-finCohortePPS); 
          }
        if( debutCohortePPS >= eventPPS){
          DelaisFixPosPPS = (unsigned long) (((debutCohortePPS-eventPPS)*47UL)/46.875);      // Délais avant l'envoi des messages NMEA par le GPS causé par le temps nécessaire pour le calcul de la position.
        } else { DelaisFixPosPPS =  (unsigned long) (((eventPPS-debutCohortePPS) * 47UL) / 46.875); //  UL après un constante comme 100UL indique un cast (unsigned long)
        }
        DeltaUTC = DelaisFixPosPPS + DureeTransitCohorte;
                                                                // On prend pour acquis de la partie fractionnaire des secondes est toujours zéro = "00" à 1 Hz.
        if ( DeltaUTC > 999 ){                                  // Bug pour l'instant inexplicable de "900mS" au lieu de "45mS" pour DureeeTransit être >= 900 ms. Regulateur de voltage capricieux ?, attendre 10+ secondes entre les rebranchements de l'alimentation.
          Serial1.write( "335" );                               // on impose une limite maximale fictive à la durée de transit d'une cohorte.
          } else {
              if ( DeltaUTC < 100 ) { Serial1.write( '0'); }    // Format du nombre avec des zéros signaficatifs avant
              if ( DeltaUTC <  10 ) { Serial1.write( '0'); }    // Format du nombre avec des zéros signaficatifs avant
              Serial1.print( DeltaUTC );                        // Différentiel entre le temps réel UTC et le temps de transit GPS->Data Loggeur. 
          }
        Serial1.write( '0');                                   // Ajout d'un zéro à la 4e position après le point comme le message $UTC de l'Applanix  
        Serial1.write("\r\n");                                 // Fin de ligne.  13 - Retour de chariot - Carriage Return - \r et 10 - Nouvelle ligne - Line Feed \n
        Serial1.write("$DELTA,Fix=" );                         // Nouveau message non standard pour afficher le différentiel UTC et temp de traitements pour information seulement.
        Serial1.print( DelaisFixPosPPS);                       // Le délais causé par le temps de calcul de la position par le GPS après l'envoi du PPS
        Serial1.print(" mS, Tx=");                             //
        Serial1.print( DureeTransitCohorte);                   // Le délais de réception et de retransmission des messages NMEA d'une même cohorte PPS.
        Serial1.print(" mS.");
        Serial1.write("\r\n");                                 // Fin de ligne.  13 - Retour de chariot - Carriage Return - \r et 10 - Nouvelle ligne - Line Feed \n

        Serial.write( "$UTC,");                          // $UTC,  Nom du message     
        Serial.write( &receivedChars[23], 4);            // $GNZDA Année
        Serial.write( &receivedChars[20], 2);            // $GNZDA Mois
        Serial.write( &receivedChars[17], 3);            // $GNZDA Jour + la virgule
        Serial.write( &receivedChars[7], 7);             // $GNZDA HHMMSS + le point
        Serial.write( "00  :  "); 

        Serial.print( " PPS= ");                                  // Debuggeur moniteur série USB 
        Serial.print( eventPPS );                               // Horodatage en milliseondes de l'arrivé du signal PPS
        Serial.write( " Debut= ");
        Serial.print( debutCohortePPS );                        // Horodatage en milliseondes de l'arrivé du premier caractère '$' des messages NMEA
        Serial.write( " Fin= ");
        Serial.print( finCohortePPS );                          // Horodatage en milliseondes de l'arrivé du dernier caractère des messages NMEA
        Serial.write( " DeltaPPS = ");
        Serial.print( (unsigned long)  round((((eventPPS -  lastEventPPS)*47UL)/46.859)));
        Serial.write( " DeltaUTC = " );
        Serial.print( DeltaUTC );   
        Serial.print ( "\t overflowUTC( " );
        Serial.print( overflowDeltaUTC );
        Serial.println ( " )" );
        if ( overflowDeltaUTC == 6UL ){
          Serial.println ( "RESET GPS..." );
          //digitalWrite(resetPin, LOW);                    // Le board Artemis est resetté, l'éxécution s'arrête ici et reprendra à "void setup()""
          //digitalWrite(resetPin, HIGH);                    // Le board Artemis est resetté, l'éxécution s'arrête ici et reprendra à "void setup()""
          overflowDeltaUTC--;
        }  
        if ( overflowDeltaUTC >= 10UL){
         Serial.println ( "RESET ARTEMIS..." );
          wdt.start();                                        // Le board Artemis est RESET par le Watchdog timer après   ?? secondes (valeur par défaut de WDT)
          while(true){}
        }  

        lastEventPPS = eventPPS;                          // Pour le caclul de la durée entre deux PPS
        
        finCohortePPS = 0;                                // Indicateur d'une nouvelle cohorte PPS à la prochaine seconde.
        debutCohortePPS = 0;                              // Indicateur d'une nouvelle cohorte PPS à la prochaine seconde.

        if (  DeltaUTC >=  500UL ) {                           // Si le temps de traitement des messages NMEA est trop long c'est notre bug. RESET GPS après 3 overflow. RESET Artemis après 4 overflowle board.  Vérification à chaque PPS
          overflowDeltaUTC += 2;
          Serial1.write( "$ERROR, DeltaUTC >= 500 mS.\r\n" );
          Serial.println("$ERROR, DeltaUTC >= 500 mS.");
          } else { if( overflowDeltaUTC >= 1 ) {overflowDeltaUTC--;}}
        }
      debutMessage = false;                               // Message traité on attend un nouveau message
      finMessage = false;                                 // Message traité on attend un nouveau message
      }        
    } else {            // S'il n'y a pas de caractères à recevoir on est entre deux cohortes NMEA donc environ 950 mS ou rien ne se passe
        }

} // end loop
//
//
// ++++++++++ FIN DU PROGRAMME ++++++++++
//
//
// Note pour la section loop()  *****
//
//
// Ici faire des chose de non-blocante et pas de niaisage.
//
// À 115200 bauds, un caractère est transmis en 86 microsecondes.
//   À 115200 baud entre chaque caractère reçu, Arduino dispose de 86 microsecondes ou 1276 instructions Arduino pour faire d'autre chose.
// Le buffer Serial est de 64 caractères, on estime 32 caractères IN et 32 caractères OUT quand le car le flow des données est relativement symétrique.
//   32 caractères * 86 microsecondes =  2,7 millisecondes de lousse.
//   Ici tous les caractères sont envoyés à mesure de leurs réception, au pif 60 caractères IN et 4 caractères OUT pour le pire cas. Donc un lousse sécuritaire entre 5,34 et 5,69 millisecondes.  
// Quand une série complète de messages est reçue (environ 512 caractères ou 45 millisecondes) il reste environ 955 millisecondes d'attente avant la prochaine série associée au PPS suivant.
//   Durant cette attente pour le prochain PPS c'est à ce moment que le message $UTC est construit de toute pièces à partir du message "$GNZDA,".
//   Le message "$GNZDA," étant le dernier message de la série à être envoyé, c'est pratique pour faire suivre immédiatemewnt par "$UTC" en profitant du temps libre inter message de 956 millisecondes. 
//
// Attention simple guillemet pour le type char et double guillemet pour le type string
//
// Ne pas utiliser les fonctions suivantes car elle sont blocantes, Arduino ne peu faire rien d'autre tant que la fonction n'est pas terminée
//   Serial.parseInt()
//   Serial.parseFloat()
//   Serial.readBytes()
//   Serial.readBytesUntil()
//
// Les MCU Arduino ont généralement très peu de mémoire RAM. La string class cause possiblement des corruption sde mémoire. Voir les alternatives ici avec des fcontions des librairies "C"http://www.cplusplus.com/reference/cstring/
//
// char myChar = 'A';
// char myChar = 65; // Les deux sont équivalent
//
// Opérateurs logiques
//
// and             &&
// and_eq          &=
// bitand          &
// bitor           |
// not             !
// not_eq          !=
// or              ||
// or_eq           |=
// xor             ^
// xor_eq          ^=
//
// La fonction millis() est basé sur un compteur interne au CPU qui divise la frequence du proceessuer par 1024
// Si le CPU a une fréquence 48 MHz ou 48 000 000 de cycle par secondes
// Alors 48 000 000 divisé par 1024 donne 46 875 cycle par secondes
// On as 46 875 divisé par 1000 = 46.875 cycle par millisecondes
// Le compteur est un nombre entier, donc on arrondi à 47 cycle par milliseconde
// le ratio de 46.85 divise par 47 donne 0,9973 
// Donc un tick d'une durée supposé de 1 millisecondes est en réalité 0.9973 milliseconde.
// Donc 1 000 millisecondes multiplié par le ratio 0,9773 et arrondis donne 977 tick de simili millisecondes pour une durée réelle de 1 000 millisecondes.
//
//
// ***** LA VRAI FIN *****