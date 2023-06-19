# NMEA_ZDA_TO_UTC
Insertion du message $UTC avec correction en temps réel du décalage avec le signal PPS.

Permet de bien faire fonctionner certains équipements qui demande le message $UTC d'Applanix, mais en utilisant un GPS générique.

Le point décimal de l'heure UTC compensée dans le message $UTC est synchro à ± 0,5 mS avec l'heure réelle UTC.
Vous pouvez donc vous servir de ce repère comme d'un émulateur du signal PPS bon marché. Ce n'est pas un signal au TOP de la seconde (! TOW).

À titre indicatif seulement :
Un test avec un analyseur logique et le GPS U-Blox ZED-F9R, montre que le premier caractère après le signal PPS, arrive environ 237 mS plus tard.
Ce délai est variable et dépend de la charge de traitement en temps réel du GPS et de son paramétrage.
Il n'est donc pas indiqué de se servir du premier caractère reçu comme d'une émulation du signal PPS.

# English : ( Google Translate !)

Insertion of the $UTC message with real-time time correction of the UTC offset with the PPS signal. 

Allows you to operate certain equipment that requires the $UTC message from Applanix, but let you use a generic GPS. 

The decimal point of the compasated UTC time in the $UTC message is synchronized at ± 0.5 mS with the actual real time UTC time. 
So you can use this cue as a cheap PPS signal emulator. This is not a Top of a Second (not TOW) signal. 

For information purposes only: 

A test with a logic analyzer and the U-Blox ZED-F9R GPS, shows that the "first character" after the PPS signal, showup about 237 mS later. 
This delay is variable and depends on the real-time processing load of the GPS and its setting. 
It's therefore not appropriate to use the "first character" received as an emulation of the PPS signal. Use the $UTC message instead.

#español ( Google Translate !)

Inserción del mensaje $UTC con corrección de offset en tiempo real con la señal PPS.

Le permite operar ciertos equipos que requieren el mensaje $UTC de Applanix, pero usando un GPS genérico.

El punto decimal de la hora UTC compensada en el mensaje $UTC se sincroniza dentro de ± 0,5 mS con la hora UTC real.
Entonces puede usar este punto de referencia como un emulador de señal PPS barato. No es una señal en el TOP del segundo (! TOW).

Solo para información:
Una prueba con un analizador lógico y el GPS U-Blox ZED-F9R, muestra que el primer carácter después de la señal PPS, llega unos 237 mS más tarde.
Este retraso es variable y depende de la carga de procesamiento en tiempo real del GPS y su configuración.
Por tanto, no es apropiado utilizar el primer carácter recibido como una emulación de la señal PPS.