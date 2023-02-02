# ArduinoTimelapse

## Contexte

Ce projet a été réalisé dans le cadre d'un projet de fin d'étude de Master 2, en partenariat avec l'INRAE.
On souhaite effectuer un prise d'image régulière en fonction de la température extérieure mesurée par capteur SHT20.

## Fonctionnement

Pour cela, les codes de deux modèles sont disponibles :<br />

### Montage à deux Arduino

D'un côté, un modèle à deux Arduino avec les codes suivants :<br />

LoRa_Arduino_Temperature se charge de la prise de température, et de l'envoi, par protocole LoRa des informations suivantes : la fréquence de prise de photos ainsi que
l'heure actuelle (afin d'éviter la désynchronisation).<br /><br />
LoRa_Arduino_Photo se charge de la réception du message LoRa ainsi que de la prise de photo par caméra ArduCAM, la sauvegarde sur carte SD et la mise à jour de l'heure actuelle.<br />
Ce dernier programme génère parfois des dysfonctionnements causés par l'ensemble des bibliothèques nécessaires, gourmandes en mémoire. Le code LoRa_Arduino_Photo_NoCam réalise
donc le même travail mais ne prend pas en charge la prise de photo ni la sauvegarde sur carte SD.
Ce programme permet tout de même de vérifier le fonctionnement de la communication LoRa ainsi que la synchronisation des RTC.<br /><br />
Ces deux programmes, après communication LoRa, sont endormis pendant une heure puis réveillés par la RTC afin de pouvoir communiquer à nouveau.<br />

### Montage à une Arduino

D'autre part, un modèle à une Arduino avec le code suivant :<br />

ArduinoTempPhoto qui se charge de la prise de température, de la prise de photo ainsi que de l'enregistrement sur carte SD.
