# Etude et développement d’une nouvelle architecture matérielle et logicielle des plateformes mobiles autonomes

**Stagiaire :** Sonia HAMDI

**Maitre de stage :** Sébastien DI MERCURIO

Dans le cadre de la réalisation du projet CMQe " Mobilité et Transport Intelligent " financé par la Caisse des Dépôts et Consignations (CDC), nous avons dû étudier et prendre en main le projet GeiCar que les étudiant de 5ème année à l'INSA font. 

Le but dans ce nouveau projet a été d'étudier une nouvelle architecture pour simplifier celle déjà existante. Cela nous a également permis de tester de nouvelles cartes STM32 et capteurs. 

**Cartes STM32 :**
- H563ZI
- L476RG

Nous avons décidé de rester sur une ancienne carte L476RG pour tester les différents capteurs.

**Capteurs :**
- IMU : Nucleo IKS01A2
- GPS : Ardusimplertk2b             
- LIDAR : LD06_LD

![image](https://github.com/user-attachments/assets/e1a77936-2115-4a85-b382-cabc8786f0d3)

Nous avons travaillé sur STM32CubeIDE et codé en C/C++. 

Dans le dossier Application vous trouverez le fichier `application.c` qui contient les différentes tâches du projet. Ces tâches se font en temps réel grâce à FreeRTOS (système d'exploitation open source en temps réel pour les microcontrôleurs).
Les Callback d'interruptions sont utilisés pour l'envoi et la réception de trames via l'UART 4. Nous avons connecté au port de cet UART un convertisseur TTL vers USB pour pouvoir envoyer et recevoir des données sans passer pas le port d'alimentation. Pour éviter la perte de données, nous avons décidé de créé une queue de message. 

En plus des différents capteurs, nous souhaitions que  la carte puisse gérer plusieurs moteurs en paramétrant des Timers en mode PWM. Pour le moment nous avons paramétré uniquement les timers 3 et 8.

**Périphériques utilisés**

- UART4 : communication avec ROS

- I2C1 : communication avec l'IMU

- USART2 : communication avec le GPS

- TIM7 et TIM8 en mode PWM (Période de 20 ms) : pilotage de servomoteurs (durée de l'état haut entre 1 ms et 2 ms)

**Tâches**

- StartUart : (Priorité Normal)
S'occupe de transmettre (``Transmit_data_to_usb()``) et recevoir (``Receive_data_from_usb()``) des données venant de l'uart4. 

- StartIMU : (Priorité High)
La fonction ``IMU_Receive_Transmit_Data()`` traite les données IMU (réception et transmission).
capteurs : humidité, température, pression, accéléromètre, magnétomètre, gyroscope.
Après réception des données, nous créons des trames contenant ces données et les envoyons dans la queue de messages.

  **Structure des trames IMU**
    1) Header : Un octet de synchronisation qui marque le début de la trame.
    2) Length : La longueur totale de la trame, en incluant tous les champs. (2 octets)
    3) Frame Type : Le type de trame, indiquant la nature des données. (1 octet)
    4) Données IMU : Les valeurs mesurées par les capteurs de l'IMU. (1 octet) 
    5) CRC : Un code de contrôle pour garantir l'intégrité de la trame.

- StartGPS : (Priorité Above Normal)
Lecture des trames NMEA du GPS, analyse, puis envoie sur la queue de messages.

  **Structure des trames NMEA**
    1) $ : Le symbole $ marque le début de chaque trame NMEA.
    2) Identifiant de la trame : Un code à 5 caractères qui identifie le type de données contenues dans la trame.
    Les trois premiers caractères (par exemple, GPG, GLG, GNR) indiquent la source (GPS, GLONASS, Galileo, etc.).
    Les deux derniers caractères indiquent le type de données spécifiques, par exemple :

        GLL : Extraction de la latitude, longitude, et heure du GPS.
        
        GGA : Trame qui donne les informations sur la position GPS et l'altitude.
        
        GSA : Extraction du mode de positionnement, du nombre de satellites utilisés, ainsi que des mesures de dilution de la précision (PDOP, HDOP, VDOP).
    
    4) Données : Chaque trame contient une série de champs délimités par des virgules, qui varient en fonction du type de trame. Ces champs représentent des informations telles que l'heure UTC, la latitude, la longitude, la vitesse, etc.
    5) Checksum : À la fin de la trame, un checksum est ajouté pour vérifier l'intégrité des données. Il est représenté par un caractère * suivi de deux caractères hexadécimaux. Le checksum est calculé en effectuant un XOR de tous les caractères entre $ et *.
    6) Les caractères de fin de ligne (Carriage Return et Line Feed) indiquent la fin de la trame NMEA.

Après avoir récupéré les données qui nous intéréssaient (altitude, longitude et latitude), nous les mettons dans une nouvelle trame à envoyer ensuite sur la queue de messages. Pour cette nouvelle trame, la structure est la même que pour les trames IMU (header, length, frame_type, data, crc).

La synchronisation temporelle des différentes tâches est principalement gérée par ``vTaskDelayUntil()``, qui garantit que chaque tâche s'exécute à des intervalles fixes, tout en laissant le CPU libre pour d'autres tâches lorsqu'une tâche est en attente.


