# Etude et développement d’une nouvelle architecture matérielle et logicielle des plateformes mobiles autonomes

**Stagiaire :** Sonia HAMDI

**Maitre de stage :** Sébastien DI MERCURIO

Dans le cadre de la réalisation du projet CMQe "Mobilité et Transport Intelligent » financé par la Caisse des Dépôts et Consignations (CDC), nous avons dû étudier et prendre en main le projet GeiCar que les étudiant de 5ème année à l'INSA font. 

Le but dans ce nouveau projet a été d'étudier une nouvelle architecture pour simplifier celle déjà existante. Cela nous a également permis de tester de nouvelles cartes STM32 et capteurs. 

**Cartes STM32 :**
- H563ZI
- L476RG
Nous avons décidé de rester sur une ancienne carte L476RG pour tester les différents capteurs.

**Capteurs :**
- IMU : Nucleo IKS01A2
- GPS :
- LIDAR :

Nous avons travaillé sur STM32CubeIDE et codé en C/C++. 

Dans le dossier Application vous trouverez le fichier application.c qui contient les différentes tâches du projet. Ces tâches se font en temps réel grâce à FreeRTOS (système d'exploitation open source en temps réel pour les microcontrôleurs).
Les Callback d'interruptions sont utilisés pour l'envoi et la réception de trames via l'UART 4. Nous avons connecté au port de cet UART un convertisseur TTL vers USB pour pouvoir envoyer et recevoir des données sans passer pas le port d'alimentation. Pour éviter la perte de données, nous avons décidé de créé une queue de message. 

En plus des différents capteurs, nous souhaitions que  la carte puisse gérer plusieurs moteurs en paramétrant des Timers en mode PWM. Pour le moment nous avons paramétré uniquement les timers 3 et 8.
