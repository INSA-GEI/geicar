# Étude d’une nouvelle architecture basée sur ROS2, simplifiant la solution actuelle tout en la rendant polyvalente 

## Lancer les programmes déjà existant

1) Se placer dans le dossier ``/ros2_ws`` 
2) Tapez ``. install/setup.bash``
3) Vous pouvez ensuite créer les 2 topics suivant : ``/serial_in`` et ``/serial_out`` 

Un topic permet de faire communiquer des noeuds entre eux. Dans notre cas, la création de ces topics va nous permettre de publier sur /serial_out les trames passant pas le port USB (dev/ttyUSB0) connecté à la carte, ainsi que de publier de nouvelles trames depuis ros (/serial_in).

Pour voir les trames venant de la STM32 tapez: ``ros2 run serial_comm1 talker`` (ne fonctionne que si la carte STM32 est alimentée et que l'adapteur TTL-USB est connecté à l'ordinateur)

Pour envoyer des trames vers la STM32 tapez: ``ros2 run serial_comm1 listener``

4) Une fois cette commande tapée, vous pourrez voir dans la liste des différents topics, les nouveaux topics créés:

Pour voir tous les noeuds existants :
``ros2 topic list``

Pour voir ce qui passe dans un noeud particulier :
``ros2 topic echo /serial_out ``

La trame PWM est de la forme : n°PWM (1 octet) + n°canal (1 octet) + valeur à mettre (4 octets)

- Pour le moment seulement le Timer 3 et 8 sont configurés pour recevoir ces trames.
- Le nombre maximum de channel est de 4.
- La valeur pour la PWM est une valeur flottante entre 1.0 ms et 2.0 ms (entre 1.0 et 1.5 le servo-moteur est en mode recule et entre 1.5 et 2.0 il avance)

``ros2 topic pub /serial_in std_msgs/msg/String "data: '3 2 1.5'"``

## Compiler un fichier ros dans le cas de modifications
Commencer par refaire les étapes 1) et 2), puis taper : 
``colcon build --packages-select nom_package``

exemple :
``colcon build --packages-select serial_comm1``

Cela va permettre de compiler les fichiers serial_publisher.cpp et serial_subscriber.cpp présent dans le projet ``/serial_comm1``.
