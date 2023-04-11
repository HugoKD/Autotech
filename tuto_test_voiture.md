# Docu test de la voiture

Lancer la jetson avec le mot de passe : Intech2022

## Généralité

Pour bien faire : penser à changer le code sur vos pc puis à git pull sur la jetson
Sur la jetson il faut build le package avec ros :
'''
source ~/.bashrc
colcon build
'''
Donc essayer le moins possible de modifier du code sur la jetson. 
Le dossier monthlery est buggé sur le github mais pas sur la jetson, c'est normal sur le github si il n'est pas accessible.
## Lidar to ai

* Alimentation à moins de 24V (23.--V)
* brancher le lidar : brancher le câble noir (la masse) en premier puis le rouge
* brancher câble ethernet
* lancer la commande : ros2 run urg_node urg_node_driver __params:=path/to/my/file.yaml
	Si ca marche pas : ros2 run urg_node urg_node_driver --ros-args --params-file ./install/urg_node/share/urg_node/launch/urg_node_ethernet.yaml
* lancer le noeud lidar_to_ai : ros2 run monthlery lidar_to_ai
* lancer le noeud ai_node : ros2 run autotech_ros2_pkg ai_node
* observer les topics et voir si le retour du lidar est cohérent et voir si la réponse du modèle l'est aussi



