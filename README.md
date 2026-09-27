# BetaBot

BetaBot est un robot humanoïde construit en LEGO à base de 2 LEGO Mindstorms NXT, d'un Odroid C2, d'une carte OpenCR, d'un LiDAR LD06 et d'une webcam.

L'objectif du projet est un robot sous ROS capable de :
- faire du **SLAM** (cartographie en temps réel avec le LiDAR),
- être **piloté à distance** (clavier, joystick, RViz),
- disposer d'une **couche d'IA** pour le contrôle vocal des mouvements et le dialogue (style Vector / wire-pod, LLM hébergé sur le Mac via Ollama).

Le scan 3D du robot est disponible ici : https://poly.cam/capture/14D15C9A-101B-442E-B5E1-CDD3CCB289C9

## Matériel

| Composant | Rôle |
|-----------|------|
| 2x LEGO Mindstorms NXT (`NXT1`, `NXT2`) | Motorisation et capteurs (touch, ultrason, couleur, lumière, son, RFID) |
| Odroid C2 (Ubuntu 18.04, ROS Melodic) | Ordinateur de bord, exécute les nœuds ROS |
| Carte OpenCR | Interface supplémentaire (rosserial, `/dev/ttyACM0`) |
| LiDAR LD06 | Scan laser pour le SLAM (`/dev/ttyS1`, 230400 bauds) |
| Webcam | Vision (`/dev/video0`, topic `head_camera`) |

### Répartition NXT

- **NXT1** : `right_bumper`, `left_bumper`, `back_bumper`, `ultrasonic_sensor` (capteurs) + `right_tread`, `left_tread` (traction), `torso_joint` (moteurs)
- **NXT2** : `sound_sensor`, `color_sensor`, `line_following_sensor`, `rfid_sensor` (capteurs) + `head_joint`, `laser_joint`, `arms_joint` (moteurs)

Chaque NXT doit contenir ses programmes `NXTx_calibrate.rxe` (et `NXT2_light_anim.rxe`, `NXT2_rfid_read.rxe` sur NXT2) pour les scripts de maintenance.

## Structure du dépôt

Ce dépôt est conçu pour être cloné dans `/betabot/` sur l'Odroid C2.

- `description/` : URDF et mesh 3D du robot (`robot_description.urdf`)
- `ros_ws/` : espace de travail catkin avec les paquets ROS
  - `nxt1_ros`, `nxt2_ros` : pilotes des deux briques NXT (USB, via la librairie nxt-python en Python 2)
  - `nxt_msgs` : messages ROS spécifiques (`Contact`, `Color`, `JointCommand`, `Light`, `RFID`, `Sound`)
  - `nxt_controllers` : contrôle de la base (`base_controller`, `base_odometry`), agrégation des états de joints et contrôle des articulations
  - `base_simple_teleop` (C++), `base_enhanced_teleop`, `joints_teleop` : téléopération clavier
  - `interactive_markers_server`, `point_operation` : contrôle interactif dans RViz
  - `emergency_shutdown` : arrêt d'urgence (batterie faible ou ordre root)
  - `ldlidar_stl` : pilote du LiDAR LD06
- `ros_launch/` : fichiers de lancement (voir `ros_launch/README.md`)
- `python_scripts/` : scripts Python de maintenance des NXT (reset, animations)
- `shell_scripts/` : scripts bash (démarrage, mises à jour)
- `media/` : ressources multimédias

## Installation

Sur l'Odroid C2 (Ubuntu 18.04 / ROS Melodic) :

```bash
cd /
git clone https://github.com/rodolphemds/BetaBot.git betabot
cd /betabot/ros_ws
rosdep install --from-paths src --ignore-src -y
catkin_make
source /betabot/ros_ws/devel/setup.bash
```

Dépendances Python 2 : `nxt-python` (librairie NXT), `python-usb`, `python-pykdl`, `python-tf-conversions`.

Règles udev pour le LiDAR (accès `/dev/ttyS1` sans sudo) :

```bash
cd /betabot/ros_ws/src/ldlidar_stl/scripts
./create_udev_rules.sh
```

## Utilisation

Démarrer le noyau robot (capteurs, moteurs, LiDAR, webcam, TF, odometrie) :

```bash
roslaunch /betabot/ros_launch/core.launch
```

Piloter la base au clavier (dans un autre terminal) :

```bash
roslaunch /betabot/ros_launch/base_enhanced_teleop.launch
```

Faire du SLAM (dans un autre terminal) :

```bash
roslaunch /betabot/ros_launch/slam.launch
```

Sauvegarder la carte obtenue :

```bash
rosrun map_server map_saver -f ~/map
```

Visualiser dans RViz (transformées, scan laser, carte) :

```bash
roslaunch /betabot/ros_launch/rviz_core.launch
```

Piloter les articulations (tête, torse, laser, bras) :

```bash
roslaunch /betabot/ros_launch/joints_teleop.launch
```

Arrêt d'urgence : publier `"LOW_BATTERY"` ou `"ROOT_ORDER"` sur le topic `emergency_shutdown_request`.

## ROS sur le Mac (OrbStack)

Les conteneurs ROS1/ROS2 tournent sur OrbStack (MacBook Air). Pour piloter le robot depuis le Mac, le plus simple est un setup multi-machine ROS1 :

```bash
# Sur l'Odroid :
export ROS_MASTER_URI=http://<odroid-ip>:11311
# Sur le Mac (conteneur ROS sur OrbStack, même réseau) :
export ROS_MASTER_URI=http://<odroid-ip>:11311
export ROS_IP=<mac-ip>
rosrun rviz rviz
```

RViz sur le Mac affiche alors le `scan` LiDAR, la carte SLAM et le modèle 3D, et les nœuds de téléopération publiés sur `cmd_vel` contrôlent le robot.

## Couche IA (voix et dialogue)

L'objectif est un fonctionnement de type Vector / wire-pod :

- **wire-pod** tourne dans un conteneur Docker sur le Mac : il capte la voix (wake word + STT) et publie les commandes reconnues.
- **Ollama** tourne sur le Mac : le LLM génère les réponses de dialogue.
- Le robot reçoit les intentions sur des topics ROS (par ex. `cmd_vel` pour les mouvements, `cmd_head_joint` pour la tête) et renvoie son état (position, capteurs) pour contextualiser les réponses.

Le pont wire-pod ↔ ROS n'est pas encore dans ce dépôt ; la structure des topics est prête à le recevoir (voir `ros_launch/`).

## Sécurité

Le script `shell_scripts/update_repository.sh` utilisait un token GitHub en dur : il a été retiré. **Révoquez ce token** sur https://github.com/settings/tokens (il est exposé dans l'historique Git du dépôt public). Utilisez désormais une authentification par clés SSH ou un token saisi à l'exécution.
