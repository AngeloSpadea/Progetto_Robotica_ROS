# Robotica 2023-2024

Robot che in loop si muove tra tre posizione fisse

## Descrizione

Il progetto si occupa di far eseguire al robot Panda il task di Pick and Place. Per fare ciò è stata implementata una macchina a stati (Wait, Goto, Pick and Place). Ogni stato comunica con il file `move_tutorial.py` per decidere in quale punto andare e se aprire o meno la pinza. Sono state definite 3 posizioni fisse: `home_pose`, `pick_pose`, `place_pose`. A questo punto, il `ControlNode` comunica sempre tramite topic con il file `move_robot.py`, che si occupa proprio del movimento simulato in Rviz.

## Struttura del Progetto

Le seguenti directory sono strutturate così:

- **launch**: Contiene il file `progetto.launch`, che è utilizzato per avviare:
  - panda_moveit_config/launch/demo.launc
  - il nodo "fsm" che avvia la fsm.py
  - il nodo "move_robot" che avvia la move_robot.py
  - il nodo "control_robot" che avvia la control_robot.py
- **src**: Contiene i seguenti file:

  - `fsm.py`: Implementa la macchina a stati.
  - `move_robot.py`: Implementa la classe `ArmController`, responsabile della gestione del robot in Rviz. Esso fornisce due metodi:
    - `move_arm(self)`: Si occupa dello spostamento del robot fornendo una posizione e un orientamento tramite il topic `target_arm`.
    - `move_eff(self)`: Si occupa dell'apertura e della chiusura della pinza. I parametri vengono forniti dal topic `endeffector_status`.
  - `control_robot.py`: Implementa la classe `ControlNode`, responsabile della gestione delle azioni in base allo stato fornito da `fsm.py` tramite il topic `/status`.

- `CMakeLists.txt`: File di configurazione CMake per il package.
- `package.xml`: File di manifest del package.

## Service

Il nodo `ControlNode` offre il seguente servizio:

- `/start` (tipo: Empty): Questo servizio deve essere richiamato per avviare il task.

```http
  $ rosservice call /start "{}"
```

## Installazione

Descrizione dei passaggi per installare il progetto, inclusi eventuali dipendenze.
# Install Catkin Tools and wstool

On your virtual machine, make sure you have the most up to date packages:

```http
  $ rosdep update
  $ sudo apt update
  $ sudo apt dist-upgrade
```


Install <a href="https://catkin-tools.readthedocs.io/en/latest/" target="_blank">catkin_tools</a>:

```http
  $ sudo apt install python3-catkin-tools
```

Install <a href="http://wiki.ros.org/wstool" target="_blank">wstool</a>:

```http
  $ sudo apt install python3-wstool
```

# Create a Catkin Workspace

You will need to create a catkin workspace setup:

```http
  $ mkdir -p ~/ws_moveit/src
  $ cd ~/ws_moveit/src
```

# Download MoveIt Source

```http
  $ wstool init .
  $ wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
  $ wstool remove moveit_tutorials  
  $ wstool update -t .
```

# Download Example Code

Within your catkin workspace, download the tutorials as well as the ``panda_moveit_config`` package:

```http
  $ cd ~/ws_moveit/src
  $ git clone https://github.com/ros-planning/moveit_tutorials.git -b master
  $ git clone https://github.com/ros-planning/panda_moveit_config.git -b noetic-devel
```

# Build your Catkin Workspace

The following will install from Debian any package dependencies not already in your workspace:

```http
  $ cd ~/ws_moveit/src rosdep install -y --from-paths . --ignore-src --rosdistro noetic
```

The next command will configure your catkin workspace:

```http
  $ cd ~/ws_moveit
  $ catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release
  $ catkin build
```
Dopodichè scaricare ed inserire tutti i file all'interno di un pacchetto situato in ~/ws_moveit/src.
## Come avviare il progetto
```http
  $ cd ~/ws_moveit
  $ roslaunch robot_pkg progetto.launch

aprire un secondo terminale e avviare il servizio  

```http
  $ rosservice call /start "{}"
