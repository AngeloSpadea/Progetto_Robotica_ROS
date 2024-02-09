# Robotica 2023-2024

Robot che in loop si muove tra tre posizione fisse

## Descrizione

Il progetto si occupa di far eseguire al robot Panda il task di Pick and Place. Per fare ciò è stata implementata una macchina a stati (Wait, Goto, Pick and Place). Ogni stato comunica con il file `move_tutorial.py` per decidere in quale punto andare e se aprire o meno la pinza. Sono state definite 3 posizioni fisse: `home_pose`, `pick_pose`, `place_pose`. A questo punto, il `ControlNode` comunica sempre tramite topic con il file `move_robot.py`, che si occupa proprio del movimento simulato in Rviz.

## Struttura del Progetto

Il progetto è strutturato in vari packages. Uno dei packages principali è `robot_pkg`, che contiene le seguenti directory e file:

- **include**: Contiene file di inclusione, se necessario.
- **launch**: Contiene il file `progetto.launch`, che è utilizzato per avviare l'applicazione.
- **src**: Contiene i seguenti file:

  - `fsm.py`: Implementa la macchina a stati.
  - `move_robot.py`: Implementa la classe `ArmController`, responsabile della gestione del robot in Rviz. Esso fornisce due metodi:
    - `move_arm(self)`: Si occupa dello spostamento del robot fornendo una posizione e un orientamento tramite il topic `target_arm`.
    - `move_eff(self)`: Si occupa dell'apertura e della chiusura della pinza. I parametri vengono forniti dal topic `endeffector_status`.
  - `move_tutorial.py`: Implementa la classe `ControlNode`, responsabile della gestione delle azioni in base allo stato fornito da `fsm.py` tramite il topic `/status`.

- `CMakeLists.txt`: File di configurazione CMake per il package.
- `package.xml`: File di manifest del package.

## Servizi Offerti

Il nodo `ControlNode` offre il seguente servizio:

- `/start` (tipo: Empty): Questo servizio deve essere richiamato per avviare il task.

## Requisiti

- ROS (versione)
- Altri requisiti specifici del sistema, se applicabile

## Installazione

Descrizione dei passaggi per installare il progetto, inclusi eventuali dipendenze.

```bash
# Esempi di comandi per l'installazione, se applicabile
