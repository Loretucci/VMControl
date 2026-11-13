# H2 — avvio di gripper, camera Zivid e VMC

Questa guida riguarda `gripper_node.py`, `zivid_camera_h2.py` e
`h2_vmc_ros.py`. Descrive l'avvio **manuale** nella sequenza gripper → camera →
VMC. I due launch file discussi per il futuro non sono ancora presenti.

I percorsi e i topic del PC2 riportati qui provengono dall'ultima procedura
documentata in [lorenzoreadme.md](lorenzoreadme.md). Verificarli sul PC2 prima
dell'uso di H2: i nuovi script non sono ancora stati provati lì come insieme.

## 1. Dove gira ogni componente

| Ambiente | Processo | Funzione |
|---|---|---|
| PC1 | `julia_torque_controller`, già avviato con la procedura del laboratorio | Pubblica lo stato FR3 e riceve le coppie |
| PC2, shell VMC A | `gripper_node.py` | Attiva il Robotiq e riceve i comandi di apertura/chiusura |
| PC2, shell Zivid | `zivid_camera_h2.py` | Acquisisce continuamente e pubblica la posa ArUco |
| PC2, shell VMC B | `h2_vmc_ros.py` | Legge stato e posa, calcola le coppie VMC, mostra il viewer e richiede la presa |

Le shell VMC A e B devono restare aperte contemporaneamente. La configurazione
precedentemente funzionante del VMC usava il container `fr3-vmc` tramite
`nsenter` con rete host e un profilo Fast DDS UDP; la preparazione è descritta
nel [README precedente, sezione 4](lorenzoreadme.md#4-pc2--accesso-al-vmc-e-profilo-udp).
Una normale shell `docker exec` potrebbe non avere lo stesso comportamento di
rete. La shell Zivid attuale va verificata: nel README precedente la camera
girava sull'host con `~/venvs/zivid/bin/python`.

## 2. Controlli preliminari sul PC2

Aggiornare la copia Git sul PC2 e verificare che i tre file siano visibili
nelle rispettive shell. I percorsi sotto sono quelli noti dall'ultima prova:
`/workspace/VMControl` nella shell VMC e `~/VMControl` sull'host PC2. Se la
shell Zivid è un altro container, usare il percorso del progetto visibile lì.

Nelle shell coinvolte, ROS 2 Humble deve essere attivo e devono coincidere
`ROS_DOMAIN_ID` e `ROS_LOCALHOST_ONLY` con il resto del sistema. L'ultima
configurazione documentata usava `ROS_DOMAIN_ID=0` e `ROS_LOCALHOST_ONLY=0`.

Prima di avviare H2, nella shell VMC verificare che la directory Menagerie sia
visibile **dentro quella shell** e contenga il modello unito:

```bash
export MUJOCO_MENAGERIE_PATH="/percorso/nel-container/mujoco_menagerie"
test -f "$MUJOCO_MENAGERIE_PATH/fr3_robotiq_2f_140/scene_fr3_robotiq.xml"
```

Sostituire il percorso tra virgolette con quello reale. Se `test` fallisce,
`h2_vmc_ros.py` non troverà il modello. La variabile va impostata nella shell
che avvia H2; il percorso sull'host non basta se non è montato nel container.

Controllare inoltre che il topic dei giunti del PC1 sia raggiungibile dal PC2:

```bash
ros2 topic echo /robot2/franka/joint_states --once
```

Se non arriva un messaggio, risolvere prima la comunicazione ROS. I valori
predefiniti del parser H2 (`/robot/joint_states` e `/mujoco/robot/torques`) **non**
sono i topic del robot usati nell'ultima prova documentata: nei comandi H2
seguenti sono sostituiti esplicitamente.

## 3. Avviare il gripper per primo — shell VMC A

Preparare la shell VMC come nel README precedente. Nell'ultima configurazione
il progetto era in `/workspace/VMControl` e Python in `/opt/venv/bin/python`:

```bash
cd /workspace/VMControl
source /opt/ros/humble/setup.bash
source /opt/venv/bin/activate
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
test -e /dev/ttyUSB0
/opt/venv/bin/python FR3/gripper_node.py
```

Il nodo usa `/dev/ttyUSB0` per impostazione predefinita. Se l'adattatore ha
un'altra porta, sostituire l'ultima riga con, per esempio:

```bash
/opt/venv/bin/python FR3/gripper_node.py --ros-args -p port:=/dev/ttyUSB1
```

All'avvio il gripper esegue l'attivazione/calibrazione e poi riceve un comando
di apertura completa. **Il messaggio `Activation complete.` indica che è stata
completata l'attivazione, non che le dita abbiano già finito di aprirsi.**
Anche `Gripper active. Listening...` compare dopo l'invio del comando di
apertura, senza attendere che il movimento finisca.

Prima di avviare H2, verificare che il gripper sia effettivamente aperto e
fermo. Da un'altra shell con ROS attivo:

```bash
ros2 topic echo /gripper/status --once
ros2 topic echo /gripper/current_distance --once
```

`/gripper/status` contiene `[gSTA, gOBJ, gFLT, gCU]`. A fine apertura ci si
aspetta `gSTA=3`, `gOBJ=3`, `gFLT=0` e una distanza vicina a `0.14 m` su
`/gripper/current_distance`. Leggere più di un aggiornamento se il gripper è
ancora in movimento. Lasciare in esecuzione il nodo gripper.

## 4. Avviare la camera — shell Zivid

`zivid_camera_h2.py` carica i settings da un **percorso relativo** alla
directory corrente:

```text
Scrivania/Tesi/First_Ex_LAB_VMC_OmarLibrary_27-10/zivid_camera_settings.yml
```

Carica inoltre la calibrazione da questo percorso assoluto:

```text
/home/lorenzoantonio/zivid_ws/calib_2026-10-21/calibration_outputs/T_cam_to_base_Zivid.npy
```

Nella precedente configurazione host, la camera si avviava così; usare questo
comando solo se la shell Zivid conserva gli stessi percorsi e lo stesso venv:

```bash
source /opt/ros/humble/setup.bash
source ~/venvs/zivid/bin/activate
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
cd /home/lorenzoantonio
test -f Scrivania/Tesi/First_Ex_LAB_VMC_OmarLibrary_27-10/zivid_camera_settings.yml
test -f /home/lorenzoantonio/zivid_ws/calib_2026-10-21/calibration_outputs/T_cam_to_base_Zivid.npy
python3 -B ~/VMControl/FR3/zivid_camera_h2.py
```

Se ora la shell Zivid è un container distinto, verificare con `pwd`,
`command -v python3` e `ls` il percorso del progetto prima di adattare il
comando. Lo script cerca attualmente il marker **ID 16**, dizionario
`aruco5x5_1000`; questi valori sono nel file Python e non sono argomenti del
parser. Pubblica `PoseStamped` su `/vision/aruco_pose` con `frame_id=base` solo
quando quel marker viene rilevato. Continua ad acquisire nuovi frame finché
non viene interrotto.

Da una shell ROS che comunichi con la camera, controllare che arrivino pose:

```bash
ros2 topic echo /vision/aruco_pose --once
```

Se il comando resta in attesa, verificare marker, impostazioni Zivid e
comunicazione ROS tra le due shell. Lasciare in esecuzione la camera.

## 5. Avviare H2 per ultimo — shell VMC B

Aprire **una seconda** shell VMC con lo stesso accesso al container, ROS 2,
venv, rete e profilo DDS usati per le prove VMC precedenti. Impostare anche
`MUJOCO_MENAGERIE_PATH` in questa shell. Se si usa il profilo UDP temporaneo
del README precedente, `FASTRTPS_DEFAULT_PROFILES_FILE` deve puntare a un
file leggibile anche qui: la variabile della shell A non si trasferisce da
sola alla shell B.

Per quella configurazione, prima del comando H2 impostare nella shell B il
percorso effettivo del profilo creato seguendo il README precedente:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE="/percorso/del/profilo-udp.xml"
test -r "$FASTRTPS_DEFAULT_PROFILES_FILE"
```

Prima prova con coppie pubblicate su un topic isolato:

```bash
cd /workspace/VMControl
source /opt/ros/humble/setup.bash
source /opt/venv/bin/activate
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export MUJOCO_MENAGERIE_PATH="/percorso/nel-container/mujoco_menagerie"
test -f "$MUJOCO_MENAGERIE_PATH/fr3_robotiq_2f_140/scene_fr3_robotiq.xml"
/opt/venv/bin/python FR3/h2_vmc_ros.py \
    --sub-topic /robot2/franka/joint_states \
    --torque-pub-topic /vmc/h2_dry_run_torques \
    --camera-topic /vision/aruco_pose \
    --duration 0
```

Questa prova isola **soltanto le coppie del FR3**: se `gripper_node.py` è
attivo, H2 può comunque pubblicare su `/hand/finger_distance` e chiudere il
gripper quando le condizioni di presa sono soddisfatte. Verificare che il
topic di prova non sia sottoscritto da un controller del robot.

H2 attende la prima posa camera valida e il primo `JointState`; poi apre il
viewer MuJoCo. Con `--duration 0` continua finché viene interrotto. Senza
`--duration 0`, il valore predefinito è **60 secondi anche con il viewer**.
Senza `--headless`, il viewer resta attivo; `--headless` lo disabilita.
Se il viewer non riesce ad aprire il display X11, seguire la
[sezione sul viewer del README precedente](lorenzoreadme.md#9-viewer-integrato).

Per la prova sul robot reale, terminare prima la prova isolata e verificare
che non restino altre istanze H2. Il topic coppie documentato per il controller
del PC1 è:

```text
/robot2/julia_torque_controller/external_torques
```

Sostituirlo a `/vmc/h2_dry_run_torques` **solo dopo** aver verificato nel
laboratorio che il controller attivo sottoscriva quel topic e che il modello,
la calibrazione e i parametri di controllo siano quelli previsti per la prova.
`h2_vmc_ros.py` invia le coppie del meccanismo virtuale; non aggiunge una
compensazione di gravità propria.

## 6. Parametri modificabili di H2

Il parser è definito in `h2_vmc_ros.py`. Alcuni valori predefiniti:

| Argomento | Default | Significato |
|---|---:|---|
| `--stiffness` | `2000` | Rigidezza di ciascuna molla virtuale, N/m |
| `--damping` | `50` | Smorzamento di ciascun damper, Ns/m |
| `--max-force` | `20` | Saturazione per componente della **molla**, N |
| `--link-length` | `0.4` | Distanza TCP–site superiore virtuale, m |
| `--pub-rate` | `1000` | Frequenza di pubblicazione delle coppie ROS, Hz |
| `--timestep` | `0.001` | Intervallo di esecuzione del loop Python, s |
| `--state-timeout` | `0.2` | Tempo massimo senza nuovo stato robot, s |
| `--duration` | `60` | Durata del ciclo, s; `0` significa continuo |

Esempio di modifica dei parametri al lancio:

```bash
/opt/venv/bin/python FR3/h2_vmc_ros.py \
    --sub-topic /robot2/franka/joint_states \
    --torque-pub-topic /vmc/h2_dry_run_torques \
    --camera-topic /vision/aruco_pose \
    --stiffness 1500 --damping 20 --max-force 15 \
    --duration 0
```

I numeri dell'esempio illustrano la sintassi, non sono parametri validati per
H2 sul robot reale. Il parser legge i valori una volta all'avvio: per cambiarli
occorre arrestare e rilanciare H2. Quando creeremo i launch file, ogni valore
che vorremo cambiare con `ros2 launch ... nome:=valore` dovrà essere dichiarato
nel launch e inoltrato come corrispondente argomento `--nome` allo script.

## 7. Arresto

Interrompere per primo H2 con `Ctrl+C` e verificare che il suo publisher di
coppie non sia più presente. Poi interrompere la camera e, quando non servono
più comandi alle dita, il nodo gripper. La chiusura del controllo PC1 segue la
procedura del laboratorio; non viene gestita da questi tre script.
