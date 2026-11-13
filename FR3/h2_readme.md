# H2 — avvio completo su PC1 e PC2

Questa guida avvia il controller FR3 sul PC1, il gripper e la camera Zivid
sull'host del PC2, e `h2_vmc_ros.py` nella shell VMC del PC2. La sequenza è:
**PC1 pronto → gripper aperto → camera attiva → H2**. Il gripper compie
attivazione/calibrazione e apertura quando parte il suo nodo; H2 va avviato
soltanto dopo che l'apertura è terminata.

Stato verificato il 13 novembre 2026: sul PC2 sono stati caricati i launch del
commit `0d522f6`; il venv host `~/venvs/vmc` importa PyModbus 3.16.2; i file
settings e calibrazione Zivid esistono; il modello FR3 + Robotiq è stato
compilato **nel container** con `nq=13`, `nv=13`, `nu=7`. Non risulta ancora una
prova integrata di H2 con il nuovo `torque_subscriber_controller`. Il nuovo
nome del controller è stato comunicato da Lorenzo; **il topic esatto delle
coppie va verificato sul PC1 prima della prova reale**. Il vecchio
[lorenzoreadme.md](lorenzoreadme.md) descrive H1 con
`julia_torque_controller`: è una prova storica, non il comando H2 da copiare.

## 1. Quale ambiente esegue cosa

| Macchina e ambiente | Processo | Python e accesso |
|---|---|---|
| PC1, container `franka_ros2` | Driver FR3 e `torque_subscriber_controller` | Procedura del laboratorio |
| PC2, host, terminale gripper | `gripper_node.py` | `~/venvs/vmc/bin/python`; USB `/dev/ttyUSB0` |
| PC2, host, terminale camera | `zivid_camera_h2.py` | `~/venvs/zivid/bin/python`; camera Zivid |
| PC2, shell VMC tramite `nsenter` | `h2_vmc_ros.py` e viewer | `/opt/venv/bin/python` nel container `fr3-vmc`, rete del PC2 |
| PC2, host, terminale diagnostica | `ros2 topic ...` | ROS 2 Humble |

Sul PC2 `~/VMControl` è montata nel container come `/workspace/VMControl`.
Il modello copiato dal Mac ha attualmente una doppia directory:

- Host: `~/VMControl/fr3_robotiq_2f_140/fr3_robotiq_2f_140/`
- Container: `/workspace/VMControl/fr3_robotiq_2f_140/fr3_robotiq_2f_140/`

Perciò `MUJOCO_MENAGERIE_PATH` deve indicare la **directory esterna**
`/workspace/VMControl/fr3_robotiq_2f_140`: H2 aggiunge da sé
`fr3_robotiq_2f_140/scene_fr3_robotiq.xml`. L'XML è la copia autonoma con le
mesh in `assets/`; non serve la copia diversa presente negli esempi del
laboratorio. La compilazione nel container ha avuto esito positivo.

### Perché `nsenter` e il profilo Fast DDS UDP

La shell host `vmc` e la shell nel container usano due Python diversi.
`docker exec fr3-vmc` usa il filesystem **e la rete bridge** del container:
è sufficiente per controllare import e compilare XML. Il comando `nsenter`
usato qui entra nel filesystem e nel Python del container, ma non nel suo
namespace di rete perché **non** contiene `--net`: il processo H2 usa quindi
la rete del PC2. È la configurazione che aveva funzionato per la prova H1.

Nella prova H1 gli host ROS e la shell VMC non condividevano correttamente il
trasporto shared memory di Fast DDS; imponendo UDPv4 **nella shell H2** la
posa della camera arrivava al VMC. Il profilo qui sotto riguarda Fast DDS,
non il preset di acquisizione Zivid. Gripper e camera, che girano sull'host,
non richiedevano quel profilo nella procedura documentata.

## 2. PC1 — preparare il controller prima di H2

Se il driver e il controller sono **già attivi**, lasciarli in esecuzione e
passare alle verifiche. Non avviare una seconda istanza. Se il sistema è
fermo, seguire la procedura fisica del laboratorio per preparare l'FR3, FCI e
il controllo; la persona responsabile del robot deve conoscere lo stato
dell'arresto di emergenza.

Nel vecchio setup il container del PC1 si chiamava `franka_ros2`. Da host PC1:

```bash
docker ps -a --format 'table {{.Names}}\t{{.Status}}'
```

Solo se il container è fermo, avviarlo con `docker start franka_ros2`. Entrare
nel container e caricare ROS:

```bash
docker exec -it franka_ros2 bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

Controllare i controller e i topic già presenti:

```bash
ros2 control list_controllers
ros2 topic list -t | grep -E 'joint_states|torque|external'
```

Se il bringup non è già avviato, il **vecchio** comando aveva questa forma:

```bash
ros2 launch franka_bringup example.launch.py \
  controller_name:=julia_torque_controller \
  robot_config_file:=/ros2_ws/src/franka_bringup/config/franka_yi.config.yaml
```

Per H2 Lorenzo ha comunicato il nuovo nome `torque_subscriber_controller`.
Prima di avviarlo, verificare che il controller sia installato e configurato
nel YAML attuale del PC1. Se il bringup e il file di configurazione sono
rimasti gli stessi, il comando corrispondente è:

```bash
ros2 launch franka_bringup example.launch.py \
  controller_name:=torque_subscriber_controller \
  robot_config_file:=/ros2_ws/src/franka_bringup/config/franka_yi.config.yaml
```

**Questo comando con il nuovo nome non è ancora stato verificato in questa
sessione.** Usare quello effettivamente valido sul PC1 se il laboratorio lo ha
cambiato. Lasciare aperto il terminale del bringup.

Verificare che `/robot2/franka/joint_states` pubblichi `JointState`. Individuare
il topic sottoscritto dal **nuovo** controller per le sette coppie, senza
dedurlo soltanto dal nome:

```bash
ros2 topic list -t | grep -E 'joint_states|torque|external'
ros2 control list_controllers
```

Un possibile nome, da **verificare**, è
`/robot2/torque_subscriber_controller/external_torques`. Per il topic
individuato usare `ros2 topic info NOME_TOPIC --verbose`: ci aspettiamo tipo
`std_msgs/msg/Float64MultiArray`, un subscriber del controller e nessun
publisher H2 prima della prova reale. Annotare il nome esatto come
`REAL_TORQUE_TOPIC`. Il vecchio topic
`/robot2/julia_torque_controller/external_torques` appartiene alla
configurazione H1 documentata in precedenza.

Il controller PC1 deve essere pronto **prima** di H2: H2 attende un
`JointState` reale per inizializzare la copia virtuale. H2 non aggiunge una
compensazione di gravità propria; la convenzione delle coppie rimane quella
del controller effettivo sul PC1.

## 3. PC2 host — avviare il gripper per primo

Aprire un terminale sull'**host PC2**, non dentro `fr3-vmc`. Controllare che
non esista già un nodo gripper e che la porta USB sia quella attesa:

```bash
pgrep -af '[g]ripper_node.py'
ls -l /dev/ttyUSB*
```

Nell'ultima verifica non c'era un nodo attivo e `/dev/ttyUSB0` esisteva.
Caricare ROS e avviare **solo** il gripper con il primo modo del launch:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 launch ~/VMControl/FR3/h2_vmc.launch.py \
  start_h2:=false \
  python_executable:=/home/lorenzoantonio/venvs/vmc/bin/python \
  gripper_port:=/dev/ttyUSB0
```

Il launch resta aperto. Il nodo fa reset, attivazione/calibrazione e invia un
comando di apertura completa. `Activation complete` e `Listening...`
compaiono **prima** che l'apertura sia necessariamente terminata.

Da un secondo terminale host con ROS caricato:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 topic echo /gripper/status --once
ros2 topic echo /gripper/current_distance --once
```

`/gripper/status` contiene `[gSTA, gOBJ, gFLT, gCU]`. Ripetere la lettura se
le dita sono ancora in moto. Prima di H2 attendere `gSTA=3`, `gOBJ=3`,
`gFLT=0` e distanza vicina a `0.14 m`. Se il nodo segnala un guasto o la porta
non è accessibile, risolverlo prima di continuare. Lasciare aperto il launch.

## 4. PC2 host — avviare la camera Zivid

In un altro terminale **host PC2**:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 launch ~/VMControl/FR3/zivid_camera_h2.launch.py \
  python_executable:=/home/lorenzoantonio/venvs/zivid/bin/python \
  camera_workdir:=/home/lorenzoantonio
```

Il launch mantiene la directory `/home/lorenzoantonio` perché
`zivid_camera_h2.py` legge i settings dal percorso relativo
`Scrivania/Tesi/First_Ex_LAB_VMC_OmarLibrary_27-10/zivid_camera_settings.yml`.
La calibrazione viene letta da
`/home/lorenzoantonio/zivid_ws/calib_2026-10-21/calibration_outputs/T_cam_to_base_Zivid.npy`.
Entrambi i file erano presenti nell'ultima verifica.

Lo script cattura continuamente; pubblica `/vision/aruco_pose` solo quando
rileva il marker configurato **ID 16**, dizionario `aruco5x5_1000`. Il messaggio
è `PoseStamped` con `header.frame_id="base"` e posizione in metri. Da un
terminale host di diagnostica:

```bash
ros2 topic echo /vision/aruco_pose --once
```

Il comando resta in attesa finché non viene vista una posa valida. Lasciare
aperto il launch camera.

## 5. PC2 — aprire la shell VMC e configurare Fast DDS

Aprire un **nuovo** terminale host PC2. Verificare che il container esista e
sia in esecuzione:

```bash
docker inspect fr3-vmc --format 'PID={{.State.Pid}} Stato={{.State.Status}}'
```

Se è fermo, avviarlo con `docker start fr3-vmc`, poi rileggere il PID. Entrare
nella shell con filesystem del container e rete dell'host:

```bash
vmc_pid=$(docker inspect -f '{{.State.Pid}}' fr3-vmc)
sudo nsenter --target "$vmc_pid" --mount --pid --root --wd /bin/bash
```

Da qui in poi i comandi di questa sezione si eseguono **dentro la shell
`nsenter`**. Preparare l'ambiente e verificare il modello:

```bash
cd /workspace/VMControl
source /opt/ros/humble/setup.bash
source /opt/venv/bin/activate
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export MUJOCO_MENAGERIE_PATH=/workspace/VMControl/fr3_robotiq_2f_140
test -f "$MUJOCO_MENAGERIE_PATH/fr3_robotiq_2f_140/scene_fr3_robotiq.xml"
```

Il modello in questo percorso ha già compilato nel container con
`nq=13, nv=13, nu=7`. Non installare `pymodbus` in `/opt/venv`: H2 non lo
importa; il nodo gripper usa il venv host.

Creare il profilo UDP per **questa shell** e renderlo disponibile ai comandi
ROS e al launch H2:

```bash
vmc_udp_profile=$(mktemp /tmp/vmc-h2-udp.XXXXXX)
cat > "$vmc_udp_profile" <<'XML'
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>h2_udp</transport_id>
      <type>UDPv4</type>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="h2_udp_profile" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>h2_udp</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
XML
export FASTRTPS_DEFAULT_PROFILES_FILE="$vmc_udp_profile"
test -r "$FASTRTPS_DEFAULT_PROFILES_FILE"
```

Ogni nuova shell `nsenter` richiede le proprie variabili d'ambiente. Il
profilo è un file temporaneo in questa shell; ricrearlo dopo un riavvio se non
esiste più. Non impostare `--net` nel comando `nsenter`, altrimenti la rete non
è quella della procedura H1 verificata.

Prima del VMC, verificare **da questa shell** che arrivino gli input:

```bash
ros2 topic echo /robot2/franka/joint_states --once
ros2 topic echo /vision/aruco_pose --once
```

Se sull'host arrivano ma qui no, controllare per primi `ROS_DOMAIN_ID`,
`ROS_LOCALHOST_ONLY` e `FASTRTPS_DEFAULT_PROFILES_FILE`. Non risolvere il
problema lanciando una seconda istanza H2.

### Viewer X11

Il viewer viene aperto da H2 nella stessa shell. Verificare `echo "$DISPLAY"`
qui e in un terminale host; se nella shell `nsenter` manca, esportare il valore
effettivo dell'host. Solo se compare un errore X11 di autorizzazione, da
terminale host dell'utente grafico eseguire:

```bash
xhost +SI:localuser:root
```

Al termine del viewer revocare con `xhost -SI:localuser:root`. Il vecchio
[README, sezione viewer](lorenzoreadme.md#9-viewer-integrato) contiene il
contesto della precedente prova.

## 6. Avviare H2 con le coppie isolate

Nella shell `nsenter`, dopo PC1, gripper e camera, avviare H2. Il topic delle
coppie nel launch è di default `/vmc/h2_dry_run_torques`, non quello del
controller reale:

```bash
ros2 launch /workspace/VMControl/FR3/h2_vmc.launch.py \
  start_h2:=true \
  python_executable:=/opt/venv/bin/python \
  sub_topic:=/robot2/franka/joint_states \
  camera_topic:=/vision/aruco_pose \
  torque_pub_topic:=/vmc/h2_dry_run_torques \
  duration:=0
```

`duration:=0` significa ciclo continuo e `headless:=false` (default) apre il
viewer. H2 attende una prima posa camera valida e un primo `JointState`, poi
crea i tre collegamenti virtuali. A ogni aggiornamento usa lo stato reale
misurato, calcola le coppie nel modello virtuale e le passa al bridge ROS.
Non chiama `mj_step` per far evolvere un secondo robot dinamico.

**Questa prova isola solo le coppie del braccio.** Se il nodo gripper è in
esecuzione, H2 può pubblicare una richiesta reale di chiusura su
`/hand/finger_distance` quando le condizioni di presa sono soddisfatte. Il
codice richiede per 0,5 s un errore TCP e un massimo sui tre collegamenti
inferiori a 1 cm, con dati robot e camera recenti. Il timeout robot predefinito
è 0,2 s; quello camera nel codice è 0,8 s.

Da terminale host, senza usare `topic echo` sul topic delle coppie durante
questa verifica (echo aggiungerebbe un subscriber diagnostico):

```bash
ros2 topic info /vmc/h2_dry_run_torques --verbose
ros2 topic list -t | grep -E 'joint_states|torque|external'
```

Il topic dry run dovrebbe avere un publisher H2 e **nessun subscriber**.
Il topic reale deve ancora avere **zero publisher H2**. Se i conteggi
differiscono, identificare i processi prima di procedere.

## 7. Solo dopo: inviare coppie al controller reale

Fermare **solo H2 dry run** con `Ctrl+C` e verificare che sia terminato:

```bash
pgrep -af '[h]2_vmc_ros.py'
```

Non avviare contemporaneamente due H2. Sul PC1 o da una shell ROS che vede il
controller, ricontrollare il **topic reale esatto** e il suo subscriber:

```bash
ros2 control list_controllers
ros2 topic list -t | grep -E 'torque|external'
ros2 topic info NOME_TOPIC_REALE --verbose
```

Sostituire `NOME_TOPIC_REALE` con il nome trovato; non usare automaticamente
il vecchio topic `julia_torque_controller`. Prima della prova reale concordare
sul robot i guadagni, la posa marker, la calibrazione e l'area di movimento.
I default del launch H2 (`stiffness=2000`, `damping=50`, `max_force=20`) non
sono stati validati qui con il nuovo controller. `max_force` limita ogni
componente di ogni molla virtuale: non limita la coppia totale ai giunti e
non satura il contributo dei damper.

Nella **stessa** shell `nsenter`, con modello, profilo DDS e viewer già
preparati, impostare il topic verificato e avviare una sola istanza H2:

```bash
export REAL_TORQUE_TOPIC=/sostituire/con/il/topic/verificato
ros2 launch /workspace/VMControl/FR3/h2_vmc.launch.py \
  start_h2:=true \
  python_executable:=/opt/venv/bin/python \
  sub_topic:=/robot2/franka/joint_states \
  camera_topic:=/vision/aruco_pose \
  torque_pub_topic:="$REAL_TORQUE_TOPIC" \
  stiffness:=VALORE_CONCORDATO \
  damping:=VALORE_CONCORDATO \
  max_force:=VALORE_CONCORDATO \
  duration:=0
```

Questi comandi **pubblicano coppie verso il robot reale** quando
`REAL_TORQUE_TOPIC` è quello del controller. H2 si avvia solo dopo la prima
posa camera e il primo stato robot; il target non viene creato con coordinate
provvisorie. Per comportamento inatteso usare la procedura di stop del
laboratorio. Il timeout Python invia zero coppia VMC aggiuntiva se lo stato
diventa vecchio, ma non sostituisce l'arresto del sistema robotico.

## 8. Parametri del launch H2

Il launch espone i parametri del parser di `h2_vmc_ros.py` usando
`nome:=valore`. Il gripper usa **lo stesso file launch** ma un'altra
invocazione con `start_h2:=false`; la camera usa il suo launch separato.

| Launch | Default | Significato |
|---|---:|---|
| `start_h2` | `false` | `false`: solo gripper; `true`: solo H2 |
| `python_executable` | `python3` | Passare l'interprete host o container esplicitamente |
| `gripper_port` | `/dev/ttyUSB0` | Porta del gripper, usata solo con `start_h2:=false` |
| `sub_topic` | `/robot2/franka/joint_states` | Stati reali letti da H2 |
| `torque_pub_topic` | `/vmc/h2_dry_run_torques` | Coppie pubblicate da H2 |
| `camera_topic` | `/vision/aruco_pose` | Posa target della Zivid |
| `pub_rate` | `1000.0` Hz | Frequenza di pubblicazione ROS |
| `link_length` | `0.4` m | Distanza TCP–site superiore |
| `stiffness` | `2000.0` N/m | Rigidezza per collegamento |
| `damping` | `50.0` Ns/m | Smorzamento per collegamento |
| `max_force` | `20.0` N | Saturazione per componente di ciascuna molla |
| `timestep` | `0.001` s | Intervallo di pacing del loop Python |
| `state_timeout` | `0.2` s | Soglia di stato robot scaduto |
| `duration` | `0` | Continuo fino a stop; valore positivo in secondi |
| `headless` | `false` | `true` non apre il viewer |

Esempio di cambio di guadagni per **dry run**:
`stiffness:=1500 damping:=20 max_force:=15`. Il parser legge i valori
all'avvio: dopo una modifica occorre fermare e rilanciare H2.

## 9. Arresto e note pratiche

Ordine normale: fermare H2 con `Ctrl+C`, verificare che non resti un publisher
di coppie, poi fermare camera e gripper. Il controller PC1 segue la procedura
del laboratorio. Il viewer è parte dello stesso processo H2: non aprire un
secondo H2 per avere una seconda finestra.

Se un H2 nella shell `nsenter` non termina, da host PC2 identificare prima il
PID con `pgrep -af '[h]2_vmc_ros.py'`; solo dopo, se necessario, terminare
**quel** PID. Non usare `docker rm` o ricreare container per l'arresto normale.

La cartella `~/VMControl/fr3_robotiq_2f_140/` contiene una copia locale del
modello da circa 34 MB. È dentro il repository Git ma non deve essere caricata
per errore con `git add .`. Se deve restare solo sul PC2, aggiungere
`/fr3_robotiq_2f_140/` a `~/VMControl/.git/info/exclude` (esclusione solo
locale, non sincronizzata su GitHub).
