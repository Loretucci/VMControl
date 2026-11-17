# H2 — avvio completo su PC1 e PC2

Il punto di pick attuale è il centro dell'ArUco traslato di **1,5 cm verso
l'alto lungo lo z della base**: `p_TCP = p_aruco + [0, 0, 0.015]` m.
La traslazione riguarda tutti e tre i target virtuali e vale per H2 reale,
simulazione con camera reale e precedente simulazione con marker fittizio.
Le pose pubblicate dalla camera restano le misure del marker.

Questa guida avvia il controller FR3 sul PC1, il gripper e la camera Zivid
sull'host del PC2, e `h2_vmc_ros.py` nella shell VMC del PC2. La sequenza è:
**PC1 pronto → gripper aperto → camera attiva → H2**. Il gripper compie
attivazione/calibrazione e apertura quando parte il suo nodo; H2 va avviato
soltanto dopo che l'apertura è terminata.

Stato al 13 novembre 2026: il modello FR3 + Robotiq è stato compilato **nel
container** con `nq=13`, `nv=13`, `nu=7`. È stato tentato il launch reale con
`/robot1`, dominio ROS 42, `stiffness=600`, `damping=30`, `max_force=10` e viewer
attivo. Il log disponibile mostra che il processo è terminato perché X11 ha
rifiutato l'accesso al display `:0`; Lorenzo riferisce che la prova non ha
ancora funzionato bene. Il topic delle coppie va verificato sul PC1 prima di
ogni nuovo avvio reale. Il vecchio
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
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

Controllare i controller e i topic già presenti:

```bash
ros2 control list_controllers
ros2 topic list -t | grep -E 'joint_states|torque|external'
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

Verificare che `/robot1/franka/joint_states` pubblichi `JointState` e che il
controller sottoscriva il topic delle sette coppie usato nel launch H2:

```bash
ros2 topic list -t | grep -E 'joint_states|torque|external'
ros2 control list_controllers
ros2 topic info /robot1/franka/joint_states --verbose
ros2 topic info /robot1/torque_subscriber_controller/external_torques --verbose
```

Per il topic delle coppie ci aspettiamo tipo `std_msgs/msg/Float64MultiArray`
e un subscriber del `torque_subscriber_controller`. Il percorso nel comando H2
è quello usato nel tentativo del 13 novembre, ma non abbiamo ancora un output
di `topic info` che ne confermi il subscriber. Se compare `Unknown topic` o
nessun subscriber del controller, identificare il topic effettivo sul PC1
prima di avviare H2. Il vecchio topic di `julia_torque_controller` appartiene
alla configurazione H1.

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
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
ros2 launch ~/VMControl/FR3/h2_vmc.launch.py \
  start_h2:=false \
  python_executable:=/home/lorenzoantonio/venvs/vmc/bin/python \
  gripper_port:=/dev/ttyUSB0 \
  gripper_force:=30
```

Il launch resta aperto. Il nodo fa reset, attivazione/calibrazione e invia un
comando di apertura completa. `Activation complete` e `Listening...`
compaiono **prima** che l'apertura sia necessariamente terminata.

Da un secondo terminale host con ROS caricato:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
ros2 topic echo /gripper/status --once
ros2 topic echo /gripper/current_distance --once
```

`/gripper/status` contiene `[gSTA, gOBJ, gFLT, gCU]`. Ripetere la lettura se
le dita sono ancora in moto. Prima di H2 attendere `gSTA=3`, `gOBJ=3`,
`gFLT=0` e distanza vicina a `0.14 m`. Se il nodo segnala un guasto o la porta
non è accessibile, risolverlo prima di continuare. Lasciare aperto il launch.

La versione aggiornata di `gripper_node.py` legge il parametro ROS `force`
a ogni comando di movimento. Dopo aver scaricato questa modifica occorre
riavviare il nodo per caricarla: il riavvio esegue di nuovo attivazione e
apertura. H2 riconosce la versione aggiornata tramite il parametro read-only
`force_runtime_configurable=True` e rifiuta di partire con una vecchia
istanza che continuerebbe a usare la forza memorizzata all'avvio.

## 4. PC2 host — avviare la camera Zivid

In un altro terminale **host PC2**:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
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
export ROS_DOMAIN_ID=42
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
      <interfaceWhiteList>
        <address>192.168.10.3</address>
        <address>127.0.0.1</address>
      </interfaceWhiteList>
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
ros2 topic echo /robot1/franka/joint_states --once
ros2 topic echo /vision/aruco_pose --once
```

Se sull'host arrivano ma qui no, controllare per primi `ROS_DOMAIN_ID`,
`ROS_LOCALHOST_ONLY` e `FASTRTPS_DEFAULT_PROFILES_FILE`. Non risolvere il
problema lanciando una seconda istanza H2.

### Viewer X11

Il tentativo del 13 novembre ha mostrato `X11: Failed to open display :0`.
Prima di rilanciare H2 con il viewer, aprire un **terminale del desktop del
PC2 come utente `lorenzoantonio`**, non la shell root `nsenter`, ed eseguire:

```bash
echo "$DISPLAY"
xhost +SI:localuser:root
```

Nella shell `nsenter` verificare `echo "$DISPLAY"`: deve indicare lo stesso
display (nel log era `:0`). Se è vuoto e il display host è `:0`, eseguire lì
`export DISPLAY=:0`. Il comando `xhost` è una prova di accesso, non una
conferma che il viewer ora funzioni: se ricompare un errore GLFW/X11,
diagnosticare il display prima di considerare riuscito l'avvio. Al termine
revocare l'accesso dal terminale host con `xhost -SI:localuser:root`. Il vecchio
[README, sezione viewer](lorenzoreadme.md#9-viewer-integrato) contiene il
contesto della precedente prova.

## 6. PC2 — avviare H2 sul robot reale

Nella shell `nsenter` preparata alla sezione 5, con controller PC1, gripper e
camera attivi, controllare che non esista già un altro processo H2 e che il
topic reale delle coppie abbia un subscriber del controller:

```bash
pgrep -af '[h]2_vmc_ros.py'
ros2 topic info /robot1/torque_subscriber_controller/external_torques --verbose
```

`pgrep` non stampa nulla quando non c'è già un H2. `topic info` deve mostrare
il tipo `std_msgs/msg/Float64MultiArray` e un subscriber del controller. Se
non è così, controllare il nome del topic sul PC1 prima del launch. Il comando
seguente usa **il controller reale** e riporta esplicitamente i valori
attuali del parser di `h2_vmc_ros.py` per le due molle, il damper e la
geometria. I guadagni della simulazione sono riportati nella sezione 11.

```bash
ros2 launch /workspace/VMControl/FR3/h2_vmc.launch.py \
  start_h2:=true \
  python_executable:=/opt/venv/bin/python \
  sub_topic:=/robot1/franka/joint_states \
  camera_topic:=/vision/aruco_pose \
  gripper_force:=30 \
  torque_pub_topic:=/robot1/torque_subscriber_controller/external_torques \
  pub_rate:=1000 \
  link_length:=0.8 \
  stiffness:=2000 \
  stiffness2:=20 \
  damping:=50 \
  max_force:=2 \
  max_force2:=8 \
  timestep:=0.001 \
  state_timeout:=0.2 \
  headless:=false \
  target_radius:=0.015 \
  duration:=0
```

`headless:=false` richiede il viewer X11; `duration:=0` mantiene H2 attivo
finché non viene fermato. H2 attende una prima posa camera valida e un primo
`JointState`, poi crea due serie di tre collegamenti virtuali in parallelo.
A ogni aggiornamento usa lo stato reale misurato, calcola le coppie nel
modello virtuale e le pubblica
verso il controller. Non chiama `mj_step` per far evolvere un secondo robot
dinamico.

Prima di costruire i collegamenti, H2 confronta la posa misurata della pinza
con due riferimenti equivalenti per la presa: quello del marker e quello
ruotato di 180 gradi attorno al suo asse z. Sceglie quello che richiede la
rotazione minore. Nel secondo caso inverte insieme gli assi x e y del
riferimento: i due target laterali si scambiano, mentre TCP, quota di pick e
target superiore restano identici. La scelta vale per le forze VMC e per gli
errori usati dal controllo di arrivo; le tolleranze restano 1 cm per 0,5 s.

Il messaggio `[TARGET] Pick yaw branch: 0 deg` oppure `180 deg` indica la
rotazione scelta rispetto al marker, non l'angolo assoluto nella base. La
scelta resta fissa durante gli aggiornamenti camera e dopo un timeout con
successiva ripresa del tracking. La stessa logica e applicata in
`h2_camera_simulation.py` e `h2_simulations.py`. La posa pubblicata dalla
camera resta la posa effettivamente rilevata del marker.

Per ciascuno dei tre punti, la prima serie usa `stiffness`, `max_force` e
`damping`; la seconda aggiunge una molla con `stiffness2` e `max_force2`,
senza un secondo damper. Il launch trasmette entrambi i gruppi al parser:
per esempio `stiffness2:=40` diventa `--stiffness2 40`. I contributi delle due
serie si sommano. La forza del gripper (`gripper_force`) è indipendente da
questi parametri delle molle virtuali.

La forza del gripper si sceglie dal parser H2 con `--gripper-force 30`, o dal
launch con `gripper_force:=30`. E un intero nella scala del registro Robotiq
`rFR` (1-255 nel nostro nodo), non una forza in Newton. Il default rimane 30.
All'avvio, prima di calcolare o pubblicare coppie VMC, H2 controlla che il
nodo gripper supporti l'aggiornamento e imposta `force` tramite i servizi ROS
`/robotiq_gripper/get_parameters` e `/robotiq_gripper/set_parameters`.
Attende la conferma: se il servizio manca, non risponde o rifiuta il valore,
H2 termina senza iniziare il controllo. La richiesta di parametro non muove
le dita; il valore viene inserito nel registro Modbus al successivo comando
di apertura/chiusura. Nel log compare `[GRIPPER] Force configured: rFR=...`.
Per leggere la configurazione dal terminale host con ROS caricato:

```bash
ros2 param get /robotiq_gripper force
```

Il valore `rFR` imposta il limite di corrente del motore secondo il
[manuale Robotiq](https://assets.robotiq.com/website-assets/support_documents/document/2F-85_2F-140_Instruction_Manual_CB_PDF_20190725.pdf).
La chiusura visuale di `h2_camera_simulation.py`, basata su un'equality, non
modella questa forza del motore reale.

La configurazione e stata verificata localmente con servizi ROS e scritture
Modbus sostituiti: valore trasmesso, aggiornamento al comando successivo e
byte `rFR` corretti; controllati anche vecchio nodo, rifiuto del parametro,
servizio assente e timeout. La comunicazione effettiva e la forza applicata
dal gripper fisico restano da verificare sul PC2.

Durante `APPROACH`, se non arriva una posa camera valida per piu di 0,8 s,
H2 salva una sola volta le posizioni correnti dei tre site, usando uno stato
robot recente, e le usa come riferimenti fissi. Entrambi i meccanismi
rimangono attivi: le coppie sono ricalcolate a ogni iterazione per richiamare
il gripper verso quella posa e smorzarne la velocita. Non viene conservato
l'ultimo vettore di coppie. Il log indica `target_mode=HOLD` e la richiesta
di chiusura resta inibita. Quando torna una posa camera valida si riprende
automaticamente l'inseguimento dell'ArUco. La perdita e rilevata tramite il
timeout, non da una notifica immediata di marker assente.

Questo e un mantenimento cedevole della posa dell'end-effector: non blocca
rigidamente i giunti e non costituisce un arresto di sicurezza. Compensazione
di gravita e comportamento del controller PC1 vanno verificati sul sistema
effettivo. Se scade anche lo stato del robot, resta il precedente comportamento
di pubblicazione di zero coppia VMC aggiuntiva.

Questa configurazione può muovere il **robot reale** e può chiedere al nodo
gripper una chiusura su `/hand/finger_distance`: la prova non è ancora stata
validata come funzionante. `max_force=2` e `max_force2=8` limitano ciascuna
componente della rispettiva molla a ogni collegamento, non la coppia totale
ai giunti né il contributo dei damper. Il timeout Python invia zero coppia
VMC aggiuntiva se lo stato diventa vecchio, ma non sostituisce l'arresto del
sistema robotico.

## 7. Verificare la comunicazione durante la prova

Sul PC2, nella shell `nsenter`, gli stati del PC1 devono arrivare anche prima
di lanciare H2. Per vedere la frequenza ricevuta:

```bash
ros2 topic hz /robot1/franka/joint_states
```

Interrompere con `Ctrl+C` dopo alcuni secondi. Sul PC1, nella shell ROS già
configurata con dominio 42, mentre H2 è attivo:

```bash
ros2 topic info /robot1/torque_subscriber_controller/external_torques --verbose
ros2 topic echo /robot1/torque_subscriber_controller/external_torques --once --qos-reliability best_effort
ros2 topic hz /robot1/torque_subscriber_controller/external_torques
```

`topic info` mostra publisher H2 e subscriber del controller; `echo --once`
conferma che almeno un messaggio arriva al PC1. `hz` misura la frequenza
**ricevuta dal comando diagnostico**, che ad alta frequenza può essere
inferiore a quella del publisher; interromperlo dopo pochi secondi. Queste
verifiche non dimostrano da sole che il controller applichi correttamente le
coppie. Se compare il problema X11 del log, H2 termina prima che il viewer si
apra: correggere l'accesso al display e non considerare riuscita la prova.

## 8. Parametri del launch H2

Il launch espone i parametri del parser di `h2_vmc_ros.py` usando
`nome:=valore`. Il gripper usa **lo stesso file launch** ma un'altra
invocazione con `start_h2:=false`; la camera usa il suo launch separato. La
tabella riporta **i valori del comando reale della sezione 6**, non i default
del file launch: usare il comando completo per riprodurre questa configurazione.
I default delle due molle, del damper, di `link_length`, `target_radius` e
`gripper_force` sono ora allineati al parser reale. Restano da specificare i
topic del robot: il default `torque_pub_topic` del launch è ancora
`/vmc/h2_dry_run_torques`; il comando della sezione 6 lo sostituisce con
quello del controller reale.

| Argomento | Valore nella sezione 6 | Significato |
|---|---|---|
| `start_h2` | `true` | Avvia H2 anziché il gripper |
| `python_executable` | `/opt/venv/bin/python` | Python VMC nella shell `nsenter` |
| `sub_topic` | `/robot1/franka/joint_states` | Stati reali letti da H2 |
| `torque_pub_topic` | `/robot1/torque_subscriber_controller/external_torques` | Coppie inviate al controller reale |
| `camera_topic` | `/vision/aruco_pose` | Posa target della Zivid |
| `gripper_force` | `30` | Registro di forza Robotiq rFR, intero 1-255, non Newton |
| `pub_rate` | `1000.0` Hz | Frequenza ROS richiesta dal launch |
| `link_length` | `0.8` m | Distanza TCP–site superiore |
| `stiffness` | `2000` N/m | Rigidezza della prima molla per collegamento |
| `stiffness2` | `20` N/m | Rigidezza della seconda molla in parallelo |
| `damping` | `50` Ns/m | Smorzamento per collegamento nella prima serie |
| `max_force` | `2` N | Saturazione per componente della prima molla |
| `max_force2` | `8` N | Saturazione per componente della seconda molla |
| `timestep` | `0.001` s | Intervallo di pacing del loop Python |
| `state_timeout` | `0.2` s | Soglia di stato robot scaduto |
| `duration` | `0` | Continuo fino a stop |
| `headless` | `false` | Apre il viewer |
| `target_radius` | `0.015` m | Raggio delle sfere che mostrano i target nel viewer |

Il parser legge i valori all'avvio: dopo una modifica occorre fermare e
rilanciare H2.

Per controllare gli argomenti disponibili senza avviare H2, dalla shell VMC:

```bash
ros2 launch /workspace/VMControl/FR3/h2_vmc.launch.py --show-args
```

Con il viewer attivo, H2 mostra i tre riferimenti `StaticObject` come sfere:
ciano per il punto superiore, verde per il sinistro e rosa per il destro.
Una sfera gialla indica il TCP desiderato, al centro dei due target laterali.
Il raggio predefinito e 1,5 cm; si cambia con `target_radius:=0.02` nel launch
oppure `--target-radius 0.02` nello script Python. Le sfere seguono i
riferimenti attivi anche in `HOLD`. Sono elementi grafici della scena utente:
non aggiungono masse, collisioni o modifiche alle tolleranze del controllo.

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

## 10. Simulazione H2 sul Mac con ROS e viewer

Questa sezione riguarda `h2_simulations.py` sul Mac. Il robot e il gripper
sono simulati: i topic `/mujoco/h2/joint_states` e `/mujoco/h2/torques`
servono alla diagnostica e non sono i topic del controller reale. Il marker
giallo sostituisce la camera e si può trascinare nel viewer. ROS, MuJoCo e il
viewer girano nello **stesso container**. Lo schermo virtuale viene mostrato
dal Mac tramite Condivisione schermo; XQuartz non è necessario.

Preparare una volta l'immagine grafica dal terminale del Mac:

```bash
cd "/Users/lorenzotucci/Desktop/Progetto Tesi/MuJoCo Projects/ROS2"
docker build --target gui -t vmc-ros-gui:humble .
```

Avviare la simulazione dal terminale del Mac:

```bash
docker run --rm -d --name h2-vmc-gui \
  -p 127.0.0.1:5901:5900 \
  -v "/Users/lorenzotucci/Desktop/Progetto Tesi:/workspace" \
  -w "/workspace/MuJoCo Projects/FR3" \
  -e PYTHONPATH=/workspace/mujoco-vmc-main \
  -e ROS_DOMAIN_ID=42 \
  -e ROS_LOCALHOST_ONLY=1 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  vmc-ros-gui:humble bash -lc '
set -e
Xvfb :99 -screen 0 1280x800x24 +extension GLX -nolisten tcp -noreset >/tmp/h2-xvfb.log 2>&1 &
for i in $(seq 1 50); do test -S /tmp/.X11-unix/X99 && break; sleep 0.1; done
test -S /tmp/.X11-unix/X99
export DISPLAY=:99
fluxbox >/tmp/h2-fluxbox.log 2>&1 &
viewer_password=$(python3 -c "import secrets; print(secrets.token_hex(4))")
x11vnc -storepasswd "$viewer_password" /tmp/h2-vnc.pass >/dev/null 2>&1
printf "VNC password: %s\n" "$viewer_password"
x11vnc -display :99 -forever -shared -rfbauth /tmp/h2-vnc.pass -rfbport 5900 -listen 0.0.0.0 -noxdamage >/tmp/h2-vnc.log 2>&1 &
source /opt/ros/humble/setup.bash
exec python3 -u h2_simulations.py --duration 0
'
```

La porta VNC è pubblicata soltanto su `127.0.0.1` del Mac. Per leggere la
password temporanea e verificare l'avvio:

```bash
docker logs h2-vmc-gui
```

Aprire l'app **Condivisione schermo** del Mac, scegliere **Nuova connessione**,
inserire `vnc://127.0.0.1:5901` e usare la password stampata nel log. La
finestra MuJoCo appare dentro la sessione VNC. Se la porta 5901 è occupata,
scegliere un'altra porta locale cambiando soltanto il primo `5901` in `-p` e
nell'indirizzo VNC.

Per leggere una coppia pubblicata, da un altro terminale del Mac:

```bash
docker exec -it h2-vmc-gui bash -lc \
  'source /opt/ros/humble/setup.bash && ros2 topic echo /mujoco/h2/torques --once --qos-reliability best_effort'
```

Terminare la prova con `docker stop h2-vmc-gui`; `--rm` rimuove il container
fermo. L'immagine `vmc-ros-gui:humble` rimane disponibile per le prove future.

## 11. PC2 — camera reale e pick simulato in MuJoCo

`h2_camera_simulation.py` riceve `/vision/aruco_pose` dalla Zivid e controlla
un FR3 con Robotiq interamente simulato. Le coppie vengono assegnate a
`qfrc_applied` e applicate con `mj_step`; la chiusura modifica solo il gripper
MuJoCo. Lo script non pubblica comandi di coppia o di chiusura e non legge gli
stati del robot reale. Per questa prova non servono il controller sul PC1 o
il nodo del gripper fisico.

Portare sul PC2 `h2_camera_simulation.py`. Il file mantiene la struttura di
`h2_vmc_ros.py`: funzioni geometriche con gli stessi nomi, connessioni
`conn_att`, `conn_left`, `conn_right` esplicite nel `main`, poi
`step_iteration()`. Include anche `conn_att_2`, `conn_left_2`, `conn_right_2`
e `mechanism_2`: tre molle aggiuntive in parallelo ai primi tre collegamenti.
La prima serie ha molla e damper, la seconda ha solo la molla. Nel ciclo
vengono applicati entrambi i meccanismi, sommando le coppie. Non importa piu
codice da `h2_simulations.py`. Servono anche il modello FR3/Robotiq con le
mesh e la libreria `mujoco_vmc` gia usati sul PC2.

Avviare la camera dall'host PC2 come nella sezione 4; se e gia in esecuzione,
usare quella istanza. La posa deve essere in metri con `frame_id="base"`.
Poi aprire la shell VMC del PC2 seguendo la sezione 5, incluso il profilo UDP,
con dominio 42 e `ROS_LOCALHOST_ONLY=0`. Verificare da quella shell:

```bash
ros2 topic echo /vision/aruco_pose --once
```

Per aprire la simulazione, nella stessa shell VMC:

```bash
cd /workspace/VMControl/FR3
/opt/venv/bin/python h2_camera_simulation.py \
  --scene /workspace/VMControl/fr3_robotiq_2f_140/fr3_robotiq_2f_140/scene_fr3_robotiq.xml \
  --camera-topic /vision/aruco_pose \
  --stiffness 600 \
  --stiffness2 2000 \
  --damping 30 \
  --max-force 10 \
  --max-force2 20 \
  --duration 0
```

`--stiffness2` e `--max-force2` controllano solo le molle della seconda serie;
i default sono rispettivamente 2000 N/m e 20 N per componente. Per escludere
il loro contributo durante un confronto si puo usare `--stiffness2 0`.
Per confrontare simulazione e controllo reale, impostare esplicitamente
gli stessi guadagni di entrambe le serie: i default dei due parser possono
essere diversi.

Correzione geometrica verificata con la posa camera del 16 novembre: il
marker ha asse +z quasi parallelo a -z della base. Il punto superiore del
gripper si trova lungo -z del TCP, quindi il suo target viene costruito con
`p_top = p_target - L * z_aruco`, per avere TCP +z allineato a marker +z.
Con quella posa, il precedente segno `+` metteva il target superiore a
z=-0,331 m e richiedeva un assetto con il corpo del gripper sotto il piano;
la simulazione restava in contatto con il pavimento. Il segno `-` mette il
target superiore a z=0,469 m. La formula e stata allineata sia nel file di
simulazione sia in `h2_vmc_ros.py`. La correzione e stata verificata nella
simulazione; la prova con il robot fisico resta da eseguire.

Il viewer e attivo per default. Usa il display Linux del PC2; se ricompare
l'errore GLFW/X11, seguire il paragrafo "Viewer X11" della sezione 5. Per una
prima verifica del topic senza grafica si puo aggiungere
`--headless --duration 10` allo stesso comando (sostituendo `--duration 0`).

Il robot simulato parte dal keyframe `home`. Come nel file reale, il `main`
attende la prima posa camera valida prima di creare i tre collegamenti e
aprire il viewer. Finche compare `[WAIT]`, non si e ancora entrati nel loop
di controllo. Dopo, la sequenza e:

1. Aggiornare i tre target dalla camera, usando il segno corretto per il
   punto superiore descritto sopra.
2. Copiare q e dq del robot simulato nella copia virtuale ed eseguire `mj_forward`.
3. Sommare le coppie VMC dei due meccanismi, aggiungere il bias del modello
   dinamico e limitare le coppie ai limiti dei giunti.
4. Eseguire `mj_step` sul robot simulato.
5. Richiedere la chiusura simulata quando TCP e tre collegamenti restano entro
   1 cm per 0,5 s con una posa camera recente.

La simulazione e allineata al mantenimento della posa di `h2_vmc_ros.py`:
durante `APPROACH`, oltre 0,8 s senza una posa valida, salva una sola volta
le posizioni correnti dei tre site e le mantiene come riferimenti fissi.
Entrambi i meccanismi restano attivi e la dinamica MuJoCo continua a evolvere
con le coppie ricalcolate verso quella posa. Il log mostra `target_mode=HOLD`;
gli errori si riferiscono alla posa salvata, non al marker non osservato.
La chiusura e inibita anche se questi errori diventano piccoli. Alla nuova
posa valida si riprende automaticamente l'inseguimento dell'ArUco, con
`target_mode=ARUCO`. Durante `CLOSING` le nuove pose continuano ad aggiornare
i target e non annullano la chiusura in entrambi gli script. Questi eventi
sono stampati insieme a un riepilogo degli errori una volta al secondo.

`CLOSED` conferma soltanto che le dita simulate hanno raggiunto l'angolo di
chiusura. La camera non ricostruisce automaticamente un oggetto fisico in
MuJoCo: questa versione verifica avvicinamento e chiusura, non il contatto o
la qualita della presa. Inoltre la configurazione iniziale e la dinamica
MuJoCo possono differire dal FR3 reale.

Verifica geometrica e dinamica del 16 novembre, senza ROS o viewer: usando
la posa ArUco comunicata da Lorenzo e i guadagni 20/1500 N/m, damping 20 Ns/m,
limiti elastici 10/2 N, il vecchio segno ha riprodotto l'errore TCP di
57,5 mm, il collegamento superiore lungo 742,5 mm e i contatti col pavimento.
Con il segno corretto, lo script e arrivato a `CLOSED`, con errore TCP di
circa 0,16 mm e massimo errore dei collegamenti di circa 6,7 mm, senza
contatti col pavimento alla fine della prova. In questa verifica l'ingresso
ROS e il clock sono stati sostituiti localmente con la posa ricevuta e un
clock di test. Le verifiche precedenti su target sintetici hanno controllato
la somma delle coppie, il timeout camera e l'aggiornamento durante la
chiusura. Il nuovo comportamento va riprovato con camera e viewer sul PC2.

I valori numerici di questa verifica storica sono stati ottenuti con offset
di pick nullo. Precedono l'aggiunta dell'offset di 1,5 cm e non costituiscono
una verifica hardware della nuova quota di pick.

Verificato localmente anche il nuovo `HOLD`, con dinamica MuJoCo e ingresso
camera/clock sostituiti: acquisizione unica dei tre site dopo il timeout,
riferimenti fissi mentre il robot continua a evolvere, entrambi i meccanismi
attivi, nessuna chiusura durante la perdita e ripresa al ritorno delle pose.
Verificato che la perdita della camera durante `CLOSING` non attivi `HOLD`
e che la chiusura simulata prosegua.
