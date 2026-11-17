# H3 — avvio sul PC2 e pick-and-place ripetuto

Questa guida riguarda:

- [zivid_camera_h3.py](zivid_camera_h3.py): camera Zivid reale, due marker.
- [h3_simulations.py](h3_simulations.py): camera reale e FR3/Robotiq simulati.
- [h3_pick_place.py](h3_pick_place.py): controllo del FR3 e del gripper fisici.

**Per la simulazione segui le sezioni 1 → 2 → 3 → 4.**
Per il robot fisico aggiungi i prerequisiti della sezione 5, rispettando
l'ordine **PC1 pronto → gripper aperto → camera H3 attiva → H3 reale**.

I comandi avviano direttamente i file H3. I launch
`h2_vmc.launch.py` e `zivid_camera_h2.launch.py` continuano ad avviare H2.

## 1. Ambienti, percorsi e marker

| Dove | Processo | Python |
|---|---|---|
| PC2 host, terminale camera | Camera H3, per entrambe le prove | `/home/lorenzoantonio/venvs/zivid/bin/python` |
| PC2, shell VMC con `nsenter` | Simulazione H3 oppure H3 reale, con viewer | `/opt/venv/bin/python` |
| PC2 host, terminale gripper | Gripper fisico, solo per H3 reale | `/home/lorenzoantonio/venvs/vmc/bin/python` |
| PC1 | Driver e `torque_subscriber_controller`, solo per H3 reale | Ambiente ROS del laboratorio |

La simulazione richiede soltanto camera e shell VMC. Non richiede controller
PC1, stati del FR3 o nodo del gripper fisico.

Il repository sull'host PC2 è `/home/lorenzoantonio/VMControl`, montato nel
container `fr3-vmc` come `/workspace/VMControl`.

Il modello ha la doppia cartella:

`/workspace/VMControl/fr3_robotiq_2f_140/fr3_robotiq_2f_140/scene_fr3_robotiq.xml`

| Riferimento | ID e dizionario configurati nella camera | Topic |
|---|---|---|
| Cubo | `16`, `aruco5x5_1000` | `/vision/aruco_pose` |
| Tavolo | `20`, `aruco6x6_50` | `/vision/aruco_table_pose` |

Entrambi i topic pubblicano `geometry_msgs/msg/PoseStamped`, con
`header.frame_id="base"` e posizione in metri. I marker stampati devono
corrispondere agli ID e ai dizionari sopra. Le due rilevazioni sono indipendenti:
quando un marker manca, la camera può continuare a pubblicare l'altro.

## 2. PC2 host — avviare la camera H3

Usa un terminale sull'host PC2. Controlla le istanze già attive:

```bash
pgrep -af '[z]ivid_camera_h[23].py'
```

Se una camera è già attiva, identifica il suo terminale prima di avviarne
un'altra. Per H3 serve lo script che pubblica entrambi i topic.

Nel terminale camera:

```bash
cd /home/lorenzoantonio
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

/home/lorenzoantonio/venvs/zivid/bin/python -u \
  /home/lorenzoantonio/VMControl/FR3/zivid_camera_h3.py
```

La directory iniziale deve essere `/home/lorenzoantonio`: i settings sono letti
dal percorso relativo
`Scrivania/Tesi/First_Ex_LAB_VMC_OmarLibrary_27-10/zivid_camera_settings.yml`.

La calibrazione attualmente usata da H3 è:

`/home/lorenzoantonio/zivid_ws/calib_board_2026-11-12_1628/T_cam_to_base_Zivid_board.npy`

Questi sono i percorsi presenti nello script; la loro esistenza sul PC2 va
confermata dall'avvio. Lascia aperto il terminale camera.

Da un altro terminale host, verifica entrambe le pose:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

ros2 topic echo /vision/aruco_pose --once
ros2 topic echo /vision/aruco_table_pose --once
```

Ogni comando attende che venga visto il relativo marker. Controlla frame e
unità prima di proseguire.

## 3. PC2 — shell VMC, rete e viewer

### 3.1 Accesso al display dall'host

Per usare il viewer, esegui questi comandi da un terminale del desktop PC2,
come utente `lorenzoantonio`:

```bash
echo "$DISPLAY"
xhost +SI:localuser:root
```

Annota il valore del display: servirà nella shell VMC. Per una prova
`--headless` il display non serve.

### 3.2 Entrare nella shell VMC

Da un nuovo terminale host:

```bash
docker inspect fr3-vmc --format 'PID={{.State.Pid}} Stato={{.State.Status}}'
```

Se il container è fermo, avvialo con `docker start fr3-vmc`, poi rileggi il PID.
Quando è in esecuzione:

```bash
vmc_pid=$(docker inspect -f '{{.State.Pid}}' fr3-vmc)
sudo nsenter --target "$vmc_pid" --mount --pid --root --wd /bin/bash
```

Il comando usa filesystem e Python del container, mantenendo la rete
dell'host. La procedura documentata usa `nsenter` senza `--net`;
un normale `docker exec` usa invece la rete bridge del container.

**I comandi seguenti si eseguono dentro la shell VMC:**

```bash
cd /workspace/VMControl
source /opt/ros/humble/setup.bash
source /opt/venv/bin/activate
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export MUJOCO_MENAGERIE_PATH=/workspace/VMControl/fr3_robotiq_2f_140

test -f "$MUJOCO_MENAGERIE_PATH/fr3_robotiq_2f_140/scene_fr3_robotiq.xml"
```

La variabile indica la cartella **esterna**: il file reale H3 aggiunge
`fr3_robotiq_2f_140/scene_fr3_robotiq.xml`. La simulazione accetta anche
il percorso completo tramite `--scene`.

### 3.3 Profilo Fast DDS UDP

Nella stessa shell VMC, crea il profilo già usato nella procedura H2:

```bash
vmc_udp_profile=$(mktemp /tmp/vmc-h3-udp.XXXXXX)
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

Gli identificativi interni `h2_udp` riutilizzano lo stesso profilo di
[h2_readme.md](h2_readme.md). Le interfacce elencate sono quelle della
configurazione PC2 documentata; se la rete cambia, verificare gli indirizzi.

Ogni nuova shell VMC richiede nuovamente source, export e profilo.

### 3.4 Display e verifica degli ingressi

Nella shell VMC:

```bash
echo "$DISPLAY"
```

Deve corrispondere al valore letto sul desktop host. Se il display host è
`:0`, il comando da usare nella shell VMC è:

```bash
export DISPLAY=:0
```

Se il desktop usa un altro valore, sostituisci `:0` con quello effettivo.

Verifica ora la ricezione da questa shell:

```bash
ros2 topic echo /vision/aruco_pose --once
ros2 topic echo /vision/aruco_table_pose --once
```

Per la simulazione questi sono gli unici ingressi ROS richiesti.
La verifica degli stati del robot appartiene alla sezione 5.

## 4. PC2 — camera reale e robot simulato

Nella shell VMC preparata alla sezione 3:

```bash
cd /workspace/VMControl/FR3
/opt/venv/bin/python -u h3_simulations.py \
  --scene /workspace/VMControl/fr3_robotiq_2f_140/fr3_robotiq_2f_140/scene_fr3_robotiq.xml \
  --camera-topic /vision/aruco_pose \
  --camera-table-topic /vision/aruco_table_pose \
  --link-length 0.8 \
  --stiffness 600 \
  --stiffness2 2000 \
  --damping 30 \
  --max-force 10 \
  --max-force2 20 \
  --restart-distance 0.05 \
  --restart-dwell 0.5 \
  --duration 0
```

Il comando esplicita i guadagni usati nella prova dinamica locale. I default
dei due file H3 restano diversi da questo comando:

| Parametro | Default dei due script H3 | Comando simulazione sopra |
|---|---:|---:|
| `--stiffness` | 2000 | 600 |
| `--stiffness2` | 20 | 2000 |
| `--damping` | 50 | 30 |
| `--max-force` | 2 | 10 |
| `--max-force2` | 8 | 20 |
| `--link-length` | 0.8 m | 0.8 m |

Nel caso locale provato, i default H3 rendevano il trasferimento troppo lento
per completare il ciclo nel tempo della prova. I valori del comando hanno
completato due cicli con dinamica MuJoCo; ciò non li valida sul robot fisico.

Il viewer si apre dopo la prima posa valida del cubo. Se compare `[WAIT]`,
lo script sta ancora aspettando quella posa. La posa del tavolo sarà
necessaria per iniziare `TRANSFER`.

Per una prova senza viewer aggiungi `--headless` e sostituisci
`--duration 0` con una durata finita, ad esempio `--duration 20`.
La durata parte dopo l'inizializzazione e l'attesa della prima posa.

### Cosa viene simulato

- `data_sim` evolve con `mj_step`; `data` calcola il VMC dalla sua q e dq.
- Le coppie dinamiche sommano VMC e bias del modello e rispettano i limiti
  articolari della scena.
- La chiusura e l'apertura muovono soltanto le dita MuJoCo.
- `GRASPED` è un feedback emulato dal raggiungimento dell'angolo di chiusura.
- Il modello non contiene automaticamente il cubo o il tavolo fisici:
  questa prova verifica movimento e sequenza, non contatti o forza di presa.

### Ripetere il ciclo in simulazione

Quando compare `WAIT_FOR_MOVE`, sposta il marker fisico del cubo di almeno
5 cm rispetto alla sua posizione osservata alla fine del ciclo. Devono
arrivare nuove osservazioni dello spostamento per almeno 0,5 s.

Il robot virtuale non può spostare il cubo fisico: per questo il riferimento
di riavvio è l'ultima posizione reale osservata, anziché il punto virtuale
di deposito. Lasciare il marker fermo mantiene la simulazione in attesa.

## 5. H3 sul robot fisico

Questa sezione riguarda `h3_pick_place.py`, che può comandare FR3 e gripper
fisici. La prova completa H3 sul sistema reale resta da validare.

Prepara gli elementi nell'ordine **PC1 → gripper aperto → camera → H3 reale**.
Se hai appena eseguito la simulazione, termina quel processo prima di
avviare H3 reale; puoi riutilizzare l'istanza camera H3.

### 5.1 PC1 — driver e controller

Prepara driver e `torque_subscriber_controller` secondo la procedura del
laboratorio descritta in [h2_readme.md](h2_readme.md), sezione 2.
Se sono già attivi, verifica quella istanza.

Nel terminale ROS del PC1, con gli ambienti del driver già caricati:

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
ros2 control list_controllers
ros2 topic info /robot1/franka/joint_states --verbose
ros2 topic info /robot1/torque_subscriber_controller/external_torques --verbose
```

Conferma il controller attivo, gli stati di tipo `sensor_msgs/msg/JointState`
e un subscriber del controller sul topic delle coppie
`std_msgs/msg/Float64MultiArray`. I nomi sopra sono quelli configurati per
robot 1: verificarli sul PC1 prima di ogni prova reale e usare quelli effettivi
nel comando H3.

H3 reale calcola le coppie senza `mj_step` e senza aggiungere `qfrc_bias`.
La convenzione delle coppie e la compensazione di gravità vanno verificate
nel controller PC1 effettivamente in uso.

### 5.2 PC2 host — gripper fisico

In un terminale host, verifica l'istanza del gripper e la porta:

```bash
pgrep -af '[g]ripper_node.py'
ls -l /dev/ttyUSB0
```

Se il nodo è già attivo, verificane stato e versione prima di riutilizzarlo.
Per avviarlo:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

/home/lorenzoantonio/venvs/vmc/bin/python -u \
  /home/lorenzoantonio/VMControl/FR3/gripper_node.py \
  --ros-args -p port:=/dev/ttyUSB0 -p force:=30
```

All'avvio esegue attivazione/calibrazione e apertura. Le stampe
`Activation complete` e `Listening...` non confermano che l'apertura sia
già terminata.

Da un altro terminale host con ROS e dominio 42 preparati:

```bash
ros2 topic echo /gripper/status --once
ros2 topic echo /gripper/current_distance --once
ros2 param get /robotiq_gripper force_runtime_configurable
```

Prima di avviare H3 attendi `gSTA=3`, `gOBJ=3`, `gFLT=0` e distanza vicina
a `0.14 m`. Il vettore di stato è `[gSTA, gOBJ, gFLT, gCU]`.

Il parametro `force_runtime_configurable` deve risultare `True`. Se il
processo gripper è una vecchia istanza, va riavviato con il file aggiornato;
il riavvio esegue nuovamente calibrazione e apertura.

### 5.3 Camera e shell VMC

Avvia la camera come nella sezione 2 e prepara la shell VMC come nella
sezione 3. Nella shell VMC verifica anche lo stato del FR3:

```bash
ros2 topic echo /robot1/franka/joint_states --once
ros2 topic echo /vision/aruco_pose --once
ros2 topic echo /vision/aruco_table_pose --once
ros2 topic info /robot1/torque_subscriber_controller/external_torques --verbose
```

### 5.4 Avvio del controllo reale

Il comando seguente esplicita i **default attuali del parser reale**.
È una configurazione del codice da validare nella prova fisica, distinta
dai guadagni della simulazione della sezione 4.

Esegui nella shell VMC, dopo aver confermato i prerequisiti sopra:

```bash
cd /workspace/VMControl/FR3
export MUJOCO_MENAGERIE_PATH=/workspace/VMControl/fr3_robotiq_2f_140

/opt/venv/bin/python -u h3_pick_place.py \
  --sub-topic /robot1/franka/joint_states \
  --torque-pub-topic /robot1/torque_subscriber_controller/external_torques \
  --camera-topic /vision/aruco_pose \
  --camera-table-topic /vision/aruco_table_pose \
  --gripper-force 30 \
  --link-length 0.8 \
  --stiffness 2000 \
  --stiffness2 20 \
  --damping 50 \
  --max-force 2 \
  --max-force2 8 \
  --state-timeout 0.2 \
  --pub-rate 1000 \
  --restart-distance 0.05 \
  --restart-dwell 0.5 \
  --duration 0
```

Il file reale risolve il modello tramite `MUJOCO_MENAGERIE_PATH`: non espone
l'opzione `--scene` della simulazione.

`--gripper-force 30` imposta il registro Robotiq `rFR`, non una forza in
Newton. H3 verifica il supporto del nodo e configura il parametro prima
di iniziare il calcolo delle coppie. Se il servizio manca o rifiuta il
valore, il controllo non parte.

L'apertura completa deve essere già terminata: la conferma del parametro
di forza non è una conferma di apertura.

## 6. Sequenza, parametri e riavvio

Il ciclo è:

`APPROACH → CLOSING → GRASPED → TRANSFER → OPENING → WAIT_FOR_MOVE → APPROACH`

- **APPROACH:** segue la posa del cubo. TCP e tre collegamenti devono restare
  entro 1 cm per 0,5 s, con dati recenti, prima della richiesta di chiusura.
- **CLOSING:** invia una sola richiesta di chiusura e attende la conferma.
- **GRASPED:** attende una posa recente del tavolo e costruisce il riferimento.
- **TRANSFER:** usa un'unica destinazione fissa: centro del marker tavolo
  più `0.035 m` lungo lo z della base. Controlla continuamente la presa.
- **OPENING:** richiede una sola apertura quando il TCP resta entro 1 cm
  dalla destinazione per 0,5 s; attende il feedback di apertura.
- **WAIT_FOR_MOVE:** mantiene il riferimento e attende uno spostamento del
  marker del cubo osservato dopo il completamento del ciclo.

Il criterio di arrivo al place usa la posizione TCP. L'orientazione finale
del cubo non è una condizione di riuscita; i tre punti virtuali continuano
a dare al gripper il riferimento geometrico costruito dalla posa del tavolo.

La quota di 3,5 cm riguarda il **TCP**. La distanza del fondo del cubo dal
piano dipende anche dalle dimensioni e dalla posizione del cubo nella presa.

| Impostazione | Valore attuale |
|---|---|
| `--restart-distance` | `0.05` m, distanza tridimensionale |
| `--restart-dwell` | `0.5` s, su nuove osservazioni camera |
| `--duration` | `0`, continua fino a interruzione |
| Timeout camera | `0.8` s; configurabile con `--camera-timeout` solo nella simulazione |
| Timeout stato FR3 reale | `--state-timeout 0.2` s |
| Timeout feedback gripper | `0.5` s |
| Timeout chiusura / apertura | `5` s ciascuno |
| Apertura richiesta | `0.140` m; conferma entro `0.005` m |
| `--close-time`, `--open-time` | `1` s ciascuno, solo per le dita simulate |

`--max-force` e `--max-force2` limitano ogni componente della forza elastica
di ciascuna molla. Non limitano il damper o la coppia totale ai giunti.
Nella simulazione le coppie dinamiche sono inoltre limitate ai valori della scena.

Nel file reale la conferma della presa richiede `detected=True`, `gSTA=3`,
`gOBJ=2`, `gFLT=0` e feedback ricevuti dopo la richiesta di chiusura.
Durante il trasporto il feedback deve continuare a essere recente.

L'apertura richiede stato attivo senza guasti, `gOBJ=3` e distanza delle dita
vicina al valore richiesto, entrambi ricevuti dopo il comando di apertura.

I due riferimenti per il riavvio sono:

- **Robot reale:** il punto TCP desiderato al deposito.
- **Simulazione:** l'ultima posizione osservata del marker fisico a fine ciclo.

Servono pose camera recenti, stato robot disponibile e gripper aperto.
Una singola immagine, la sola ricomparsa del marker o un movimento sotto
soglia non bastano a far ripartire il ciclo.

### Timeout e FAILED

Durante `APPROACH`, oltre il timeout della camera vengono fissati i tre
riferimenti sulla posa corrente del gripper. Il VMC mantiene quella posa;
nuove osservazioni valide fanno riprendere l'inseguimento.

Dopo aver scelto il place, la perdita della camera non sposta la destinazione
già fissata. La scadenza dei dati del gripper durante `GRASPED` o `TRANSFER`,
la perdita della conferma della presa o il fallimento di chiusura/apertura
portano a `FAILED`.

In `FAILED` il VMC mantiene una posa misurata e non riavvia automaticamente
il ciclo. Nel reale, con stato robot scaduto pubblica zero coppia VMC
aggiuntiva e fissa la posa al ritorno di dati recenti. Questo mantenimento
è cedevole; non sostituisce l'arresto del sistema fisico.

## 7. Arresto e problemi di avvio

Termina H3 con `Ctrl+C` o chiudendo il viewer. La durata predefinita è
illimitata. Poi termina la camera; nella prova reale completa l'arresto
di gripper e driver con la procedura del laboratorio. H3 non invia
un'apertura automatica alla propria uscita.

Dopo la chiusura del viewer, dal terminale desktop host puoi revocare
l'accesso al display:

```bash
xhost -SI:localuser:root
```

| Sintomo | Controllo da fare |
|---|---|
| `[WAIT]` e viewer assente | Posa cubo in `/vision/aruco_pose`, `frame_id=base`, ingresso visibile dalla shell VMC |
| `GRASPED` senza trasferimento | Posa tavolo recente, marker 20 e dizionario corretti |
| Topic visibili sull'host ma non nella shell VMC | Dominio 42, `ROS_LOCALHOST_ONLY=0`, profilo UDP e uso di `nsenter` |
| `X11: Failed to open display` | Display del desktop e della shell coerenti; `xhost` eseguito sull'host |
| Simulazione ferma a lungo in `TRANSFER` | Errori stampati e guadagni effettivi; confrontare il comando della sezione 4 |
| Errore sui servizi del gripper nel reale | Nodo host aggiornato, servizi raggiungibili e `force_runtime_configurable=True` |
| `WAIT_FOR_MOVE` senza riavvio | Spostamento sopra soglia su nuove pose, gripper aperto e dati recenti |
| `FAILED` | Leggere il motivo stampato; risolvere la causa prima di terminare e rilanciare il controllo |

Le verifiche già svolte sono locali: controlli della logica con dati fittizi
e due cicli dinamici MuJoCo della simulazione con ingresso camera, ROS e clock
sostituiti. Questa guida non attesta una nuova prova completa sul PC2 o sul
robot fisico.
