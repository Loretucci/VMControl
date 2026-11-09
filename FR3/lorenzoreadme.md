# Lorenzo — avvio FR3 VMC + camera Zivid

Aggiornato: 2 novembre 2026. Procedura ricostruita dai comandi e dagli output
della sessione. File creato sul PC1; non si sincronizza automaticamente sul PC2.
Non modifica gli script Python.

## 1. Architettura e terminali

| PC / terminale | Funzione | Ambiente |
|---|---|---|
| PC1, terminale controllo | Driver FR3, julia_torque_controller e controllo già preparato | Container franka_ros2 |
| PC2, terminale A | VMC (nelle prove era il terzo terminale) | Shell nsenter, filesystem container e rete host |
| PC2, terminale B | Camera e verifiche | Host, utente lorenzoantonio |
| PC2, terminale C opzionale | Lettura posa / diagnostica | Host o nsenter, specificato di volta in volta |

PC1: pacmate-MD34400. PC2 camera/VMC: pacmate22.
ROS 2 Humble, ROS_DOMAIN_ID=0, ROS_LOCALHOST_ONLY=0 sul PC2.

Rete osservata: PC1 192.168.10.2, PC2 192.168.10.3; robot 192.168.0.2,
collegato al PC1 (192.168.0.1). Verificare gli indirizzi se cambia il cablaggio.

| Topic | Tipo / funzione |
|---|---|
| /robot2/franka/joint_states | sensor_msgs/msg/JointState, stati reali |
| /vision/aruco_pose | geometry_msgs/msg/PoseStamped, frame base |
| /vmc/dry_run_torques | std_msgs/msg/Float64MultiArray, sola prova |
| /robot2/julia_torque_controller/external_torques | std_msgs/msg/Float64MultiArray, comanda il robot |

Il viewer è integrato in h1_vmc_ros.py. Non servono un secondo viewer o uno
script di sincronizzazione. Non usare --mock-stream per queste prove.

## 2. PC1 — controllo del robot

Se il controllo è già funzionante, lasciarlo invariato: NON avviare una seconda
istanza e NON interrompere o sostituire eventuali processi del laboratorio che
mantengono il flusso di controllo e la compensazione di gravità.

Solo per avvio da sistema fermo, seguire la procedura del laboratorio: robot
preparato, FCI attivo, area libera e stop presidiato. Questa guida non sostituisce
la preparazione fisica del robot.

PC1, terminale controllo, host: controllare il container esistente.

```bash
docker ps -a --format 'table {{.Names}}\t{{.Status}}'
```

Solo se fermo:

```bash
docker start franka_ros2
```

PC1, terminale controllo, host: entrare.

```bash
docker exec -it franka_ros2 bash
```

PC1, stesso terminale, dentro il container, solo se non esiste già un bringup:

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch franka_bringup example.launch.py \
    controller_name:=julia_torque_controller \
    robot_config_file:=/ros2_ws/src/franka_bringup/config/franka_yi.config.yaml
```

Lasciare aperto. Non omettere il file YAML: quello predefinito può configurare
un altro robot. Il comando non documenta né avvia eventuali ulteriori componenti
di gravity compensation personalizzati: mantenerne la procedura già funzionante.
Lo script VMC non aggiunge gravità; non dedurre la convenzione dal solo nome del
controller. Nel sorgente letto julia_torque_controller copia i sette valori sulle
interfacce effort e, dopo 100 ms senza messaggi, comanda zero. Questo timeout
non è un sostituto dello stop o della procedura di arresto del laboratorio.

## 3. PC2 — file e ambienti esistenti

Host:
- Progetto: /home/lorenzoantonio/VMControl
- Camera: ~/venvs/zivid/bin/python (SDK Zivid, ROS, SciPy verificati)
- ~/VMControl/.venv non aveva Zivid né MuJoCo nelle verifiche eseguite.
- ~/venvs/vmc aveva ROS e MuJoCo, ma non un import funzionante di RobotROSBridge
  dalla copia locale della libreria. Non è l'ambiente usato per la prova finale.

Container fr3-vmc:
- Progetto: /workspace/VMControl, bind mount in sola lettura dall'host.
- Python usato: /opt/venv/bin/python.
- MuJoCo e mujoco-vmc installati nel venv.
- Il container originale usa rete bridge. La procedura nsenter sotto mantiene
  la rete dell'host per il processo VMC senza ricreare il container.

Non ricreare i container per il normale utilizzo: le installazioni aggiunte nel
container potrebbero non essere salvate nell'immagine.

### Aggiornamento facoltativo dei soli due script

PC2, terminale B host, con VMC/camera fermi e dopo aver salvato eventuali modifiche:

```bash
git -C ~/VMControl status --short
git -C ~/VMControl fetch origin
git -C ~/VMControl restore --source=origin/main -- FR3/h1_vmc_ros.py FR3/zivid_camera.py
```

restore sovrascrive le modifiche locali ai due file. Non aggiorna automaticamente
modelli o librerie richiesti da nuove versioni: ricontrollare eventuali dipendenze.
La procedura seguente si riferisce alla versione provata in sessione.

## 4. PC2 — accesso al VMC e profilo UDP

PC2, terminale A inizialmente host:

```bash
docker inspect fr3-vmc --format 'PID={{.State.Pid}} Stato={{.State.Status}}'
```

Se fermo, avviarlo e rileggere il PID:

```bash
docker start fr3-vmc
docker inspect fr3-vmc --format 'PID={{.State.Pid}} Stato={{.State.Status}}'
```

PC2, terminale A host: sostituire PID_CORRENTE con il numero appena ottenuto.
Non riutilizzare 59749, che era soltanto il PID della sessione documentata.

```bash
sudo nsenter --target PID_CORRENTE --mount --pid --root --wd /bin/bash
```

La nuova shell è root, usa filesystem e venv del container, ma mantiene la rete
host perché non si specifica --net. Non è equivalente all'isolamento Docker
standard. Non cambia globalmente la configurazione di rete del container.

PC2, terminale A, dentro questa shell:

```bash
cd /workspace/VMControl
source /opt/ros/humble/setup.bash
source /opt/venv/bin/activate
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

vmc_test_profile=$(mktemp /tmp/vmc-udp-test.XXXXXX.xml)
cat > "$vmc_test_profile" <<'XML'
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>test_udp</transport_id>
      <type>UDPv4</type>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="test_udp_profile" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>test_udp</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
XML
```

Tenere questa shell: vmc_test_profile è una variabile locale al terminale.
Il file temporaneo può sparire al riavvio; ricrearlo ogni nuova sessione.
Passare FASTRTPS_DEFAULT_PROFILES_FILE a ogni avvio VMC, come sotto.
Non basta attivare il venv.

## 5. PC2 — verifiche prima dell'avvio

PC2, terminale B host:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
pgrep -af '[h]1_vmc_ros.py|[z]ivid_camera.py'
ros2 topic list -t --no-daemon --spin-time 5
ros2 topic info /robot2/julia_torque_controller/external_torques --verbose
```

Non avviare due VMC. Se esistono publisher reali, identificarli prima di agire:
non sostituire o terminare altri componenti di controllo già funzionanti.
Nella prova documentata il topic reale aveva un subscriber e zero publisher.

PC2, terminale B host, controllo del valore h nel file effettivo:

```bash
rg -n 'h =|reference_position|stiffness|damping|max-force' ~/VMControl/FR3/h1_vmc_ros.py
```

Nella copia provata h = 0.0; riferimento iniziale della punta (0.4, 0, 0.1) m.

## 6. Dry run: coppie isolate dal robot

PC2, terminale A nella shell nsenter con profilo UDP:

```bash
FASTRTPS_DEFAULT_PROFILES_FILE="$vmc_test_profile" \
PYTHONDONTWRITEBYTECODE=1 \
/opt/venv/bin/python FR3/h1_vmc_ros.py \
    --sub-topic /robot2/franka/joint_states \
    --torque-pub-topic /vmc/dry_run_torques \
    --camera-topic /vision/aruco_pose \
    --stiffness 2000 --damping 20 --max-force 20 \
    --headless --duration 0
```

Attendere Received initial real robot joint state. La riga Publishing computed
torques deve riportare ESATTAMENTE /vmc/dry_run_torques.
duration 0 significa esecuzione continua, non una prova di durata zero.

PC2, terminale B host:

```bash
ros2 topic info /vmc/dry_run_torques --verbose
ros2 topic info /robot2/julia_torque_controller/external_torques --verbose
ros2 topic info /vision/aruco_pose --verbose
```

Per la configurazione documentata richiedere:
- Dry run: 1 publisher, 0 subscriber.
- Topic reale: 0 publisher, 1 subscriber julia_torque_controller.
- Visione: subscriber PoseStamped presente.

Fermarsi se il topic di prova ha destinatari non identificati o il VMC pubblica
sul topic reale. Non usare topic echo sulle coppie mentre si verifica il conteggio
zero subscriber: echo aggiunge un subscriber diagnostico.
Unknown topic prima dell'avvio non dimostra né un errore né l'isolamento finale.

## 7. Acquisizione camera (dry run o prova reale già preparata)

PC2, terminale B host: verifiche facoltative delle dipendenze e dei file.

```bash
~/venvs/zivid/bin/python -B -c 'import zivid, rclpy; from scipy.spatial.transform import Rotation; from geometry_msgs.msg import PoseStamped; print("Import camera OK")'
ls -l \
  ~/Scrivania/Tesi/First_Ex_LAB_VMC_OmarLibrary_27-10/zivid_camera_settings.yml \
  ~/zivid_ws/calib_2026-10-21/calibration_outputs/T_cam_to_base_Zivid.npy
```

PC2, terminale B host, acquisizione:

```bash
cd /home/lorenzoantonio
~/venvs/zivid/bin/python -B ~/VMControl/FR3/zivid_camera.py
```

Avviare dalla home: lo script carica le impostazioni con percorso relativo
Scrivania/Tesi/First_Ex_LAB_VMC_OmarLibrary_27-10/zivid_camera_settings.yml.
La calibrazione ha invece percorso assoluto. Non confondere l'esistenza del
file con la validazione geometrica della calibrazione.

Marker: aruco5x5_1000, ID 16. Posizione convertita da mm a metri, frame base;
quaternione ROS [x,y,z,w], convertito dal VMC in [w,x,y,z].
Lo script acquisisce una volta, aspetta un subscriber, pubblica e termina.
Per una nuova acquisizione, rilanciare lo stesso comando. Non è uno streaming.
Se il marker non è rilevato o manca la connessione camera, fermarsi sull'errore.

Il log Pubblicato ArUco non dimostra da solo che il VMC abbia elaborato la posa.
Nel terminale A verificare il cambiamento di errore posizione/orientamento.
Con h=0 il target della punta coincide con la posizione del marker.
Anche con robot fermo l'errore cambia se cambia il target.

## 8. Prova reale — procedura eseguita in sessione

QUESTI COMANDI INVIANO COPPIE AL ROBOT. Non sono parte del dry run.
La prova è stata dichiarata riuscita dall'operatore con stiffness 2000,
damping 20, max-force 20. Questo non garantisce stabilità per altre pose,
carichi, calibrazioni o condizioni di rete.

Una precedente prova con damping 50 aveva prodotto forti vibrazioni e arresto
manuale. Non usare i default senza ricontrollare: il default damping letto era 50.
max-force 20 limita ogni componente di ciascuna molla, NON le coppie totali;
lo smorzamento non è saturato da quel parametro.

Prima della prova: area libera, stop presidiato, controllo PC1 già funzionante,
posa iniziale e posa marker previste. Non intervenire sul flusso di controllo PC1.

PC2, terminale A: interrompere SOLO il dry run con Ctrl+C e attendere il prompt.
PC2, terminale B host: verificare che non restino istanze VMC.

```bash
pgrep -af '[h]1_vmc_ros.py'
ros2 topic info /robot2/julia_torque_controller/external_torques --verbose
```

Nel caso documentato: nessuna istanza VMC e zero publisher sul topic reale.
Non proseguire con publisher concorrenti o istanze non identificate.

PC2, terminale A nella stessa shell nsenter, con profilo UDP ancora presente:

```bash
test -r "$vmc_test_profile" && cat "$vmc_test_profile"
```

Se il file non è leggibile, ricrearlo al punto 4 prima di proseguire.
PC2, terminale A, avvio reale:

```bash
FASTRTPS_DEFAULT_PROFILES_FILE="$vmc_test_profile" \
PYTHONDONTWRITEBYTECODE=1 \
/opt/venv/bin/python FR3/h1_vmc_ros.py \
    --sub-topic /robot2/franka/joint_states \
    --torque-pub-topic /robot2/julia_torque_controller/external_torques \
    --camera-topic /vision/aruco_pose \
    --stiffness 2000 \
    --damping 20 \
    --max-force 20 \
    --headless --duration 0
```

Sequenza effettiva del codice:
1. Appena riceve gli stati robot, applica il VMC verso la punta (0.4, 0, 0.1) m
   e l'orientamento definito dai tre target iniziali. Può muovere subito.
2. Attendere una risposta stabile. Non acquisire durante oscillazioni.
3. PC2, terminale B host: eseguire il comando camera del punto 7.
4. La pubblicazione cambia subito posizione E orientamento target verso il marker.
   Non esiste una rampa o un'abilitazione separata dopo la posa.
5. Per comportamento inatteso usare lo stop del laboratorio, non attendere
   il timeout ROS o la chiusura del viewer.

La camera trasmette una misura, non una traiettoria interpolata. Non acquisire
una posa arbitraria supponendo che la risposta sia identica alla posa già provata.

## 9. Viewer integrato

--headless disabilita intenzionalmente il viewer. Nei comandi finali della
sessione il viewer non si apriva per questo motivo.

Per usarlo, scegliere prima dell'avvio lo stesso comando VMC appropriato
(dry run o reale), togliendo --headless e mantenendo --duration 0.
Il viewer appartiene allo stesso processo; non è attivabile a caldo.
Non avviare una seconda istanza per aggiungerlo.

IMPORTANTE: riavviare il VMC perde la posa camera memorizzata e riparte dal
riferimento fisso. Nella prova reale non riavviarlo solo per aprire il viewer
senza preparare questo movimento e la gestione del flusso di controllo.

In precedenza era stato necessario autorizzare root sul display X11.
Solo se compare X11: Failed to open display / Authorization required:

PC2, terminale B host, utente della sessione grafica:

```bash
echo "$DISPLAY"
xhost +SI:localuser:root
```

PC2, terminale A nsenter, prima dell'avvio con viewer: verificare DISPLAY;
se assente, impostarlo al valore appena letto sull'host (in sessione era :0).
Non usare un valore diverso da quello effettivo della sessione grafica.

```bash
echo "$DISPLAY"
```

L'autorizzazione è locale a root e alla sessione grafica, non una modifica dei
file Python. A viewer terminato, PC2 terminale B host:

```bash
xhost -SI:localuser:root
```

## 10. Arresto e processi duplicati

Arresto normale del VMC: PC2 terminale A, Ctrl+C e attendere il prompt.
Con viewer, usare anche la X della finestra se necessario. Il codice attuale
chiama bridge.stop() nel finally; verificare comunque che il processo termini.
Non interrompere il controllo PC1 o altri publisher necessari al laboratorio.
Un arresto del publisher non sostituisce la procedura di sicurezza fisica.

PC2, terminale B host, se il processo non termina o ci sono istanze duplicate:

```bash
pgrep -af '[h]1_vmc_ros.py'
```

Identificare il PID corretto dalla riga di comando. Poi, PC2 terminale B host:

```bash
sudo kill -TERM PID_DA_TERMINARE
pgrep -af '[h]1_vmc_ros.py'
```

sudo serve perché i processi nsenter girano come root. Non terminare tutti i
processi Python. Solo se TERM non funziona, ultima opzione sullo stesso PID:

```bash
sudo kill -KILL PID_DA_TERMINARE
```

Prima di ripristinare il robot dopo uno stop, assicurarsi che non rimanga un
VMC che pubblica comandi reali. Non riutilizzare i PID annotati nelle vecchie prove.

PC2, terminale A, solo dopo aver terminato tutti i test/processi che lo usano,
si può eliminare il file temporaneo creato in questa shell:

```bash
rm -- "$vmc_test_profile"
unset vmc_test_profile
```

La chiusura del controllo PC1 segue la procedura del laboratorio; non è richiesta
per terminare il solo dry run. Evitare docker rm/down/ricreazioni come arresto ordinario.

## 11. Problemi incontrati e diagnosi verificata

### Camera pubblica, VMC non aggiorna

- La posa arrivava a ros2 topic echo sull'host PC2.
- Non arrivava a echo nell'ambiente nsenter con trasporto predefinito.
- Host e container vedevano /dev/shm diversi, con oggetti Fast DDS separati.
- Forzando UDPv4 su echo la posa arrivava.
- Forzando UDPv4 sul VMC, il target veniva aggiornato senza modifiche Python.

Il risultato supporta un problema di trasporto DDS locale/shared memory fra gli
ambienti; non prova un bug dello script né attribuisce la causa alle sole versioni.
Host: Fast DDS 2.6.10 / rmw_fastrtps_cpp 6.2.8.
Container: Fast DDS 2.6.12 / rmw_fastrtps_cpp 6.2.10.

Riferimento profilo UDP:
https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/transport/udp/udp.html

### Due publisher dry run

Erano rimaste due istanze VMC. pgrep le ha confermate; è stata terminata solo
la precedente, mantenendo quella UDP. Il conteggio finale era uno.
_NODE_NAME_UNKNOWN_ non dimostra da solo la causa; verificare la consegna dati.

### Import differenti

python - dalla root del progetto può caricare /workspace/VMControl/mujoco_vmc,
mentre python FR3/h1_vmc_ros.py aveva caricato /opt/venv/.../site-packages/mujoco_vmc.
Controllare il percorso effettivo prima di analizzare una libreria diversa.
L'errore RobotROSBridge sull'host non è stato risolto/installando nuovi pacchetti:
la procedura finale usa il venv del container.

### Stop e traceback

User Stop pressed! nel log libfranka documentava il fungo premuto, non la causa
delle vibrazioni precedenti. KeyboardInterrupt dopo Ctrl+C indica l'interruzione;
GLXBadContext era comparso in una chiusura del viewer. Non rilanciare alla cieca
se il driver è terminato: prima arrestare i publisher reali residui.

## 12. Risultati e limiti

Verificati con output: ricezione stati, pubblicazione dry run isolata, ricezione
posa camera tramite UDP e aggiornamento target. L'operatore ha poi riferito esito
corretto del movimento reale prima verso il riferimento fisso, poi verso ArUco,
con parametri 2000 / 20 / 20. Non è una certificazione di stabilità generale.

Limiti del codice letto:
- riferimento fisso attivo prima della prima misura camera;
- cambio target immediato, senza rampa;
- ultima posa conservata senza timeout visione (received_at non usato);
- camera one-shot: nessun aggiornamento automatico muovendo il marker;
- assenza di un limite complessivo alle coppie nello script;
- nessuna validazione automatica della zona raggiungibile del target;
- impostazioni camera relative alla cartella di avvio;
- riavvio VMC resetta il target alla posa fissa.

Non sono stati modificati gli script durante la risoluzione del problema DDS.
Il solo file di configurazione creato per il trasporto era temporaneo e locale.
