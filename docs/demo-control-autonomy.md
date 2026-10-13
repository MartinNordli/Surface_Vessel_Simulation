# Demo med Control & Autonomy: steg for steg

Denne veiledningen gjelder demoen tirsdag 13. oktober 2026, der DP-regulatoren
til Control & Autonomy (C&A) skal styre den simulerte båten. Den sier hva
simuleringsgruppa gjør, hva C&A gjør, hvilke kommandoer som kjøres, og hva
simulatoren forventer inn og gir ut.

- **Demo-PC-en** er en vanlig Linux-PC (Ubuntu 24.04) med NVIDIA-skjermkort.
  Simulatoren kjører der.
- **Simuleringsgruppa** gjør avsnitt 2 før demoen og kjører simulatoren på dagen.
- **C&A** gjør avsnitt 3 før demoen og starter sine noder på dagen (avsnitt 4).

Bakgrunn og flere detaljer:
[control-autonomy-setpoints.md](control-autonomy-setpoints.md) (grensesnittet),
[running.md](running.md) (kjøremiljøet),
[team-integration.md](team-integration.md) (eget ROS 2-workspace) og
[demo-control-autonomy-gui.md](demo-control-autonomy-gui.md) (de samme
kjøringene med Gazebo- og RViz-vinduene, steg for steg).

## 1. Hva simulatoren gir ut og tar inn

```
 Simulatoren                      C&A sine noder (simulatormodus)                Simulatoren
 ───────────                      ───────────────────────────────                ───────────
 /njord/odometry  ──▶ konverter ──▶ /vessel/state_est ─┐
   (ENU, FLU)          til NED/FRD   (VesselStateEst)   ├─▶ DP ─▶ allokering ──▶ /thruster_1/command
 /njord/setpoint  ──▶ konverter ──▶ DPSetpoint ────────┘                         …
   (ENU)               til NED                                                   /thruster_4/command
 /clock           ──▶ use_sim_time:=true på alle noder                           (newton)
```

Simulatoren bruker ROS-standarden: `map` er ENU (x øst, y nord), og båtens
ramme `base_link` er FLU (x fram, y babord, z opp). C&A regner i NED og FRD og
konverterer selv i sine noder. Simulatoren bygger ikke inn C&A sine
meldingstyper.

### Ut fra simulatoren (inn til C&A)

| Topic | Type | Innhold | Takt og QoS |
|---|---|---|---|
| `/njord/odometry` | `nav_msgs/Odometry` | Posisjon og heading for `base_link` i `map` (ENU). Fart i `twist` er i båtens ramme (FLU). `header.stamp` er simuleringstid. Fasit med `STATE_SOURCE=truth`, ellers EKF-estimat. | 30 Hz. Abonner best effort eller reliable. |
| `/njord/setpoint` | `geometry_msgs/PoseStamped` | Målet: `pose.position.x/y` i meter og heading som `pose.orientation`, i `map` (ENU). Siste punkt gjelder til et nytt kommer. | Sendes når punktet gis, og gjentas hvert sekund i et løp med **samme** stempel. Abonner reliable. |
| `/clock` | `rosgraph_msgs/Clock` | Simuleringstid | Alle noder må kjøre med `use_sim_time:=true`. |

### Inn til simulatoren (ut fra C&A)

| Topic | Type | Innhold | Krav |
|---|---|---|---|
| `/thruster_1/command` … `/thruster_4/command` | `std_msgs/Float64` | Kraft i newton langs thrusterens retning. Negativ verdi er revers. | Send alle fire i hver syklus, omtrent 20 Hz. Er én kommando eldre enn 0,5 s simuleringstid, settes all kraft til null. Kraften begrenses til ±500 N per thruster. |

Ingen namespace, og `ROS_DOMAIN_ID=42`.

### Konverteringen C&A gjør

| Størrelse | Fra simulatoren | I NED / FRD |
|---|---|---|
| Posisjon | `position.x` (øst), `position.y` (nord) | `x = position.y`, `y = position.x` |
| Heading | `yaw`, mot klokka fra øst (fra kvaternionen) | `psi = π/2 − yaw`, med klokka fra nord, wrappet til [−π, π] |
| Fart | `twist.linear.x`, `twist.linear.y` (mot babord) | `u = linear.x`, `v = −linear.y` |
| Girrate | `twist.angular.z`, mot klokka | `r = −angular.z` |
| Bias | finnes ikke | `b = 0` |

**Samme konvertering gjelder for målpunktet:** `x = pose.position.y`,
`y = pose.position.x`, `psi = π/2 − yaw`.

Eksempler som brukes i resten av veiledningen:

| | Fra simulatoren (ENU) | I NED |
|---|---|---|
| Startposisjon i alle kursene | x = 0, y = 0, yaw = 0° (peker øst) | x = 0, y = 0, psi = 90° (1,5708 rad) |
| Målet i `hold_position` | x = 0, y = 0, heading 0° | x = 0, y = 0, psi = 90° |
| Målet i `station_keeping` | x = 10, y = 0, heading 0° | x = 0, y = 10, psi = 90° |

### Thrusterne på Munin-plassholderen (`munin_v0.yaml`)

Oppsettet er **antatt** («X») til Naval har målt. Allokeringen til C&A må
bruke samme oppsett, ellers går båten feil vei.

| Navn | Plass | Simulatoren (FLU): [x, y] m, retning | C&A (FRD): [x, y] m, retning | Positiv kraft skyver |
|---|---|---|---|---|
| `thruster_1` | baug, babord | [1,1, 0,55], −45° | [1,1, −0,55], +45° | fram og mot styrbord |
| `thruster_2` | baug, styrbord | [1,1, −0,55], +45° | [1,1, 0,55], −45° | fram og mot babord |
| `thruster_3` | akter, babord | [−1,1, 0,55], +45° | [−1,1, −0,55], −45° | fram og mot babord |
| `thruster_4` | akter, styrbord | [−1,1, −0,55], −45° | [−1,1, 0,55], +45° | fram og mot styrbord |

Retningen er vinkelen til positiv kraft fra rett fram: mot klokka i
simulatoren, med klokka i FRD. Maks kraft er 500 N per thruster i begge
retninger. Bruker C&A et annet oppsett, endrer simuleringsgruppa
`njord_sim/config/vessels/munin_v0.yaml`. Det krever ikke ny bygging.

## 2. Simuleringsgruppa: før demoen (på demo-PC-en)

Alle kommandoer kjøres fra rota av repoet, i en terminal på skrivebordet til
demo-PC-en (ikke over SSH, ellers åpnes ikke vinduene).

### 2.1 Sjekk maskinen

```bash
lsb_release -ds     # forventet: Ubuntu 24.04
nvidia-smi          # forventet: en tabell med skjermkortet og driverversjonen
```

Feiler `nvidia-smi`, mangler NVIDIA-driveren. Den må installeres først;
oppsettet under installerer aldri en skjermkortdriver. Har maskinen ikke
NVIDIA-kort, kan `NJORD_CPU=1` settes foran alle `njord`-kommandoer. Det er
tregere, og målpunktkursene er ikke prøvd slik.

### 2.2 Hent koden og sett opp Docker

```bash
git clone https://github.com/MartinNordli/Surface_Vessel_Simulation.git
cd Surface_Vessel_Simulation
sudo bash scripts/setup-host.sh     # én gang: Docker, Compose og NVIDIA Container Toolkit
sudo usermod -aG docker $USER       # så docker virker uten sudo; logg ut og inn igjen etterpå
```

Forventet siste linje fra oppsettet: `Docker is configured. …`. Etter ny
innlogging skal `docker ps` virke uten `sudo`. Medlemskap i `docker`-gruppa gir
i praksis root-tilgang på maskinen, så det er eieren av maskinen som bestemmer
det. Uten det må `njord`-kommandoene kjøres med `sg docker -c '…'`.

### 2.3 Hent imaget og sjekk det

```bash
./scripts/njord pull      # laster ned imaget CI har bygget for denne commiten (flere GB)
./scripts/njord doctor
```

Forventet:
- `pull` ender med `Image sha256:… matches this checkout (source)`.
- `doctor` viser `nvidia-smi`-tabellen to ganger (verten og inne i
  containeren) og ender med samme `matches this checkout`-linje.

Svarer `pull` med `No published image …`, er ikke CI ferdig for den nyeste
commiten ennå. Vent til CI er grønn på GitHub, eller kjør
`./scripts/njord build`. En bygging uten cache kompilerer Gazebo og VRX og
tar lang tid, så gjør det i god tid.

### 2.4 Test at simulatoren virker

```bash
./scripts/njord test        # ca. 30 s
./scripts/njord selftest    # starter simulatoren og sjekker kameraer, lidar og navigasjon
```

Forventet: `test` ender med `Ran 310 tests … OK (skipped=2)`. `selftest` ender
uten feilmelding og med exit-kode 0 (`echo $?`).

### 2.5 Generalprøve A: simulatorens egen regulator

Denne kjøringen bruker ingen C&A-kode. Den viser at simulatoren, båten og
målingen virker på demo-PC-en.

```bash
SEED=1 ENVIRONMENT=current VESSEL_CONFIG=/config/vessels/munin_v0.yaml ./scripts/njord demo hold_position
```

Forventet i terminalen etter en kort oppstart og 60 s løp (hver linje har i
tillegg et prefiks med containernavn og tid):

```
[evaluator]: Setpoint 1/1 'hold' issued at 0.0 s: x=0.00 y=0.00 heading=0.0 deg
[evaluator]: Required heartbeats and contacts ready; race started
[evaluator]: Setpoint 1/1 'hold' reached after 60.0 s: error 0.23 m / 0.2 deg
[evaluator]: Race completed; metrics: /outputs/run_metrics.json, report: /outputs/report.html
Report: …/outputs/run-<tid>/report.html
```

`echo $?` skal gi 0. Åpne `report.html` i en nettleser.

### 2.6 Generalprøve B: ekstern regulator i NED

Her er simulatorens regulator slått av. I stedet kjører
`validation/ned_dp_standin.py`, som gjør det C&A sine noder skal gjøre:
konverterer til NED/FRD, regulerer og sender kraft per thruster. Den er
**ikke** C&A sin regulator; den viser bare at grensesnittet virker.

Terminal 1 (regulatoren først):

```bash
export NJORD_UID=$(id -u) NJORD_GID=$(id -g)
mkdir -p outputs/demo
VESSEL_CONFIG=/config/vessels/munin_v0.yaml docker compose -p njord-ca run --rm autonomy \
  python3 /opt/njord/validation/ned_dp_standin.py
```

Forventet: `Stand-in controller for 4 thrusters; waiting for /njord/setpoint`.

Terminal 2 (løpet):

```bash
STATE_SOURCE=truth CONTROLLER=external SEED=1 ENVIRONMENT=current \
  VESSEL_CONFIG=/config/vessels/munin_v0.yaml ./scripts/njord demo hold_position
```

Forventet i terminal 1 når løpet starter:

```
New setpoint in NED: x=0.00 m (north), y=0.00 m (east), psi=90.0 deg
Sending thrust
```

Forventet i terminal 2: samme fire linjer som i generalprøve A, med
`reached after 60.0 s: error 0.31 m / 0.0 deg`, og exit-kode 0. Stopp
regulatoren med Ctrl-C i terminal 1.

Bytt `hold_position` med `station_keeping` for å prøve et sprang på 10 m.
Da skriver terminal 1 `x=0.00 m (north), y=10.00 m (east), psi=90.0 deg`, og
løpet ender med `reached after 65.9 s`.

### 2.7 Generalprøve C: med vinduer

```bash
SEED=1 ENVIRONMENT=current VESSEL_CONFIG=/config/vessels/munin_v0.yaml ./scripts/njord gui hold_position
```

Forventet: ett Gazebo-vindu med båten og ett RViz-vindu med målpunktet som en
gul skive (grønn når det er nådd) og sporet til båten. Løpet går og avslutter
som i generalprøve A. RViz-vinduet blir stående etter løpet; lukk det selv,
eller kjør `docker rm -f njord-sim-rviz-1`.

Åpnes ingen vinduer, kjører dere trolig over SSH eller som en annen bruker
enn den som er logget inn på skrivebordet.

Hva vinduene viser, hvordan dere zoomer inn i RViz, og generalprøve B og fri
kjøring med vinduer står i
[demo-control-autonomy-gui.md](demo-control-autonomy-gui.md).

## 3. Control & Autonomy: før demoen

### 3.1 Dette må nodene deres gjøre i simulatormodus

1. **Tilstandsnoden** abonnerer på `/njord/odometry` og publiserer
   `/vessel/state_est` (`VesselStateEst`) med konverteringen i avsnitt 1.
2. **DP-noden** abonnerer på `/njord/setpoint` og gjør det om til en
   `DPSetpoint` med samme konvertering.
3. **Allokeringsnoden** publiserer som avtalt på `/thruster_1/command` …
   `/thruster_4/command`, med thrusteroppsettet i avsnitt 1.
4. Alle noder startes med `use_sim_time:=true`, uten namespace og med
   `ROS_DOMAIN_ID=42`. Både ROS 2 Humble og Jazzy virker (avsnitt 4.1).

Regler som er lette å glemme:

- **Gjentatt målpunkt er ikke et nytt punkt.** I et løp kommer det aktive
  punktet på nytt hvert sekund med samme stempel. Sammenlign stempel og pose
  med punktet dere har, og ikke nullstill referansefilter eller integrator
  når de er like.
- **Send null kraft selv** når tilstand eller målpunkt mangler. Simulatoren
  stopper også kraften hvis kommandoene blir eldre enn 0,5 s.
- **Løpet starter først når dere er koblet til:** minst én node abonnerer på
  `/njord/setpoint`, og alle fire thrustertopicene har en publisher.
- **Behold stempelet** fra `/njord/odometry` i `VesselStateEst.header.stamp`.
  Ikke sett det til «nå».

### 3.2 Eksempel på konverteringen (Python)

Bytt `deres_msgs` med pakken som har meldingstypene deres. En kjørbar
referanse som er prøvd mot simulatoren ligger i
[`validation/ned_dp_standin.py`](../validation/ned_dp_standin.py).

```python
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from deres_msgs.msg import DPSetpoint, VesselStateEst


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_enu(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# I __init__ til tilstandsnoden:
#   self.create_subscription(Odometry, '/njord/odometry', self.on_odometry, qos_profile_sensor_data)
def on_odometry(self, msg):
    out = VesselStateEst()
    out.header.stamp = msg.header.stamp            # simuleringstid, uendret
    out.x = msg.pose.pose.position.y               # nord
    out.y = msg.pose.pose.position.x               # øst
    out.psi = wrap(math.pi / 2 - yaw_enu(msg.pose.pose.orientation))
    out.u = msg.twist.twist.linear.x
    out.v = -msg.twist.twist.linear.y
    out.r = -msg.twist.twist.angular.z
    out.b = 0.0
    self.state_pub.publish(out)


# I __init__ til DP-noden (standard QoS er reliable):
#   self.create_subscription(PoseStamped, '/njord/setpoint', self.on_setpoint, 10)
def on_setpoint(self, msg):
    key = (msg.header.stamp.sec, msg.header.stamp.nanosec, msg.pose.position.x, msg.pose.position.y,
           msg.pose.orientation.z, msg.pose.orientation.w)
    if key == self.last_setpoint_key:
        return                                     # gjentakelse av samme punkt
    self.last_setpoint_key = key
    out = DPSetpoint()
    out.header.stamp = msg.header.stamp
    out.x = msg.pose.position.y
    out.y = msg.pose.position.x
    out.psi = wrap(math.pi / 2 - yaw_enu(msg.pose.orientation))
    self.use_setpoint(out)
```

### 3.3 Test konverteringen uten simulatoren

Start nodene deres med `use_sim_time:=false` (det finnes ingen `/clock` uten
simulatoren) og send faste verdier fra en annen terminal:

```bash
export ROS_DOMAIN_ID=42
ros2 topic pub -r 20 /njord/odometry nav_msgs/msg/Odometry "{header: {stamp: now, frame_id: map}, child_frame_id: base_link, pose: {pose: {position: {x: 3.0, y: 4.0}, orientation: {w: 1.0}}}, twist: {twist: {linear: {x: 1.0, y: 0.5}, angular: {z: 0.2}}}}"
```

Forventet på `/vessel/state_est`: `x = 4.0`, `y = 3.0`, `psi = 1.5708`,
`u = 1.0`, `v = -0.5`, `r = -0.2`.

Sett så farten til null i kommandoen over (`linear: {x: 0.0, y: 0.0}`,
`angular: {z: 0.0}`) og posisjonen til `x: 0.0, y: 0.0`, og send ett målpunkt
om gangen fra en tredje terminal:

```bash
# 2 m rett fram for båten (øst)
ros2 topic pub --once /njord/setpoint geometry_msgs/msg/PoseStamped "{header: {stamp: now, frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}"
```

| Målpunkt (ENU) | Forventet `DPSetpoint` | Forventet fortegn på `/thruster_1..4/command` |
|---|---|---|
| `x: 2.0, y: 0.0`, `orientation: {w: 1.0}` (2 m rett fram) | x = 0, y = 2, psi = 1,5708 | + + + + |
| `x: 0.0, y: 2.0`, `orientation: {w: 1.0}` (2 m mot babord) | x = 2, y = 0, psi = 1,5708 | − + + − |
| `x: 0.0, y: 0.0`, `orientation: {z: 0.7071, w: 0.7071}` (snu mot nord) | x = 0, y = 0, psi = 0 | − + − + |

Fortegnene gjelder en regulator som skyver rett mot målet, med
thrusteroppsettet i avsnitt 1. Sjekk dem med
`ros2 topic echo /thruster_1/command` osv. Stemmer ikke fortegnene, er det
allokeringen eller konverteringen som avviker, og det er mye raskere å finne
nå enn på demoen.

### 3.4 Send dette til simuleringsgruppa før demoen

- Lenke til repoet og branchen, og hvordan det bygges (`colcon build`?).
- Hvilken ROS 2-versjon dere bruker (Humble eller Jazzy), og avhengigheter
  utover den. Simulator-imaget (Jazzy) har `g++`, Eigen og NumPy, men for
  eksempel ikke SciPy.
- Thrusteroppsettet allokeringen deres regner med, hvis det avviker fra
  tabellen i avsnitt 1.
- Frekvensen på kontrollsløyfen.

## 4. På demodagen

### 4.1 Koble C&A sine noder til simulatoren

Simulatoren kjører ROS 2 Jazzy. **Noder på ROS 2 Humble virker også mot den**
(prøvd, se avsnitt 6), med disse forbeholdene:

- Humble-terminalene skriver `sequence size exceeds remaining buffer` omtrent
  hvert sekund. Data kommer likevel riktig fram. Feilen gjelder trolig
  ROS sin interne oversikt over noder, som har byttet format etter Humble.
- `ros2 node list` på Humble viser ikke simulatorens noder, og omvendt.
  `ros2 topic list`, `echo` og `hz` virker.
- Bare standardtypene som krysser mellom simulatoren og C&A er prøvd
  (`Odometry`, `PoseStamped`, `Float64`, `Clock`). De er like i Humble og Jazzy.
- ROS 2 lover ikke at ulike versjoner kan snakke sammen. Bruk standard DDS
  (Fast DDS, altså ikke sett `RMW_IMPLEMENTATION`).

**Alternativ A (tryggest): kjør C&A sine noder i en container på demo-PC-en.**
Da er nettverket mellom maskiner ute av bildet. `~/ca_ws/src` inneholder C&A
sitt repo.

For kode skrevet for **Humble**:

```bash
docker run --rm -it --network host -e ROS_DOMAIN_ID=42 -e HOME=/tmp \
  -u $(id -u):$(id -g) -v $HOME/ca_ws:/ca_ws -w /ca_ws ros:humble-ros-base bash
# inne i containeren:
colcon build                       # forventet: Summary: N packages finished
source install/setup.bash
ros2 launch <pakke> <launchfil> use_sim_time:=true
```

For kode som bygger på **Jazzy**, i simulator-imaget (kjøres fra simulator-repoet):

```bash
export TEAM_WS_HOST=$HOME/ca_ws NJORD_UID=$(id -u) NJORD_GID=$(id -g)
docker compose -p njord-ca -f compose.yaml -f compose.team.yaml run --rm team \
  colcon --log-base log-sim build --build-base build-sim --install-base install-sim
docker compose -p njord-ca -f compose.yaml -f compose.team.yaml run --rm team bash
# inne i containeren:
source /team_ws/install-sim/setup.bash
ros2 launch <pakke> <launchfil> use_sim_time:=true
```

Feiler byggingen på en manglende avhengighet, må den installeres i
containeren først (start den da uten `-u …`, altså som root, og kjør
`apt-get update && rosdep install --from-paths src -y --ignore-src`), eller
bruk alternativ B.

**Alternativ B: C&A sin egen maskin** med ROS 2 Humble eller Jazzy, koblet med
kabel til samme svitsj som demo-PC-en. Trådløse nett som eduroam slipper
normalt ikke gjennom oppdagelsestrafikken (multicast) til ROS 2.

```bash
source /opt/ros/humble/setup.bash            # eller jazzy
source <deres_ws>/install/setup.bash
export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
ros2 launch <pakke> <launchfil> use_sim_time:=true
```

På Jazzy settes i tillegg `export FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, som i
simulatorens containere. Sjekk koblingen fra C&A sin maskin mens simulatoren
går (4.2):

```bash
ros2 topic list | grep njord       # skal vise /njord/odometry og /njord/setpoint
ros2 topic hz /njord/odometry      # ca. 30 Hz
```

Alternativ B er ikke prøvd mellom to maskiner. Sett av tid til det før
demoen, og ha alternativ A som reserve.

### 4.2 Sjekk koblingen før første løp

Simuleringsgruppa starter en fri kjøring uten løp:

```bash
STATE_SOURCE=truth CONTROLLER=external ENVIRONMENT=current \
  VESSEL_CONFIG=/config/vessels/munin_v0.yaml GUI=1 ./scripts/njord lab hold_position
```

C&A starter nodene sine (4.1). Sjekk så fra en ny terminal i simulator-repoet:

```bash
docker compose exec autonomy /entrypoint.sh ros2 topic hz /njord/odometry                    # ca. 30 Hz
docker compose exec autonomy /entrypoint.sh ros2 topic info /njord/setpoint                  # Subscription count: minst 2 (evaluatoren og DP-noden)
docker compose exec autonomy /entrypoint.sh ros2 topic info /thruster_1/command              # Publisher count: 1
docker compose exec autonomy /entrypoint.sh ros2 topic hz /thruster_1/command                # ca. 20 Hz
docker compose exec autonomy /entrypoint.sh ros2 topic echo --once /njord/guard_status --field status
```

Den siste skal vise `message='ok'` når alle fire thrusterkommandoene kommer.
Ellers sier meldingen hva som mangler, for eksempel
`thruster 3 command missing` eller `… stale in simulation time`.

Send så et lite steg og se at båten går riktig vei:

```bash
./scripts/njord goto 2 0 0      # 2 m øst, heading øst (rett fram fra start)
./scripts/njord goto 2 2 90     # 2 m nord for forrige punkt, heading nord
```

Forventet fra `goto`: `Sent /njord/setpoint to 2 subscriber(s): …`
(evaluatoren og DP-noden; flere med RViz). Forventet i simulatorterminalen:
`Setpoint 1 'setpoint_1' issued …` og litt senere `… reached after … s`.
Kommandoen bruker omtrent seks sekunder, fordi den venter på noder som
oppdages sent.

Går båten motsatt vei eller sideveis, er det konverteringen eller
thrusteroppsettet som avviker (avsnitt 1). Avslutt med Ctrl-C i
simulatorterminalen; da skrives rapporten.

### 4.3 Kjøreplan

C&A sine noder kan stå og gå mellom løpene. Start dem før løpet, så driver
ikke båten mens den venter.

| # | Hva | Kommando (simuleringsgruppa) | Forventet |
|---|---|---|---|
| 1 | Simulatoren alene, med vinduer | `SEED=1 ENVIRONMENT=current VESSEL_CONFIG=/config/vessels/munin_v0.yaml ./scripts/njord gui hold_position` | `reached after 60.0 s`, exit 0 |
| 2 | C&A holder posisjon i vind og strøm | `STATE_SOURCE=truth CONTROLLER=external SEED=1 ENVIRONMENT=current VESSEL_CONFIG=/config/vessels/munin_v0.yaml ./scripts/njord gui hold_position` | `race started` når nodene er koblet til, så `reached after 60.0 s` |
| 3 | C&A kjører 10 m og holder | Som 2, med `station_keeping` | `reached after … s`. Spranget på 10 m kan være for stort for en regulator som forventer glatt referanse. |
| 4 | Små steg for hånd | Som 4.2, med `./scripts/njord goto X Y HEADING` | Én `reached`-linje per punkt |
| 5 | Valgfritt: estimat i stedet for fasit | Som 2, uten `STATE_SOURCE=truth` | Se under |

`hold_position` gir målet i startposisjonen, altså ingen sprang. Kravet er
1,0 m og 10° i 60 s, og løpet gir opp etter 200 s. Bruk `demo` i stedet for
`gui` for å kjøre uten vinduer. `ENVIRONMENT=calm` fjerner vind og strøm.

**Bruk `STATE_SOURCE=truth` i demoen.** Det tilsvarer en ideell Seapath. Uten
den får nodene et EKF-estimat der farten er svært støyete: i generalprøven
var feilen i surge-fart 0,4–0,7 m/s RMS, like stor som farten selv, og
regulatoren brukte 2,7 ganger så mye kraft.

Står løpet og venter, skriver simulatoren hva som mangler:
`Waiting for a subscriber on /njord/setpoint, a publisher on /thruster_1/command … (the controller) to start`.

### 4.4 Les resultatet

Hver kjøring skriver til `outputs/run-<tid>/`. Åpne `report.html` i en
nettleser: kart med spor og målpunkt, tabell per punkt og grafer over
avstand, headingfeil, fart og kraft per thruster. Tallene ligger også i
`run_metrics.json` (`setpoints[]`), og `timeseries.csv` har 10 Hz-data.

Til sammenligning, generalprøven 12. oktober 2026 med stand-in-regulatoren
(ikke C&A sin) på Munin-plassholderen, `ENVIRONMENT=current`:

| Kjøring | Tilstand | Hold: RMS-avstand | Hold: maks avstand | Kraftimpuls |
|---|---|---|---|---|
| `hold_position` | fasit | 0,30 m | 0,33 m | 2525 N·s |
| `hold_position` | estimat | 0,36 m | 0,72 m | 6721 N·s |
| `station_keeping` | fasit | 0,33 m | 1,00 m | 4218 N·s |

Stand-in-regulatoren er en ren PD-regulator uten integralvirkning, så den
blir stående omtrent 0,3 m fra målet i vind og strøm.

## 5. Feilsøking

| Symptom | Årsak og tiltak |
|---|---|
| Løpet starter aldri | Les `Waiting for …`-linja. Feil topicnavn, namespace, `ROS_DOMAIN_ID`, eller nodene ser ikke simulatoren (4.1). |
| Båten står stille | `ros2 topic echo /njord/guard_status`: meldingen sier hvilken kommando som mangler eller er for gammel. |
| `stale in simulation time` | Noden mangler `use_sim_time:=true`, eller sender sjeldnere enn hvert 0,5 s. |
| Båten går feil vei eller snurrer | Konverteringen (avsnitt 1) eller thrusteroppsettet. Kjør testen i 3.3. |
| Båten rykker fram og tilbake | Kjører uten `STATE_SOURCE=truth` (støyete fart), eller målpunktet behandles som nytt hvert sekund. |
| Topicen heter `/<ns>/thruster_1/command` | Noden kjører i et namespace. Start uten. |
| `ros2 topic list` på C&A sin maskin viser ingen `/njord/…` | Ulik `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY` er satt, brannmur, eller nettet blokkerer multicast. Bruk alternativ A. |
| `sequence size exceeds remaining buffer` gjentas i en Humble-terminal | Ufarlig når Humble snakker med Jazzy (4.1); data kommer fram. |
| To publishers på en thrustertopic | `CONTROLLER=external` er glemt, så simulatorens regulator kjører også. |
| `Image … has a missing or mismatched … digest` | Koden er endret etter at imaget ble hentet. Kjør `./scripts/njord pull` eller `build`. |
| Ingen vinduer med `gui` | Kjør fra en terminal på skrivebordet til demo-PC-en, som brukeren som er logget inn. |

## 6. Hva som er prøvd

Prøvd 12. oktober 2026 på GPU (RTX 5090, WSL2), `munin_v0`
([validation.md](validation.md#control--autonomy-demo-rehearsal-2026-10-12)):

- Generalprøve A og B, også med `station_keeping` og uten `STATE_SOURCE=truth`:
  alle fullført med exit 0.
- Fri kjøring med åtte `goto`-punkt og stand-in-regulatoren startet etter
  simulatoren: alle åtte nådd.
- `gui hold_position`: Gazebo og RViz startet, og løpet ble fullført.
  Vinduene ble ikke sett på den dagen; det ble gjort 13. oktober
  ([demo-control-autonomy-gui.md](demo-control-autonomy-gui.md#10-hva-som-er-prøvd)).
- Et workspace med `VesselStateEst` og `DPSetpoint` bygger i team-containeren
  (Jazzy) og i `ros:humble-ros-base`.
- **ROS 2 Humble mot simulatoren:** en regulatornode i en
  `ros:humble-ros-base`-container på samme maskin fullførte `hold_position` og
  `station_keeping` og fikk åtte av åtte `goto`-punkt. Alle 36 topics var
  synlige fra Humble.

Ikke prøvd:

- En Linux-PC uten WSL, og vinduene der.
- C&A sin egen kode, og alternativ B mellom to maskiner (verken med Humble
  eller Jazzy).
- `NJORD_CPU=1` på målpunktkursene.

`munin_v0` er en plassholder med antatt skrog og thrusteroppsett. Resultatene
sier noe om regulatoren og koblingen, ikke om hvordan Munin vil oppføre seg.
