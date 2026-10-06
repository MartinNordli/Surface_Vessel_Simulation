# Teste «kjør til posisjon» og thrusterkraft

Kort oppskrift for simuleringsgruppa. Den viser hvordan dere tester de to
grensesnittene Control & Autonomy (C&A) skal bruke, med og uten GUI og med
valgfri båt:

1. **Kjør til posisjon:** et målpunkt på `/njord/setpoint` (`PoseStamped` i `map`).
2. **Kraft per thruster:** newton på `/thruster_N/command` (`std_msgs/Float64`).

Detaljer om grensesnittet står i
[control-autonomy-setpoints.md](control-autonomy-setpoints.md), og
kjøremiljøet i [running.md](running.md).

## 0. Før dere starter

```bash
./scripts/njord build      # eller ./scripts/njord pull
./scripts/njord test       # hele testsuiten i containeren, ca. 30 s
```

Alle kommandoene kjøres fra rota av repoet. GPU er forventet. Uten
NVIDIA-GPU kan `NJORD_CPU=1` settes foran kommandoene. Det er tregere, og
renderingen er ikke den samme ([running.md](running.md)).

## 1. Velg båt og miljø

Båten velges med `VESSEL_CONFIG`. Miljøet velges med `ENVIRONMENT`.

| Båt | `VESSEL_CONFIG=` | Thrustere | Merk |
|---|---|---|---|
| WAM-V (standard) | `/config/vessels/wamv.yaml` | 2, parallelle akter | VRX-modellen; kan ikke skyve sideveis |
| Njord-testskrog | `/config/vessels/njord_v1.yaml` | 2, parallelle akter | Analytisk boks, ukalibrert |
| Munin-plassholder | `/config/vessels/munin_v0.yaml` | 4, «X»-oppsett | Fullaktuert (DP); antatt oppsett, ikke Munin |
| Egen båt | `/config/vessels/<fil>.yaml` | Fra fila | Kopier en av filene over til `njord_sim/config/vessels/` og endre den |

- `njord_sim/config/` er montert som `/config`. En ny eller endret vesselfil
  virker derfor uten `build`. Nye scenariofiler krever `build`.
- `ENVIRONMENT` er `calm` (standard), `windy` eller `current`. `current` virker
  bare på Njord-profilene (`njord_v1`, `munin_v0`), ikke på WAM-V.
- `SEED=1` gir repeterbare kjøringer.

Eksempel: `SEED=1 VESSEL_CONFIG=/config/vessels/munin_v0.yaml ENVIRONMENT=windy ./scripts/njord demo station_keeping`

## 2. Med eller uten GUI

| Kommando | GUI | Bruk |
|---|---|---|
| `./scripts/njord demo <kurs>` | Nei | Målt løp som avslutter selv. Exit 0 = alle punkt nådd. |
| `./scripts/njord gui <kurs>` | Gazebo + RViz | Samme målte løp med vinduer |
| `./scripts/njord lab <kurs>` | Nei | Fri kjøring. Dere sender mål eller kraft selv. Ctrl-C avslutter. |
| `GUI=1 ./scripts/njord lab <kurs>` | Gazebo + RViz | Fri kjøring; målpunkt kan klikkes ut i RViz |

Kursene er `goto_square`, `goto_retarget` og `station_keeping` (se tabellen i
[control-autonomy-setpoints.md](control-autonomy-setpoints.md#2-kom-i-gang-på-fem-minutter-uten-egne-noder)).
GUI bruker X-skjermen til verten. På WSL2 legger `scripts/njord` selv til
`compose.wsl.yaml`.

Alle kjøringer skriver til `outputs/run-<tid>/`:
- `report.html`: kart, tabell per punkt og grafer, inkludert kraft per thruster
- `run_metrics.json`
- `timeseries.csv` (10 Hz, med `force_1_n` …)

## 3. Test A: kjør til posisjon med simulatorens kontroller

Denne testen sjekker hele kjeden fra målpunkt til fysikk, uten C&A-kode:

```bash
SEED=1 VESSEL_CONFIG=/config/vessels/munin_v0.yaml ./scripts/njord demo goto_square   # uten GUI
SEED=1 VESSEL_CONFIG=/config/vessels/munin_v0.yaml ./scripts/njord gui goto_square    # med GUI
```

**Forventet:**
- Én logglinje per punkt, for eksempel `Setpoint 1/4 'east' reached after 19.1 s: error 0.43 m / 0.2 deg`.
- `Report: …` til slutt og exit-kode 0.

Bytt `VESSEL_CONFIG` for å teste en annen båt.

## 4. Test B: send målpunkt selv (lab)

```bash
VESSEL_CONFIG=/config/vessels/munin_v0.yaml ./scripts/njord lab goto_square   # terminal 1
./scripts/njord goto 15 5 90     # terminal 2: x=15 m, y=5 m, heading 90° (nord, ENU)
./scripts/njord goto 0 0         # uten heading: pek langs linja dit
```

- Med GUI: start terminal 1 med `GUI=1`. I RViz velger dere «2D Goal Pose»
  (tast G), klikker på målet og drar i ønsket heading.
- Hvert punkt måles med 1,5 m / 15° / hold 5 s til et nytt kommer.
- Ctrl-C i terminal 1 skriver rapporten.

## 5. Test C: kraft direkte per thruster

Her er simulatorens kontroller slått av (`CONTROLLER=external`), og dere
publiserer newton på hver thruster, slik C&A sin allokeringsnode vil gjøre:

```bash
CONTROLLER=external VESSEL_CONFIG=/config/vessels/munin_v0.yaml ./scripts/njord lab goto_square   # terminal 1

# terminal 2: én verdi i newton per thruster, i rekkefølgen i vesselfila, i 15 s
docker compose exec autonomy /entrypoint.sh bash -c '
F=(100 100 100 100)
for i in "${!F[@]}"; do
  timeout 15 ros2 topic pub -r 20 /thruster_$((i+1))/command std_msgs/msg/Float64 "{data: ${F[$i]}}" >/dev/null &
done; wait'
```

- `F` må ha like mange verdier som båten har thrustere: 2 på WAM-V og
  `njord_v1`, 4 på `munin_v0`.
- Når `timeout` går ut, stopper kommandoene. Guarden setter da kraften til
  null, og båten driver videre.

| Båt | `F` | Forventet bevegelse |
|---|---|---|
| `munin_v0` | `(100 100 100 100)` | Rett fram, ingen sidefart |
| `munin_v0` | `(-100 100 -100 100)` | Dreier mot klokka på stedet |
| WAM-V / `njord_v1` | `(100 100)` | Rett fram |
| WAM-V / `njord_v1` | `(-100 100)` | Dreier mot klokka (venstre) |

**Sjekk underveis** (i en tredje terminal):

```bash
docker compose exec autonomy /entrypoint.sh ros2 topic echo /njord/guard_status --field status   # «ok» mens kraft slippes gjennom
docker compose exec autonomy /entrypoint.sh ros2 topic echo --once /njord/actuator_forces        # kraften guarden sendte til fysikken
docker compose exec autonomy /entrypoint.sh ros2 topic echo --once /sim/ground_truth/odometry --field twist.twist
```

Målt 2026-10-06:
- `munin_v0` med `(100 100 100 100)` i 10 s: 1,7 m/s fram og null sidefart.
- `munin_v0` med `(-100 100 -100 100)`: 2,6 rad/s mot klokka uten fart.
- WAM-V med `(-100 100)`: 0,21 rad/s mot klokka.
- Guarden svarte `thruster 1 command missing` før start, `ok` under kjøring
  og `thruster 1 command stale in simulation time` etter at kommandoene
  stoppet.

## 6. Test D: ekstern kontroller, slik C&A kobler seg på

Her spiller simulatorens kontroller rollen som C&A sin node. Den startes for
hånd etter simulatoren, som en ekstern node:

```bash
CONTROLLER=external VESSEL_CONFIG=/config/vessels/munin_v0.yaml SEED=1 ./scripts/njord demo goto_square   # terminal 1 (eller gui)

# terminal 2, når terminal 1 skriver «Waiting for a subscriber on /njord/setpoint, a publisher on /thruster_1/command …»:
docker compose exec autonomy /entrypoint.sh bash -c '
python3 -c "import json,os,yaml; p=json.load(open(os.environ[\"OUTPUT_DIR\"]+\"/public_parameters.json\"))[\"setpoint_controller\"]; yaml.safe_dump({\"setpoint_controller\": {\"ros__parameters\": p}}, open(\"/tmp/sc.yaml\", \"w\"))"
ros2 run njord_sim setpoint_controller --ros-args -p use_sim_time:=true --params-file /tmp/sc.yaml'
```

Den første linja henter båtens thrusteroppsett og grenser fra kjøringen, slik
at kontrolleren passer til båten som er valgt.

**Forventet:**
- Løpet venter til kontrolleren abonnerer på målpunktet og publiserer på alle
  thrustertopicene. Deretter står `race started`, og løpet går som i test A
  med exit 0.
- RViz eller `ros2 topic echo /njord/setpoint` alene starter ikke løpet.

For C&A sine ekte noder, se avsnitt 3 i
[control-autonomy-setpoints.md](control-autonomy-setpoints.md#3-koble-til-deres-egne-noder).

## 7. Feilsøking

| Symptom | Sjekk |
|---|---|
| Båten står stille | `ros2 topic echo /njord/guard_status`. Meldingen sier hva som mangler, for eksempel `thruster 3 command missing`. |
| `… stale in simulation time` | Kommandoene kommer for sjelden, eller noden mangler `use_sim_time:=true`. |
| Løpet starter aldri | Evaluatoren skriver hva den venter på: feil topicnavn, namespace, antall thrustere eller `ROS_DOMAIN_ID` (standard 42). |
| To publishers på en thrustertopic | `CONTROLLER=external` er glemt, så simulatorens kontroller kjører også. |

## 8. Hva som er verifisert

- **Kjørt uten GUI på GPU (RTX 5090), 2026-10-06, på `feat/setpoint-missions`:**
  - test A med `munin_v0`: 4/4 punkt, 0,24–0,43 m feil, exit 0
  - test C med `munin_v0` og WAM-V
  - test D med `munin_v0`: 4/4 punkt, exit 0
  - test B ved forrige gjennomgang ([validation.md](validation.md#setpoint-courses-2026-10-06))
- **Ikke verifisert ennå:** `gui`, `GUI=1` og klikk i RViz. Rapporter
  resultatet når dere har kjørt dem.
- `munin_v0` er en plassholder. Resultatene sier noe om kontrolleren og
  koblingen, ikke om hvordan Munin vil oppføre seg.
