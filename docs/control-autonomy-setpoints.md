# Control & Autonomy: «kjør til denne posisjonen» og thrusterkraft

Denne guiden viser nøyaktig hvordan nodene til Control & Autonomy (C&A) kobles
til simulatoren slik at:

1. båten får et signal om hvilken posisjon (og heading) den skal kjøre til,
   også flere signaler etter hverandre, og
2. C&A sender kraft per thruster, og båten beveger seg ut fra det,

og hvordan dere ser hvor godt båten faktisk gjorde det, både med og uten GUI.

> **Foreløpige navn.** Topicnavn og meldingstyper for målpunkt er valgt av
> simulatorgruppa og ikke avtalt med C&A ennå. Hvert navn har ett hjem i koden,
> og [avsnitt 11](#11-navn-og-typer-som-må-bekreftes-med-ca) sier hvor det
> endres. Thrustertopicene `/thruster_N/command` er allerede avtalt med C&A.

Generelle detaljer om topics, rammer og QoS står i [interfaces.md](interfaces.md),
og oppsett av eget ROS 2-workspace i [team-integration.md](team-integration.md).

## 1. Slik henger det sammen

```
 Hvor kommer målpunktet fra?            C&A sine noder                         Simulatoren
 ───────────────────────────            ──────────────                         ───────────
 scenariofil (evaluatoren gir     ┐
   punktene i rekkefølge)         │
 ./scripts/njord goto X Y [H]     ├─▶ /njord/setpoint ──▶ guidance/regulator ──▶ /thruster_1/command … /thruster_N/command
 RViz «2D Goal Pose»              │    (PoseStamped)       + allokering            (Float64, newton)
 deres egen missionnode           ┘                                                   │
                                                                                      ▼
                                       /njord/odometry, /sensors/gps/fix,      command guard ──▶ Gazebo-fysikk
                                       /sensors/imu/data, /clock  ◀─────────────────────────────────┘
 Evaluatoren (bruker fasit, bare til måling):
   run_metrics.json, timeseries.csv, report.html, /sim/setpoint_status, RViz-markører
```

- **Målpunktet** er en posisjon og heading i kartrammen `map`.
- **C&A** regner ut kraft per thruster i newton.
- **Command guard** slipper kraften gjennom bare så lenge alle kommandoene er
  ferske. Ellers setter den kraften til null, og båten driver.
- **Evaluatoren** er dommer i et løp: den gir neste målpunkt når det forrige er
  nådd og holdt, og måler alt på fasit fra simuleringen.

## 2. Kom i gang på fem minutter (uten egne noder)

Simulatoren har en enkel referansekontroller (`setpoint_controller`) som gjør
akkurat dette. Kjør den først, så dere ser hva simulatoren forventer:

```bash
./scripts/njord pull          # eller ./scripts/njord build
SEED=1 ./scripts/njord demo goto_square                                              # WAM-V, to thrustere
SEED=1 VESSEL_CONFIG=/config/vessels/munin_v0.yaml ./scripts/njord demo goto_square  # Munin-plassholder, fire thrustere
./scripts/njord gui goto_square                                                      # samme løp med Gazebo-vindu og RViz
```

- **Under kjøring** skriver terminalen én linje per hendelse, for eksempel
  `Setpoint 1/4 'east' reached after 26.9 s: error 0.34 m / 0.2 deg`.
- **Til slutt** skrives `Report: outputs/run-…/report.html`. Åpne fila i en
  nettleser.
- **Exit-koden** er 0 bare når alle påkrevde målpunkt ble nådd uten kollisjon.

Målpunktkursene som følger med:

| Kurs | Hva den tester |
|---|---|
| `goto_square` | Fire hjørner av et kvadrat på 20 m, hvert med en heading som skal holdes i 5 s |
| `goto_retarget` | Nye målpunkt kommer før det forrige er nådd (`advance_after_s`), så et siste punkt som må nås |
| `station_keeping` | Kjør til ett punkt og hold posisjon og heading i 60 s (DP). Bruk `ENVIRONMENT=windy` (alle båter) eller `ENVIRONMENT=current` (bare Njord-profiler som `munin_v0`). WAM-V med to thrustere klarte ikke å holde headingen mot sidevind ([validation.md](validation.md#setpoint-courses-2026-10-06)); `munin_v0` klarte det. |

## 3. Koble til deres egne noder

1. **Velg båt.** Munin har fire faste thrustere. Inntil Naval har tall, bruk
   plassholderen `VESSEL_CONFIG=/config/vessels/munin_v0.yaml` (se
   [avsnitt 4](#thrustere-på-munin_v0)).
2. **Start nodene deres først**, med ROS 2 Jazzy:
   - samme `ROS_DOMAIN_ID` som simulatoren (standard 42)
   - **uten namespace**, slik at `thruster_1/command` blir `/thruster_1/command`
   - med `use_sim_time:=true` på alle noder

   ```bash
   source /opt/ros/jazzy/setup.bash
   source /sti/til/ca_ws/install/setup.bash
   export ROS_DOMAIN_ID=42
   ros2 launch <pakke> <launchfil> use_sim_time:=true
   ```

   Pakken kan også bygges og kjøres i simulator-imaget med team-containeren
   (`compose.team.yaml`); se [team-integration.md](team-integration.md#3-connect-your-own-ros-2-workspace).
   Gi den da et eget Compose-prosjektnavn, for eksempel
   `docker compose -p njord-ca -f compose.yaml -f compose.team.yaml run --rm team bash`.
   Ellers tilhører containeren samme prosjekt som løpet. Da kan
   `./scripts/njord demo` bli hengende etter at løpet er ferdig, til containeren
   deres avslutter (sett i én av to testkjøringer).
3. **Start en kjøring der simulatorens kontroller er slått av:**

   ```bash
   # Målt løp: evaluatoren gir punktene i scenariet ett og ett og avslutter kjøringen.
   CONTROLLER=external VESSEL_CONFIG=/config/vessels/munin_v0.yaml SEED=1 ./scripts/njord demo goto_square
   # Fri kjøring: dere sender punktene selv (avsnitt 6) og avslutter med Ctrl-C.
   CONTROLLER=external VESSEL_CONFIG=/config/vessels/munin_v0.yaml ./scripts/njord lab goto_square
   # Fasit i stedet for EKF-estimat på /njord/odometry (bare for å skille regulator- fra estimatorfeil):
   STATE_SOURCE=truth CONTROLLER=external VESSEL_CONFIG=/config/vessels/munin_v0.yaml ./scripts/njord demo goto_square
   ```

   Med `CONTROLLER=external` (eller `AUTONOMY=external`) på en målpunktkurs
   kjører følgende:
   - simulatoren, sensorene og estimeringen (EKF, eller fasit med `STATE_SOURCE=truth`),
   - command guard,
   - evaluatoren.

   Simulatorens egen kontroller kjører ikke. Ingen andre publiserer på
   thrustertopicene.
4. **Sjekk koblingen** i en annen terminal:

   ```bash
   docker compose exec simulator /entrypoint.sh ros2 topic info /thruster_1/command --verbose   # nøyaktig én publisher: deres node
   docker compose exec simulator /entrypoint.sh ros2 topic echo /njord/setpoint --once
   docker compose exec simulator /entrypoint.sh ros2 topic echo /njord/guard_status             # hvorfor kraft slippes eller stoppes
   ```

Et målpunktløp starter først når navigasjonen er klar, minst én node
abonnerer på `/njord/setpoint` **og** hver thrustertopic i vesselfila har en
publisher. Det spiller derfor ingen rolle om nodene deres starter før eller
etter simulatoren, heller ikke med GUI: RViz og `ros2 topic echo` abonnerer
også på målpunktet, men sender ikke kraft, så de alene starter ikke løpet. Til
da skriver evaluatoren hva den venter på, for eksempel
`Waiting for a publisher on /thruster_3/command (the controller) to start`.
Stopper thrusterkommandoene etter start, holder guarden kraften på null, og
løpet ender med `setpoints_failed` eller `simulation_timeout`.

## 4. Grensesnittet

### Inn til C&A

| Topic (foreløpig navn) | Type | Innhold | QoS og takt | Endres i |
|---|---|---|---|---|
| `/njord/setpoint` | `geometry_msgs/PoseStamped` | Målposisjon `pose.position.x/y` i meter og heading som `pose.orientation` (rotasjon om z), i rammen `map`. `header.stamp` er simuleringstiden da punktet ble gitt. | Reliable, publiseres transient local, depth 1. Sendes når punktet gis og gjentas deretter hvert sekund med **samme** stempel; samme stempel og pose betyr samme punkt. | `SETPOINT_TOPIC` i `njord_sim/njord_sim/constants.py` |
| `/njord/setpoint_sequence` | `nav_msgs/Path` | Valgfri lookahead: aktivt punkt først, så resten av rekkefølgen. Bare i et målt løp (`demo`/`gui`). | Som over, oppdatert når et nytt punkt gis | `SETPOINT_SEQUENCE_TOPIC` i `constants.py` |
| `/njord/odometry` | `nav_msgs/Odometry` | Pose av `base_link` i `map`. Hastighet (twist) i body-ramma (x fram, y babord). EKF-estimat, eller fasit med `STATE_SOURCE=truth`. | Best effort (sensor-QoS), ca. 20–30 Hz | [interfaces.md](interfaces.md) |
| `/sensors/gps/fix`, `/sensors/imu/data` | `sensor_msgs/NavSatFix`, `sensor_msgs/Imu` | Simulert GNSS/IMU med støy, for egen estimator | Best effort | `GPS_TOPIC`, `IMU_TOPIC` i `constants.py` |
| `/clock` | `rosgraph_msgs/Clock` | Simuleringstid. Krever `use_sim_time:=true`. | | |

### Ut fra C&A

| Topic | Type | Innhold | Krav | Endres i |
|---|---|---|---|---|
| `/thruster_1/command` … `/thruster_N/command` | `std_msgs/Float64` | Kraft i **newton** langs thrusterens retning. Positiv = skyv i retningen `yaw_deg`, negativ = revers. | Send alle thrusterne hver syklus, normalt 20 Hz. Guarden abonnerer best effort (depth 1), så både `RELIABLE` og `BEST_EFFORT` fra dere virker. Hver kommando må være under 0,5 s gammel i simuleringstid. Ellers setter guarden *alle* thrustere til null. | Navnene: `thrusters[].name` i vesselfila. Malen `/{name}/command`: `THRUSTER_COMMAND_TOPIC` i `constants.py`. |

Kraften begrenses til thrusterens grense i vesselfila og til `guidance.max_thrust`
i `algorithms.yaml` (500 N). Fysikken legger på en førsteordens forsinkelse
(`response_time_s`, 0,1 s på `munin_v0`). Null kraft stopper ikke båten; den
fortsetter med sin egen fart og driver med vind og strøm.

### Thrustere på `munin_v0`

Plasseringen er antatt («X»-oppsett) til Naval har målt. Posisjon er i
body-ramma `base_link` (x fram, y babord, z opp). `yaw_deg` er retningen på
positiv kraft, mot klokka fra rett fram.

| Navn | Posisjon [x, y, z] (m) | `yaw_deg` | Positiv kraft skyver | Grense fram/bak |
|---|---|---|---|---|
| `thruster_1` | [1.1, 0.55, -0.1] (baug, babord) | -45 | fram og mot styrbord | 500 / 500 N |
| `thruster_2` | [1.1, -0.55, -0.1] (baug, styrbord) | 45 | fram og mot babord | 500 / 500 N |
| `thruster_3` | [-1.1, 0.55, -0.1] (akter, babord) | 45 | fram og mot babord | 500 / 500 N |
| `thruster_4` | [-1.1, -0.55, -0.1] (akter, styrbord) | -45 | fram og mot styrbord | 500 / 500 N |

Med dette oppsettet kan båten skyve rett sideveis og dreie på stedet
(fullaktuert). WAM-V og `njord_v1` har to parallelle akterthrustere og kan ikke
skyve sideveis.

## 5. Regler som må følges

- **Koordinater.** `map` er ENU: x øst, y nord, origo i datumet i `constants.py`.
  Heading måles i radianer mot klokka fra øst (0 = øst, π/2 = nord). Bruker dere
  NED, gjelder: `heading_NED = π/2 − heading_ENU` og `(x_NED, y_NED) = (y_ENU, x_ENU)`.
  Flere formler finnes i [team-integration.md](team-integration.md#control--autonomy-munin-state-source-and-frames).
- **Siste punkt gjelder.** Et nytt målpunkt erstatter det aktive med en gang.
  Punktet utløper ikke; hold det til et nytt kommer.
- **Rekkefølge i et løp.** Evaluatoren gir neste punkt først når båten har vært
  innenfor toleransen i `hold_s` sekunder, når punktets `timeout_s` er ute
  (punktet teller da som feilet), eller etter `advance_after_s` (punktet er da
  valgfritt).
- **Fri heading.** Er headingen fri i scenariet, settes `orientation` til
  retningen fra forrige punkt, og headingen måles ikke.
- **Gammel input gir null kraft.** Nodene deres må sende null kraft selv når
  odometri eller målpunkt mangler eller er ugyldig. Guarden er bare et ekstra
  sikkerhetsnett: mangler en thrusterkommando i 0,5 s simuleringstid (eller
  2 s veggtid hvis klokka står), slippes ingen kraft.
- **`use_sim_time:=true` overalt.** Uten den sammenlignes simuleringsstempler
  med veggklokka, og alt ser gammelt ut.
- **QoS.** Abonner på `/njord/setpoint` med `RELIABLE`. Både `VOLATILE`
  (standard) og `TRANSIENT_LOCAL` virker. Evaluatoren gjentar det aktive
  punktet hvert sekund med uendret stempel, så en node som kobler seg på midt i
  et løp får punktet innen ett sekund. Behandle en melding med samme stempel og
  pose som det punktet dere allerede har, ikke som et nytt.
- **Én publisher.** I et løp er evaluatoren den eneste som skal publisere på
  `/njord/setpoint`; den advarer i loggen hvis det er flere. På hver
  thrustertopic skal det bare være deres node.
- **Ikke bruk `/sim/*`.** Alt under `/sim` er fasit fra simuleringen og bare til
  visning og måling.

## 6. Sende målpunkt

### a) Fra en scenariofil (repeterbart og målt)

Lag en fil i `scenarios/`, for eksempel `scenarios/ca_test.yaml`, med
`goto_square.yaml` som mal. Kjør `./scripts/njord build`, siden kursene ligger i
imaget, og deretter `./scripts/njord demo ca_test`.

```yaml
setpoints:
  defaults: {tolerance_m: 1.5, heading_tolerance_deg: 15.0, hold_s: 5.0, timeout_s: 100.0}
  sequence:
  - {name: p1, position: [25.0, 0.0], heading_deg_enu: 0.0}
  - {name: p2, position: [25.0, 20.0], heading_deg_enu: 90.0, hold_s: 10.0}
  - {name: p3, position: [0.0, 20.0]}                           # heading fri
  - {name: bytt, position: [10.0, 5.0], advance_after_s: 8.0}   # erstattes etter 8 s, nådd eller ikke
  - {name: hjem, position: [0.0, 0.0], heading_deg_enu: -90.0}
```

| Felt | Betydning |
|---|---|
| `name` | Unikt navn, brukt i logg og rapport |
| `position` | [x, y] i meter i `map` (ENU) |
| `heading_deg_enu` | Ønsket heading i grader mot klokka fra øst; utelatt = fri |
| `tolerance_m` | Maks avstand fra punktet for å regnes som «inne» |
| `heading_tolerance_deg` | Maks headingavvik for å regnes som «inne» (når heading ikke er fri) |
| `hold_s` | Hvor lenge båten må være inne uten avbrudd før punktet er nådd |
| `timeout_s` | Punktet feiler hvis det ikke er nådd innen denne tiden |
| `advance_after_s` | Gi neste punkt etter så mange sekunder uansett; punktet er da valgfritt. Kan ikke brukes på siste punkt. |

Feltene kan settes per punkt eller under `defaults`; det finnes ingen skjulte
standardverdier. En fil har enten `gates` (portløp) eller `setpoints`, aldri
begge. Den må ha minst ett hinder under `obstacles`, slik at kollisjoner kan
måles; malen har en bøye langt unna ruta. `timeout_s` på toppnivå er grensen
for hele løpet.

### b) Fra terminalen under `lab`

```bash
CONTROLLER=external VESSEL_CONFIG=/config/vessels/munin_v0.yaml ./scripts/njord lab goto_square   # terminal 1
./scripts/njord goto 15 5 90      # terminal 2: x=15 m, y=5 m, heading 90° (nord)
./scripts/njord goto 0 0          # uten heading: båten bes peke langs linja fra der den er
```

I `lab` publiserer ikke evaluatoren punkter selv; den måler hvert punkt dere
sender til det neste kommer. Uten `CONTROLLER=external` kjører simulatorens
referansekontroller dem. Akseptansen er fast i lab:
`OBSERVED_SETPOINT_ACCEPTANCE` i `constants.py` (1,5 m, 15°, hold 5 s).

### c) Fra RViz

`GUI=1 ./scripts/njord lab goto_square` åpner Gazebo og RViz. Velg «2D Goal
Pose» (tast G), klikk på målet og dra i ønsket heading-retning. Verktøyet
publiserer på `/njord/setpoint`.

### d) Fra deres egen node eller kommandolinje

Alt som publiserer en `PoseStamped` i `map` på `/njord/setpoint`, fungerer
(i `lab`; i et løp eier evaluatoren topicen):

```bash
ros2 topic pub --once /njord/setpoint geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 20.0, y: 10.0}, orientation: {z: 0.7071, w: 0.7071}}}"
```

## 7. Med eller uten GUI

| Kommando | Vindu | Hva skjer |
|---|---|---|
| `./scripts/njord demo <kurs>` | Ingen (headless) | Målt løp. Avslutter selv, exit-kode = resultat. |
| `./scripts/njord gui <kurs>` | Gazebo + RViz | Samme målte løp. Å lukke RViz stopper ikke løpet. |
| `./scripts/njord lab <kurs>` | Ingen | Fri kjøring. Punkt sendes med `goto`/egen node, og hvert punkt måles. Ctrl-C avslutter og skriver rapporten. |
| `GUI=1 ./scripts/njord lab <kurs>` | Gazebo + RViz | Som `lab`, pluss målpunkt med musa i RViz |

RViz viser:
- målpunktene som skiver med toleranseradius og headingpil, farget etter
  tilstand (grå venter, gul aktiv, grønn nådd, rød feilet, blå erstattet),
- det kjørte sporet,
- det aktive målpunktet.

## 8. Se hvor godt båten gjorde det

**Under kjøringen:**

- Terminalen viser én linje når et punkt gis og én når det ender, med sluttfeil i meter og grader.
- `ros2 topic echo /sim/setpoint_status` viser løpende status: aktivt punkt, avstand, headingavvik, tid siden punktet ble gitt og hvor lenge båten har vært innenfor.
- RViz viser det samme grafisk (avsnitt 7).

**Etterpå,** i `outputs/run-<tid>/`:

| Fil | Innhold |
|---|---|
| `report.html` | Rapport som åpnes i nettleser, uten internett: resultat, kart sett ovenfra med spor og målpunkt, tabell per punkt og grafer over avstand, headingfeil, fart og thrusterkraft over tid. `./scripts/njord report [mappe]` lager den på nytt. |
| `run_metrics.json` | Alle tall, pluss proveniens (commit, image, seed og scenario-hash) |
| `timeseries.csv` | 10 Hz: tid, posisjon, heading, fart, aktivt punkt, avstand, headingfeil og kraft per thruster |

Feltene per målpunkt i `run_metrics.json` (`setpoints[]`):

| Felt | Betydning |
|---|---|
| `outcome` | `reached`, `timeout`, `advanced` (erstattet etter `advance_after_s`), `superseded` (erstattet i lab) eller `unfinished` |
| `time_to_reach_s` | Tid fra punktet ble gitt til båten første gang var innenfor toleransen |
| `time_to_settle_s` | Tid til starten av holdet som fullførte punktet |
| `final_distance_m`, `final_heading_error_deg` | Feil da punktet endte |
| `hold_rms_distance_m`, `hold_max_distance_m`, `hold_rms_heading_error_deg`, `hold_max_heading_error_deg` | Presisjon under holdet (DP-mål) |
| `overshoot_m` | Hvor langt båten gikk forbi punktet, målt langs retningen fra der punktet ble gitt til punktet |
| `max_cross_track_m`, `path_length_m`, `path_efficiency` | Hvor rett båten kjørte: største avvik fra den rette linja før punktet ble nådd, kjørt distanse, og forflytning delt på kjørt distanse (1 = helt rett) |
| `force_impulse_ns` | ∫ Σ\|F\| dt for kraften guarden slapp gjennom (N·s). Et mål på pådrag, **ikke** elektrisk energi. |

**Status for hele kjøringen** (`status`):

| Status | Betydning |
|---|---|
| `completed` | Alle påkrevde punkt er nådd |
| `setpoints_failed` | Sekvensen ble ferdig, men minst ett påkrevd punkt ble ikke nådd |
| `collision` | Kollisjon med båten |
| `geometric_overlap` | Skrogets omhylling (envelope) traff et hinder |
| `simulation_timeout` | Tiden for løpet gikk ut |
| `wall_timeout`, `odometry_timeout`, `contact_monitor_timeout` | Infrastrukturen sviktet |
| `interrupted` | Løpet ble avbrutt |
| `stopped` | En lab-kjøring ble avsluttet |

All måling bruker fasit fra simuleringen. `state_source` i metrics sier om
nodene navigerte på estimat eller fasit.

## 9. Feilsøking

| Symptom | Sjekk |
|---|---|
| Båten står stille | `ros2 topic echo /njord/guard_status`. Meldingen sier hva som mangler, for eksempel `thruster 3 command missing` eller `stale in simulation time`. |
| `stale in simulation time` | Nodene mangler `use_sim_time:=true`, eller sender for sjelden. |
| Topic heter `/<ns>/thruster_1/command` | Nodene kjører i et namespace. Start uten, eller remap til `/thruster_N/command`. |
| Ser ingen topics | Samme `ROS_DOMAIN_ID`? Kjører dere utenfor Docker og ser ingenting, prøv `export FASTDDS_BUILTIN_TRANSPORTS=UDPv4` (samme som containerne). |
| Løpet starter aldri (`Waiting for a subscriber on /njord/setpoint` eller `Waiting for a publisher on /thruster_N/command`) | Ingen node abonnerer på målpunktet, eller en thrustertopic mangler publisher: feil topicnavn, namespace, antall thrustere eller `ROS_DOMAIN_ID`. |
| Flere publishers på en thrustertopic | Glemt `CONTROLLER=external`. Da kjører også simulatorens referansekontroller. |

## 10. Begrensninger

- **`munin_v0` er en plassholder.** Skrog, masse og hydrodynamikk er kopiert fra
  en analytisk testmodell, og thrusteroppsettet er antatt. Resultatene sier
  hvordan *regulatoren deres* oppfører seg mot denne modellen, ikke hvordan
  Munin vil oppføre seg.
- **Kraft, ikke PWM.** Simulatoren tar newton. Omregning fra PWM til kraft
  trenger målinger fra Electrical (åpent spørsmål 16 i
  [control-autonomy-questions.md](control-autonomy-questions.md)).
- **Referansekontrolleren** er en enkel sammenligningsbase med uinnstilte
  forsterkninger (`setpoint_control` i `algorithms.yaml`). Den unngår ikke
  hindringer.
- **Njord-profilene** (`munin_v0`, `njord_v1`) har flatt vann: konstant vind og
  strøm, ingen bølger eller vindkast.
- **Målpunktkursene** er åpent vann med én fjern bøye. Kartlegging,
  bøyedeteksjon og stiplanlegging fra referansekjeden kjører ikke på dem.

## 11. Navn og typer som må bekreftes med C&A

Alle navn under er valgt foreløpig. For å endre et navn: rediger stedet i
kolonnen «Endres i», oppdater RViz-oppsettet `njord_sim/config/njord.rviz` hvis
navnet står der, kjør `./scripts/njord build` og `./scripts/njord test`. Testen
`tests/test_parameter_contracts.py` sjekker at RViz-oppsettet bruker de samme
topicnavnene som `constants.py`.

| Hva | Foreløpig valg | Endres i | Å avklare med C&A |
|---|---|---|---|
| Topic for målpunkt | `/njord/setpoint` | `SETPOINT_TOPIC`, `njord_sim/njord_sim/constants.py` | Navn og namespace |
| Meldingstype for målpunkt | `geometry_msgs/PoseStamped` | Koden som bruker topicen: `evaluator_node.py`, `setpoint_controller_node.py`, `scripts/send_setpoint.py` | Trenger dere ønsket fart, toleranse eller ID i meldingen? (Krever i så fall en egen meldingstype eller et tillegg.) |
| Lookahead-sekvens | `/njord/setpoint_sequence`, `nav_msgs/Path` | `SETPOINT_SEQUENCE_TOPIC`, `constants.py` | Ønsker dere den i det hele tatt? |
| Heading-konvensjon | ENU, mot klokka fra øst | Fast i ROS-konvensjonen (REP 103); NED-adapter er åpent spørsmål 8 | Konverterer dere selv? |
| Thrustertopics | `/thruster_1/command` … `/thruster_4/command` | Navn: `thrusters[].name` i vesselfila. Mal: `THRUSTER_COMMAND_TOPIC` i `constants.py`. | Avtalt (september 2026); rekkefølge og plassering venter på Naval |
| Thrustertype | `std_msgs/Float64`, newton | `command_guard_node.py`, `setpoint_controller_node.py`, `guidance_node.py` | PWM senere? |
| Navigasjonstilstand | `/njord/odometry`, `nav_msgs/Odometry`, `map` → `base_link` | [interfaces.md](interfaces.md); rammenavn `MAP_FRAME`, `BASE_FRAME` i `constants.py` | Holder dette, eller vil dere ha egne meldinger fra GNSS/attitude-noden? |
| Toleranser og hold i lab | 1,5 m, 15°, 5 s | `OBSERVED_SETPOINT_ACCEPTANCE`, `constants.py` | Hvilke krav gjelder for DP (spørsmål 14)? |
| Visning og måling | `/sim/setpoint_status`, `/sim/setpoint_markers`, `/sim/trajectory` | `SETPOINT_STATUS_TOPIC`, `SETPOINT_MARKERS_TOPIC`, `TRAJECTORY_TOPIC`, `constants.py` | Bare til simulatorens visning; ikke for C&A-noder |
