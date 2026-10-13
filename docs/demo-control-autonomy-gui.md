# Demo med Control & Autonomy: med vinduer (GUI)

Denne veiledningen viser de samme kjøringene som
[demo-control-autonomy.md](demo-control-autonomy.md), men med Gazebo- og
RViz-vinduene åpne: hva som startes, hva dere ser i hvert vindu, hva dere
kan klikke på, og hva som skjer når kjøringen er ferdig.

Grensesnittet, konverteringen til NED og oppsettet av demo-PC-en er likt med
og uten vinduer og står bare i
[demo-control-autonomy.md](demo-control-autonomy.md) (avsnitt 1–3). Gjør
avsnitt 2.1–2.4 der før dere begynner her.

## 1. Hva vinduene endrer

| Kommando | Vinduer | Hva skjer |
|---|---|---|
| `./scripts/njord demo <kurs>` | Ingen | Målt løp. Avslutter selv, og exit-koden er resultatet. |
| `./scripts/njord gui <kurs>` | Gazebo + RViz | Samme målte løp, samme rapport og samme exit-kode. |
| `./scripts/njord lab <kurs>` | Ingen | Fri kjøring. Dere sender målpunkt selv. Ctrl-C avslutter og skriver rapporten. |
| `GUI=1 ./scripts/njord lab <kurs>` | Gazebo + RViz | Samme frie kjøring med vinduer. |

`gui <kurs>` og `GUI=1 demo <kurs>` er det samme. Alle andre innstillinger
(`SEED`, `ENVIRONMENT`, `VESSEL_CONFIG`, `STATE_SOURCE`, `CONTROLLER`) virker
likt med og uten vinduer. Vinduene viser bare; fysikken, målingen og
rapporten er de samme.

## 2. Før dere starter

- Kjør fra en terminal **på skrivebordet** til demo-PC-en, som brukeren som er
  logget inn. Over SSH åpnes ingen vinduer.
- Sjekk at terminalen har en skjerm: `echo $DISPLAY` skal skrive noe, for
  eksempel `:0` eller `:1`.
- På WSL2 trengs ingenting ekstra; `scripts/njord` legger selv til
  `compose.wsl.yaml`, og vinduene kommer opp som vanlige Windows-vinduer.
- De to vinduene åpnes oppå hverandre (RViz 1440 × 900, Gazebo 1000 × 845).
  Flytt dem ved siden av hverandre med en gang: et målt løp starter 7–8 s
  etter kommandoen når regulatoren er klar.

## 3. De to vinduene

### Gazebo: selve simuleringen

![Gazebo-vinduet under hold_position med munin_v0](media/demo-gui-gazebo.png)

- Viser havet, rutenettet og båten. `munin_v0` er en plassholder og tegnes som
  en grå kasse, ikke som et skrog.
- Kameraet står stille og følger ikke båten. Båten holder seg i bildet i
  `hold_position` og `station_keeping`; etter lange `goto`-steg må dere flytte
  kameraet selv.
- Nede til høyre står sanntidsfaktoren. Den viste 90–94 % i kjøringene i
  avsnitt 10.
- `Entity Tree` til høyre lister `vessel` (båten) og `reference_buoy` (en bøye
  25 m unna som bare er der for kollisjonsmålingen).
- **Ikke lukk Gazebo-vinduet under en kjøring.** Gazebo spør
  `Do you really want to exit?`. Velg `CANCEL`; simuleringen går videre mens
  dialogen står der. Med `OK` stopper simulatoren: et målt løp ender da med
  `Race contact_monitor_timeout` og exit-kode 2, og en fri kjøring blir
  stående uten simulator til dere trykker Ctrl-C.
- **Ikke trykk pause eller reset** (de oransje knappene nede til venstre).
  Simuleringstiden stopper da for alle noder. Etter koden avslutter
  evaluatoren løpet med `odometry_timeout` når simuleringen har stått stille
  i 30 s, og med `clock_reset` hvis klokka går bakover. Dette er ikke prøvd.
- Gazebo viser ikke målpunktet. Det ser dere i RViz.

### RViz: målpunkt, spor og sensorer

![RViz zoomet inn på båten: ett nådd og ett aktivt målpunkt](media/demo-gui-rviz.png)

Bildet er fra en fri kjøring: punkt 1 er nådd (grønt), punkt 2 er aktivt
(gult), den røde kassa er båten, og den hvite streken er sporet.

| I RViz | Betyr |
|---|---|
| Rød kasse | Båten (`munin_v0`) |
| Skive | Et målpunkt. Radien er toleransen: 1,0 m i `hold_position` og `station_keeping`, 1,5 m i fri kjøring. |
| Farge på skiva | Gul: aktivt. Grønn: nådd. Rød: ikke nådd i tide. Blå: erstattet av et nytt punkt. Grå: venter. |
| Pil på skiva | Headingen båten skal ha |
| Tekst over skiva | Nummer, navn og tilstand, for eksempel `1 hold (active)` |
| Hvit strek | Sporet båten har kjørt (fasit) |
| Rutenett | 5 m per rute. x (øst) er mot høyre og y (nord) oppover i startvisningen. |
| `Front left camera`, `Front right camera` | Bildene fra de to kameraene på båten |

`Observed occupancy`, `D Star Lite route` og `Current mission goal` i lista
`Displays` er tomme i målpunktkursene, og `Observed occupancy` har et oransje
varselikon. Det er som forventet: kart, planlegger og oppdragsnode kjører bare
i portkursene.

**RViz starter langt unna.** Startvisningen dekker hele kursområdet (95 m
avstand), så båten og målpunktet er bare noen få piksler til venstre for
midten. Gjør dette først:

1. Velg `Focus Camera` i verktøylinja (tast C) og klikk på båten. Da blir
   båten midtpunktet for visningen.
2. Rull med musehjulet for å zoome inn. Zoomen går mot midtpunktet, ikke mot
   musepekeren, så uten steg 1 forsvinner båten ut av bildet.
3. Venstre museknapp og dra roterer, midtre museknapp og dra flytter
   visningen.

Hold verktøyet `Move Camera` valgt (tast M) når dere bare ser på.

## 4. Målt løp med vinduer, simulatorens egen regulator

Dette er generalprøve A i [demo-control-autonomy.md](demo-control-autonomy.md)
med vinduer. Ingen C&A-kode brukes.

```bash
SEED=1 ENVIRONMENT=current VESSEL_CONFIG=/config/vessels/munin_v0.yaml ./scripts/njord gui hold_position
```

Det som skjer, i rekkefølge:

1. Terminalen skriver `Image … matches this checkout (executable)` og starter
   containerne.
2. RViz-vinduet og Gazebo-vinduet åpnes. Løpet startet 7–8 s etter
   kommandoen.
3. Terminalen skriver (hver linje har i tillegg et prefiks med containernavn
   og tid):

   ```
   [evaluator]: 2 publishers on /njord/setpoint; the evaluator must be the only one during a setpoint race
   [evaluator]: Setpoint 1/1 'hold' issued at 0.0 s: x=0.00 y=0.00 heading=0.0 deg
   [evaluator]: Required heartbeats and contacts ready; race started
   ```

   Den første linja er en advarsel som bare kommer med vinduer. Den andre
   publisheren er verktøyet `2D Goal Pose` i RViz. Den er ufarlig så lenge
   ingen bruker verktøyet under et målt løp (avsnitt 6).
4. I RViz ligger en gul skive med teksten `1 hold (active)` under båten. I
   Gazebo ligger båten omtrent stille mot vind og strøm.
5. Etter 60 s simuleringstid:

   ```
   [evaluator]: Setpoint 1/1 'hold' reached after 60.0 s: error 0.24 m / 0.1 deg
   [evaluator]: Race completed; metrics: /outputs/run_metrics.json, report: /outputs/report.html
   Report: …/outputs/run-<tid>/report.html
   ```

   Skiva i RViz blir grønn, og teksten blir `1 hold (reached)`.
6. Gazebo-vinduet lukkes av seg selv, og kommandoen avslutter. `echo $?` skal
   gi 0. **RViz-vinduet blir stående** (avsnitt 7).

## 5. Målt løp med vinduer, ekstern regulator

Dette er generalprøve B med vinduer, og rad 2 og 3 i kjøreplanen på demodagen.
Simulatorens regulator er slått av, og en ekstern regulator sender kraft per
thruster. Under står stand-in-regulatoren `validation/ned_dp_standin.py`; på
demoen starter C&A sine egne noder i stedet
([demo-control-autonomy.md](demo-control-autonomy.md), avsnitt 4.1).

Terminal 1 (regulatoren først, så driver ikke båten mens den venter):

```bash
export NJORD_UID=$(id -u) NJORD_GID=$(id -g)
mkdir -p outputs/demo
VESSEL_CONFIG=/config/vessels/munin_v0.yaml docker compose -p njord-ca run --rm autonomy \
  python3 /opt/njord/validation/ned_dp_standin.py
```

Forventet: `Stand-in controller for 4 thrusters; waiting for /njord/setpoint`.

Terminal 2 (løpet med vinduer):

```bash
STATE_SOURCE=truth CONTROLLER=external SEED=1 ENVIRONMENT=current \
  VESSEL_CONFIG=/config/vessels/munin_v0.yaml ./scripts/njord gui hold_position
```

Vinduene og terminallinjene er som i avsnitt 4. Terminal 1 skriver i tillegg
når løpet starter:

```
New setpoint in NED: x=0.00 m (north), y=0.00 m (east), psi=90.0 deg
Sending thrust
```

Forventet til slutt i terminal 2: `reached after 60.0 s: error 0.31 m / 0.0 deg`
og exit-kode 0.

Bytt `hold_position` med `station_keeping` for et sprang på 10 m. Da ligger
den gule skiva to ruter (10 m) til høyre for båten i RViz, og båten kjører
dit og holder (`reached after 66.0 s`). Regulatoren i terminal 1 kan stå og
gå mellom løpene.

Står løpet og venter, skriver terminal 2 hva som mangler:
`Waiting for a subscriber on /njord/setpoint, a publisher on /thruster_1/command … (the controller) to start`.
RViz abonnerer også på `/njord/setpoint`, men teller ikke som regulator:
løpet starter først når alle fire thrustertopicene har en publisher.

## 6. Fri kjøring med vinduer

Her går det ikke noe løp. Dere sender målpunkt selv og ser båten kjøre dit.
Dette er sjekken før første løp på demodagen og «små steg for hånd» i
kjøreplanen.

Terminal 1: regulatoren, som i avsnitt 5 (eller C&A sine noder).

Terminal 2:

```bash
STATE_SOURCE=truth CONTROLLER=external ENVIRONMENT=current \
  VESSEL_CONFIG=/config/vessels/munin_v0.yaml GUI=1 ./scripts/njord lab hold_position
```

Vinduene åpnes som før, men RViz viser ingen skive ennå: i fri kjøring finnes
det ikke noe målpunkt før dere sender ett. **Båten driver med vind og strøm
til det første punktet kommer.** I kjøringen i avsnitt 10 drev den 7,1 m på
33 s. Send derfor det første punktet med en gang vinduene er oppe.

Terminal 3 (i simulator-repoet):

```bash
./scripts/njord goto 2 0 0      # x = 2 m øst, y = 0, heading 0° (øst)
./scripts/njord goto 6 3 90     # x = 6 m, y = 3 m, heading 90° (nord)
```

Forventet:

- `goto` svarer `Sent /njord/setpoint to 3 subscriber(s): x=2.00 m, y=0.00 m, heading=0.0 deg (ENU, from east)`.
  De tre er evaluatoren, regulatoren og RViz.
- Terminal 2 skriver `Setpoint 1 'setpoint_1' issued at … s: x=2.00 y=0.00 heading=0.0 deg`
  og litt senere `… reached after … s: error … m / … deg`.
- I RViz kommer en gul skive med pil og teksten `1 setpoint_1 (active)`.
  Båten kjører dit, og skiva blir grønn når båten har vært innenfor 1,5 m og
  15° i 5 s. Sendes et nytt punkt før det forrige er nådd, blir det forrige
  blått.
- Går båten motsatt vei eller sideveis, er det konverteringen eller
  thrusteroppsettet som avviker
  ([demo-control-autonomy.md](demo-control-autonomy.md), avsnitt 1).

**Målpunkt med musa:** velg `2D Goal Pose` i verktøylinja i RViz (tast G),
klikk der båten skal, hold knappen inne og dra i den retningen baugen skal
peke. RViz sender da punktet på `/njord/setpoint`, på samme måte som `goto`.
Verktøyet er koblet til riktig topic, men selve klikket er ikke prøvd
(avsnitt 10). Bruk det bare i fri kjøring. I et målt løp eier evaluatoren
målpunktet, og et klikk ville gitt regulatoren et punkt som ikke måles.

Avslutt med Ctrl-C i terminal 2. Gazebo-vinduet lukkes, og terminalen skriver
`Report: …/outputs/run-<tid>/report.html` med ett avsnitt per punkt dere
sendte. RViz blir stående.

## 7. Etter kjøringen

- **RViz blir stående.** RViz kjører i en egen container, så å lukke vinduet
  stopper aldri en kjøring, men det blir også stående etter at kjøringen er
  ferdig, med sporet og skivene fra den. Neste `gui`- eller `GUI=1`-kommando
  bytter det ut med et nytt vindu av seg selv. Etter siste kjøring lukker
  dere vinduet, eller kjører `docker rm -f njord-sim-rviz-1`.
- **Rapporten** ligger i `outputs/run-<tid>/report.html` og er den samme som
  uten vinduer: kart med spor og målpunkt, tabell per punkt og grafer over
  avstand, headingfeil, fart og kraft per thruster.
- **Regulatoren i terminal 1** stoppes med Ctrl-C når dere er ferdige.

## 8. Demodagen med vinduer

Kjøreplanen står i [demo-control-autonomy.md](demo-control-autonomy.md),
avsnitt 4.3, og bruker allerede `gui`. Med vinduer blir rekkefølgen:

| # | Hva | Kommando | I vinduene |
|---|---|---|---|
| 0 | Sjekk koblingen | Avsnitt 6 med C&A sine noder | Skive per `goto`, båten kjører dit |
| 1 | Simulatoren alene | Avsnitt 4 | Gul skive under båten, grønn etter 60 s |
| 2 | C&A holder posisjon | Avsnitt 5, `hold_position` | Som 1, men med C&A sin regulator |
| 3 | C&A kjører 10 m og holder | Avsnitt 5, `station_keeping` | Skiva 10 m øst for båten |
| 4 | Små steg for hånd | Avsnitt 6 | Én skive per punkt |

Hver kjøring åpner begge vinduene på nytt. Flytt dem fra hverandre og zoom
inn i RViz igjen (avsnitt 3); visningen huskes ikke fra forrige kjøring.

## 9. Feilsøking for vinduene

| Symptom | Årsak og tiltak |
|---|---|
| Ingen vinduer åpnes | Kjør fra en terminal på skrivebordet, ikke over SSH, og som brukeren som er logget inn. `echo $DISPLAY` må skrive noe. |
| Bare RViz åpnes, eller bare Gazebo | Se i terminalen etter feil fra `simulator-1` (Gazebo) eller kjør `docker logs njord-sim-rviz-1` (RViz). |
| RViz er nesten tomt | Startvisningen er 95 m unna. Bruk `Focus Camera` på båten og zoom inn (avsnitt 3). |
| RViz viser spor og skiver etter at kjøringen er ferdig | RViz blir stående til neste kjøring bytter det ut. Lukk det selv etter siste kjøring (avsnitt 7). |
| Ingen skive i RViz i fri kjøring | Det er ikke sendt noe målpunkt ennå. Kjør `./scripts/njord goto …`. |
| Båten driver bort i fri kjøring | Regulatoren har ikke fått noe målpunkt, eller den er ikke startet. Send et punkt, og sjekk `guard_status` ([demo-control-autonomy.md](demo-control-autonomy.md), avsnitt 4.2). |
| `2 publishers on /njord/setpoint` i terminalen | Forventet med vinduer: RViz sitt `2D Goal Pose`. Ufarlig når ingen klikker under et målt løp. |
| Løpet avslutter med `contact_monitor_timeout` og exit-kode 2 | Gazebo-vinduet ble lukket med `OK`. Start kjøringen på nytt, og velg `CANCEL` hvis dialogen kommer. |
| Løpet avslutter med `odometry_timeout` | Simuleringen sto trolig på pause i Gazebo. Start kjøringen på nytt. |
| Båten forsvinner ut av Gazebo-bildet | Kameraet følger ikke båten. Følg med i RViz, eller flytt kameraet i Gazebo. |

Andre symptomer (løpet starter aldri, båten går feil vei, `stale in
simulation time`) er like med og uten vinduer og står i
[demo-control-autonomy.md](demo-control-autonomy.md), avsnitt 5.

## 10. Hva som er prøvd

Prøvd 13. oktober 2026 på GPU (RTX 5090, WSL2 med WSLg), `munin_v0`,
`ENVIRONMENT=current`, seed 1, ved commit e3ecf11. Vinduene er lest fra
skjermbilder som ble tatt av hvert vindu mens kjøringene gikk; de to bildene
over er derfra. Ingen person har sett på dem ennå.

| Kjøring | Resultat |
|---|---|
| `gui hold_position`, simulatorens regulator | Fullført, exit 0: `reached after 60.0 s: error 0.24 m / 0.1 deg`. Hold: 0,36 m RMS, 0,72 m maks. Skiva gikk fra gul til grønn. |
| `gui hold_position`, stand-in, `STATE_SOURCE=truth CONTROLLER=external` | Fullført, exit 0: `reached after 60.0 s: error 0.31 m / 0.0 deg`. Hold: 0,30 m RMS, 0,33 m maks. |
| `gui station_keeping`, stand-in, samme innstillinger og samme regulatorprosess | Fullført, exit 0: `reached after 66.0 s: error 0.31 m / 0.0 deg` (5,9 s fram, 60 s hold). Hold: 0,33 m RMS, 0,99 m maks. |
| `GUI=1 lab hold_position`, stand-in, `STATE_SOURCE=truth CONTROLLER=external` | `goto 2 0 0` nådd etter 12,2 s (0,52 m / 0,0°), `goto 6 3 90` nådd etter 6,7 s (0,26 m / 0,3°). Ctrl-C skrev rapporten med begge punktene. |

Også prøvd:

- Rulling, venstre dra og midtre dra i RViz, med syntetiske musehendelser.
- At RViz står igjen etter både et løp og en fri kjøring, at neste
  `gui`-kommando bytter det ut med ett nytt vindu, og at
  `docker rm -f njord-sim-rviz-1` fjerner det.
- Å lukke Gazebo-vinduet: dialogen kom, og `OK` ga
  `Race contact_monitor_timeout` og exit-kode 2 i et løp. I en fri kjøring
  stoppet simulatoren, og kommandoen ble stående til Ctrl-C.

Ikke prøvd:

- Et klikk med `2D Goal Pose` eller `Focus Camera` i RViz. Gazebo-vinduet lå
  oppå verktøylinja, så verktøyene lot seg ikke velge med syntetiske klikk.
  `ros2 topic info /njord/setpoint` viste at RViz har en publisher der.
- Pause, reset og kameraflytting i Gazebo.
- Vinduene på en Linux-PC uten WSL.
- C&A sin egen kode.

`munin_v0` er en plassholder med antatt skrog og thrusteroppsett. Det dere
ser i vinduene sier noe om regulatoren og koblingen, ikke om hvordan Munin
vil oppføre seg eller se ut.
