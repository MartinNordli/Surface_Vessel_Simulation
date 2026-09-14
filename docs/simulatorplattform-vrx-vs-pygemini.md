# Valg av simulatorplattform for Njord NTNU

**Beslutningsgrunnlag: PyGemini sammenlignet med VRX, Gazebo og ROS 2**

**Dato:** 14. september 2026

## 1. Anbefaling

**Njord bør velge VRX + Gazebo + ROS 2 som hovedplattform for samlet testing av perception, control og autonomy.** Min vurdering er at denne kombinasjonen gir den mest direkte, dokumenterte veien til simulatoren laget trenger: en virtuell båt som mottar kontrollkommandoer, beveger seg fysisk og leverer nye sensormålinger tilbake til lagets programvare.

Anbefalingen gjelder **teknologifundamentet for videre utvikling**. Kodebasen i dette repositoriet er en demo som undersøker gjennomførbarheten. Den skal ikke vurderes som et ferdig alternativ som allerede dekker alle lagets behov.

Det viktigste beslutningskriteriet er derfor:

> Hvilken plattform lar simulation-gruppen bruke mest mulig av arbeidet på Njord-spesifikke modeller, scenarioer og tester, og minst mulig på å utvikle grunnleggende simulatorinfrastruktur?

VRX/Gazebo tilbyr dokumenterte byggeklosser for fartøymodeller, sensorer, aktuatorer, miljø og ROS-integrasjon. Dette er grunnlaget for anbefalingen. Se [VRX-prosjektet][vrx], [ROS 2-integrasjon][ros] og [Gazebos marine modeller][marine].

## 2. Formålet med simulatoren

Simulation-gruppens hovedoppgave er å gjøre det mulig å teste lagets programvare digitalt før testing på vann. Simulatoren bør støtte både enkeltgrupper og hele systemet:

| Gruppe | Hva simulatoren skal gjøre mulig |
|---|---|
| Perception | Teste deteksjon, sporing, sensorfusjon og kartlegging mot simulerte målinger. |
| Control | Teste regulatorer mot kraftbasert bevegelse, treghet, motstand og miljøpåvirkning. |
| Autonomy | Teste planlegging, beslutninger, oppdrag og håndtering av hindringer. |
| Hele laget | Teste hvordan algoritmene virker sammen, inkludert forsinkelser, feil og bortfall. |

Den sentrale testsløyfen er:

```text
Simulerte sensorer
       ↓
Lagets perception og tilstandsestimering
       ↓
Lagets autonomy og control
       ↓
Aktuatorer og fysisk båtrespons
       ↓
Nye sensormålinger → tilbake til lagets programvare
```

Simulatoren bør tilpasses lagets ROS-grensesnitt. Referansealgoritmene i demoen er utskiftbare hjelpemidler for å prøve denne sløyfen.

## 3. Hva sammenligningen omfatter

Vi sammenligner to **utviklingsplattformer**, inkludert realistiske utvidelsesmuligheter. Manglende funksjoner i demoen er ikke automatisk begrensninger ved VRX/Gazebo. Tilsvarende må mulige utvidelser av PyGemini tas med.

| Betegnelse | Betydning |
|---|---|
| **Tilgjengelig** | Dokumenterte byggeklosser finnes. Konfigurasjon og integrasjon gjenstår. |
| **Utviklingsarbeid** | En konkret implementasjonsvei finnes, men laget må bygge eller koble sammen funksjonalitet. |
| **Uavklart** | Kildene er utilstrekkelige til å fastslå støtte eller arbeidsmengde. |

Vurderingen av PyGemini bygger på [forfatternes artikkel fra juni 2025][pygemini]. Et oppdatert maritimt koderepositorium og Njord-gruppens implementasjon er ikke kontrollert. Nyere tillegg kan derfor endre sammenligningen.

Vurderingen av VRX/Gazebo bygger på offisiell dokumentasjon og den tidligere gjennomgangen av demoen. Dokumenterte plattformmuligheter er ikke en bekreftelse på at alle funksjonene virker sammen i dagens container. Valgte versjoner og plugins må integreres og testes.

## 4. Muligheter med VRX + Gazebo + ROS 2

| Behov | Plattformens muligheter | Arbeid som gjenstår for Njord |
|---|---|---|
| **Teste ROS 2-noder** | Tilgjengelig kommunikasjon mellom ROS 2 og Gazebo gjennom `ros_gz`. | Tilpasse topics, meldingstyper, QoS, TF og simuleringstid. Enkelte meldinger krever adaptere. |
| **Teste hele algoritmekjeden** | Sensorer, fysikk og aktuatorer kan inngå i én lukket testsløyfe. | Integrere lagets noder og definere testkriterier. |
| **Modellere Njord-båten** | Egne fartøymodeller med geometri, masse, treghet, ledd og sensorer. Ingen binding til WAM-V. | Lage og kalibrere modell av skrog, fremdrift og sensoroppsett. |
| **Akselerasjon, bremsing og svinging** | Kraftbaserte aktuatorer og fysisk bevegelse. | Tilpasse motorkurver, grenser og respons. |
| **Hydrodynamikk** | Modeller for oppdrift, demping, tilleggstreghet og Coriolis-ledd finnes. | Velge en sammenhengende modell og bestemme parametere. |
| **Vind og bølger** | VRX har konfigurerbare modeller. | Definere relevante miljøforhold og kontrollere fartøyets respons. |
| **Strøm** | Gazebos hydrodynamikk støtter vannstrøm. | Integrere dette med valgt fartøy- og bølgemodell. |
| **Kamera, LiDAR, GPS og IMU** | Ferdige sensormodeller finnes. | Tilpasse plassering, frekvens, synsfelt, oppløsning og støy. |
| **Flere kameratyper** | Dybde-, RGB-D-, vidvinkel- og termiske kameraer finnes. | Konfigurere sensor og ROS-grensesnitt etter behov. |
| **Merkede treningsbilder** | Kameraer for segmentering og bounding boxes finnes. | Lage datasettgenerator, scenevariasjon og eksport. |
| **Andre fartøy og hindringer** | Flere modeller kan styres av egne kontrollere eller plugins. | Utvikle trafikkatferd, møtescenarioer og eventuell COLREGs-logikk. |
| **Sensor- og aktuatorfeil** | Kan implementeres med adaptere og plugins. | Lage modeller for bortfall, forsinkelse, bias og redusert motorkraft. |
| **Automatiske testserier** | Kjøring uten GUI og med sensorrendering støttes. | Bygge testorkestrering, scoring og resultatarkivering. |
| **Spesialisert radar og vannoptikk** | Plattformen kan utvides, men en egnet ferdig løsning er ikke verifisert her. | Undersøke kompatible tillegg eller utvikle egne modeller. Potensielt omfattende arbeid. |

Grunnlaget for tabellen er [ROS-integrasjonen][ros], [sensorbiblioteket][sensors], [marine modeller og aktuatorer][marine], [hydrodynamikk og strøm][hydro], [egne plugins][plugins], [kjøring uten GUI][headless] og [VRXs tilpasningsveiledninger][tutorials].

### Modellvalg er ikke plattformgrenser

At demoen bruker WAM-V, en bestemt sensoroppløsning eller null tilleggstreghet i enkelte retninger, betyr ikke at plattformen er begrenset til disse valgene. De kan endres eller erstattes.

Gazebo dokumenterer både hydrodynamiske parametere og utvidelse gjennom egne systemplugins. Det gir en konkret vei til mer avanserte modeller. Ulike fysikkmotorer og versjoner har forskjellige egenskaper, og samme kraftbidrag skal ikke beregnes av flere modeller samtidig. Se [hydrodynamikkdokumentasjonen][hydro].

## 5. Muligheter med PyGemini

| Område | Dokumentert grunnlag og utvidelsesmulighet |
|---|---|
| **ROS 2** | Native node gjennom `rclpy`. |
| **Kamera og LiDAR** | Sensorfunksjoner er dokumentert. |
| **Rekonstruksjon og treningsdata** | Gaussian Splatting, annotering og bildeaugmentering. |
| **Bevegelse** | Baner og SeaState-forstyrrelser. |
| **Kraftbasert fartøydynamikk** | Mangler i 2025-beskrivelsen; integrasjon av eksterne fysikkbiblioteker foreslås. |
| **Komplett fysisk autonomisløyfe** | Beskrevet som videre arbeid. |
| **Egne utvidelser** | Modulær Python-arkitektur. |
| **Radar** | Planlagt i 2025-beskrivelsen. |

Kilde: [PyGemini-artikkelen, del IV og V][pygemini]. Funksjoner som krever videreutvikling, skal ikke omtales som umulige. PyGemini har et relevant utgangspunkt for sensor- og dataarbeid.

## 6. Hvilken plattform passer oppgaven best?

Dette er en teknisk vurdering av forventet utviklingsarbeid, ikke en måling av utviklingstid eller en direkte ytelsesbenchmark.

| Prioritet | Vurdering |
|---|---|
| **Samlet testing av perception, control og autonomy** | VRX/Gazebo anbefales: dokumenterte byggeklosser dekker hele den fysiske testsløyfen. |
| **Njord-spesifikk fartøymodell** | VRX/Gazebo gir en dokumentert vei gjennom egne modeller og marine fysikksystemer. Parametere må fremskaffes uansett plattform. |
| **Spesialisert generering av treningsdata** | PyGemini bør vurderes nærmere dersom dette blir en hovedleveranse. |
| **Lav terskel for lagets ROS-brukere** | Prioriter et stabilt ROS-grensesnitt. Unngå at algoritmegruppene må kjenne simulatorens interne arkitektur. |
| **Overlevering mellom studentkull** | Vektlegg dokumentasjon, vedlikeholdere og tilgjengelig hjelp. Et større upstream-prosjekt fritar ikke laget fra å dokumentere egne tillegg. |

VRX har offentlig kildekode, veiledninger, issue tracker og dokumenterte utgivelser, inkludert støtte for ROS 2 Jazzy og Gazebo Harmonic. Dette gir et etterprøvbart grunnlag for å vurdere vedlikehold og støtte. Se [prosjektbeskrivelsen][vrx] og [utgivelsene][releases].

## 7. Argumentene for å velge VRX/Gazebo

### Arbeidet kan konsentreres om Njords behov

Med sensorer, aktuatorer, fysikk og ROS-kobling tilgjengelig kan gruppen prioritere båtmodell, scenarioer og integrasjon. Dette er min hovedbegrunnelse for valget. Den forventede gevinsten er mindre utvikling av grunnleggende infrastruktur.

### Control trenger en fysisk responsmodell

Regulatoren må kunne testes mot akselerasjon, treghet, motstand og svingerespons. En feil kommando bør få konsekvenser for den videre båttilstanden og dermed for kommende sensormålinger. Dette gjør det mulig å undersøke samspillet mellom kontroll, planlegging og perception.

### Simulatoren kan utvikles gradvis

En første modell kan gi verdi gjennom testing av meldingsflyt, koordinatsystemer, algoritmesamspill og feiltilstander. Deretter kan modellen forbedres for å svare mer presist på spørsmål om manøvrering. All kalibrering trenger ikke være ferdig før simulatoren tas i bruk.

### Demoen undersøker en relevant teknisk retning

Demoens rolle er å vise at valgte byggeklosser kan kobles sammen i et konkret prosjekt. Hvor god demonstrasjonsregulatoren er, eller hvilke scenarioer den klarer, bør ikke styre plattformvalget. Lagets egne algoritmer skal kunne overta.

## 8. Usikkerheter som faktisk påvirker beslutningen

| Usikkerhet | Betydning |
|---|---|
| **Nyere PyGemini-kode og gruppens tillegg** | Kan redusere forskjellen i nødvendig grunnarbeid. Bør gjennomgås før et endelig plattformvedtak. |
| **Tilgjengelig NTNU-kompetanse** | God lokal veiledning kan påvirke utviklingskostnaden betydelig. |
| **Behov for visuell realisme** | Avklar hvilke sensorfeil perception faktisk trenger å teste. Fine bilder alene dokumenterer ikke sensorrealisme. |
| **Data om Njord-båten** | Presise prediksjoner krever relevante modellparametere og sammenligning med målinger. |
| **Maskinvare og ytelse** | Mål ytelsen med ønsket sensoroppsett og lagets noder. Ingen generell garanti om sanntid eller bitidentiske resultater legges til grunn. |
| **Personavhengighet** | Flere medlemmer må kunne bygge, konfigurere og videreutvikle simulatoren. |

Påstandene om få PyGemini-brukere, liten utviklingsaktivitet og gruppens fremdrift er ikke tilstrekkelig verifisert. De bør ikke brukes som hovedargumenter. Heller ikke påstanden om lavere læringskurve er målt; dette bør vurderes gjennom en konkret integrasjonsoppgave.

## 9. Forslag til beslutningstekst

> Njord velger VRX/Gazebo som hovedretning for videre utvikling av simulatoren. Plattformen tilbyr dokumenterte byggeklosser for ROS 2-integrasjon, simulerte sensorer, fysisk bevegelse og egne fartøymodeller. Dette gir simulation-gruppen et egnet fundament for å teste perception, control og autonomy samlet før forsøk på vann.
>
> Den eksisterende demoen er en utprøving av gjennomførbarhet. Videre arbeid prioriteres mot lagets grensesnitt, relevante testscenarioer og gradvis tilpasning av båtmodellen. PyGemini vurderes som et mulig supplement for spesialisert sensor- og dataarbeid, og nyere funksjoner gjennomgås før det endelige plattformvedtaket.

[vrx]: https://github.com/osrf/vrx
[releases]: https://github.com/osrf/vrx/releases
[tutorials]: https://github.com/osrf/vrx/wiki/tutorials
[ros]: https://gazebosim.org/docs/harmonic/ros2_integration/
[sensors]: https://gazebosim.org/api/sensors/8/namespacegz_1_1sensors.html
[marine]: https://gazebosim.org/api/sim/8/underwater_vehicles.html
[hydro]: https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1Hydrodynamics.html
[plugins]: https://gazebosim.org/api/sim/8/createsystemplugins.html
[headless]: https://gazebosim.org/api/sim/8/headless_rendering.html
[pygemini]: https://arxiv.org/html/2506.06262v1
