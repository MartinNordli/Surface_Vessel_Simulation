# Åpne spørsmål til Control & Autonomy og Naval

Simulatoren skal kjøre koden til Control & Autonomy (C&A) uendret og oppføre
seg som Munin før båten er bygget. Svarene under bestemmer grensesnittet og
modellen. Før inn svaret og datoen under hvert spørsmål når det er avklart, og
lenk til endringen i simulatoren hvis svaret krevde en.

Ting som ikke er bestemt ennå, er helt greit: skriv «ikke bestemt» og hva det
venter på.

## Allerede avklart (svar fra C&A, september 2026)

- Munin får **fire faste thrustere**, ikke azimuth. Thrusterne styres med PWM;
  omregning fra kraft til PWM gjøres senere av C&A og Electrical, så
  simulatoren tar kraft i newton.
- Allokeringsnoden publiserer `std_msgs/Float64` på `thruster_1/command` …
  `thruster_4/command`. Simulatoren lytter nå på `/thruster_1/command` …
  `/thruster_4/command` (se [interfaces.md](interfaces.md)).
- En node skal motta GNSS og attitude fra sensorene; den er ikke skrevet ennå.
- Ingen masse-, treghets- eller hydrodynamikkdata før båten er ferdig
  (februar). Ingen rosbags fra Munin ennå.
- Forstyrrelser: bølger og strøm er trolig små i konkurranse, men det er
  ønskelig å kunne teste mot dem.
- Både fasit fra simuleringen og simulert GNSS/IMU til egen estimator er
  aktuelt.
- Måltall tar utgangspunkt i Autodrone-reglene: banefølgingsfeil, tid og
  kollisjoner.
- Mange kjøringer raskere enn sanntid er aktuelt for parametersøk og
  eventuelt RL.
- Viktigst dette halvåret: vanlig fremdrift med stiplanlegging gjennom
  hindringer, og dynamisk posisjonering (DP).

## Thrustere og aktuering (Naval Lead Jenny Grinden og C&A)

Fram til disse er besvart bruker simulatoren plassholderen
[`munin_v0.yaml`](../njord_sim/config/vessels/munin_v0.yaml) med et antatt
X-oppsett.

1. **Plassering:** Hvor sitter de fire thrusterne, som posisjon [x, y, z] i
   meter fra et avtalt origo, og hvilken retning skyver hver av dem? Er det
   X-oppsett (vinklede thrustere), to akter pluss to tunnelthrustere, eller
   noe annet?
   - Svar:
2. **Origo:** Hvilket punkt er båtens referansepunkt i matten til C&A:
   tyngdepunktet, midtskips i vannlinja, eller noe annet?
   - Svar:
3. **Nummerering:** Hvilken fysisk thruster er `thruster_1`, `thruster_2`,
   `thruster_3` og `thruster_4`?
   - Svar:
4. **Fortegn og grenser:** Betyr positiv verdi på `thruster_i/command` skyv i
   thrusterens retning, målt i newton? Hva er omtrentlig maks kraft forover og
   bakover per thruster?
   - Svar:
5. **Namespace:** Kjører allokeringsnoden i et ROS-namespace? Topicen
   `thruster_1/command` er relativ og blir `/thruster_1/command` bare uten
   namespace.
   - Svar:
6. **Takt:** Publiseres alle fire kommandoene i hver kontrollsyklus, med
   samme takt? Simulatoren slipper bare kraft gjennom mens alle fire er
   ferske.
   - Svar:

## Sensorer og tilstand

7. **Meldingstyper:** Hvilke meldingstyper får GNSS- og attitude-noden på
   båten? `sensor_msgs/NavSatFix` og `sensor_msgs/Imu` (som simulatoren sender
   i dag), eller driverens egne typer, f.eks. heading eller
   `geometry_msgs/TwistStamped` for fart?
   - Svar:
8. **Konvensjon:** Regner dere i ENU og FLU (x fram, y babord, z opp; ROS-
   standarden og det simulatoren bruker) eller NED og FRD? Skal simulatoren
   tilby en adapter, eller konverterer dere selv?
   - Svar:
9. **Hardware:** Hvilken GNSS (med eller uten RTK) og hvilken IMU er planlagt?
   Datablad gir realistiske verdier for støy, bias og drift i simuleringen.
   - Svar:
10. **Fasit eller estimat:** Vil dere normalt kjøre på fasit
    (`STATE_SOURCE=truth`), på simulatorens estimat, eller på deres egen
    estimator fra rå GNSS/IMU?
    - Svar:

## Kontroll og timing

11. **Frekvens:** Hvilken frekvens planlegger dere for kontrollsløyfen, selv
    grovt (10, 20 eller 50 Hz)?
    - Svar:
12. **Simuleringstid:** Bruker nodene `use_sim_time`? Det kreves for at
    tidsstempler og timere skal stemme med simuleringen, og for å kunne kjøre
    raskere enn sanntid.
    - Svar:

## DP og evaluering

13. **Settpunkt:** Hvordan vil dere få DP-settpunktet: som
    `geometry_msgs/PoseStamped` på `/njord/goal` (posisjon og heading i
    `map`), eller på en annen måte?
    - Svar:
14. **Toleranser:** Hvilke krav gjelder for DP (for eksempel ±1 m og ±5°), og
    hvor lenge skal posisjonen holdes?
    - Svar:
15. **Måltall:** Hvilke Autodrone-måltall er aktuelle: banefølgingsfeil mot
    hvilken referanse (planlagt sti eller linjen mellom portene), tid,
    kollisjoner, energibruk?
    - Svar:

## Senere

16. **PWM til kraft:** Når kan Electrical måle sammenhengen mellom PWM og
    kraft per thruster? Den trengs hvis simulatoren skal ta imot PWM direkte.
    - Svar:
