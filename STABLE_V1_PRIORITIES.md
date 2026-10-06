# Stable V1 Priorities (ESP32-Stepper-DMX-board)

Stand: 2026-08-28
Ziel: eine technisch stabile, betriebssichere erste Version fuer Show-Einsatz.

## 1) P0 Kritisch (MUSS vor Stable V1)

### P0-3: WS2813/Pixel-Stabilitaet nach DMX-Verlust (nur erste ~10 LEDs / random)
- Symptom: nach DMX-Loss oder sporadisch arbeiten nur erste Pixel korrekt.
- Risiko: sichtbarer Show-Fehler.
- Wahrscheinliche technische Ursachen:
  - Keine harte Paketlaengen-Validierung vor Kanalzugriffen.
  - Treiber ist auf WS2812B konfiguriert, Hardwaremeldung nennt WS2813.
- Entscheidung fuer V1:
  - DMX Paketgroesse strikt validieren (nur verarbeiten, wenn voller Footprint vorhanden).
  - Bei unvollstaendigen Frames: fail-safe (letzter gueltiger Zustand oder definierter Blackout).
  - LED Typ/Timing passend zur realen Hardware fixieren und validieren.

### P0-4: Position + kontinuierliche Rotation verliert Referenz/systematischer Fehler
- Symptom: Positionsdrift/Lost Position bei gleichzeitiger Pan-Position und Rotation.
- Risiko: mechanische Fehlposition im Betrieb.
- Hinweis: wirkt algorithmisch/systematisch, nicht nur Step-Loss.
- Entscheidung fuer V1:
  - Bewegungsmodell vereinheitlichen: klarer Zustandsautomat fuer Position, Hold, Rotation.
  - Offset- und Uebergangslogik (0/128/rotating) deterministisch machen.
  - Reproduzierbarer Testfall als Release-Gate (mehrere Minuten kontinuierlich).

### P0-2: DMX Timeout Anforderung stimmt nicht mit Implementierung
- Anforderung: DMX Timeout = 60s.
- Ist-Code: Timeout steht auf 5s.
- Risiko: unnoetige Motor-Disable/Homing-Zyklen bei kurzen DMX-Aussetzern.
- Entscheidung fuer V1:
  - Timeout auf exakt 60s setzen.
  - Bei Timeout: Stepper disable.
  - Bei DMX Rueckkehr: zwingend Rehoming vor normalem Betrieb.
  - Timeout/Homing Verhalten mit realen DMX-Ausfaellen testen und protokollieren.

### P0-1: COB Fan schaltet nicht sicher bei hoher Last
- Symptom: Luefter startet nicht zuverlaessig, auch wenn COB-Last hoch ist.
- Risiko: thermische Ueberlastung, Leistungsabfall, Hardware-Schaden.
- Beobachtung aus Code:
  - Fan wird ueber einen Integrator mit langer Rise-Time geregelt.
  - Aktivierung erst ueber FAN_ENABLE_THRESHOLD.
- Entscheidung fuer V1:
  - Fan-Sicherheitsmodus einfuehren: bei hoher Last (z. B. >=80% Dimmer oder Pixel-Last >=70%) sofort Fan ON.
  - Thermal-Schutz als harte Prioritaet behandeln (nie ohne aktive Kuehlung bei hoher Last).
  - Kurzer Hardwaretest unter Dauerlast (mind. 20-30 min) als Release-Gate.

### P0-5: Feste DMX-Adressen fuer alle 8 ESP-Controller
- Symptom: uneinheitliche oder nicht final festgelegte DMX-Adressierung im Gesamtsystem.
- Risiko: Patching-Fehler, unvorhersehbares Verhalten im Show-Betrieb, hoher Setup-Aufwand.
- Entscheidung fuer V1:
  - Feste, dokumentierte Startadressen pro Controller definieren.
  - Adressplan in Doku + Konfiguration synchron halten.
  - Endtest mit kompletter 8-ESP-Konstellation als Release-Gate.

## 2) P1 Hoch (sollte in V1 wenn Zeit reicht)

### P1-1: RDM Zuverlaessigkeit im Vollbetrieb
- Symptom: bekannte RDM Probleme.
- Risiko: schlechte Erkennbarkeit/Steuerbarkeit an Lichtpulten.
- Beobachtung:
  - Es gibt separate rdm_test Firmware -> Hinweis auf noch nicht robuste Integration im Hauptbuild.
- Entscheidung fuer V1:
  - RDM Grundfunktionen als Mindestumfang absichern: Discovery, DMX Start Address, Personality, Device Label.
  - Lasttests mit aktivem Pixel/Dimmer-Rendering + RDM Polling.
  - Falls nicht voll stabil: RDM Scope fuer V1 klar begrenzen und dokumentieren.

### P1-2: Fehlerpfad bei Homing-Fehlern
- Symptom: Fehler werden zwar geloggt, aber kaum Recovery/Benutzerfeedback.
- Risiko: unklarer Betriebszustand nach fehlgeschlagenem Homing.
- Entscheidung fuer V1:
  - Klaren Safe-State definieren (Ausgaenge aus, Bewegung stop, Fehlerflag).
  - Sichtbares Fehlerfeedback (z. B. Diagnose-LED oder serieller Fehlercode) definieren.

## 3) P2 Mittel (nach Stable V1 verschiebbar)

### P2-1: Placeholder/TODOs ohne direkten Show-Blocker
- RDM Identify Callback ist nur Platzhalter.
- Geplante Weiterleitung DMX -> Serial2 ist TODO.

### P2-2: Hardware/Validierungsaufgaben aus TODO.md
- Mechanik-/Elektroniktests sind nur teilweise dokumentiert.
- Fuer V1 reicht fokussierter Abnahmetest, umfassende Validierung kann in V1.1 folgen.

## 4) Konkrete Stable-V1 Zieldefinition (MUSS)

1. Pixel-Robustheit: kein "nur erste LEDs" Fehler nach DMX-Loss/Reconnect.
2. Bewegungsstabilitaet: keine systematische Positionsdrift bei Position+Rotation.
3. DMX Fail-Safe: 60s Timeout, dann Stepper disable; bei Rueckkehr obligatorisches Rehoming.
4. Thermische Sicherheit: Fan startet sicher unter hoher Last und verhindert Ueberhitzung.
5. Feste DMX-Adressen fuer die insgesamt 8 verschiedenen ESPs.

## 5) Release Gates (Go/No-Go Tests)

1. 10x DMX Reconnect-Test ohne Pixel-Ausfall/Random-Muster.
2. 20 min Bewegungstest mit gemischter Position/Rotation ohne Referenzverlust.
3. 10x DMX Ausfall/Rueckkehr-Zyklus mit 60s Timeout, jedes Mal korrektes Disable/Rehome.
4. 30 min Vollast-Test (Dimmer + Pixel) ohne thermischen Fehler.
5. RDM Discovery und Parameter-Aenderung von mindestens 2 unterschiedlichen Lichtpult/Tools.

## 6) Empfohlene Reihenfolge (wenn wenig Zeit)

1. Pixel/WS2813 Stabilitaet nach DMX-Loss (P0-3)
2. Rotation+Position Drift fixen (P0-4)
3. DMX Timeout auf 60s + Rehome-Flow absichern (P0-2)
4. Fan/Thermal Fix + Test (P0-1)
5. Feste DMX-Adressen fuer 8 ESPs finalisieren (P0-5)
6. RDM Mindeststabilitaet verifizieren (P1-1)
