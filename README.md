# 3D-Steuerung Grab-it Roboterarm

Python-Anwendung, die den Joy-IT Grab-it (Robot02) in 3D im Browser darstellt. Der Werkzeugpunkt (TCP) lässt sich mit der Maus verschieben. Aus seiner Position berechnet die inverse Kinematik die Gelenkwinkel, und der echte Arm wird per MQTT entsprechend angesteuert.

## Funktionsübersicht

- 3D-Darstellung des Arms (URDF-Modell) mit viser im Browser
- Verschiebbarer Marker am TCP; das Modell folgt der Markerposition
- Regler für Greiferneigung (`psi`), Greiferdrehung (`q5`) und Greifer (auf/zu)
- Vorwärtskinematik `fk()` als Matrixkette und inverse Kinematik `ik()` in geschlossener Form
- Umrechnung der Gelenkwinkel in Servowinkel (Richtung, Offset, Begrenzung)
- Ansteuerung der Arm-Steuerung per MQTT mit fester Rate und Geschwindigkeitsrampe
- Probelauf-Modus (`--dry-run`), der die Nachrichten nur ausgibt
- STOPP-Knopf in der Oberfläche

## Funktionsweise

Eine Bewegung durchläuft diese Stufen:

1. Der TCP-Marker oder ein Regler wird in der Oberfläche verändert.
2. Die inverse Kinematik berechnet daraus die Gelenkwinkel `q1` bis `q4` (siehe unten). `q5` und der Greifer kommen direkt von den Reglern.
3. Die Winkel werden auf die Gelenkgrenzen begrenzt, und ein Sprung-Schutz prüft die Änderung gegenüber der letzten gültigen Pose.
4. Das 3D-Modell zeigt die neuen Winkel sofort an.
5. Eine Sendeschleife rechnet die Gelenkwinkel in Servowinkel um und führt sie mit einer Geschwindigkeitsrampe dem Sollwert nach.
6. Die Servowinkel gehen als MQTT-Nachrichten an die Arm-Steuerung.

### Projektstruktur

```
grabit_app/
  grabit.urdf          Robotermodell (Maße in Metern, Winkel in Radiant)
  config.yaml          Servo-Richtungen, Offsets, Motornummern, MQTT-Einstellungen
  kinematics.py        fk(), ik() und die Transformationsmatrizen
  app.py               viser-Oberfläche, Servo-Mapping, MQTT
  test_kinematics.py   pytest-Tests für fk() und ik()
  README.md            diese Datei
```

### Modell

Der Arm hat fünf Achsen plus Greifer (sechs Servos). In der Nullstellung stehen alle Gelenke auf 0°, der Arm zeigt senkrecht nach oben, und der TCP liegt bei z = 420 mm. Ein positiver Winkel an Schulter, Ellbogen oder Handgelenk kippt den Arm nach vorne (+x).

| Gelenk | Motor | Dreht um | Bewegung |
|---|---|---|---|
| `base_yaw` (q1) | 0 | z | Arm nach links/rechts schwenken |
| `shoulder_pitch` (q2) | 1 | y | Oberarm nach vorne kippen |
| `elbow_pitch` (q3) | 2 | y | Unterarm kippen |
| `wrist_pitch` (q4) | 3 | y | Greifer kippen |
| `wrist_roll` (q5) | 4 | Greiferachse | Greifer um sich selbst drehen |
| `gripper` | 5 | – | Greifer öffnen/schließen |

Längen: Grundplatte bis Drehteller 50 mm, Drehteller bis Schulter 40 mm, Oberarm L1 = 105 mm, Unterarm L2 = 95 mm, Handgelenk bis TCP L3 = 130 mm (40 mm bis zum Drehservo, 90 mm bis zum TCP). Die Längen im URDF sind in Metern angegeben.

### Kinematik (`kinematics.py`)

**Vorwärtskinematik.** `fk(q)` bildet die Kette der homogenen Transformationen (`trans()`, `rot_y()`, `rot_z()` als 4×4-Matrizen) und liefert die Pose des TCP. Weil Schulter, Ellbogen und Handgelenk um parallele y-Achsen drehen, addieren sich ihre Winkel, und der Arm bewegt sich in einer senkrechten Ebene, die der Drehteller um `q1` schwenkt.

**Inverse Kinematik.** `ik(x, y, z, psi)` löst in fünf Schritten ohne Iteration:

1. Drehteller: `q1 = atan2(y, x)`
2. Handgelenkpunkt: vom TCP um L3 entgegen der Greiferrichtung zurück, relativ zur Schulter
3. Erreichbarkeit: der Abstand D von der Schulter zum Handgelenkpunkt muss zwischen |L1 − L2| und L1 + L2 liegen, sonst liefert `ik()` `None`
4. Ellbogen und Schulter über den Kosinussatz
5. Handgelenk: `q4 = psi − q2 − q3`

`psi` ist die Greiferneigung gegen die Senkrechte (180° = senkrecht nach unten, 90° = waagerecht). Sie wird vorgegeben, weil fünf Achsen Position und Neigung nicht unabhängig wählbar machen. Es gibt zwei Lösungen für den Ellbogen; `ik()` prüft zuerst „Ellbogen oben“ (`q3 > 0`), dann „Ellbogen unten“ und liefert die erste, die in die Gelenkgrenzen passt.

Sonderfälle: Liegt das Ziel hinter dem Arm (`q1` außerhalb ±90°), wird `q1` um 180° gedreht und `r` negativ gerechnet. Liegt es auf der Drehachse, behält `q1` den letzten Wert. Gerundete Zielwerte werden mit einer kleinen Toleranz an den Gelenkgrenzen (0,5°) akzeptiert und auf die Grenze geklemmt. `fk()` und `ik()` rechnen in Metern und Radiant.

### Gelenkgrenzen

- `q1` bis `q3`, `q5`: ±90°
- `q4` (Handgelenk kippen): −5° bis 175°, abgeleitet aus `servo = 175 − q4` und dem Servobereich 0° bis 180°. Die Obergrenze ist mechanisch _geprüft bis … / nicht geprüft_.
- Greifer: 0° (offen) bis 40° (geschlossen)

Die Grenzen sind im URDF und in `kinematics.py` (`Q_LIMIT`, `Q_MIN`, `Q_MAX`) hinterlegt.

### Servo-Mapping (`jointToServo`)

Jeder Gelenkwinkel `q` (in Grad) wird mit der Formel

```
servo = 90° + direction · q + offset
```

in einen Servowinkel umgerechnet und anschließend auf 0° bis 180° begrenzt. `direction` ist +1, wenn ein positiver Gelenkwinkel den Servowinkel erhöht, sonst −1. `offset` ist die Korrektur in Grad, damit der Servowinkel 90° plus Offset zur Nullstellung des Modells passt. Diese Werte stehen pro Servo in der `config.yaml`.

Die Arm-Steuerung rechnet nur Servowinkel in Pulsweiten um. Richtung, Offset und Begrenzung passieren ausschließlich in dieser Anwendung, jeweils einmal.

### Oberfläche (`app.py`)

- **Marker:** Gizmo am TCP, nur Verschieben, keine Drehung
- **`psi`:** Greiferneigung, 0° bis 180°. Ist ein Ziel mit dem eingestellten Wert unerreichbar, sucht die Anwendung das nächstgelegene `psi`, mit dem es erreichbar ist, und stellt den Regler selbst um.
- **`q5`:** Greifer drehen, ±90°
- **`Greifer`:** 0° bis 40°
- **Status:** „OK“, „Ziel unerreichbar“, „Sprung zu groß (…°)“ oder „GESTOPPT“
- **STOPP:** Setzt ein Flag, das das Senden anhält. Der Arm bleibt auf der zuletzt befohlenen Position stehen. Es gibt keinen Weiter-Knopf; nach einem Stopp ist ein Neustart der Anwendung vorgesehen.

Bei unerreichbaren Zielen und bei Sprüngen über 90° (`JUMP_LIMIT_DEG`) werden Marker und Regler auf die letzte gültige Pose zurückgesetzt, das Modell bleibt dort stehen.

### Senden und Geschwindigkeitsrampe

Eine Schleife läuft mit der Rate `rate_hz` (25 Hz). In jedem Takt:

1. Der Sollwert wird in Servowinkel umgerechnet.
2. Jeder Servo bewegt sich höchstens um `max_step_deg` (2°, also 50°/s) in Richtung des Sollwerts. Große Sprünge werden so zu einer Fahrt.
3. Die Werte werden auf ganze Grad gerundet. Es werden nur die Motoren gesendet, deren gerundeter Wert sich seit der letzten Nachricht geändert hat.

Beim Start wird die Nullstellung einmal komplett gesendet, ohne Rampe. Das 3D-Modell zeigt das Ziel sofort, der echte Arm folgt mit der Rampe.

## MQTT-Schnittstelle

| Eigenschaft | Wert |
|---|---|
| Broker | `10.20.3.82`, Port `1883` |
| Topic | `Robo/Command` |
| Payload | Text: `<Motornummer> <Winkel>`, eine Nachricht pro Servo |
| Beispiel | `3 175` (Motor 3 auf 175°) |
| Einheit | Servowinkel in Grad, ganze Zahlen, 0 bis 180 |
| QoS / retain | QoS 0, kein `retain` |

Motornummern 0 bis 5 in der Reihenfolge der Gelenke im URDF (siehe Tabelle oben). QoS 0 ohne `retain` ist für Bewegungsdaten gewählt, weil ein verlorener Wert vom nächsten ersetzt wird und ein gespeicherter Altwert den Arm beim nächsten Start unerwartet bewegen könnte.

## Konfiguration (`config.yaml`)

```yaml
mqtt:
  host: "10.20.3.82"
  port: 1883
  topic: "Robo/Command"
  rate_hz: 25
  max_step_deg: 2.0

servos:
  base_yaw:       {motor: 0, direction: ..., offset: ...}
  shoulder_pitch: {motor: 1, direction: ..., offset: -3.0}
  elbow_pitch:    {motor: 2, direction: ..., offset: ...}
  wrist_pitch:    {motor: 3, direction: -1,  offset: 85.0}
  wrist_roll:     {motor: 4, direction: ..., offset: ...}
  gripper:        {motor: 5, direction: ..., offset: ...}
```

Die Gelenknamen entsprechen den Namen im URDF.

## Kalibrierwerte

| Gelenk | Motor | Richtung | Offset | Bemerkung |
|---|---|---|---|---|
| `base_yaw` | 0 | _eintragen_ | _eintragen_ | |
| `shoulder_pitch` | 1 | _eintragen_ | −3 | Servo 87° = senkrecht |
| `elbow_pitch` | 2 | _eintragen_ | _eintragen_ | |
| `wrist_pitch` | 3 | −1 | 85 | Servo 175° = Handgelenk gerade, 90° = etwa rechter Winkel |
| `wrist_roll` | 4 | _eintragen_ | _eintragen_ | |
| `gripper` | 5 | _eintragen_ | _eintragen_ | Modell 0° = offen; Servoweg von offen bis zu: _eintragen_ |

Nachgemessene Längen (im URDF mit `MESSEN` markiert): _eintragen oder „wie im URDF“_.

## Aufruf

```
python app.py             # sendet per MQTT an die Arm-Steuerung
python app.py --dry-run   # gibt die Nachrichten nur aus, stellt keine Verbindung zum Broker her
```

Die Oberfläche ist danach im Browser unter der im Terminal ausgegebenen Adresse erreichbar, standardmäßig `http://localhost:8080`.

### Installation

Benötigt wird Python 3.10 oder neuer mit den Paketen `numpy`, `viser`, `yourdfpy`, `pyyaml`, `paho-mqtt` (ab Version 2) und `pytest`. `ikpy` kann zum Gegenprüfen der Kinematik verwendet werden.

```
python -m venv .venv
python -m pip install numpy viser yourdfpy pyyaml paho-mqtt pytest
```

## Tests

```
pytest -v
```

`test_kinematics.py` enthält Tests für `fk()` (die fünf Prüfwerte auf 0,1 mm genau) und für `ik()` (dieselben Werte, unerreichbare Ziele, Gelenkgrenzen, Ziele hinter dem Arm, Ziele auf der Drehachse, Rundreise mit 1000 zufälligen Posen: FK → IK → FK unter 0,1 mm Abweichung).

## Schutzmechanismen und Sicherheit

Die Anwendung enthält folgende Schutzmechanismen:

- **Gelenkgrenzen:** `ik()` liefert nur Lösungen innerhalb der Grenzen; sonst `None`, und es wird nichts gesendet.
- **Begrenzung der Servowinkel:** Jeder gesendete Wert liegt zwischen 0° und 180°.
- **Sprung-Schutz:** Ändert sich ein Gelenk gegenüber der letzten gültigen Pose um mehr als `JUMP_LIMIT_DEG`, wird die Bewegung abgelehnt.
- **Geschwindigkeitsrampe:** Jeder Servo bewegt sich pro Takt höchstens um `max_step_deg`.
- **Probelauf:** Mit `--dry-run` wird kein MQTT-Client angelegt, die Nachrichten werden nur ausgegeben.
- **Startpose:** Beim Start wird die Nullstellung gesendet. Die Servos springen beim Einschalten in ihre Sollposition, und die Startpose wird ohne Rampe übertragen.
- **STOPP-Knopf:** Hält das Senden an.

Die Servos sind kräftig (21,5 kg·cm laut Hersteller) und können klemmen oder Gegenstände vom Tisch fegen. Die Servo-Stromversorgung hat einen eigenen Schalter, der unabhängig von der Anwendung wirkt; der STOPP-Knopf ersetzt ihn nicht, weil bereits gesendete Nachrichten nicht zurückgeholt werden können und die Anwendung selbst hängen kann.

## Bekannte Grenzen

- **Keine Rückmeldung:** Die Servos melden ihre Position nicht zurück. Das Modell zeigt nur, was befohlen wurde. Hängt der Arm durch, blockiert er oder ist schlecht kalibriert, weicht die Realität ab, ohne dass die Software es bemerkt.
- **Modell und Arm laufen auseinander:** Das Modell zeigt das Ziel sofort, der echte Arm folgt mit der Rampe (50°/s) und hängt bei schnellen Bewegungen sichtbar hinterher.
- **Greiferneigung:** `psi` kann sich beim Ziehen des Markers ändern, weil die Anwendung bei Bedarf ein erreichbares `psi` sucht.
- **Sprung-Schutz:** Auch sehr schnelle Mausbewegungen können eine Winkeländerung über 90° pro Ereignis erzeugen und werden dann zurückgesetzt. Beim Überqueren der Linie x = 0 hinter dem Drehteller springt `q1` um etwa 180°, was der Schutz ebenfalls ablehnt.
- **Singularitäten:** Bei voll gestrecktem Arm (z. B. in der Nullstellung) ändert sich der Ellbogenwinkel bei kleinen Markerbewegungen stark.
- **Keine Kollisionsprüfung:** Das Modell kennt weder Tisch noch Selbstkollisionen.
- **STOPP:** Kein Weiter-Knopf; nach einem Stopp ist ein Neustart vorgesehen.
- **Genauigkeit:** _eintragen, falls gemessen (5 Punkte mit dem Maßband), sonst „nicht gemessen“_.
