"""3D-Steuerung für den Grab-it Roboterarm.

Die Anwendung zeigt den Arm in 3D im Browser (viser). Der Werkzeugpunkt (TCP) lässt sich
mit der Maus verschieben, die inverse Kinematik berechnet die Gelenkwinkel, und der echte
Arm wird per MQTT angesteuert.

Weg einer Bewegung:
  Marker/Regler -> IK -> Sprung-Schutz -> 3D-Modell (sofort)
                                       -> Sollwert -> Sendeschleife (Rampe, 25 Hz)
                                       -> Servowinkel -> MQTT -> Arm

Start:  python app.py            (sendet per MQTT)
        python app.py --dry-run  (gibt die Nachrichten nur aus)
"""
from pathlib import Path
import time
import argparse
 
import numpy as np
import viser
import yaml
from viser.extras import ViserUrdf
import paho.mqtt.client as mqtt
 
from kinematics import fk, ik


# Pfade relativ zu dieser Datei
URDF_PATH = Path(__file__).parent / "grabit.urdf"      # Robotermodell für die Anzeige
CONFIG_PATH = Path(__file__).parent / "config.yaml"    # Servo-Richtungen, Offsets, MQTT

# Größte erlaubte Änderung eines Gelenks pro Mausereignis (Grad). Fängt z. B. den
# 180-Grad-Sprung des Drehtellers beim Überqueren der Linie x = 0 ab.
JUMP_LIMIT_DEG = 90.0

def loadConfig() -> dict:
    """config.yaml einlesen (Servo-Richtungen, Offsets, MQTT-Einstellungen)."""
    with open(CONFIG_PATH) as f:
        return yaml.safe_load(f)

def jointToServo(servos: dict, name: str, qRad: float) -> float:
    """Gelenkwinkel (Radiant) -> Servowinkel (Grad), geklemmt auf 0..180."""
    cfg = servos[name]
    # 90 Grad ist die Servomitte. direction (+1/-1) dreht die Richtung um,
    # offset gleicht aus, dass der Servo nicht genau in der Modell-Nullstellung eingebaut ist.
    servo = 90 + cfg["direction"] * np.rad2deg(qRad) + cfg["offset"]
    # Letzter Schutz: der Servo kennt nur 0..180 Grad.
    return float(np.clip(servo, 0.0, 180.0))

def findPsi(x: float, y: float, z: float, psi0Deg: float):
    """Sucht das zu psi0Deg nächstgelegene psi (0..180 Grad), mit dem das Ziel
    erreichbar ist. Gibt (psiDeg, q) zurück oder None."""
    # Vom gewünschten psi aus abwechselnd nach unten und oben suchen (Abstand i = 0, 1, 2, ...).
    for i in range(181):
        for psiDeg in (psi0Deg - i, psi0Deg + i):
            if 0 <= psiDeg <= 180:
                q = ik(x, y, z, np.deg2rad(psiDeg))
                if q is not None:
                    return psiDeg, q
    return None  # für kein psi erreichbar



def main() -> None:

    # --- Kommandozeile: --dry-run gibt die Nachrichten nur aus ---
    parser = argparse.ArgumentParser()
    parser.add_argument("--dry-run", action="store_true", help="Nachrichten nur ausgeben, nicht senden")
    args = parser.parse_args()

    # --- Konfiguration ---
    config = loadConfig()
    servos = config["servos"]       # pro Gelenk: motor, direction, offset
    mqttCfg = config["mqtt"]        # host, port, topic, rate_hz, max_step_deg
    maxStep = mqttCfg["max_step_deg"]   # größte Servo-Änderung pro Takt (Rampe)

    # --- MQTT-Client: nur anlegen, wenn wirklich gesendet wird ---
    client = None
    if not args.dry_run:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        client.connect(mqttCfg["host"], mqttCfg["port"])
        client.loop_start()     # Netzwerk-Thread im Hintergrund

    # --- 3D-Szene: Modell und verschiebbarer Marker am TCP ---
    TCP_START = tuple(fk(np.zeros(4))[:3, 3])   # TCP-Position in der Nullstellung (0, 0, 0.42)
    server = viser.ViserServer()
    robot = ViserUrdf(server, URDF_PATH)
    server.scene.set_up_direction("+z")
    tcp = server.scene.add_transform_controls(
        "/tcp_marker",
        scale=0.1,
        position=TCP_START,
        disable_rotations=True,     # nur verschieben, nicht drehen
    )

    # --- STOPP-Knopf ---
    stopButton = server.gui.add_button("STOPP", color="red")
    state = {"stopped": False}      # als Dictionary, damit der Callback den Wert ändern kann

    @stopButton.on_click
    def _(_event) -> None:
        state["stopped"] = True     # sendMove() sendet danach nichts mehr
        # "status" wird weiter unten angelegt; der Callback läuft erst beim Klick, also später
        status.value = "GESTOPPT"
        

    # dict: Gelenkname -> (untere, obere Grenze), Reihenfolge wie bei update_cfg
    limits = robot.get_actuated_joint_limits()
    jointNames = list(limits)       # 6 Namen: base_yaw ... gripper (Greifer-Spiegelgelenk zählt nicht)
    print("Bewegliche Gelenke (Reihenfolge wie bei update_cfg):", jointNames)
    robot.update_cfg(np.zeros(len(jointNames)))     # Modell in die Nullstellung setzen

    # --- Oberfläche: Status und Regler (alle in Grad) ---
    status = server.gui.add_text("Status", initial_value="OK", disabled=True)
    psiSlider = server.gui.add_slider(
        "psi (Greiferneigung)", min=0.0, max=180.0, step=1.0, initial_value=0.0
    )
    q5Slider = server.gui.add_slider(
        "q5 (Greifer drehen)", min=-90.0, max=90.0, step=1.0, initial_value=0.0
    )
    gripperSlider = server.gui.add_slider(
        "Greifer", min=0.0, max=40.0, step=1.0, initial_value=0.0
    )


    # --- Gemeinsamer Zustand (Dictionaries, damit die Callbacks sie ändern können) ---
    # Letzte gültige Pose: darauf wird bei abgelehnten Bewegungen zurückgesetzt.
    lastValid = {
        "tcp": list(TCP_START),
        "psi": 0.0,
        "q5": 0.0,
        "gripper": 0.0,
        "q": np.zeros(4),  # letzte gültige Winkel q1..q4 in Radiant
    }
    # Zuletzt gesendete Servowinkel (ganze Grad); None = noch nichts gesendet
    lastSent = {"servo": None, }
    target = {"q": np.zeros(len(jointNames))}  # aktueller Sollwert für die Sendeschleife (6 Winkel)
    # Aktuell befohlene Servowinkel (Rampe); Start = Servowinkel der Nullstellung
    current = {"servo": np.array([jointToServo(servos, n, 0.0) for n in jointNames])}


    def sendMove(dryRun ) -> None:
        """Ein Takt: Rampenschritt Richtung Ziel, geänderte Servos senden."""
        if state["stopped"]:
            return      # nach STOPP nichts mehr senden (die Rampe wird auch nicht weitergerechnet)
        # Sollwert (Gelenkwinkel) in Servowinkel umrechnen
        targetServo = np.array([jointToServo(servos, n, q) for n, q in zip(jointNames, target["q"])])
        # Rampe: jeder Servo darf sich pro Takt höchstens um maxStep bewegen
        step = np.clip(targetServo - current["servo"], -maxStep, maxStep)
        current["servo"] = current["servo"] + step
 
        # Auf ganze Grad runden (das Format der Nachricht) und mit dem zuletzt Gesendeten vergleichen
        rounded = np.round(current["servo"]).astype(int)
        if lastSent["servo"] is None:
            changed = np.ones(len(rounded), dtype=bool)  # Startpose komplett senden
        else:
            changed = rounded != lastSent["servo"]
        if not changed.any():
            return      # nichts Neues, nichts zu senden
        lastSent["servo"] = rounded
 
        # Pro geändertem Servo eine Nachricht im Format "<Motornummer> <Winkel>"
        for name, servoDeg, isChanged in zip(jointNames, rounded, changed):
            if not isChanged:
                continue
            motor = servos[name]["motor"]
            message = f"{motor} {servoDeg}"
            if dryRun:
                print(message)
            else:
                client.publish(mqttCfg["topic"], message, qos=0, retain=False)

 

    def rejectMove(text: str) -> None:
        """Marker und Regler auf die letzte gültige Pose zurücksetzen."""
        tcp.position = lastValid["tcp"]
        psiSlider.value = lastValid["psi"]
        q5Slider.value = lastValid["q5"]
        gripperSlider.value = lastValid["gripper"]
        status.value = text

    def updatePose() -> None:
        """Marker und Regler lesen, IK rechnen, Modell nachführen, Sollwert merken."""
        # Aktuellen Zustand aus der Oberfläche lesen
        x, y, z = tcp.position
        psi = psiSlider.value
        q5 = np.deg2rad(q5Slider.value)
        gripper = np.deg2rad(gripperSlider.value)


        # Gültiges psi und die zugehörigen Winkel suchen
        ergebnis = findPsi(x, y, z, psi)
        if ergebnis is None:
            rejectMove("Ziel unerreichbar")
            return
        psiDeg, q = ergebnis
        q = np.array(q)


        # Sprung-Schutz: größte Winkeländerung gegenüber der letzten gültigen Pose
        sprungDeg = np.rad2deg(np.abs(q - lastValid["q"]).max())
        if sprungDeg > JUMP_LIMIT_DEG:
            rejectMove(f"Sprung zu groß ({sprungDeg:.0f}°)")
            return

        # Pose übernehmen
        cfg = np.array([*q, q5, gripper])   # 6 Werte in der Reihenfolge der Gelenke
        robot.update_cfg(cfg)               # 3D-Modell sofort nachführen
        target["q"] = cfg                   # Sollwert für die Sendeschleife
        status.value = "OK"
        if psi != psiDeg:
            psiSlider.value = psiDeg        # findPsi hat ein anderes psi gewählt: Regler anpassen
        # Diese Pose als "letzte gültige" merken
        lastValid["tcp"][:] = [x, y, z]
        lastValid["psi"] = psiSlider.value
        lastValid["q5"] = q5Slider.value
        lastValid["gripper"] = gripperSlider.value
        lastValid["q"] = q

    # Jede Änderung am Marker oder an einem Regler löst updatePose() aus
    tcp.on_update(lambda _event: updatePose())
    psiSlider.on_update(lambda _event: updatePose())
    q5Slider.on_update(lambda _event: updatePose())
    gripperSlider.on_update(lambda _event: updatePose())

    # --- Sendeschleife: feste Rate, immer der aktuellste Sollwert ---
    try:
        while True:
            time.sleep(1 / mqttCfg["rate_hz"])  # 25 Hz -> alle 40 ms ein Takt
            sendMove(args.dry_run)
    except KeyboardInterrupt:
        pass                                    # Strg+C beendet das Programm
    finally:
        # Verbindung sauber schließen (nur wenn ein Client existiert)
        if client is not None:
            client.loop_stop()
            client.disconnect()
 
if __name__ == "__main__":
    main()
