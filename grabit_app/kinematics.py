"""Kinematik des Grab-it Roboterarms.

Dieses Modul enthält
  - die Maße des Arms,
  - Hilfsfunktionen für homogene 4x4-Transformationen (trans, rot_y, rot_z),
  - die Vorwärtskinematik fk(): Gelenkwinkel -> Position/Lage des TCP,
  - die inverse Kinematik ik(): TCP-Position + Greiferneigung -> Gelenkwinkel.

Einheiten: Längen in Metern, Winkel in Radiant.
Koordinatensystem: x zeigt nach vorne, y nach links, z nach oben.
Nullstellung: alle Gelenke 0 -> Arm steht senkrecht, der TCP liegt bei z = 0.42 m.
"""
import numpy as np

# ---------------------------------------------------------------------------
# Maße des Arms (in Metern)
# ---------------------------------------------------------------------------
H_BASE = 0.05      # Grundplatte bis Drehteller
H_SHOULDER = 0.04  # Drehteller bis Schulterachse
L1 = 0.105         # Oberarm (Schulter bis Ellbogen)
L2 = 0.095         # Unterarm (Ellbogen bis Handgelenk)
L_WRIST = 0.04     # Handgelenk bis Drehservo des Greifers
L_TOOL = 0.09      # Drehservo bis TCP (Punkt zwischen den Fingerspitzen)
H = H_BASE + H_SHOULDER  # Höhe der Schulterachse über dem Boden (0.09 m)
L3 = L_WRIST + L_TOOL    # Handgelenk bis TCP (0.13 m)

# ---------------------------------------------------------------------------
# Gelenkgrenzen (Radiant)
# ---------------------------------------------------------------------------
Q_LIMIT = np.deg2rad(90.0)  # Grenze für q1 (Drehteller): -90 bis +90 Grad
# Grenzen für q2, q3, q4 (Schulter, Ellbogen, Handgelenk), jeweils (untere / obere)
Q_MIN = np.deg2rad([-90.0, -90.0, -5.0])
Q_MAX = np.deg2rad([90.0, 90.0, 175.0])
EPS = 1e-9                   # Rechentoleranz gegen Rundungsfehler
LIMIT_TOL = np.deg2rad(0.5)  # Toleranz an den Gelenkgrenzen für gerundete Zielwerte


# ---------------------------------------------------------------------------
# Homogene Transformationen
# Eine 4x4-Matrix fasst Drehung (oberer linker 3x3-Block) und Verschiebung
# (letzte Spalte) zusammen. Verkettet wird durch Multiplikation mit "@".
# Die Reihenfolge ist wichtig: Matrixmultiplikation ist nicht vertauschbar.
# ---------------------------------------------------------------------------
def trans(x: float, y: float, z: float) -> np.ndarray:
    """Reine Verschiebung als 4x4-Matrix."""
    T = np.eye(4)            # Einheitsmatrix: keine Drehung, keine Verschiebung
    T[:3, 3] = [x, y, z]     # Verschiebung in die letzte Spalte eintragen
    return T


def rot_y(angle: float) -> np.ndarray:
    """Drehung um die y-Achse (Winkel in Radiant) als 4x4-Matrix."""
    c, s = np.cos(angle), np.sin(angle)
    T = np.eye(4)
    T[:3, :3] = [[c, 0.0, s],
                 [0.0, 1.0, 0.0],
                 [-s, 0.0, c]]
    return T


def rot_z(angle: float) -> np.ndarray:
    """Drehung um die z-Achse (Winkel in Radiant) als 4x4-Matrix."""
    c, s = np.cos(angle), np.sin(angle)
    T = np.eye(4)
    T[:3, :3] = [[c, -s, 0.0],
                 [s, c, 0.0],
                 [0.0, 0.0, 1.0]]
    return T


# ---------------------------------------------------------------------------
# Vorwärtskinematik
# ---------------------------------------------------------------------------
def fk(q) -> np.ndarray:
    """Gelenkwinkel (q1..q4, optional q5, Radiant) -> 4x4-Pose des TCP.

    Die Position des TCP steht in der letzten Spalte: fk(q)[:3, 3].
    """
    q = list(q)
    q1, q2, q3, q4 = q[0], q[1], q[2], q[3]
    q5 = q[4] if len(q) > 4 else 0.0  # q5 dreht nur den Greifer, die Position bleibt gleich
    # Die Kette von der Grundplatte bis zum TCP, von links nach rechts gelesen:
    # "ein Stück gehen, dann drehen". Pro Gelenk: Verschiebung zum Gelenk, dann Drehung.
    return (
        trans(0, 0, H_BASE) @ rot_z(q1)          # Drehteller (Drehung um die senkrechte Achse)
        @ trans(0, 0, H_SHOULDER) @ rot_y(q2)    # Schulter (Oberarm kippen)
        @ trans(0, 0, L1) @ rot_y(q3)            # Ellbogen (Unterarm kippen)
        @ trans(0, 0, L2) @ rot_y(q4)            # Handgelenk (Greifer kippen)
        @ trans(0, 0, L_WRIST) @ rot_z(q5)       # Greifer um seine eigene Achse drehen
        @ trans(0, 0, L_TOOL)                    # bis zum TCP
    )


# ---------------------------------------------------------------------------
# Inverse Kinematik
# ---------------------------------------------------------------------------
def _wrap(angle: float) -> float:
    """Winkel auf (-pi, pi] bringen."""
    return float((angle + np.pi) % (2 * np.pi) - np.pi)


def ik(x: float, y: float, z: float, psi: float, last_q1: float = 0.0):
    """TCP-Position (m) und Greiferneigung psi (rad, gegen die Senkrechte)
    -> (q1, q2, q3, q4) in Radiant, oder None, wenn nicht erreichbar.

    psi = pi: senkrecht nach unten greifen, psi = pi/2: waagerecht.
    last_q1: wird verwendet, wenn das Ziel auf der Drehachse liegt (r ~ 0).

    Idee: Der Arm lässt sich in eine Drehung um die senkrechte Achse (q1) und
    einen ebenen Arm (q2, q3, q4) zerlegen. Für den ebenen Arm bilden Oberarm,
    Unterarm und die Strecke Schulter-Handgelenk ein Dreieck (Kosinussatz).
    Die Greiferneigung psi muss vorgegeben werden, weil der Arm mit fünf Achsen
    nicht jede Position mit jeder Neigung erreichen kann.
    """
    # Schritt 1: Drehteller. atan2 kennt alle vier Quadranten und teilt nie durch null.
    r = float(np.hypot(x, y))  # waagerechter Abstand des Ziels von der Drehachse
    q1 = last_q1 if r < 1e-6 else float(np.arctan2(y, x))  # auf der Achse: q1 beliebig, alten Wert behalten

    # Ziel hinter dem Arm: q1 läge außerhalb von +-90 Grad.
    # Dann um 180 Grad drehen und r negativ rechnen (Arm greift nach hinten über sich hinweg).
    if abs(q1) > Q_LIMIT + EPS:
        q1 = _wrap(q1 - np.sign(q1) * np.pi)
        r = -r
    if abs(q1) > Q_LIMIT + EPS:
        return None  # auch nach dem Umdrehen außerhalb der Grenzen

    # Schritt 2: Handgelenkpunkt. Vom TCP um L3 entgegen der Greiferrichtung zurückgehen
    # und relativ zur Schulter angeben (r_w waagerecht, z_w senkrecht).
    r_w = r - L3 * np.sin(psi)
    z_w = z - H - L3 * np.cos(psi)
    D = float(np.hypot(r_w, z_w))  # Abstand Schulter - Handgelenk

    # Schritt 3: Erreichbarkeit. Das Dreieck mit den Seiten L1, L2 und D existiert nur,
    # wenn |L1 - L2| <= D <= L1 + L2 gilt.
    if D > L1 + L2 + EPS or D < abs(L1 - L2) - EPS:
        return None

    # Schritt 4: Ellbogenwinkel q3 über den Kosinussatz.
    cos_q3 = (D**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_q3 = float(np.clip(cos_q3, -1.0, 1.0))  # Rundungsfehler abfangen, sonst liefert arccos NaN
    q3_abs = float(np.arccos(cos_q3))

    # Es gibt zwei Lösungen: q3 > 0 ("Ellbogen oben") und q3 < 0 ("Ellbogen unten").
    # Beide werden gegen die Gelenkgrenzen geprüft, die erste passende wird genommen.
    # Erst mit exakter Grenze (tol = EPS), dann mit kleiner Toleranz (LIMIT_TOL) für
    # gerundete Zielwerte; ein Ergebnis in der Toleranz wird auf die Grenze geklemmt.
    for tol in (EPS, LIMIT_TOL):
        for sign in (+1.0, -1.0):
            q3 = sign * q3_abs
            # Schulterwinkel q2. Die Winkel werden gegen die Senkrechte gemessen,
            # deshalb steht atan2 hier mit den Argumenten (r, z) statt (z, r).
            q2 = float(np.arctan2(r_w, z_w)
                       - np.arctan2(L2 * np.sin(q3), L1 + L2 * np.cos(q3)))
            # Schritt 5: Handgelenk. Die Winkel der drei parallelen Achsen addieren sich zu psi.
            q4 = _wrap(psi - q2 - q3)
            q_arr = np.array([q2, q3, q4])
            if np.all(q_arr >= Q_MIN - tol) and np.all(q_arr <= Q_MAX + tol):
                q2, q3, q4 = (float(a) for a in np.clip(q_arr, Q_MIN, Q_MAX))
                return (q1, q2, q3, q4)
    return None  # keine der beiden Lösungen passt in die Gelenkgrenzen
