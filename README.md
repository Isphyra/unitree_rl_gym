# 🤖 Unitree G1 MuJoCo Sim2Sim:

CC: https://github.com/unitreerobotics/unitree_rl_gym

Dieses Projekt ermöglicht die Simulation des Unitree G1 Roboters in MuJoCo unter Verwendung einer trainierten Reinforcement Learning Policy (`motion.pt`).



**Besonderheiten dieser Version:**
Das Skript `deploy_mujoco.py` wurde stark erweitert ("Ultimate Edition") und bietet:
* 🎮 **Xbox Controller Support** (Laufen, Drehen, Sonderfunktionen).
* ⌨️ **WASD Tastatur-Steuerung** (als Backup).
* 👊 **Schubs-Funktion** (Perturbations zum Testen der Balance).
* ♻️ **Reset-Funktion** (Ohne Neustart des Programms).

---

## 🛠️ Installation & Setup

**Voraussetzung:** Python 3.10 (Wichtig für MuJoCo Kompatibilität).

### 1. Repository klonen
```bash
git clone https://github.com/unitreerobotics/unitree_rl_gym.git
cd unitree_rl_gym
```
### ...mit der Steuerung
```bash
git clone https://github.com/Isphyra/unitree_rl_gym
cd unitree_rl_gym
```

### 2. Umgebung einrichten
# Erstelle die virtuelle Umgebung
```bash
py -3.10 -m venv g1_deploy_env

# Aktiviere sie
.\g1_deploy_env\Scripts\Activate.ps1
```
### 3. Abhängigkeiten installieren
Wichtig: Öffne zuerst die setup.py im Hauptordner und kommentiere die Zeilen isaacgym und rsl-rl aus (setze ein # davor), da diese Pakete für die reine MuJoCo-Simulation nicht benötigt werden.

Führe danach folgende Befehle aus:
```bash
# Projekt im Editable-Mode installieren (ohne Dependencies-Check)
pip install -e . --no-deps

# Notwendige Pakete nachinstallieren
pip install numpy torch scipy mujoco mujoco-python-viewer matplotlib tensorboard keyboard pygame pyyaml
```
### 4. Policy platzieren
Stelle sicher, dass deine motion.pt Datei (das "Gehirn" des Roboters) im richtigen Ordner liegt: unitree_rl_gym/deploy/pre_train/g1/motion.pt

🚀 Starten der Simulation

Manuell über Terminal
```bash
# 1. Umgebung aktivieren
.\g1_deploy_env\Scripts\Activate.ps1

# 2. In den Ordner wechseln
cd deploy\deploy_mujoco

# 3. Starten
python deploy_mujoco.py g1
```
### 🎮 Steuerung
Das Skript erkennt automatisch, ob ein Xbox-Controller angeschlossen ist. Falls nicht, wird automatisch die Tastatur verwendet.

| Funktion | Xbox Controller 🎮 | Tastatur ⌨️ | Beschreibung |
| :--- | :--- | :--- | :--- |
| **Laufen** | Linker Stick | `W`, `A`, `S`, `D` | Vorwärts, Rückwärts, Seitwärts laufen. |
| **Drehen** | Rechter Stick | `Q`, `E` | Roboter nach links oder rechts drehen. |
| **Schubsen** | Taste `A` (Grün) | `Leertaste` | Gibt dem Roboter einen Stoß (Balance-Test). |
| **Reset** | Taste `Back` (Select) | `R` | Setzt den Roboter zurück zum Startpunkt. |

