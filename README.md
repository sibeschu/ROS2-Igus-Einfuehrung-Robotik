TODO:
  - PLATZHALTER mit meinem Git Repository ersetzen, zum füllen des src-Ordners.
  - ORDNERSTRUKTUR mit Ordnerstruktur nach Installation colcon und erstem Mal bauen ersetzen.

# Einführung in die Robotik mit dem Igus-Rebel und ROS2

## Schnell-Installation

``bash
git clone --recurse-submodules https://github.com/sibeschu/ROS2-Igus-Einfuehrung-Robotik
```

oder

```bash
git clone https://github.com/sibeschu/ROS2-Igus-Einfuehrung-Robotik
cd ROS2-Igus-Einfuehrung-Robotik
git submodule update --init --recursive
```

## Inhaltsverzeichnis

- [Grundwissen](#grundwissen)
- [Anlegen eines eigenen Workspaces](#anlegen-eines-eigenen-workspaces)
- [ROS2 Installation](#ros2-installation)
- [Igus Installation](#igus-rebel-roboterarm-installation)
- [Kamera Installation](#realsense-kamera-installation)

## Hinweis

Bitte lesen Sie genau und vollständig. Dieser Text beinhaltet einige Linuxbefehle die `im Text` vorkommen und einige die
```bash
in einem Code-Fenster angezeigt werden.
```
Meistens sind beide Formatierungen gleich wichtig.

## Grundwissen

### Pakete
Für Ubuntu Noble (24.04) gibt es Debian Pakete für ROS2 Jazzy Jalisco. (Mehr über [Debian Pakete](https://de.wikipedia.org/wiki/Debian-Paket))

Im Gegensatz zu Windows wird hier keine .exe-Datei runtergeladen und ausgeführt, sondern das Programm über eine einfache Terminal-Eingabe installiert.

### Versionen
Noble und Jazzy Jalisco sind Namen für die jeweiligen Versionen von Ubuntu oder ROS2. So gibt es zum Beispiel ROS2 Versionen die Iron Irwini, Humble Hawksbill oder Foxy Fitzroy heißen und Ubuntu Versionen, die Trusty Tahr (14.04) oder Bionic Beaver (18.04) heißen. Jazzy Jalisco wird von Mai 2024 bis Mai 2029 gepflegt. Kilted Kaiju bis November 26 und Rolling Ridley von Juni 2020 bis "Ongoing".

Wir nutzen hier Jazzy, da die verwendeten ROS2-Pakete unter Jazzy getestet sind.

## ROS2 Installation

## Schritt 1

### Vorbereitung

#### System Einstellungen

Zuerst gehen wir sicher, dass unser Ubuntu System UTF-8 nutzt. Das ist wichtig für die Zeichenkodierung in z.B. Pfaden, Zeichenketten oder Meldungen. Sonst kann es zu Zeichenfehlern, Encoding-Problemen oder Programmfehlern kommen.

Mit <kbd>STRG</kbd> + <kbd>ALT</kbd> + <kbd>T</kbd> rufen wir ein neues Terminal auf.

Wir geben `locale` ein und prüfen, ob unser System mit UTF-8 arbeitet.

Falls nicht können wir dies mit

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

sicherstellen.

(Falls man lieber die deutsche Sprache als Systemsprache verwendet, kann man überall in den Befehlen "en_US" durch "de_DE" ersetzen.)

Mit `locale` überprüfen wir erneut die aktuelle Einstellung.

#### Erforderliche Repositories

Zuerst gehen wir sicher, dass das Ubuntu Universe Verzeichnis aktiviert ist. (Mehr über [Verzeichnisse/Repositories](https://help.ubuntu.com/community/Repositories/Ubuntu))

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Wir installieren ros2-apt-source Pakete. Diese konfigurieren ROS2 Verzeichnisse für unser System. Updates werden dann automatisch angezeigt, wenn neue Versionen für unsere Pakete veröffentlicht werden. Die Installation der ros2-apt-source Pakete erfolgt so:

```bash
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

Da wir später ROS2 Pakete bauen wollen, installieren wir auch die entsprechenden Entwicklungswerkzeuge.

```bash
sudo apt update && sudo apt install ros-dev-tools
```

## Schritt 2
### Installation

Mit `sudo apt update`aktualisieren wir unseren "Repository cache". (Diesen Befehl haben wir bereits mehrfach verwendet, aber falls bei diesem Schritt eingestiegen wird steht er hier erneut.)

Bevor wir ROS2 installieren updaten wir unsere Systemdateien mit `sudo apt upgrade`.

Wir installieren nun ROS, RViz, demos und tutorials mit

```bash
sudo apt install ros-jazzy-desktop
```

Um unsere ROS2 Entwicklungsumgebung zu aktivieren schreiben wir `source /opt/ros/jazzy/setup.bash`.

Um die Installation zu verifizieren geben wir `ros2 run demo_nodes_cpp talker` ein.
Wir sollten nun Nachrichten wie diese sehen:
```bash
[INFO] [1760010581.519717306] [talker]: Publishing: 'Hello World: 1'
[INFO] [1760010582.519676343] [talker]: Publishing: 'Hello World: 2'
```

Wenn wir diesen Output sehen, öffnen wir ein neues Terminal mit <kbd>STRG</kbd> + <kbd>ALT</kbd> + <kbd>T</kbd> und geben diesmal `ros2 run demo_nodes_py listener` ein.
Wir sollten in diesem zweiten Terminal nun Nachrichten wie diese sehen:
```bash
[INFO] [1760010764.592431582] [listener]: I heard: [Hello World: 5]
[INFO] [1760010765.584406703] [listener]: I heard: [Hello World: 6]
```

Falls es zu Problemen kommt oder etwas nicht funktioniert kann die offizielle Dokumentation helfen: [Jazzy Jalisco Installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

### Anlegen eines eigenen Workspaces

Da wir mit unser Projekt mit Colcon bauen wollen, müssen wir dies auch installieren mit:

```bash
sudo apt install python3-colcon-common-extensions
```

Nun legen wir unseren eigenen ROS2 Workspace an. Dafür geben wir ein:
(Wir befinden uns im ~/ Ordner)
```bash
mkdir -p ~/ROS2-Igus-Einfuehrung-Robotik/src
```

mit `cd ~/ROS2-Igus-Einfuehrung-Robotik/src` wechseln wir in das neu angelegte Verzeichnis.

Jetzt wollen wir unseren src-Ordner mit den entsprechenden Repositories füllen.
Dafür nutzen wir
```bash
git clone PLATZHALTER
```

```bash
cd ~/ROS2-Igus-Einfuehrung-Robotik
```

Wir bauen unseren Workspace, aus dem Workspace-Ordner heraus, mit
```bash
colcon build --symlink-install
```

Unsere Ordnerstruktur sollte jetzt so aussehen:

ORDNERSTRUKTUR
## Igus-Rebel Roboterarm Installation

Das ROS2 Paket für den Igus Rebel von Truphysics kann momentan nur genutzt werden indem es "from source" gebaut wird.

```bash
cd ROS2-Igus-Einfuehrung-Robotik/src
git clone https://bitbucket.org/truphysics/igus_rebel_ros2.git
cd ..
rosdep install --from-paths . --ignore-src -r -y
colcon build
```

Die IP des Roboterarms wird in `src/igus_rebel/include/Rebel.hpp`angegeben mit `192.168.3.11:3920`. Für unser Projekt funktioniert das, sollte die IP sich aber mal ändern, kann man sie dort anpassen.

### Nutzung

#### Auf einem realen Igus Rebel Arm :

Hardware Interface Controler um mit dem Roboter zu kommunizieren.
```bash
ros2 launch igus_rebel rebel.launch.py
```

Moveit Motion Planner und "teleoperation mode" um den Roboter zu steuern.
```bash
ros2 launch igus_rebel_moveit_config igus_rebel_motion_planner.launch.py use_gui:=true
```
#### In einer Simulation :

Simulation und Kontrolle des Robotermodels in Gazebo.
```bash
ros2 launch igus_rebel_moveit_config igus_rebel_simulated.launch.py
```

#### Digitale Outputs setzen

Die digitalen Ausgänge des Roboterarms können mit Aufrufen eines ROS2 Services genutzt werden.
Der Service heißt `/set_digital_output`.

Dieser braucht einen Input als `DigitalOutput`Nachrichtentyp, die wie folgt aussieht:
```bash
int8 output # index vom Output dessen Status gesetzt werden soll
bool is_on # der Status der gesetzt werden soll (True/False = an/aus)
```

Die Serviceantwort (Service output) ist immer
```bash
bool success # immer "True"
string message # Immer leer
```

#### Bedienung mit einer Tastatur :
```bash
ros2 run igus_rebel_moveit_config rebel_servo_teleop_keyboard
```

Um den TCP (Tool Center Point) im "World Frame" zu steuern, benutzen wir <KBD>W</KBD> und <KBD>T</KBD>. Dann nutzen wir <KBD>.</KBD> und <KBD>;</KBD>, um den TCP vorwärts, rückwärts, nach links, nach rechts, nach unten,oder nach oben zu bewegen.

## RealSense Kamera Installation
## Schritt 1
### librealsense2 / Intel RealSense SDK 2.0 installieren

`sudo apt install ros-jazzy-librealsense2*` ---> NICHT GETESTET ! ! !

Alternative:

Zuerst registrieren wir den "server's public key":

```bash
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
```

Dann stellen wir sicher, dass "apt HTTPS support" installiert ist:

```bash
sudo apt-get install apt-transport-https
```

Nun fügen wir den Server zu unserer Liste der Repositories hinzu:

```bash
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
```

Wir installieren die librealsense2-Pakete mit:

```bash
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
```

Wir können die Installation mit `realsense-viewer` überprüfen.
Um sicherzugehen, dass der "Kernel" aktualisiert wurde können wir `modinfo uvcvideo | grep "version:"`verwenden. Dies sollte dann `realsense`enthalten.

### ROS2 RealSense installieren
Option 1 Debian Package:

Alle RealSense-ROS2-Pakete installieren mit
```bash
sudo apt install ros-jazzy-realsense2-*
```

Option 2 "from source":

Im src-Ordner des Workspace
```bash
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master
cd ..

### Install dependencies with
sudo apt-get install python3-rosdep -y
sudo rosdep init # "sudo rosdep init --include-eol-distros" for Foxy and earlier
rosdep update # "sudo rosdep update --include-eol-distros" for Foxy and earlier
rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y

### Build with
colcon build

### Source environment
ROS_DISTRO=Jazzy
source /opt/ros/Jazzy/setup.bash
cd ~/ROS2-Igus-Einfuehrung-Robotik
. install/local_setup.bash
```
