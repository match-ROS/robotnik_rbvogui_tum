# robotnik_rbvogui_tum

Eigenständiges ROS-2-Paket für das Spawnen einer Robotnik RB-VOGUI-XL in Gazebo.
Das Modell besteht aus der RB-VOGUI-XL-Basis, 285-mm-Lenkrollen, einem Ewellix-
900-mm-Lift und einem UR-Arm (standardmäßig UR20). Es enthält die lokale
Integrations-URDF und die dazugehörigen Räder, Controller-Konfigurationen und
Pose-/Swerve-Hilfsknoten. Die offiziellen Robotnik- und Universal-Robots-Quellen
werden als Abhängigkeiten importiert und bleiben unverändert.

## Voraussetzungen

Getestet ist die Integration für ROS 2 Jazzy mit Gazebo Harmonic. Benötigt werden
`vcs`, `rosdep` und die üblichen ROS-2-Gazebo-Pakete.

## Minimales Setup

Lege dieses Repository in den `src`-Ordner eines ROS-2-Workspaces und importiere
die öffentlichen Quellen:

```bash
cd ~/ros2_ws
vcs import src < src/robotnik/robotnik_rbvogui_tum/dependencies/robotnik_rbvogui_tum.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to robotnik_rbvogui_tum
source install/setup.bash
```

Danach startet der folgende Befehl eine leere Gazebo-Welt, erzeugt daraus die
`robot_description` und spawnt das Modell als Entity `robot`:

```bash
ros2 launch robotnik_rbvogui_tum rbvogui_ur_standard_control.launch.py gui:=true
```

Für einen Headless-Start:

```bash
ros2 launch robotnik_rbvogui_tum rbvogui_ur_standard_control.launch.py gui:=false
```

Nützliche Argumente:

```bash
ros2 launch robotnik_rbvogui_tum rbvogui_ur_standard_control.launch.py \
  robot_id:=robot arm_type:=ur20 x:=0.0 y:=0.0 z:=0.1
```

`arm_type` wird an die offizielle `ur_description` weitergereicht. Es muss ein
dort unterstütztes UR-Modell sein.

## Inhalt und Abgrenzung

- `urdf/rbvogui_ur_standard_control.urdf.xacro`: lokale Gesamtbeschreibung.
- `urdf/wheels`: lokale 285-mm-Radvarianten.
- `config/rbvogui_standard_controllers.yaml`: Standard-Joint-Controller für die
  Simulation.
- `scripts/rbvogui_swerve_controller.py`: wandelt `Twist` in Lenk- und
  Radbefehle um.
- `scripts/tf_model_pose_to_pose_stamped.py`: veröffentlicht die Modellpose als
  `/robot_pose`.
- `scripts/current_pose_from_tf.py`: veröffentlicht die TCP-Pose als
  `/current_tcp_pose`.

Der Launch veröffentlicht `robot_description` unter `/<robot_id>/robot_description`
und spawnt daraus das Gazebo-Modell. Der Fahrbefehl für die lokale Simulation ist:

```text
/<robot_id>/robotnik_base_control/cmd_vel_unstamped
```

Dieses Repository enthält keine Kopie der offiziellen Robotnik-Description. Die
Upstream-Abhängigkeiten werden über die Datei in `dependencies/` bezogen.

## Vor dem ersten Push

Passe in `package.xml` die Maintainer-E-Mail-Adresse an. Anschließend kann das
Repository normal auf GitHub veröffentlicht werden:

```bash
git add .
git commit -m "Initial standalone RB-VOGUI Gazebo simulation"
git branch -M main
git remote add origin <DEIN-GITHUB-REPOSITORY>
git push -u origin main
```
