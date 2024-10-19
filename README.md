# ROS2 + Gazebo Harmonic docker

Repozytorium zawiera kod do wykorzystania przez studentów w celu realizacji zajęć z Systemów i Algorytmów Percepcji w Pojazdach Autonomicznych (SiAPwPA) w semestrze zimowym 2024/2025.

## Requirements
- [Docker](https://docs.docker.com/engine/install/ubuntu/)
- [Nvidia Docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#container-device-interface-cdi-support)
- [VS Code devcontainer plugin](https://code.visualstudio.com/docs/devcontainers/containers#_quick-start-open-an-existing-folder-in-a-container)

> [!IMPORTANT]
System operacyjnym Ubuntu jest wymagany ze względu na obsługę GUI (wymaga innego podejścia przy wykorzystaniu Windowsa).

## Start
Otwórz VS Code w katalogu z projektem.
Przejdź do lewego dolnego rogu i kliknij niebieską ikonę z dwiema strzałkami skierowanymi do siebie. Z rozwijanego menu wybierz **"Open Folder in Container... ”** i poczekaj, aż docker się zbuduje. Może to potrwać do 10 minut przy wolniejszym połączeniu internetowym.

Po zalogowaniu się do dockera będzie on działał w sposób podobny do uruchamiania ROS na komputerze hosta. Wszystkie aplikacje GUI będą korzystać z domyślnego menedżera okien hosta, będziesz mieć również dostęp do wszystkich urządzeń na hoście, a także akceleracji GPU.
Docker ma preinstalowany [ROS 2 Humble](https://docs.ros.org/en/humble/Tutorials.html) i większość potrzebnych zależności oraz symulator [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/getstarted/). 

## First step
Dla osób, które nie miały doczynienia ze środowiskiem ROS 2 + Gazebo, zachęcam do przerobienia tutorialu: [Gazebo Tutorial](https://gazebosim.org/docs/harmonic/tutorials/). Pozwoli to zaznajomić się z tym środowiskiem i tworzyć w przyszłości zaawansowane symulacje.

Następnie pomocne będzie odpowiednia kontrola robotami w środowisku symulacyjnym, na dobry start proszę zaznajomić się z repozytorium: [Gazebo ROS 2 Control](https://github.com/ros-controls/gz_ros2_control/).

Na sam koniec pewnym podsumowaniem, a także praktycznym podejściem do tematu jest dostarczony od [Husariona](https://husarion.com/tutorials/ros2-tutorials/1-ros2-introduction/) tutorial dla ich kilku robotów.


>[!IMPORTANT]
> Należy pamiętać, aby po zbudowaniu wywołać komendę lub pracować w nowym terminalu: 
> ``` bash
> source ~/.bashrc
> ```
> W tym pliku dodane są już dwie ważne ścieżki:
> ``` bash
> /opt/ros/$ROS_DISTRO/setup.bash
> /home/developer/ros2_ws/install/setup.bash
> ```

# ROS 2 -- Getting Started

## ROS2 Examples
Repozytorium z przykładami dla ROS2 Humble:
```bash
git clone https://github.com/ros/ros_tutorials.git -b humble
```

## Dependencies
Dobrą praktyką jest sprawdzanie zależności za każdym razem, gdy dodajemy coś do obszaru roboczego.
Wywołujemy komende z poziomu folderu np. `~/ros2_ws`, który zawiera `/src`
```bash
rosdep install -i --from-path src --rosdistro humble -y
```

W przypadku powodzenia pojawi się komunikat: `#All required rosdeps installed successfully`

## Build the workspace 
Budowanie obszaru roboczego odbywa się z poziomu foldera `~/ros2_ws`, używając:
```bash
colcon build --symlink-install
```
Podczas budowania za pomocą `colcon` można wykorzystać następujące flagi ([Więcej informacji](https://colcon.readthedocs.io/en/released/reference/verb/build.html)): 

`--packages-up-to` builds the package you want, plus all its dependencies, but not the whole workspace (saves time).  
`--symlink-install` saves you from having to rebuild every time you tweak python scripts.  
`--event-handlers` console_direct+ shows console output while building (can otherwise be found in the log directory).  
`--executor` sequential processes the packages one by one instead of using parallelism.
`--merge-install` With this option most of the paths added to environment variables will be the same, resulting in shorter environment variable values. The disadvantage of using this option is that it doesn’t provide proper isolation between packages.  

W celu przetestowania poprawności zbudowania obszaru roboczego można wywołać:
```bash
colcon test
```

Usunięcie folderów `build`, `install`, `log` odbywa się przy użyciu:
```
colcon clean workspace
```

## Gazebo Harmonic

Gazebo Harmonic posiada wiele gotowych modeli obiektów oraz światów, zarówno po włączeniu symulatora można wybrać przykładowe środowiska, ale także ściągnąć ze strony [Gazebo](https://app.gazebosim.org/dashboard).

Uruchomienie przykładowej symulacji:
```bash
gz sim sonoma.sdf
```

W przypadku problemów z uruchomieniem Gazebo:
```bash
ps -ef | grep "gz sim"
pkill -f gz
pkil -f "gz sim"
```



## Cheat Sheet

Przykład pobierający kod źródłowy bezpośrednio z repozytoriów na podstawie odpowiedniego pliku
```
vcs import --recursive < /home/developer/ros2_ws/src/px4.repos src

rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y

BUILD_TYPE=RelWithDebInfo
colcon build \
        --merge-install \
        --symlink-install \
        --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
        -Wall -Wextra -Wpedantic

source install/setup.bash
```

Budowanie workspace'a:
``` bash
colcon build
```

W celu sprawdzenia dostępnych `topicow` w nowym oknie terminala uruchom:
``` bash
ros2 topic list
```
