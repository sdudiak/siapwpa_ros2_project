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

## Glossary 

* `Underlying communication layer (DDS)` - It provides high-performance, reliable, real-time Underlying communication layer (DDS) data communication and integration capabilities, thereby establishing fundamental support for messaging and service invocation between nodes in ROS2.
* `Node` - Node is the smallest unit of processing running in ROS. It is typically an executable file. Each node Node can use topics or services to communicate with other nodes.
* `Message` - The variables of data types such as int, float and Message boolean.
* `Topic` - A one-way asynchronous communication mechanism. By publishing messages to topics or subscribing to topics, the data transmission Topic between nodes can be realized. The topic type is determined by the type of corresponding message.
* `Publishing` - Send data with a message type corresponding to Publishing the topic content.
* `Publishers` - For publishing to take place,the publisher node registers various information such as its topics on Publishers the master node, and transmits messages to subscribing nodes that wish to subscribe.
* `Subscribing` - Receive data with a message type corresponding topic content.
* `Subscribers` - For subscription to take place,the subscribing node registers various information such as its topics on the master node. Subsequently, it receives all messages from publisher nodes that have published topics of interest to this node, via the master node.
* `Services` - A bidirectional synchronous communication mechanism where the service is provided to the client request corresponding to a specific task and service servers gives the service response.
* `Service Servers` - A node taking requests as input, and providing responses as output.
* `Service Clients` - A node taking responding as input, and providing requests as output.
* `URDF file` - A model file describing robot’s entire elements, including link, joint, kinematics parameters, dynamics parameters, visual models and collision detection models.
* `Srv file` - It is stored in the `srv` folder used to define ROS service messages, consisting of two parts: request and respond. The request and respond are separated by the “---” symbol.
* `Msg file` - It is stored in the `msg` folder used to define ROS topic messages.
* `package.xml` - Descriptio of the package attributes, including the package name,version number,authorship and other information.
* `CmakeLists.txt` - Compile the configuration file using Cmake.
* `launch` - Launch files contain system-wide instructions for launching nodes and services required for the robot to operate.

## Cheat Sheet

### Clone repositories `vcs import`
```
vcs import --recursive < /home/developer/ros2_ws/src/px4.repos src
```

### Update dependencies `rosdep update`  
```properties
rosdep update --rosdistro $ROS_DISTRO
```

### Install dependencies `rosdep install`  
Dobrą praktyką jest sprawdzanie zależności za każdym razem, gdy dodajemy coś do obszaru roboczego.
Wywołujemy komende z poziomu folderu np. `~/ros2_ws`, który zawiera `/src`
```bash
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
```
W przypadku powodzenia pojawi się komunikat: `#All required rosdeps installed successfully`

### Build the workspace `colcon build`  
Budowanie obszaru roboczego odbywa się z poziomu foldera `~/ros2_ws`.
```bash
colcon build --symlink-install 
```

Podczas budowania za pomocą `colcon` można wykorzystać następujące flagi ([Więcej informacji](https://colcon.readthedocs.io/en/released/reference/verb/build.html)): 

`--packages-up-to` builds the package you want, plus all its dependencies, but not the whole workspace (saves time).  
`--symlink-install` saves you from having to rebuild every time you tweak python scripts.  
`--event-handlers` console_direct+ shows console output while building (can otherwise be found in the log directory).  
`--executor` sequential processes the packages one by one instead of using parallelism.  
`--merge-install` With this option most of the paths added to environment variables will be the same, resulting in shorter environment variable values. The disadvantage of using this option is that it doesn’t provide proper isolation between packages.  


### Test the workspace
W celu przetestowania poprawności zbudowania obszaru roboczego można wywołać:
```bash
colcon test
```


### Clean the workspace
Usunięcie folderów `build`, `install`, `log`.
```bash
colcon clean workspace
```

## ROS2 Package


### C++
* CMakeLists.txt file that describes how to build the code within the package  
* `include/<package_name>` directory containing the public headers for the package  
* `package.xml` file containing meta information about the package  
* `src` directory containing the source code for the package

```bash
ros2 pkg create --build-type ament_cmake --node-name my_node my_package
```

### Python
* `package.xml` file containing meta information about the package  
* `resource/<package_name>` marker file for the package  
* `setup.cfg` is required when a package has executables, so `ros2 run` can find them  
* `setup.py` containing instructions for how to install the package  
* `<package_name>` - a directory with the same name as your package, used by ROS 2 tools to find your package, contains `__init__.py`

```bash
ros2 pkg create --build-type ament_python --node-name my_node my_package
```

### ROS2 command
``` bash
ros2 topic list/info/echo/pub
ros2 pkg list/create/xml
ros2 node list/info
ros2 interface list/show
```

```bash
ros2 pkg create --build-type ament_python/ament_cmake --node-name <node_name> <package_name>
```

``` bash
ros2 launch <package_name> <launch_file_name>
```


## Tutorial

[URDF Tutorial](https://github.com/ros/urdf_tutorial)
[ROS2 + Gazebo](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)
[Gazebo YT](https://www.youtube.com/watch?v=DsjJtC8QTQY&ab_channel=TheConstruct)



