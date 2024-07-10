# PFC

Partindo do uso de um sistema operacional Ubuntu 22.04.4 LTS, os primeiros programas a instalar foram:
1. GitHub Desktop - Para facilitar a comunicação intuitiva com o diretório de arquivos do projeto.
   - sudo wget https://github.com/shiftkey/desktop/releases/download/release-3.3.3-linux1/GitHubDesktop-linux-amd64-3.3.3-linux1.deb
   - sudo apt install ./GitHubDesktop-linux-amd64-3.3.3-linux1.deb -y
2. VSCode - Para facilitar a manipulação de arquivos.
   - Foi utilizada a própria loja de programas do software.
3. ROS Humble - Que será a base de todo o trabalho.
   - É suficiente seguir o passo a passo de https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html.
4. Gazebo Harmonic - Que será a base das simulações.
   - É suficiente seguir o passo a passo de https://gazebosim.org/docs/harmonic/install_ubuntu.
  
A fim de inicializar o espaço de trabalho do projeto, segue-se os tutoriais a seguir:
- https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html
- https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html
- sudo apt install python3-colcon-common-extensions

Então, deve-se referenciar os endereços a seguir para poder utilizar as linhas de comando ros2:
- source /opt/ros/humble/setup.bash
- source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

Após a build ter sido finalizada, deve-se referenciar também:
- source ~/PFC/pfc_ws/install/setup.bash

A partir daqui, iniciou-se uma atuação com o material utilizado pelo Ten Wenis - CTEx, de modo que foi gerado uma outra área de trabalho. Para evitar confusões, a fim de mudar a área do projeto em que se está trabalhando é necessário editar as linhas de referência do documento ~/.bashrc.
- sudo apt install ros-humble-moveit && sudo apt install ros-humble-grasping-msgs && sudo apt install ros-humble-gazebo-ros2-control && sudo apt install ros-humble-ros2-control && sudo apt install ros-humble-joint-state-broadcaster && sudo apt install ros-humble-joint-trajectory-controller && sudo apt install ros-humble-velocity-controllers && sudo apt install ros-humble-pcl-ros

Para o espaço de trabalho do Wenis, utilizar:
- ros2 launch arm_pkg main_arm.launch.py
- ros2 launch arm_pkg start_moveit.launch.py




