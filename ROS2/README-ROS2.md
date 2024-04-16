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
  
A fim de inicializar o espaço de trabalho do projeto, segue-se o tutorial a seguir:
- https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html
- sudo apt install python3-colcon-common-extensions
- cd /usr/share/colcon_argcomplete/hook/
- gedit ~/.bashrc
Então, deve-se referenciar os endereços a seguir para poder utilizar as linhas de comando ros2:
- source /opt/ros/humble/setup.bash
- source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
Iniciando o trabalho a partir da criação de um espaço pfc_ws, foram criados os arquivos necessários a partir de:
- source .bashrc
- mkdir pfc_ws
- cd pfc_ws
- mkdir src
- colcon build
Os arquivos desejados podem ser encontrados no endereço a seguir, mas a pasta zip foi anexada junto ao diretório deste projeto.
- https://github.com/ros-controls/gazebo_ros2_control

