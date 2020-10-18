HackSEA USP - Equipe 15
- Gabriela Yuri Ishikawa - [gabiishikawa@usp.br](mailto:gabiishikawa@usp.br)   
Engenahria Mecatrônica - Poli-USP
- Letícia Miyuki Kimoto - [leticiakimoto@usp.br](mailto:leticiakimoto@usp.br)   
Engenharia Mecatrônica - Poli-USP
- Pedro Pimentel Fuoco - [fuoco@usp.br](mailto:fuoco@usp.br)   
Engenharia Mecatrônica - Poli-USP
- Tomaz Maia Suller - [tomaz.suller@usp.br](mailto:tomaz.suller@usp.br)   
Engenharia de Computação - Poli-USP 

# Projeto Allnighter
O Projeto Allnighter visa possibilitar a adoção de drones autônomos na entrega rápida de alimentos. Está implementado o algoritmo de detecção de zona de pouso descrito na nossa solução, que permite a detecção de um "H" na imagem de uma câmera RGB.

## Motivação e Funcionamento
Para a operação autônoma de drones em sistemas de entrega, é essencial que os veículos sejam capazes de corrigir suas trajetórias de pouso ao se aproximarem de zonas de pouso. Para tanto, são implementados dois programas: um, `h.cpp`, para detecção de um "H" demarcando a zona de pouso, que também pulbica em uma mensagem ROS a localização relativa do H na imagem e a área que ele ocupa; e um sistema de controle PID que garante a estabilidade do drone sobre o H, implementado em `cv_control.py`.

## Instalação
### Dependências
#### Programas independentes
- [ROS Melodic](http://wiki.ros.org/melodic/Installation)
- [Catkin](http://wiki.ros.org/catkin)
- [PX4](https://dev.px4.io/v1.9.0/en/setup/dev_env.html)
- [MavROS](https://dev.px4.io/master/en/ros/mavros_installation.html)
- [Gazebo](http://gazebosim.org/tutorials?cat=install)
- Python > 2
- g++
#### Bibliotecas
- OpenCV (C++)
- cv_bridge (C++)
- numpy (Python)
- rospy (Python)
- simple_pid (Python)

## Uso
(admitindo operação em sistemas Linux, testado no Ubuntu 16.04 LTS)
- **Simulação**
1. Criar um catkin workspace   
`mkdir catkin_ws`   
`cd catkin_ws && mkdir src && cd src`   
`catkin_init_workspace`
2. Clonar este repositório dentro da `src` do workspace   
`git clone https://github.com/pedro-fuoco/thales2020.git`
3. Compilar o wokrspace   
`cd ..`
`catkin build thales2020`
4. Gerar o mundo da simulaço no Gazebo   
`source scripts/simulate.sh`
5. Executar arquivo da missão   
`roslaunch thales2020 pouso.launch`

- **Teste de detecção com a webcam**
1. Clonar este repositório   
`git clone https://github.com/pedro-fuoco/thales2020.git`
2. Compilar o arquivo `h_sem_ros.cpp`   
`cd thales2020/src`   
<p>g++ h_sem_ros.cpp -o webcam_h `pkg-config --cflags --libs opencv`</p>
3. Executar
`./webcam_h
