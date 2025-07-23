# Manipulador SCARA com Garra – Simulação ROS + Gazebo

Este projeto consiste em uma simulação de um manipulador SCARA com uma garra Robotiq 2F-85, utilizando **ROS** e **Gazebo**. O robô é controlado via `ros_control` e sua cinemática é baseada em parâmetros DH. A simulação inclui controle PID e visualização 3D da cena.

## ✅ Requisitos

- Ubuntu 20.04
- ROS Noetic Ninjemys
- Gazebo 11
- Catkin Tools (opcional, mas recomendado)
- RViz (para visualização)
- `joint_state_publisher_gui`

### Instalação dos Pacotes Necessários

```bash
sudo apt update
sudo apt install ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-joint-state-publisher-gui
```

### Clonando os Pacotes Relevantes

```bash
cd ~/catkin_ws/src
git clone https://github.com/gustavoborges58/engg02-grasping.git

cd ~/catkin_ws
catkin build
source devel/setup.bash
```

## ▶️ Como Rodar a Simulação

### 1. Lance a simulação no Gazebo:

```bash
roslaunch e05 gazebo_e05_with_gripper.launch
```

### 2. Adicione o cilindro no RViz (simulando a garrafa):

Em outro terminal, com o workspace já "sourced":

``` bash
rosrun e05 cylinder_marker.py
```

Esse script publica um marcador visual (RViz) em formato de cilindro, que pode ser usado para testes de cinemática e movimentação da garra.

Para inserir o cilindro no ambiente de visualização, basta ir na lista de visualizações e realizar os seguintes passos:

* `Add`
* `By topic`
* `/beer` > `Marker`
* `Ok`

Após realizar essas etapas, o cilindro será inserido no ambiente.

### Publicar os status da junta móvel e garra

Usando a Joint State Publisher GUI, manipular a posição da garra e ângulo de abertura.

## ⚙️ Notas Técnicas

* O gripper utiliza o pacote oficial da robotiq_2f_85.
* Os ganhos PID estão configurados no arquivo `/e05/config/pid_gains.yaml`

## 🐍 Modelagem em Python

Link para script em python no Google Colab, contendo a visualização esquemática animada da vista superior do movimento de pega da garrafa e implementação dos modelos dinâmicos do robô criado e do robô real, juntamente com gráfico comparativo das forças de contato aplicadas por cada robô: [Implementação Modelo Gripper Robotiq 2F-85 - Tecnicas de Grasping.ipynb](https://colab.research.google.com/drive/1TsliUVON-hOqVyTDr-TxEpC-8tPIIfzb?usp=sharing "Link do Google Colab").