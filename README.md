# Dojo de Programação ROS-Works

Bem vindos ao repositório oficial do Dojo de Programação ROS Works, promovido pelo capitulo IEEE Control Systems Society UnB.

### Processo Seletivo IEEE CSS
- Dia 1 (25/04/2018)
    - Desenvolver funções de controle de malha aberta no simulador Turtlesim
    - Arquivo fonte: [scripts/turtle\_open\_loop.py](https://github.com/CSS-UnB/ros-works-dojos/blob/master/scripts/turtle_open_loop.py)

- Dia 2 (26/04/2018)
    - Desenvolver funções de controle de malha aberta no simulador Turtlesim
    - Arquivo fonte: [scripts/turtle\_closed\_loop.py](https://github.com/CSS-UnB/ros-works-dojos/blob/master/scripts/turtle_closed_loop.py)

### Testes Unitarios
Para adicionar testes unitarios, deve-se adiciona-los a pasta
```
    test/
```
Ao compilar o workspace, e necessaria a chamada:
```
    catkin build --catkin-make-args run_tests
```
