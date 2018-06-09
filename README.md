# Dojo de Programação ROS-Works

Bem vindos ao repositório oficial do Dojo de Programação ROS Works, promovido pelo capitulo IEEE Control Systems Society UnB.

### Coding Dojo (09/06/2018)
- Aplicar TDD (Test Driven Development)
- Parte 1 (14h - 16h)
    - Desenvolver funções de controle de malha aberta no simulador Turtlesim
    - Arquivo fonte: [coding_dojo_2018/turtle\_open\_loop.py](https://github.com/CSS-UnB/ros-works-dojos/blob/master/coding_dojo_2018/turtle_open_loop.py)
- Parte 2 (16h - 18h)    
    - Desenvolver funções de controle de malha aberta no simulador Turtlesim
    - Arquivo fonte: [coding_dojo_2018/turtle\_closed\_loop.py](https://github.com/CSS-UnB/ros-works-dojos/blob/master/coding_dojo_2018/turtle_closed_loop.py)
    
### Processo Seletivo IEEE CSS
- Dia 1 (25/04/2018)
    - Desenvolver funções de controle de malha aberta no simulador Turtlesim
    - Arquivo fonte: [dojo_ps_css/turtle\_open\_loop.py](https://github.com/CSS-UnB/ros-works-dojos/blob/master/scripts/turtle_open_loop.py)

- Dia 2 (26/04/2018)
    - Desenvolver funções de controle de malha aberta no simulador Turtlesim
    - Arquivo fonte: [dojo_ps_css/turtle\_closed\_loop.py](https://github.com/CSS-UnB/ros-works-dojos/blob/master/scripts/turtle_closed_loop.py)

### Testes de Unitarios
Para adicionar testes unitarios, deve-se adiciona-los a pasta
```
    [dojo_folder]/test/
```
e verificar se existe a diretiva no arquivo CMakeLists.txt:
```
    catkin_add_nosetests([dojo_folder]/test/)
```
Ao compilar o workspace, e necessaria a chamada:
```
    catkin build --catkin-make-args run_tests
```


### Testes de Instegracao
Testes de integracao possuem uma sintaxe semelhante a launch files.
Eles devem ser armazenados na pasta
```
    test/
```
e, apos compilados, chamados pela funcao
```
    rostest ros-works-dojo [testname.test]
```
