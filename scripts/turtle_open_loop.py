#!/usr/bin/env python
# coding: utf-8

import rospy
from turtlesim.msg import Pose
import math

from turtle_kinematics import TurtleKinematics

class TurtleOpenLoop(TurtleKinematics):
    ''' Classe para o controle em malha aberta da tartaruga.
    O movimento da tartaruga deve se basear na cinematica classica:
        Deslocamento = Velcidade x Tempo
    '''
    def __init__(self):
        ''' Construtor da classe TurtleOpenLoop
        Todo metodo da classe deve ter como primeiro parametro o ponteiro da classe (self)

        Analogamente, variaveis inicializadas dentro da classe devem ser chamadas e declaradas
        pelo o prefixo self.var_name.
        '''
        TurtleKinematics.__init__(self) # calls parent class constructor
        # Posicao linear e angular da tartaruga
        self.turtle_pose = Pose(x=0, y=0, theta=0)
        # equivalent to:
        #   self.turtle_pose.x = 0
        #   self.turtle_pose.y = 0
        #   self.turtle_pose.theta = 0

    def move_untill(self, time, vel, ang):
        ''' Realiza movimento durante uma quantidade de tempo definida.
        O movimento eh definido por velocidade angular e velocidade linear.
        O intervalo de tempo, em segundos deve ser informado para a funcao.
        '''
        now = rospy.get_time()
        wait_time = now + time
        rate = rospy.Rate(10) # loop rate (Hz)
        while now < wait_time:
            self.move_general(vel, ang)
            now = rospy.get_time()
            rate.sleep()
        self.stop()

    def go_to_point_relative(self, x, y, vel_lin=0.5, vel_ang=0.5):
        ''' Recebe ponto relativo a tartaruga em 2D e vai ate o ponto.
        Dica: tente primeiro o movimento angular e depois o linear.
        '''
        pass

    def go_to_point_absolute(self, x, y, vel_lin=0.5, vel_ang=0.5):
        ''' Movimenta tartaruga até coordenadas x e y absolutas
        Dica: use self.turtle_pose como referência.
        '''
        pass
