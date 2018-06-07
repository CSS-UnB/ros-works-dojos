#!/usr/bin/env python
# coding: utf-8
import rospy
from turtle_open_loop import TurtleOpenLoop

def print_menu():
    ''' Imprime menu e retorna opcao escolhida '''
    print '--------- ROS WORKS TURTLESIM SWARM --------- (v0.1)'
    print 'Relative or absolute coordinates?'
    print '1 - Relative'
    print '2 - Absolute'
    choice = get_value('Your choice >> ')
    print 'Where do you want to go?'
    coord_x = get_value('X coordinate >> ')
    coord_y = get_value('Y coordinate >> ')
    return choice, coord_x, coord_y

def get_value(message):
    ''' Imprime pergunta e recebe valor inteiro '''
    value = raw_input(message)
    value = float(value)
    return value

def main():
    ''' Nó executável para o controle em malha aberta '''
    rospy.init_node('turtle_open_loop')
    turtle = TurtleOpenLoop()

    while not rospy.is_shutdown():
        try:
            choice, coord_x, coord_y = print_menu()
            if choice == 1:
                ''' Relative coordinates '''
                turtle.go_to_point_relative(coord_x, coord_y)
            if choice == 2:
                ''' Absolute coordinates '''
                turtle.go_to_point_absolute(coord_x, coord_y)
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    main()
