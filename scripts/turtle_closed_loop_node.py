#!/usr/bin/env python
# coding: utf-8
import rospy
import turtle_closed_loop

def print_menu():
    ''' Imprime menu e retorna opcao escolhida '''
    print '--------- ROS WORKS TURTLESIM CLOSED LOOP --------- (v0.1)'
    print 'Where do you want to go?'
    coord_x = get_value('X coordinate >> ')
    coord_y = get_value('Y coordinate >> ')
    return coord_x, coord_y

def get_value(message):
    ''' Imprime pergunta e recebe valor inteiro '''
    value = raw_input(message)
    value = float(value)
    return value

def main():
    ''' Nó executável para o controle em malha fechada '''
    rospy.init_node('turtle_closed_loop')

    turtle = turtle_closed_loop.TurtleClosedLoop()

    while not rospy.is_shutdown():
        try:
            x, y = print_menu()
            turtle.go_to_point(x, y)
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    ''' Chamada da funcao principal executavel '''
    main()
