# -*- coding: utf-8 -*-
"""
Created on Fri Oct 26 11:45:57 2018

@author: Sergio
"""
#TODO corregir posiciones fraccionarias, pues no esta llegando a la posicion de 8250 posiblemente porque se convierte en 82.5 -> 82.0 
import argparse
try:
    import numpy as np
    import matplotlib.pyplot as plt
    is_numpy = True
    is_plt = True
except:
    is_numpy = False
    is_plt = False  

def border_points(width, hight, xi, yi):
    
    border_points = []
    for i in range(2):
        for j in range(2):
            p_coord = (i*width + xi, j*hight + yi)
            border_points.append(p_coord)
            
    border_points[-1], border_points[-2] = border_points[-2], border_points[-1]
    return border_points


def join_points(points):
    '''
    points = [(x1, y1), (x2, y2)]
    '''
    
    ox = []
    oy = []
    
    x_range = int(abs(points[0][0] - points[1][0]))
    y_range = int(abs(points[0][1] - points[1][1]))
    
    
    min_x = min([points[0][0], points[1][0]])
    min_y = min([points[0][1], points[1][1]])
    
    if x_range != 0: 
        x_range += 1
    
    if y_range != 0: 
        y_range += 1
    
    for x in range(x_range):
        ox.append(x + min_x)
        oy.append(points[0][1])

    for y in range(y_range):
        oy.append(y + min_y)
        ox.append(points[0][0])
    
    return ox, oy

def convert_to_quad_obstacle(points):
    
    '''
    points = [(x1, y1), (x2, y2)] - puntos diagonoles que definen el cuadrilatero
    '''

    width = abs(points[0][0] - points[1][0])
    hight = abs(points[0][1] - points[1][1])
    
    obstacle_points = border_points(width, hight, points[0][0], points[0][1])
    
    return obstacle_points
    

def create_quad(quad_points):
    '''
    quad_points = [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
    '''    
    ox = []
    oy = []
    
    ext_quad_points = 2*quad_points
    
    for i in range(len(quad_points)):
        couple_to_join = ext_quad_points[i: i+2]
        #print 'join: ', couple_to_join
        ox_, oy_ = join_points(couple_to_join)
        ox += ox_ 
        oy += oy_
        
        
    return ox, oy

def read_obstacle_from_file(file_adress):
    obstacle_file = open(file_adress, 'r')
    
    lines = []

    for line in obstacle_file:
        line = line.split(" ")[1:]
        for i in range(len(line)):
            line[i] = line[i].strip(",")
            line[i] = line[i].strip('\n')
            line[i] = float(line[i])
            line[i] = float(line[i]/100.0)
        lines.append(line)

    width, hight = lines[0]
    
    q0 = tuple(lines[1])
    qf = tuple(lines[2])
    qL0 = tuple(lines[3])
    qLm =  tuple(lines[4])
    qLf = tuple(lines[5])
    n_obstacles = lines[6]
    
    obs_points = []
    
    i = 0

    while i < len(lines[7:]) - 1:
        obs = [tuple(lines[7 + i]), tuple(lines[7 + i + 1])]
        obs_points.append(obs)
        i += 2
    
    
    obstacle_file.close()
    return width, hight, q0, qf, qL0, qLm, qLf, n_obstacles, obs_points


def create_map(width, hight, obs_points):
    '''
    width: dim 1
    hight = dim 2
    obs_points = [[(x1, y1), (x2, y2)], [(x1, y1), (x2, y2)], ...]
    '''
    bd_points = border_points(width, hight, 0, 0)
    ox, oy = create_quad(bd_points)
    
    for obstacle in obs_points:
        quad_obs_points = convert_to_quad_obstacle(obstacle)
        ox_ , oy_ = create_quad(quad_obs_points)
        ox += ox_ 
        oy += oy_
        
    obs_coordinates = zip(ox, oy)
    obs_coordinates = sorted(set(obs_coordinates), key = obs_coordinates.index)
    obs_coordinates = zip(*obs_coordinates)
    ox, oy = obs_coordinates
    ox, oy = list(ox), list(oy)

    return ox, oy


def main(file_adress):
    
    #ap = argparse.ArgumentParser()
    #ap.add_argument("-f", "--ofile", default = 'datos-pruebas/Escenario-Base.txt',  help = "path to the txt map file")
    #file_adress = vars(ap.parse_args())
    width, hight, q0, qf, qL0, qLm, qLf, n_obstacles, obs_points = read_obstacle_from_file(file_adress)#'datos-pruebas/Escenario-Base.txt')
    
    #width, hight = width + 1, hight + 1
    ox, oy = create_map(width, hight, obs_points)
    #print(ox)
    
    bd_points = border_points(width, hight, 0, 0)
    #plt.scatter(ox, oy, s=2)
    if is_numpy and is_plt:
        plt.scatter(np.array(bd_points)[:, 0], np.array(bd_points)[:, 1])
        
    return ox, oy, width, hight, q0, qf

    
if __name__ == '__main__':
    main()
