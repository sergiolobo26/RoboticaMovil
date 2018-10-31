# -*- coding: utf-8 -*-
from __future__ import division 
from AriaPy import *
import sys
import math
import create_map_test as c_map

# ----------------------------------------------------------------------------------------------------------------- #
# INITIALIZATION
Aria_init()
parser = ArArgumentParser(sys.argv)
parser.loadDefaultArguments()

# CREATE ROBOT
robot = ArRobot()
print "Connecting..."
con = ArRobotConnector(parser, robot)
if not Aria_parseArgs():
    Aria_logOptions()
    Aria_exit(1)

if not con.connectRobot():
    print "Could not connect to robot, exiting"
    Aria_exit(1)

# CREATE LASER
laserCon = ArLaserConnector(parser, robot, con)
print 'Connecting to laser and waiting 1 sec...'
laser = None
if laserCon.connectLasers() :
  print 'Connected to lasers as configured in parameters'
  laser = robot.findLaser(1)
else :
  print 'Warning: unable to connect to lasers. Continuing anyway!'

# CREATE FILES
## ROBOT DATA FILE
fileRobot = open( "./MapasGenerados/DataRobot.map", "w" )
fileRobot.write( "2D-Map\n" )
fileRobot.write( "DATA\n" )

## MAP DATA FILE
fileMap = open( "./MapasGenerados/DataMap.map", "w" )
fileMap.write( "2D-Map\n" )
fileMap.write( "DATA\n" )
fileMap.close()


#GLOBAL VARIABLES
OBSTACLE_FILE_PATH = 'datos-pruebas/Escenario-Base.txt'
PATH_COORDINATES = 'path_coordinates.txt'
# ----------------------------------------------------------------------------------------------------------------- #


# FUNCTIONS
## PROCESS LASER READINGS
def processData( C ) :

    fileMap = open( "./MapasGenerados/DataMap.map", "a" )
    if laser:
        laser.lockDevice()
        readings = laser.getRawReadingsAsVector()
        for r in readings:
            p = r.getPose( )
            x = p.getX( )
            y = p.getY( )
            fileRobot.write( str( int( x ) ) + " " + str( int( y ) ) + "\n" )
            
            X = x*math.cos( C[2] ) - y*math.sin( C[2] ) + C[0]
            Y = x*math.sin( C[2] ) + y*math.cos( C[2] ) + C[1]
            fileMap.write( str( int( X ) ) + " " + str( int( Y ) ) + "\n" )
            
        laser.unlockDevice()
    fileMap.close()


def append_map_coordinates():
    '''
    This function creates the document EscenaCuadrada-superposicion.map
    con la superposicion de la escena y los datos del robot transformados. 

    '''
    mapa_cuadrada = open("./"+ OBSTACLE_FILE_PATH[:-4] +".map", "r")
    fileRobot = open( "./MapasGenerados/DataRobot.map", "r" )
    fileMap = open( "./MapasGenerados/DataMap.map", "r" )
    
    mapa_cuadrada_sup = open("./MapasGenerados/"+ OBSTACLE_FILE_PATH[:-4] + "-superposicion.map", "w")
    mapa_cuadrada_sup_robot = open("./MapasGenerados/"+ OBSTACLE_FILE_PATH[:-4] +"-Robot-superposicion.map", "w")
    
    
    lines = mapa_cuadrada.readlines()
    for line in lines:
        mapa_cuadrada_sup.write(line)
        mapa_cuadrada_sup_robot.write(line)

    lines = fileMap.readlines()
    counter = 0
    for line in lines:
        if counter > 1:
            mapa_cuadrada_sup.write(line)
        counter += 1

    lines = fileRobot.readlines()
    counter = 0
    for line in lines:
        if counter > 1:
            mapa_cuadrada_sup_robot.write(line)
        counter += 1

    fileMap.close()
    mapa_cuadrada.close()
    mapa_cuadrada_sup.close()    
    mapa_cuadrada_sup_robot.close()
    fileRobot.close()

## CHANGE CONFIGURATIONS TO ROBOT FRAMEWORK
def robotConfigs( configs, initConfig ) :

    
    x0 = initConfig[0]
    y0 = initConfig[1]
    th0 = math.radians( initConfig[2] )

    x = - x0*math.cos( th0 ) - y0*math.sin( th0 )
    y = - y0*math.cos( th0 ) + x0*math.sin( th0 )

    rConfigs = []

    for c in configs :
        xT = c[0]
        yT = c[1]
        thT = c[2]

        configT = [ 
            xT*math.cos( th0 ) + yT*math.sin( th0 ) + x,
            -xT*math.sin( th0 ) + yT*math.cos( th0 ) + y, thT- initConfig[2]
        ]

        rConfigs.append( configT )
    
    initConfig[2] = th0
    return initConfig, rConfigs


def read_distance_angle(startAngle, endAngle):
    
    dThe = 1.0
    theta = startAngle
    d = laser.currentReadingPolar(theta, theta + dThe)
    dist = []
    angle = 0.0
    ang = []
    while theta < endAngle:
        d_temp = laser.currentReadingPolar(theta, theta + dThe)
        if d_temp < d:
            d = d_temp
            angle = theta + dThe/2.0
            dist.append(d)
            ang.append(angle)
        theta += dThe
        
    prom_d = sum(dist[0:3])/3.0
    r_angle = 0.0
    for i, d in enumerate(dist[0:-2]):
        prom_dist_temp = sum(dist[i:i+3])/3.0
        if prom_dist_temp < prom_d:
            prom_d = prom_dist_temp
            r_angle = ang[i+1]
    return r_angle, prom_d
        


# Lista de Configuraciones

def read_configurations():
    path_coordinates = open(PATH_COORDINATES, 'r')

    configurations = []

    for line in path_coordinates:
        line = line.split(" ")
        for i in range(len(line)):
            line[i] = line[i].strip('\n')
            line[i] = float(line[i])*100
        line.append(0)
        configurations.append(line)
    configurations.reverse()
    path_coordinates.close()
    return configurations
    
# MAIN
    
def main():
    
    print "Running..."
    robot.runAsync(True)
    width, hight, q0, qf, qL0, qLm, qLf, n_obstacles, obs_points = c_map.read_obstacle_from_file(OBSTACLE_FILE_PATH)
    C = read_configurations()
  
    nC = len( C )
    print "Total Configurations: {}".format( nC )

    print "Transforming into Robot Framework..."
    initConfig, rC = robotConfigs( C, [q0[0]*100, q0[1]*100, q0[2]*100])
    print initConfig
    processData( initConfig )

    tDelay = 20
    robot.enableMotors()
    robot.setTransAccel( 30 )
    
    print('Dirigiendose al punto de localizacion Qf')
    for i, config in enumerate(rC) :
        nextPose = ArPose( config[0], config[1])
    
        print "\tMoving to G-config: {}".format(C[i])
        print "\tMoving to R-config: {}".format( nextPose )
    
    
        nTh = robot.findAngleTo( nextPose )
        #print "\t\tAngle to {} deg".format( nTh )    
        robot.setHeading( nTh )
    
        while not robot.isHeadingDone( 0.0 ) : 
            ArUtil_sleep( tDelay )
            #processData( initConfig )
    
        ArUtil_sleep( 10 )

        nD = robot.findDistanceTo( nextPose )
        #print "\t\tDistance to {} mm".format( nD )
        robot.move( nD )
    
        while not robot.isMoveDone( 0.0 ) : 
            ArUtil_sleep( tDelay )
            #processData( initConfig )
    
        ArUtil_sleep( 10 )
    
        #print "\t\tSetting angle to {} deg".format( config[2] )
        #robot.setHeading( config[2] )
    
        while not robot.isHeadingDone( 0.0 ): 
            ArUtil_sleep( tDelay )
            #processData( initConfig )
    
        ArUtil_sleep( 100 )

    _, final_C = robotConfigs( [[qf[0]*100, qf[1]*100, qf[2]*100]], [q0[0]*100, q0[1]*100, q0[2]*100])
    robot.stop()
    ArUtil_sleep( 1000 )
    robot.setHeading(final_C[0][2])
    
    ArUtil_sleep( 1000 )
    while not robot.isHeadingDone( 0.0 ): 
        ArUtil_sleep( 10 )
      
    ArUtil_sleep( 1000 )
        
    print''
    print "... ... Localizando ... ..."
    print''
    a1, d1 = read_distance_angle(-105, -85)
    print'DISTANCIA OBSTACULO 1: ', d1, ' mm'#, '  Angulo OBST 1: ', a1)
    print'DISTANCIA ESPERADA: ', '900 mm'
    error_x = abs(d1 - 900.0)
    print'Error en x = ', error_x, ' mm'
    print ''
    a2, d2 = read_distance_angle(-20, 20)
    print'DISTANCIA OBSTaculo 2: ', d2, ' mm'#, '  Angulo OBST 2: ', a2)
    print'DISTANCIA ESPERADA: ', '300 mm'
    error_y = abs(d2 - 300)
    print'Error en y = ', error_y, ' mm'
    
    print(" ")
    
    print "\n Odometry Pose: ",  robot.getPose()
    print"Corrigiendo locailizacion ..."
    
    _, newC = robotConfigs( [[4200 + d1, d2, 0]], [q0[0]*100, q0[1]*100, q0[2]*100])
    new_pose = ArPose(newC[0][0], newC[0][1], robot.getTh())
    new_pose = ArPose(robot.getX() - error_y , robot.getY() - error_x , robot.getTh())
    robot.moveTo(new_pose)
    #append_map_coordinates()
    print "\tNew Odometry Pose: ",  robot.getPose()

    #stallRecover = ArActionStallRecover()
    #avoidSide = ArActionAvoidSide("avoid_side", 300, 5)
    #avoid_front = ArActionAvoidFront()
    #limit_forward = ArActionLimiterForwards("limitFron", 300, 1000, 15)
    #constantVel = ArActionConstantVelocity('ConstVel', 20)
    
    #robot.addAction(stallRecover, 100)
    #robot.addAction(avoidSide, 98)
    #robot.addAction(avoid_front, 95)
    #robot.addAction(limit_forward, 90)
    #robot.addAction(constantVel, 80)
    robot.setTransAccel( 10 )

    _, final_C2 = robotConfigs( [[qLm[0]*100, qLm[1]*100, qLm[2]*100]], [q0[0]*100, q0[1]*100, q0[2]*100])
    
    nextPose = ArPose( final_C2[0][0], final_C2[0][1])
    nTh = robot.findAngleTo( nextPose )
        #print "\t\tAngle to {} deg".format( nTh )    
    robot.setHeading( nTh )
    ArUtil_sleep( 4000 )
    nD = robot.findDistanceTo( nextPose )
        #print "\t\tDistance to {} mm".format( nD )
    robot.move( nD )
    while not robot.isMoveDone( 0.0 ) : 
            ArUtil_sleep( 50 )
    ArUtil_sleep( 3000 )
    
    _, final_C2 = robotConfigs( [[qLm[0]*100, qLm[1]*100, qLm[2]*100]], [q0[0]*100, q0[1]*100, q0[2]*100])
    
    nextPose = ArPose( final_C2[0][0], final_C2[0][1])
    nTh = robot.findAngleTo( nextPose )
        #print "\t\tAngle to {} deg".format( nTh )    
    robot.setHeading( nTh )
    ArUtil_sleep( 5000 )
    nD = robot.findDistanceTo( nextPose )
        #print "\t\tDistance to {} mm".format( nD )
    robot.move( nD )
    while not robot.isMoveDone( 0.0 ) : 
            ArUtil_sleep( 50 )
    ArUtil_sleep( 1000 )    
    print "\n New Odometry Pose: ",  robot.getPose()

    _, final_C3 = robotConfigs( [[qLf[0]*100, qLf[1]*100, qLf[2]*100]], [q0[0]*100, q0[1]*100, q0[2]*100])
    print "\n Moving to G-config: {}".format( [qLf[0]*100, qLf[1]*100, qLf[2]*100] )
    print "Moving to R-config: {}".format(final_C3[0])
    nextPose = ArPose( final_C3[0][0], final_C3[0][1])
    nTh = robot.findAngleTo( nextPose )
        #print "\t\tAngle to {} deg".format( nTh )    
    robot.setHeading( nTh )
    ArUtil_sleep( 3000 )
    nD = robot.findDistanceTo( nextPose )
        #print "\t\tDistance to {} mm".format( nD )
    robot.move( nD )
    while not robot.isMoveDone( 0.0 ) : 
            ArUtil_sleep( tDelay )
    ArUtil_sleep( 1000 )
    
    print "\n New Odometry Pose: ",  robot.getPose()

    print "Exiting."

    
    

    Aria_shutdown()

    fileRobot.close( )
    fileMap.close( )

if __name__ == '__main__':
    main()


