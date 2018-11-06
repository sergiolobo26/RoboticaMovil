** Solución para el Proyecto 2 **
** Planificación de Caminos Geométricos, Ejecución (Modo Autónomo) y Localización **

Autores: Sergio David Lobo Bolaño
	 Gilberto Díaz García

Oct, 2018

***********************************************
Liberías Necesarias
***********************************************
Para poder ejecutar la solución propuesta se necesitan las siguientes librerías. 

- Python Version 2.7 32-bit
- ARIA Version 2.9.4 32-bit
- MobileSim Version 0.7.5
- Mapper3 2.2.5. Esta última sólo es necesaria para visualizar los datos más fácilmente.

***********************************************
Funcionamiento
***********************************************

DESCRIPCIÓN DEL CONTENIDO DE LA CARPETA: *****************************************************************

- datos-pruebas: Esta carpeta contiene los mapas y los archivos con las coordenadas de los obstáculos.
- MapasGenerados: Esta carpeta guarda los mapas generados por el robot al tilizar el láser. 
- a_star2.py: Este script de python se encarga de generar las trayectorias de planificación.
- create_map_test.py: Este script de python lee las coordenadas de los obstáculos y genera los mapas en formas de listas para su uso en a_star2.py
- proyecto2.py: Este script contiene las instrucciones para mover el robot en planificación y localización.
- path_coordinates.txt: Este archivo contiene las coordenadas en sistema Escena de la trayectoria resultado de a_star2.py

INSTRUCCIONES DE EJECUCIÓN: ******************************************************************************

Para ejecutar el proyecto de forma adecuada, se recomienda python 2.7 e instalar los requerimientos en el archivo requierements.txt.
La ejecución consta de 2 partes: 
	1. Generación de trayectoria
	2. Ejecución de trayectoria + localización


******* 1. Generación de trayectoria:
Para generar la trayectoria, lo primero que se debe hacer es ubicar en la carpeta 'datos_prueba' el archivo .map y .txt con el mapa a utilizar.
Luego, se debe entrar a a_star2.py y modificar la variable MAP_FILE en la función main() (línea 216) con el path del archivo .txt del mapa. 
Después de esto, se ejecuta el script. Desde la terminal, puede hacerse así:
	python a_star2.py 
esto automáticamente generará o actualizará el archivo path_coordinates.txt.

******** 2. Ejecución de trayectoria + localización

Una vez generado el archivo de trayectoria, solo es necesario eecutar el script proyecto2.py y conectarlo al robot desde la terminal:

	python proyecto2.py -remoteHost <IP-Robot> -connectLaser -laserPort <IP-Robot> laserIncrementByDegrees 1.0 -laserPortType tcp -laserType urg 

O, de forma equivalente, 
	
	python proyecto2.py -rh <IP-Robot> -cl -lp <IP-Robot> -libd 1.0 -lpt tcp -lt urg 

IP-Robot: 157.253.173.241

Si, por el contrario, solamente desea simular un robot ejecute MobileSim y cargue la escena que desea simular. Luego ejecute en su consola, 
	
	python proyecto2.py -remoteIsSim -connectLaser -laserIncrementByDegrees 1.0 laserPortType tcp -laserType urg 

O, de forma equivalente, 

	python Proyecto1_Final.py -ris -cl -libd 1.0 -lpt tcp -lt urg

En consola se imprimirán los resultados de localización del robot. 

**********************************************************************************************************