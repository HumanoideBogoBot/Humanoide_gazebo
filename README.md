# Humanoide_gazebo
Simulacion del humanoide en Gazebo

- Launch para iniciar el gazebo del humanoide

```
roslaunch Humanoide_gazebo world.launch
```

- Si deseas hacer que haga un movimiento simple corre 
```
rosrun humanoide_mov Mov_joint.py
```

- En el archivo 
```
humanoide_mov/src/Mov_joint.py
```
- Esta la forma de como mover el robot si deseas hacer otra trayectoria basate en ese codigo

- Instala el siguiente paquete para poder controlar el robot en gazebo sino va a aparecer tirado:
```
sudo apt install ros-melodic-joint-trajectory-controller
```