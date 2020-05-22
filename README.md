# Clonar y compilar
```sh
git clone https://github.com/JorgeAmil/gazebo.git
catkin_make
```

# Packages
get_camera
```sh
Lee los mensajes del tópico /camera/depth/points y los publica 
de nuevo casteados al tipo sensor_msgs::PointCloud2
```
get_pointclouds
```sh
Sincroniza los mensajes del mando y los publicados por get_camera, 
en caso de estar pulsado el R2 la nube de puntos se guarda en un fichero 
```
registro
```sh
A partir de las nubes almacenadas en el formato .pcd registra un modelo 3D
```

# Ejecutar
Para generar los .pcd
```sh
roslaunch load_model init.launch
rosrun joy joy_node
rosrun mando mando
rosrun get_camera get_camera_node
rosrun get_pointclouds get_pointclouds_node get_cloud
```
Para registrar un objeto 3D a partir de nubes en archivos pcd
```sh
rosrun registro registro_node
```