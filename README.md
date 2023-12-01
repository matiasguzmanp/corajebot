# Coraje, el robot cobarde
Este proyecto busca resolver el problema del "robot famoso", donde un robot debe huir de un robot paparazzi. Éste último se asume estático y se simplifica a un Apriltag para conocer su posición rápidamente.

# Setup

Este robot funciona con ROS Noetic

Una vez instalado copiado el repositorio en un *workspace*, correr el siguiente comando

    $ rosdep install --from-paths src --ignore-src -r -y

Dentro de la carpeta src del workspace, ejecutar los siguientes comandos.

    $ git clone https://github.com/koide3/gazebo_apriltag
    $ cp -R src/gazebo_apriltag/models/* src/corajebot/corajebot_gazebo/models/

Si la carpeta ~/.gazebo/models/ no existe, crearla con mkdir.
