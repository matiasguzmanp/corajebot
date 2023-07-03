# famous_project
Proyecto

Una vez instalado, correr el siguiente comando

    $ rosdep install --from-paths src --ignore-src -r -y

Dentro de la carpeta src del workspace, ejecutar los siguientes comandos.

    $ git clone https://github.com/koide3/gazebo_apriltag
    $ cp -R src/gazebo_apriltag/models/* src/corajebot/corajebot_gazebo/models/

Si la carpeta ~/.gazebo/models/ no existe, crearla con mkdir.
