.. redirect-from::

    Tutorials/Understanding-ROS2-Actions

.. _ROS2Actions:

Comprender las acciones
=======================

**Objetivo:** Examinar las acciones en ROS 2.

**Nivel del Tutorial:** Principiante

**Tiempo:** 15 minutos

.. contents:: Contents
   :depth: 2
   :local:

Historial
---------

Las acciones son uno de los tipos de comunicación en ROS 2 y están destinadas a tareas de larga duración.
Consisten en tres partes: un objetivo, retroalimentación y un resultado.

Las acciones se basan en topics y servicios.
Su funcionalidad es similar a los servicios, excepto que las acciones pueden cancelarse.
También proporcionan comentarios constantes, a diferencia de los servicios que devuelven una sola respuesta.

Las Acciones utilizan un modelo cliente-servidor, similar al modelo publicador-suscriptor (descrito en :doc:`topics tutorial <../Understanding-ROS2-Topics/Understanding-ROS2-Topics>`).
Un nodo de "Cliente de Acción" envía un objetivo a un nodo "Servidor de acción" que reconoce el objetivo y devuelve comentarios de realimentación y un resultado.

.. image:: images/Action-SingleActionClient.gif

Requisitos previos
------------------

Este tutorial desarrolla conceptos como :doc:`nodos <../Understanding-ROS2-Nodes/Understanding-ROS2-Nodes>` y  :doc:`topics <../Understanding-ROS2-Topics/Understanding-ROS2-Topics>`, tratados en tutoriales previos.

Este tutorial utiliza :doc:`el paquete turtlesim <../Introducing-Turtlesim/Introducing-Turtlesim>`.

Como siempre, no olvides hacer un source ROS 2 en :doc:`cada terminal nueva<../Configuring-ROS2-Environment>`.

Tareas
------

1 Configuración
^^^^^^^^^^^^^^^

Inicia dos nodos de turtlesim, ``/turtlesim`` y ``/teleop_turtle``.

Abre una terminal e inicia el nodo con el comando:

.. code-block:: console

    ros2 run turtlesim turtlesim_node

Abre otra terminal nueva e inicia el segundo nodo con el comando:

.. code-block:: console

    ros2 run turtlesim turtle_teleop_key


2 Utilizar acciones
^^^^^^^^^^^^^^^^^^^

Cuando ejecutes el nodo ``/teleop_turtle``, ver+as el siguiente mensaje en tu terminal:

.. code-block:: console

    Use arrow keys to move the turtle.
    Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.

Centrémonos en la segunda línea, que corresponde a una acción.
(La primera instrucción corresponde al tema "cmd_vel", discutido anteriormente en :doc:`topics tutorial <../Understanding-ROS2-Topics/Understanding-ROS2-Topics>`.)

Observe que las teclas de la letra ``G|B|V|C|D|E|R|T`` forman una "caja" alrededor de la tecla ``F`` en un teclado US Qwerty (Si no estás utilizando un teclado Qwerty, puedes ver `this link <https://upload.wikimedia.org/wikipedia/commons/d/da/KB_United_States.svg>`__).
La posición de cada llave alrededor de ``F`` corresponde a esa orientación en Turtlesim.
Por ejemplo, el ``E`` girará la orientación de la tortuga a la esquina superior izquierda.

Presta atención al terminal donde se está ejecutando el nodo ``/turtlesim``.
Cada vez que presiones una de estas teclas, estás enviando un objetivo a un servidor de acción que forma parte del nodo ``/turtlesim``.
El objetivo es rotar la tortuga hacia una dirección particular.
Una vez que la tortuga complete su rotación, aparecerá un mensaje con el resultado del objetivo:

.. code-block:: console

    [INFO] [turtlesim]: Rotation goal completed successfully

La tecla ``F`` cancela la acción en medio de la ejecución.

Intenta presionar la tecla ``C`` y luego presiona la tecla ``F`` antes de que la tortuga pueda completar su rotación.
En el terminal donde se está ejecutando el nodo ``/turtlesim``, verás el mensaje:

.. code-block:: console

  [INFO] [turtlesim]: Rotation goal canceled

Tanto el lado del cliente (la entrada en telop), como el lado del servidor (el nodo ``/turtlesim``) pueden detener un objetivo.
Cuando el lado del servidor elige dejar de procesar un objetivo, se dice que "aborta" el objetivo.

Intenta presionar la tecla ``D``, luego la tecla ``G`` antes de que pueda completarse la primera rotación.
En el terminal donde se está ejecutando el nodo ``/turtlesim``, podrás ver el mensaje:

.. code-block:: console

  [WARN] [turtlesim]: Rotation goal received before a previous goal finished. Aborting previous goal

Como se puede ver en el mensaje anterior, el servidor de acción eligió abortar el primer objetivo porque obtuvo uno nuevo.
Podría haber elegido algo más, como rechazar el nuevo objetivo o ejecutar el segundo gol después de que el primero terminó.
No asumas que cada servidor de acción elegirá abortar el objetivo actual cuando obtenga uno nuevo.

3 ros2 node info
^^^^^^^^^^^^^^^^

Para ver las acciones disponibles en el nodo ``/turtlesim``, abre un nueva terminal y ejecuta el comando:

.. code-block:: console

    ros2 node info /turtlesim

Que devolverá una lista de suscriptores, publicadores, servicios, servidores de acción y clientes de acción del nodo ``/turtlesim``:

.. code-block:: console

  /turtlesim
    Subscribers:
      /parameter_events: rcl_interfaces/msg/ParameterEvent
      /turtle1/cmd_vel: geometry_msgs/msg/Twist
    Publishers:
      /parameter_events: rcl_interfaces/msg/ParameterEvent
      /rosout: rcl_interfaces/msg/Log
      /turtle1/color_sensor: turtlesim/msg/Color
      /turtle1/pose: turtlesim/msg/Pose
    Service Servers:
      /clear: std_srvs/srv/Empty
      /kill: turtlesim/srv/Kill
      /reset: std_srvs/srv/Empty
      /spawn: turtlesim/srv/Spawn
      /turtle1/set_pen: turtlesim/srv/SetPen
      /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
      /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
      /turtlesim/describe_parameters: rcl_interfaces/srv/DescribeParameters
      /turtlesim/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
      /turtlesim/get_parameters: rcl_interfaces/srv/GetParameters
      /turtlesim/list_parameters: rcl_interfaces/srv/ListParameters
      /turtlesim/set_parameters: rcl_interfaces/srv/SetParameters
      /turtlesim/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    Service Clients:

    Action Servers:
      /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
    Action Clients:

Puedes notar que la acción ``/turtle1/rotate_absolute`` para el nodo ``/turtlesim`` está en la categoría ``Action Servers``.
Esto significa que el nodo ``/turtlesim`` responde y provee retroalimentación  para la acción ``/turtle1/rotate_absolute``.

Si ejecutas el comando acontinuación, podás observar que el nodo ``/teleop_turtle`` tiene el nombre ``/turtle1/rotate_absolute`` en la categoría ``Action Clients``, lo que significa que envía objetivos para esa acción.

.. code-block:: console

    ros2 node info /teleop_turtle

La salida debería verse así:

.. code-block:: console

  /teleop_turtle
    Subscribers:
      /parameter_events: rcl_interfaces/msg/ParameterEvent
    Publishers:
      /parameter_events: rcl_interfaces/msg/ParameterEvent
      /rosout: rcl_interfaces/msg/Log
      /turtle1/cmd_vel: geometry_msgs/msg/Twist
    Service Servers:
      /teleop_turtle/describe_parameters: rcl_interfaces/srv/DescribeParameters
      /teleop_turtle/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
      /teleop_turtle/get_parameters: rcl_interfaces/srv/GetParameters
      /teleop_turtle/list_parameters: rcl_interfaces/srv/ListParameters
      /teleop_turtle/set_parameters: rcl_interfaces/srv/SetParameters
      /teleop_turtle/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    Service Clients:

    Action Servers:

    Action Clients:
      /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute

4 ros2 action list
^^^^^^^^^^^^^^^^^^

Para identificar todas las acciones en el grafo de ROS, ejecuta el comando:

.. code-block:: console

    ros2 action list

La salida debería verse así:

.. code-block:: console

    /turtle1/rotate_absolute

Esta es la única acción en el grafo de ROS en este momento.
Controla la rotación de la tortuga, como viste anteriormente.
También sabes que hay un cliente de acción (proveniente del nodo ``/teleop_turtle``) y un servidor  de acción (proveniente del nodo ``/turtlesim``) para esta acción, lo cual puedes verificar con el comando ``ros2 node info <node_name>``.

4.1 ros2 action list -t
~~~~~~~~~~~~~~~~~~~~~~~

Las acciones tienen tipos, similares a los topics y servicios.
Para obtener el tipo de ``/turtle1/rotate_absolute``, ejecuta el siguiente comando:

.. code-block:: console

    ros2 action list -t

La salida debería verse así:

.. code-block:: console

    /turtle1/rotate_absolute [turtlesim/action/RotateAbsolute]

En los paréntesis a la derecha de cada nombre de acción (en este caso solo ``/turtle1/rotate_absolute``) se encuentra el tipo de la acción, ``turtlesim/action/RotateAbsolute``.
Necesitarás esto cuando desees ejecutar una acción desde la línea de comando o desde el código.

5 ros2 action info
^^^^^^^^^^^^^^^^^^

Puedes inspeccionar aún más la acción ``/Turtle1/Rotate_absolute`` con el comando:

.. code-block:: console

    ros2 action info /turtle1/rotate_absolute

La salida debería verse así:

.. code-block:: console

  Action: /turtle1/rotate_absolute
  Action clients: 1
      /teleop_turtle
  Action servers: 1
      /turtlesim

Esto indica lo que aprendiste de ejecutar ``info de nodo ROS2`` en cada nodo:
El nodo ``/teleop_turtle`` tiene un cliente de acción y el nodo ``/turtlesim`` tiene un servidor de acción para la acción ``/turtle1/rotate_absolute``.


6 ros2 interface show
^^^^^^^^^^^^^^^^^^^^^

Una información más que necesitarás antes de enviar o ejecutar un objetivo de acción es la estructura del tipo de acción.

Recuerda que identificaste el tipo de ``/turtle1/totate_absolute`` al ejecutar el comando ``ros2 action list -t``.
Ingresa el siguiente comando con el tipo de acción en tu terminal:

.. code-block:: console

  ros2 interface show turtlesim/action/RotateAbsolute

La salida debería verse así:

.. code-block:: console

  # The desired heading in radians
  float32 theta
  ---
  # The angular displacement in radians to the starting position
  float32 delta
  ---
  # The remaining rotation in radians
  float32 remaining

La primera sección de este mensaje, por encima del ``--- ``, es la estructura (tipo de datos y nombre) de la solicitud del objetivo.
La siguiente sección es la estructura del resultado.
La última sección es la estructura de la retroalimentación.


7 ros2 action send_goal
^^^^^^^^^^^^^^^^^^^^^^^

Ahora intenta enviar un objetivo de acción desde la línea de comando con la siguiente sintaxis:

.. code-block:: console

    ros2 action send_goal <action_name> <action_type> <values>

``<values>`` tiene que estar en formato YAML.

Mantente atento a la ventana de turtlesim e ingresa el siguiente comando en tu terminal:

.. code-block:: console

    ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"

Deberías ver la tortuga girando, así como el siguiente mensaje en tu terminal:

.. code-block:: console

  Waiting for an action server to become available...
  Sending goal:
     theta: 1.57

  Goal accepted with ID: f8db8f44410849eaa93d3feb747dd444

  Result:
    delta: -1.568000316619873

  Goal finished with status: SUCCEEDED


Todos los objetivos tienen una identificación única, que se muestra en el mensaje de retorno.
También puedes ver el resultado, un campo con el nombre ``delta``, que es el desplazamiento desde la posición inicial.

Para ver el mensaje de retroalimentación de este objetivo, agregua ``--feedback`` al comando ``ros2 action send_goal``:

.. code-block:: console

    ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.57}" --feedback

En tu terminal deberías ver un mensaje como el siguiente:

.. code-block:: console

  Sending goal:
     theta: -1.57

  Goal accepted with ID: e6092c831f994afda92f0086f220da27

  Feedback:
    remaining: -3.1268222332000732

  Feedback:
    remaining: -3.1108222007751465

  …

  Result:
    delta: 3.1200008392333984

  Goal finished with status: SUCCEEDED

Continuarás recibiendo mensajes de retroalimentación, los radianos restantes, hasta que se complete el objetivo.

Resumen
-------

Las acciones son como servicios que te permiten ejecutar tareas de larga duración, proporcionar mensajes de retroalimentación y son cancelables.

Un sistema de robots probablemente usaría acciones para la navegación.
El objetivo de una acción podría decirle a un robot que viaje a una posición.
Mientras el robot navega a la posición, este puede enviar actualizaciones en el camino (es decir, retroalimentación), y luego un mensaje de resultado final una vez que llegue a su destino.

El nodo turtlesim tiene un servidor de acción al que los clientes de acción pueden enviar objetivos para girar las tortugas.
En este tutorial, introspectaste esa acción, ``/turtle1/rotate_absolute``, para tener una mejor idea de qué son las acciones y cómo funcionan.

Pasos siguientes
----------------

Con esto cubriste todos los conceptos fundamentales de ROS 2.
Los últimos tutoriales en el conjunto "Usuarios" te presentarán algunas herramientas y técnicas que facilitarán el uso de ROS 2, comenzando con :doc:`../Using-Rqt-Console/Using-Rqt-Console`.

Contenido Relacionado
---------------------

Puedes leer más sobre las decisiones de diseño detrás de las acciones en ROS 2 `aquí <https://design.ros2.org/articles/actions.html>`__.
