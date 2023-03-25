.. redirect-from::

    Tutorials/Services/Understanding-ROS2-Services

.. _ROS2Services:

Comprender los servicios
========================

**Objetivo:** Aprender sobre los servicios en ROS 2 utilizando herramientas de línea de comandos.

**Nivel del Tutorial:** Principiante

**Tiempo:** 10 minutos

.. contents:: Contenido
   :depth: 2
   :local:

Historial
---------

Los servicios son otro método de comunicación para nodos en el grafo ROS.
Los servicios se basan en un modelo de llamada y respuesta, a diferencia del modelo publicador-suscriptor de los topic.
Si bien los topic permiten que los nodos se suscriban al flujo de datos y obtengan actualizaciones continuas, los servicios solo proporcionan datos cuando un cliente les llama específicamente.

.. image:: images/Service-SingleServiceClient.gif

.. image:: images/Service-MultipleServiceClient.gif

Requisitos previos
------------------

Algunos conceptos son mencionados en este tutorial como :doc:`Nodes <../Understanding-ROS2-Nodes/Understanding-ROS2-Nodes>` y :doc:`Topics <../Understanding-ROS2-Topics/Understanding-ROS2-Topics>`, fueron cubiertos en tutoriales anteriores.

Requieres tener el :doc:`turtlesim package <../Introducing-Turtlesim/Introducing-Turtlesim>`.

Como siempre, no olvides hacer un source ROS 2 en :doc:`cada terminal nueva <../Configuring-ROS2-Environment>`.

Tareas
------

1 Configuración
^^^^^^^^^^^^^^^
Inicia los nodos, ``/turtlesim`` y ``/teleop_turtle``.

Abre una nueva terminal e introduce el siguiente comando:

.. code-block:: console

    ros2 run turtlesim turtlesim_node

Abre una nueva terminal e introduce el siguiente comando:

.. code-block:: console

    ros2 run turtlesim turtle_teleop_key

2 ros2 service list
^^^^^^^^^^^^^^^^^^^

Ejecutar el comando ``ros2 service list`` en un nuevo terminal devolverá una lista de todos los servicios actualmente activos en el sistema:

.. code-block:: console

  /clear
  /kill
  /reset
  /spawn
  /teleop_turtle/describe_parameters
  /teleop_turtle/get_parameter_types
  /teleop_turtle/get_parameters
  /teleop_turtle/list_parameters
  /teleop_turtle/set_parameters
  /teleop_turtle/set_parameters_atomically
  /turtle1/set_pen
  /turtle1/teleport_absolute
  /turtle1/teleport_relative
  /turtlesim/describe_parameters
  /turtlesim/get_parameter_types
  /turtlesim/get_parameters
  /turtlesim/list_parameters
  /turtlesim/set_parameters
  /turtlesim/set_parameters_atomically

Puedes observar que ambos nodos tienen los mismos seis servicios con ``parameters`` en sus nombres.
Casi todos los nodos en ROS 2 tienen estos servicios de infraestructura de los que se construyen los parámetros.
Aprenderás más sobre los parámetros en el próximo tutorial.
En este tutorial, los servicios de parámetros se omitirán de la discusión.

Por ahora nos centraremos en los servicios específicos de turtlesim, ``/clear``, ``/kill``, ``/reset``, ``/spawn``, ``/turtle1/set_pen``, ``/turtle1/teleport_absolute``, y ``/turtle1/teleport_relative``.
Si no recuerdas como interactuar con alguno de estos servicios utilizando rqt puedes dirigirte al tutorial :doc:`Utilizar tutrlesim y rqt <../Introducing-Turtlesim/Introducing-Turtlesim>`.



3 ros2 service type
^^^^^^^^^^^^^^^^^^^

Los servicios tienen tipos que describen cómo se estructuran los datos de la solicitud y respuesta de un servicio.
Estos tipos se definen de manera similar a los tipos de topics, excepto que los tipos de servicio tienen dos partes: un mensaje para la solicitud y otro para la respuesta.

Para averiguar el tipo de servicio, use el comando:

.. code-block:: console

  ros2 service type <service_name>

Prueba echar un vistazo al servicio ``/clear``.
Abre una nueva terminal e introduce el siguiente comando:

.. code-block:: console

  ros2 service type /clear

El terminal devolverá:

.. code-block:: console

  std_srvs/srv/Empty

El tipo ``Empty`` significa que la llamada al servicio no envía datos al hacer una solicitud y no recibe datos al recibir una respuesta.

3.1 ros2 service list -t
~~~~~~~~~~~~~~~~~~~~~~~~

Para ver los tipos de todos los servicios activos al mismo tiempo, puedes agregar la opción ``--show-types``, abreviada como ``-t``, al comando ``list``:

.. code-block:: console

  ros2 service list -t

El terminal devolverá:

.. code-block:: console

  /clear [std_srvs/srv/Empty]
  /kill [turtlesim/srv/Kill]
  /reset [std_srvs/srv/Empty]
  /spawn [turtlesim/srv/Spawn]
  ...
  /turtle1/set_pen [turtlesim/srv/SetPen]
  /turtle1/teleport_absolute [turtlesim/srv/TeleportAbsolute]
  /turtle1/teleport_relative [turtlesim/srv/TeleportRelative]
  ...

4 ros2 service find
^^^^^^^^^^^^^^^^^^^

Para ver todos los servicios de un tipo específico, puedes usar el comando:

.. code-block:: console

  ros2 service find <type_name>

Por ejemplo, puedes encontrar todos los tipos de sevicio ``Empty`` mediante el comando:

.. code-block:: console

  ros2 service find std_srvs/srv/Empty

El terminal devolverá:

.. code-block:: console

  /clear
  /reset

5 ros2 interface show
^^^^^^^^^^^^^^^^^^^^^

Los sevicios pueden ser llamados desde la línea de comandos, pero primero debes conocer la estructura de los argumentos de entrada.

.. code-block:: console

  ros2 interface show <type_name>

Para conocer la estructura del servicio de tipo ``Empty`` escribe en la terminal:

.. code-block:: console

  ros2 interface show std_srvs/srv/Empty

El terminal devolverá:

.. code-block:: console

  ---

El ``---`` separa la estructura de solicitud (arriba), de la estructura de respuesta (debajo).
Pero, como aprendiste recientemente, el tipo ``Empty`` no envía ni recibe datos.
Entonces, naturalmente, su estructura está en blanco.

Ahora prueba analizar un servicio que envíe y reciba datos, como ``/spawn``.
De los resultados de la lista de servicios ``ros2 service list -t``, sabemos que el sevicio ``/spawn`` tiene un tipo: ``turtlesim/srv/Spawn``.

Para ver la estructura del servicio ``/spawn``, ejecuta el comando:

.. code-block:: console

  ros2 interface show turtlesim/srv/Spawn

El terminal devolverá:

.. code-block:: console

  float32 x
  float32 y
  float32 theta
  string name # Opcional. Se creará y devolverá un nombre único si está vacío
  ---
  string name

La información por encima de la línea ``---`` indica los argumentos necesarios para llamar ``/spawn``.
``x``, ``y`` y ``theta`` determinan la ubicación inicial de la tortuga, y ``name`` es opcional.

La información debajo de la línea no es algo que necesitas saber en este caso, pero puede ayudarte a comprender el tipo de datos de la respuesta que obtiene de la llamada.

6 ros2 service call
^^^^^^^^^^^^^^^^^^^

Now that you know what a service type is, how to find a service’s type, and how to find the structure of that type’s arguments, you can call a service using:
Ahora que sabes qué es el tipo de un servicio, cómo encontrar el tipo de un servicio y cómo encontrar la estructura de ese tipo de argumentos, puedes llamar a un servicio usando:

.. code-block:: console

  ros2 service call <service_name> <service_type> <arguments>

La parte ``<arguments>`` es opcional.
Por ejemplo, de la sección anterior sabes que los servicios ``Empty`` no tienen ningún argumento:

.. code-block:: console

  ros2 service call /clear std_srvs/srv/Empty

Este comando limpiará la ventana turtlesim de cualquier línea que la tortuga haya dibujado.

.. image:: images/clear.png

Ahora crea una nueva tortuga llamando al servicio ``/spawn`` e ingresando argumentos.
La parte ``<arguments>`` en una llamada de servicio desde la línea de comandos debe estar en la formato YAML.

Ingresa el comando:

.. code-block:: console

  ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"

Obtendrás un mensaje en la terminal con la información de la solicitud, y luego la respuesta del servicio:

.. code-block:: console

  requester: making request: turtlesim.srv.Spawn_Request(x=2.0, y=2.0, theta=0.2, name='')

  response:
  turtlesim.srv.Spawn_Response(name='turtle2')

Tu ventana turtlesim se actualizará de inmediato con la tortuga recién generada:

.. image:: images/spawn.png

Summary
-------

Los nodos pueden comunicarse usando servicios en ROS 2.
A diferencia de un topic, un patrón de comunicación unidireccional donde un nodo publica información que puede consumir uno o más suscriptores, un servicio es un patrón de solicitud/respuesta en el que un cliente realiza una solicitud a un nodo que proporciona el servicio, el servicio procesa la solicitud de la solicitud y genera una respuesta.

Generalmente no deseas utilizar un servicio para llamadas continuas. Los temas o incluso las acciones serían más adecuadas.

En este tutorial, utilizaste las herramientas de línea de comandos para identificar, elaborar y llamar a los servicios.

Next steps
----------

En el siguiente tutorial, :doc:`../Understanding-ROS2-Parameters/Understanding-ROS2-Parameters`, aprenderás acerca de los parámetros, un valor de configuración de un nodo.

Related content
---------------

Puedes mirar `este tutorial <https://discourse.ubuntu.com/t/call-services-in-ros-2/15261>`_; es una excelente aplicación realista de los servicios ROS que utilizan un brazo robot Robotis.
