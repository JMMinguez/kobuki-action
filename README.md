[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-24ddc0f5d75046c5622901739e7c5dd533143b0c8e959d652212380cedb1ea36.svg)](https://classroom.github.com/a/upfI9pif)
# 2024-P5-ActionForwardTurn

## Introduccion
El objetivo de la practica es conseguir que el [Kobuki](https://robots.ros.org/kobuki/) avanza la distancia exacta que manda el cliente al servidor, para ello haré uso de TFs y comunicaciones entre servidor y cliente mediante acciones.

## Creación de paquetes
Esta práctica hace uso de 2 paquetes:
- En el paquete [action_forward_turn_interfaces](https://github.com/Docencia-fmrico/p4-actionforwardturn-jmartinm2021/tree/main/action_forward_turn_interfaces) están definidos los servicios, mensajes y acciones.
- El paquete [action_forward_turn](https://github.com/Docencia-fmrico/p4-actionforwardturn-jmartinm2021/tree/main/action_forward_turn_interfaces) contiene los nodos de los clientes y servidor.

## Descripcion y procedimiento
En esta práctica, he utilizado como base el paquete [**ASR_2024**](https://github.com/Docencia-fmrico/ASR_2024) proporcionado por [fmrico](https://github.com/fmrico). 

En cuanto el servidor recibe el mensaje del cliente con una distancia, el servidor toma la posicion del robot.
```cpp
if (tf_buffer_.canTransform("odom", "base_footprint", tf2::TimePointZero, &error)) {
   auto odom2bf_msg = tf_buffer_.lookupTransform(
      "odom", "base_footprint", tf2::TimePointZero);
    tf2::fromMsg(odom2bf_msg, odom2bf_);
}
```

Posteriormente el server va comprobando la posicion del robot en ese mmoemnto y saca la transformada entre la posicion del robot y la posicion inicial para sacar la distancia
```cpp
tf2::Transform bf2bfa = odom2bf_inverse * odom2bfa;

//  Extracts the x and y coordinates from the obtained transformation.
double x = bf2bfa.getOrigin().x();
double y = bf2bfa.getOrigin().y();

//  Calculate the distance between (0,0) and (x,y)
actual_distance_ = sqrt(x * x + y * y);
```

Dependiendo de la distancia y del cliente que mande la solicitud (forward manda un 0 y turn un 1), el robot se movera o girara la disntacia pedida
```cpp
if (goal_handle_->get_goal()->command == 0) {
   m_vel_.linear.x = 0.3;
   m_vel_.angular.z = 0;

} else if (goal_handle_->get_goal()->command == 1) {
   m_vel_.linear.x = 0;
   m_vel_.angular.z = 0.3;
}
vel_->publish(m_vel_);
```

Si mientras se esta ejecutando un comando el otro cliente hace una solicitud, automaticamente el servidor cancela al anterior cliente y procesa la solicitud nueva:
```shell
[ERROR] [1709039333.494944182] [action_forward_turn_action_client]: Goal was canceled
```
  
## Ejecución
Tambien se puede ejecutar escribiendo en una terminal el server
```shell
ros2 run action_server_main
```

Y en otra terminal el cliente con la distancia deseada (action_client_forward_main o action_client_turn_main dependiendo si deseamos que gire o avance)
```shell
ros2 run action_client_forward_main '[distancia]'
```
## Video demostración
  [Grabación de pantalla desde 27-02-24 14:15:39.webm](https://github.com/Docencia-fmrico/p4-actionforwardturn-jmartinm2021/assets/92941332/1a4d8456-a222-418d-871f-7743a65d9fb2)

## Enunciado
En esta práctica debes crear 2 paquetes:

1. En un paquete debes definir los interfaces necesarios.
2. En otro paquete debes crear tres nodos, en tres ejecutables distintos:
    * **Nodo servidor**: Este nodo implementa una acción por el que se le envían comandos de avance o de giro (uno u otro, no ambos), indicando la distancia a recorrer o los radianes a girar. El nodo está latente hasta que recibe la petición. Cuando ha acabado, se para y espera una nueva petición. Si recibe una petición mientras está ejecutando otro, el nuevo comando se empieza a ejecutar inmediatamente, expulsando el anterior. El robot notifica:
        *  feedback frecuente (>3 Hz) con la distancia avanzada o el giro realizado, y lo que le queda para completar la petición.
        *  resultado con la distancia avanzada o el giro realizado y el tiempo que le llevó realizarlo.
    * **Nodo cliente de Avance**: Este nodo se encarga de recibir como argumento la distancia a recorrer, y hace la petición al servidor, mostrando por la consola el feedback y el resultado de la petición. Cuando ha finalizado la acción, la ejecución finaliza.
    * **Nodo cliente de Giro**: Este nodo se encarga de recibir como argumento los radiantes a girar, y hace la petición al servidor, mostrando por la consola el feedback y el resultado de la petición. Cuando ha finalizado la acción, la ejecución finaliza.
