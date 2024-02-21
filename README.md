# 2024-P5-ActionForwardTurn

En esta práctica debes crear 2 paquetes:

1. En un paquete debes definir los interfaces necesarios.
2. En otro paquete debes crear tres nodos, en tres ejecutables distintos:
    * **Nodo servidor**: Este nodo implementa una acción por el que se le envían comandos de avance o de giro (uno u otro, no ambos), indicando la distancia a recorrer o los radianes a girar. El nodo está latente hasta que recibe la petición. Cuando ha acabado, se para y espera una nueva petición. Si recibe una petición mientras está ejecutando otro, el nuevo comando se empieza a ejecutar inmediatamente, expulsando el anterior. El robot notifica:
        *  feedback frecuente (>3 Hz) con la distancia avanzada o el giro realizado, y lo que le queda para completar la petición.
        *  resultado con la distancia avanzada o el giro realizado y el tiempo que le llevó realizarlo.
    * **Nodo cliente de Avance**: Este nodo se encarga de recibir como argumento la distancia a recorrer, y hace la petición al servidor, mostrando por la consola el feedback y el resultado de la petición. Cuando ha finalizado la acción, la ejecución finaliza.
    * **Nodo cliente de Giro**: Este nodo se encarga de recibir como argumento los radiantes a girar, y hace la petición al servidor, mostrando por la consola el feedback y el resultado de la petición. Cuando ha finalizado la acción, la ejecución finaliza.
