(define (domain cooperation)

    (:requirements 
        :strips
        :typing
        :equality 
        :fluents
        :durative-actions
        :preferences
        :universal-preconditions
    )

    (:types
        ugv
        uav
        navmode
        pan
        tilt
        point
    )

    (:predicates
        ; robot está en la posición
        (in ?r - (either ugv uav) ?po - point) 

        ; realizada fotografía en una posición con una orientación
        (have-image ?r - (either ugv uav) ?po - point ?p - pan ?t - tilt)

        ; orientación pan de la camara en el robot
        (camera-pan-orientation ?r - (either ugv uav) ?p - pan)

        ; orientación tilt de la camara en el robot
        (camera-tilt-orientation ?r - (either ugv uav) ?t - tilt)

        ; robot acoplado a la estación base
        (coupled ?r - (either ugv uav))

        ; robot no acoplado a la estación base
        (disengaged ?r - (either ugv uav))

        ; modo de navegación
        (navigation-mode ?r - (either ugv uav) ?m - navmode)

        ; localización de la estación base de cada robot
        (base-station ?r - (either ugv uav) ?po - point)

        ; imagen transmitida desde un robot
        ; (transmitted-image  ?po - point ?p - pan ?t - tilt)
        (transmitted-image ?r - (either ugv uav) ?po - point ?p - pan ?t - tilt)

        ; ángulos adyacentes
        (adjacent ?a1 ?a2 - (either pan tilt))
    )

    (:functions
        ; valor de la batería del robot
        (battery ?r - (either ugv uav))

        ; batería baja
        ; (low-battery ?r)

        ; valor de la bateria maximo
        (battery-capacity ?r - (either ugv uav))

        ; ratio de descarga en función de la velocidad
        (discharge-ratio ?m - navmode)

        ; distancia entre posiciones
        ; implícito que estan conectadas
        (distance ?po1 ?po2 - point)

        ; velocidad con el modo seleccionado
        (speed ?m - navmode)

        ; valor numérico del punto
        ; (value ?po - point)


        ; ------ consumos ------

        ; consumo de cambio de modo
        (mode-change-consumption)

        ; consumo de acople
        (dock-consumption)

        ; consumo de desacople
        (undock-consumption)

        ; consumo tomar imagen
        (take-image-consumption)

        ; consumo cambio de orientación pan cámara
        (change-pan-orientation-consumption)

        ; consumo cambio de orientación tilt cámara
        (change-tilt-orientation-consumption)


        ; ------ duraciones ------

        ; duración transmisión de una imagen
        (transmit-duration)

        ; duración acople
        (dock-duration)

        ; duración desacople
        (undock-duration)

        ; duración del cambio de modo
        (change-mode-duration)


        ; ------ valores totales ------

        ; consumo transmisión una imagen
        (transmit-image-consumption)

        ; energía total consumida
        (total-energy-consumed)

        ; distancia total recorrida
        (total-distance-traveled)

        ; duración total
        (total-duration)
    )

    ; Desplazar el robot desde una localización a otra con una velocidad (rápido/lento)
    (:durative-action go
        :parameters (?r - (either ugv uav) ?po1 ?po2 - point ?m - navmode)
        :duration (
            ; duración en función de la distancia y la velocidad t=d/v
            = ?duration (/ (distance ?po1 ?po2) (speed ?m))
        )
        :condition (and 
            ; consumo bateria = distacia ente posiciones * factor de velocidad (rápido/lento)
            (at start (>= (battery ?r) (* (distance ?po1 ?po2) (discharge-ratio ?m))))

            ; robot está en la posición inicial
            (at start (in ?r ?po1))

            ; las posiciones son diferentes y esten conectadas, forzado predicado distancia se cumple

            ; el robot este en el modo seleccionado
            (over all (navigation-mode ?r ?m))

            ; robot desacoplado
            (over all (disengaged ?r))
        )
        :effect (and
            ; robot está en la posición final
            (at end (in ?r ?po2))

            ; robot no está en la posición inicial
            (at start (not (in ?r ?po1)))

            ; descarga de la batería en el cálculo
            (at end (decrease (battery ?r) (* (distance ?po1 ?po2) (discharge-ratio ?m))))

            ; incrementar energía total
            (at end (increase (total-energy-consumed) (* (distance ?po1 ?po2) (discharge-ratio ?m))))

            ; contabilizar distancia recorrida
            (at end (increase (total-distance-traveled) (distance ?po1 ?po2)))

            ; contabilizar la duración total
            (at end (increase (total-duration) (/ (distance ?po1 ?po2) (speed ?m))))
        )
    )

    ; Cambiar modo de un robot
    (:durative-action change-mode   ; modo antiguo y nuevo
        :parameters (?r - (either ugv uav) ?m1 ?m2 - navmode)
        :duration (= ?duration (change-mode-duration)) ;cte
        :condition (and
            ; modos no sean iguales
            (over all (not (= ?m1 ?m2)))

            ; modo de navegación anterior
            (at start (navigation-mode ?r ?m1))

            ; haya batería suficiente
            (at start (>= (battery ?r) (mode-change-consumption)))
        )
        :effect (and
            ; eliminar el modo de navegación anterior
            (at start (not (navigation-mode ?r ?m1)))

            ; modo de navegación nuevo
            (at end (navigation-mode ?r ?m2))

            ; consumir bateria
            (at end (decrease (battery ?r) (mode-change-consumption)))

            ; incrementar energía total
            (at end (increase (total-energy-consumed) (mode-change-consumption)))

            ; contabilizar la duración total
            (at end (increase (total-duration) (change-mode-duration)))
        )
    )

    ; Acoplarse a la estación base
    (:durative-action dock
        :parameters (?r - (either ugv uav) ?po - point)
        :duration (= ?duration (dock-duration)) ;cte
        :condition (and
            ; robot este en las coordenadas
            (over all (in ?r ?po))

            ; robot está en la posición de la estación base
            (over all (base-station ?r ?po))

            ; robot desacoplado
            (at start (disengaged ?r))

            ; haya batería suficiente
            (at start (>= (battery ?r) (dock-consumption)))
        )
        :effect (and
            ; incrementar la carga hasta su máxima capacidad
            (at end (assign (battery ?r) (battery-capacity ?r)))
                
            ; robot acoplado
            (at end (coupled ?r))

            ; robot no desacoplado
            (at start (not (disengaged ?r)))

            ; incrementar energía total
            (at end (increase (total-energy-consumed) (dock-consumption)))

            ; contabilizar la duración total
            (at end (increase (total-duration) (dock-duration)))
        )
    )

    ; Desacoplarse a la estación base
    (:durative-action undock
        :parameters (?r - (either ugv uav) ?po - point)
        :duration (= ?duration (undock-duration)) ;cte
        :condition (and

            ; robot este en las coordenadas
            (over all (in ?r ?po))

            ; robot está en la posición de la estación base
            (over all (base-station ?r ?po))

            ; robot acoplado
            (at start (coupled ?r))

            ; haya batería suficiente
            (at start (>= (battery ?r) (undock-consumption)))
        )
        :effect (and
            ; robot no acoplado
            (at start (not (coupled ?r)))

            ; robot desacoplado
            (at end (disengaged ?r))

            ; consumir bateria
            (at end (decrease (battery ?r) (undock-consumption)))

            ; incrementar energía total
            (at end (increase (total-energy-consumed) (undock-consumption)))

            ; contabilizar la duración total
            (at end (increase (total-duration) (undock-duration)))
        )

    )

    ; Tomar una fotografía en un posición
    (:durative-action take-image
        :parameters (?r - (either ugv uav) ?po - point ?p - pan ?t - tilt)
        :duration (= ?duration 5)   ; como hacerla variable ??????
        :condition (and
            ; robot este en las coordenadas
            (over all (in ?r ?po))
                
            ; orientación pan de la cámara por defecto
            (over all (camera-pan-orientation ?r ?p))

            ; orientación tilt de la cámara por defecto
            (over all (camera-tilt-orientation ?r ?t))

            ; haya batería suficiente
            (at start (>= (battery ?r) (take-image-consumption)))
        )
        :effect (and
            ; realizada fotografía en una posición con una orientación
            (at end (have-image ?r ?po ?p ?t))

            ; consumir bateria
            (at end (decrease (battery ?r) (take-image-consumption)))

            ; incrementar energía total
            (at end (increase (total-energy-consumed) (take-image-consumption)))

            ; contabilizar la duración total
            (at end (increase (total-duration) 5))
        )
    )

    ; Cambiar la orientación pan de la cámara en el robot
    (:durative-action change-pan-orientation
        :parameters (?r - (either ugv uav) ?p1 ?p2 - pan)
        :duration (= ?duration 5) ; como hacerla variable ??????
        :condition (and
            ; orientación pan actual de la cámara
            (at start (camera-pan-orientation ?r ?p1))

            ; pan no sea igual
            (over all (not (= ?p1 ?p2)))

            ; los ángulos de giro sean adyacentes
            (over all (adjacent ?p1 ?p2))

            ; haya batería suficiente
            (at start (>= (battery ?r) (change-pan-orientation-consumption)))
        )
        :effect (and
            ; borrar la orientación pan anterior
            (at start (not (camera-pan-orientation ?r ?p1)))

            ; cambiar la orientación pan camara
            (at end (camera-pan-orientation ?r ?p2))

            ; consumir bateria
            (at end (decrease (battery ?r) (change-pan-orientation-consumption)))

            ; incrementar energía total
            (at end (increase (total-energy-consumed) (change-pan-orientation-consumption)))

            ; contabilizar la duración total
            (at end (increase (total-duration) 5))
        )
    )

    ; Cambiar la orientación tilt de la cámara en el robot
    (:durative-action change-tilt-orientation
        :parameters (?r - (either ugv uav) ?t1 ?t2 - tilt)
        :duration (= ?duration 5) ; como hacerla variable ??????
        :condition (and
            ; orientación tilt actual de la cámara
            (at start (camera-tilt-orientation ?r ?t1))

            ; tilt no sea igual
            (over all (not (= ?t1 ?t2)))

            ; los ángulos de giro sean adyacentes
            (over all (adjacent ?t1 ?t2))

            ; haya batería suficiente
            (at start (>= (battery ?r) (change-tilt-orientation-consumption)))
        )
        :effect (and
            ; borrar la orientación tilt anterior
            (at start (not (camera-tilt-orientation ?r ?t1)))

            ; cambiar la orientación tilt camara
            (at end (camera-tilt-orientation ?r ?t2))

            ; consumir bateria
            (at end (decrease (battery ?r) (change-tilt-orientation-consumption)))

            ; incrementar energía total
            (at end (increase (total-energy-consumed) (change-tilt-orientation-consumption)))

            ; contabilizar la duración total
            (at end (increase (total-duration) 5))
        )
    )

    ; Transmitir una fotografía
    (:durative-action transmit-image
        :parameters (?r - (either ugv uav) ?po - point ?p - pan ?t - tilt)
        :duration (= ?duration (transmit-duration))  ;cte
        :condition (and
            ; se ha tomado la fotografia
            (over all (have-image ?r ?po ?p ?t))

            ; haya batería suficiente
            (at start (>= (battery ?r) (transmit-image-consumption)))
        )
        :effect (and
            ; se transmite la fotografía
            (at end (transmitted-image ?r ?po ?p ?t))

            ; consumir bateria
            (at end (decrease (battery ?r) (transmit-image-consumption)))

            ; incrementar energía total
            (at end (increase (total-energy-consumed) (transmit-image-consumption)))

            ; contabilizar la duración total
            (at end (increase (total-duration) (transmit-duration)))
        )
    )

)
