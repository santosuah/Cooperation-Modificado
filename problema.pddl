(define (problem problema)

    (:domain cooperation)

    (:objects

        ; puntos
        P0610 P1002 P0509 P1613 P0707 P0702 - point

        ; vehículos
        Leader - ugv     ; Unmanned Ground Vehicle
        Follower0 - uav  ; Unmanned Aerial Vehicle

        ; modos de navegación
        N0 N1 - navmode

        ; orientación de la cámara
        P_0 P_45 P_90 P_135 P_180 P_225 P_270 P_315 - Pan
        T_0 T_45 T_90 T_270 T_315 - Tilt
    )

    (:init

        ; ---- consumos ----
        
        ; consumo de cambio de modo
        (= (mode-change-consumption) 1)

        ; consumo de acople
        (= (dock-consumption) 1)

        ; consumo de desacople
        (= (undock-consumption) 1)

        ; consumo tomar imagen
        (= (take-image-consumption) 1)

        ; consumo cambio de orientación pan cámara
        (= (change-pan-orientation-consumption) 0)

        ; consumo cambio de orientación tilt cámara
        (= (change-tilt-orientation-consumption) 0)

        ; consumo transmisión una imagen
        (= (transmit-image-consumption) 1)

        ; ---- duraciones ----

        ; duración transmisión una imagen
        (= (transmit-duration) 5)

        ; duración acople
        (= (dock-duration) 5)

        ; duración desacople
        (= (undock-duration) 5)

        ; duración cambio de modo
        (= (change-mode-duration) 5)

        ; --------------------

        ; valores de las velocidades
        (= (speed N0) 1)
        (= (speed N1) 2)

        ; distancia entre puntos

        ; base station Leader = P0610
        ; base station Follower0 = P1002

        ; objective Leader = P0509
        ; objective Follower0 = P1613

        ; init position Leader = P0707
        ; init position Follower0 = P0702

        ; base station Leader a los demas
        (= (distance P0610 P1002) 8.94) (= (distance P1002 P0610) 8.94)
        (= (distance P0610 P0509) 1.41) (= (distance P0509 P0610) 1.41)
        (= (distance P0610 P1613) 10.44) (= (distance P1613 P0610) 10.44)
        (= (distance P0610 P0707) 3.16) (= (distance P0707 P0610) 3.16)
        (= (distance P0610 P0702) 8.06) (= (distance P0702 P0610) 8.06)

        (= (distance P1002 P0509) 8.6) (= (distance P0509 P1002) 8.6)
        (= (distance P1002 P1613) 12.53) (= (distance P1613 P1002) 12.53)
        (= (distance P1002 P0707) 5.83) (= (distance P0707 P1002) 5.83)
        (= (distance P1002 P0702) 3) (= (distance P0702 P1002) 3)

        (= (distance P0509 P1613) 11.7) (= (distance P1613 P0509) 11.7)
        (= (distance P0509 P0707) 2.83) (= (distance P0707 P0509) 2.83)
        (= (distance P0509 P0707) 7.28) (= (distance P0707 P0509) 7.28)

        (= (distance P1613 P0707) 10.82) (= (distance P0707 P1613) 10.82)
        (= (distance P1613 P0702) 14.21) (= (distance P0702 P1613) 14.21)

        (= (distance P0707 P0702) 5) (= (distance P0702 P0707) 5)


        ; distancia 0
        ; (= (distance P0610 P0610) 0)
        ; (= (distance P1002 P1002) 0)
        ; (= (distance P0509 P0509) 0)
        ; (= (distance P1613 P1613) 0)
        ; (= (distance P0707 P0707) 0)
        ; (= (distance P0702 P0702) 0)


        ; ángulos adyacentes

        ; PAN 
        (adjacent P_0 P_45) (adjacent P_45 P_0)
        (adjacent P_45 P_90) (adjacent P_90 P_45)
        (adjacent P_90 P_135) (adjacent P_135 P_90)
        (adjacent P_135 P_180) (adjacent P_180 P_135)
        (adjacent P_180 P_225) (adjacent P_225 P_180)
        (adjacent P_225 P_270) (adjacent P_270 P_225)

        (adjacent P_270 P_315) (adjacent P_315 P_270)
        ; (adjacent P_315 P_0) (adjacent P_0 P_315)

        ; TILT 
        (adjacent T_0 T_45) (adjacent T_45 T_0)
        (adjacent T_45 T_90) (adjacent T_90 T_45)
        (adjacent T_90 T_270) (adjacent T_270 T_90)
        (adjacent T_270 T_315) (adjacent T_315 T_270)
        ; (adjacent T_315 T_0) (adjacent T_0 T_315) estos angulos no estan conectados

        ; localización de la estación base
        (base-station Leader P0610)
        (base-station Follower0 P1002)

        ; ratio de descarga en función de la velocidad
        ; si va rápido gasta el doble
        (= (discharge-ratio N0) 1)
        (= (discharge-ratio N1) 2)

        ; capacidad máxima bateria
        (= (battery-capacity Leader) 100)
        (= (battery-capacity Follower0) 100)


        ; ---- estado inicial ----

        ; posicion inicial del robot
        (in Leader P0707)
        (in Follower0 P0702)

        ; no esta en la base
        (disengaged Leader)
        (disengaged Follower0)

        ; valor de la batería del robot inicial
        (= (battery Leader) 15)
        (= (battery Follower0) 20)

        ; valor de navegación defecto
        (navigation-mode Leader N0)
        (navigation-mode Follower0 N0)

        ; orientación inicial de la cámera
        (camera-pan-orientation Leader P_0)
        (camera-tilt-orientation Leader T_0)

        (camera-pan-orientation Follower0 P_135)
        (camera-tilt-orientation Follower0 T_90)

        ; energía total consumida
        (= (total-energy-consumed) 0)

        ; distancia total recorrida
        (= (total-distance-traveled) 0)

        ; duración total
        (= (total-duration) 0)
    )

    (:goal
        (and
            (transmitted-image Leader P0509 P_0 T_0)
            (transmitted-image Follower0 P1613 P_0 T_0)
        )
    )

    (:constraints 
        (and
            (preference uav-uncopled
                ; para todos los uav siempre estar desacoplados
                (forall (?a - uav) (always (disengaged ?a)))
            )

            (preference ugv-uncopled
                ; para todos los ugv alguna vez estar desacoplados
                (forall (?b - ugv) (sometime (disengaged ?b)))
            )

        )
    )

    (:metric minimize 
        (+
            (* (total-duration) 0.25)
            (* (total-energy-consumed) 0.25)

            (* (is-violated uav-uncopled) 0.25)
            (* (is-violated ugv-uncopled) 0.25)
        )
    )

)
