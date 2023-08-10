
(define (problem kitting)
    (:domain kitting)
    (:objects
        ; number of parts can be retrieved from /ariac/bin_parts
        part1 part2 part3 - part
        ; bins with parts. Can be retrieved from /ariac/bin_parts
        bin1 - bin
        ; hard coded
        QUADRANT1 QUADRANT2 QUADRANT3 QUADRANT4 - quadrant
        ; trays at stations. Can be retrieved from camera topics
        tray1 tray2 tray3 - tray
        ; hard coded
        table1 table2 - table
        ; part types and colors. Can be retrieved from /ariac/bin_parts
        ; or from cameras
        BLUEBATTERY GREENPUMP - part-type-color
        ; can be retrieved from camera topics
        TRAYID1 TRAYID2 - tray-id
        ; can be retrieved from /ariac/orders
        agv1 agv2 - agv

    )

    (:init
        (non-competition-started)
        (non-competition-ended)
        (non-submitted-order)
        (= (current-num-parts-in-kit) 0)
        (= (expected-num-parts-in-kit) 3)
        (agv-at-kitting-station agv1)
        (agv-at-kitting-station agv2)

        (has-part-type-color part1 BLUEBATTERY)
        (has-part-type-color part2 GREENPUMP)
        (has-part-type-color part3 BLUEBATTERY)
        (has-tray-id tray1 TRAYID1)
        (has-tray-id tray2 TRAYID1)
        (has-tray-id tray3 TRAYID2)
        (in-bin part1 bin1)
        (in-bin part2 bin1)
        (in-bin part3 bin1)
        (on-table tray1 table1)
        (on-table tray2 table1)
        (on-table tray3 table2)
        ; hard coded
        (part-gripper-mounted)
        (gripper-empty)
    )
    (:goal
        (and
            (on-agv tray1 agv1)
            (part-in-tray BLUEBATTERY TRAYID1 QUADRANT1)
            (part-in-tray GREENPUMP TRAYID1 QUADRANT2)
            (part-in-tray BLUEBATTERY TRAYID1 QUADRANT3)
            (competition-ended)
        )
    )
)