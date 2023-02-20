(define (problem problem1) 
(:domain domain_sherlock)
    (:objects 
        sherlock - robot
        w1 w2 w3 w4 oracle_room - waypoint
        
    )

    (:init
        
    (in_position oracle_room sherlock) 
    (=(distance w1 w3) 5)
    (=(distance w3 w1) 5)
    (=(distance w1 w4) 5)
    (=(distance w4 w1) 5)
    (=(distance w2 w3) 5)
    (=(distance w3 w2) 5)
    (=(distance w2 w4) 5)
    (=(distance w4 w2) 5)
    (=(distance w1 oracle_room) 3.5) 
    (=(distance w2 oracle_room) 3.5)
    (=(distance w3 oracle_room) 3.5)
    (=(distance w4 oracle_room) 3.5)
    (=(distance oracle_room w1) 3.5)
    (=(distance oracle_room w2) 3.5)
    (=(distance oracle_room w3) 3.5)
    (=(distance oracle_room w4) 3.5)
    (=(distance w1 w2) 7.25) 
    (=(distance w3 w4) 7.25)
    (=(distance w2 w1) 7.25)
    (=(distance w4 w3) 7.25)
    ;all the distances are proportional to the original distance, based on this the planning happens

    (=(waypoints) 0)
    (=(cost) 0)
    (not_can_check)
    (ontology_updated)
    (not_get_hint w1)
    (not_get_hint w2)
    (not_get_hint w3)
    (not_get_hint w4)
    (not_visited w1)
    (not_visited w2)
    (not_visited w3)
    (not_visited w4)
    (not_visited oracle_room)

    )

    (:goal 
        (and
        (visited w1)
        (visited w2)
        (visited w3)
        (visited w4)
        (in_position oracle_room sherlock)
        (end_game)

        )
    )
    (:metric minimize cost)
)
