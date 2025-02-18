(define (problem move_reminder)
(:domain shr_domain)
(:objects
    current_loc dest_loc home - Landmark
    nathan - Person
    t1 t2 t3 t4 t5 - Time
    reminder_1_msg - Msg
    first_reminder  - ReminderAction
    w1 w2 w3 w4 w5 - WaitAction
    na1 na2 na3 - NoAction
)
(:init
    ;;(person_at t1 nathan current_loc)
    ;;(robot_at dest_loc)
    ;;(robot_at_time t1 dest_loc)

    ;;(no_action)

    (DetectPerson_enabled)
    (GiveReminder_enabled)

    (current_time t1)
    (next_time t1 t2)
    (next_time t2 t3)
    (next_time t3 t4)
    (next_time t4 t5)

    (oneof (person_at t2 nathan current_loc) (person_at t2 nathan dest_loc) )
    (oneof (person_at t3 nathan current_loc) (person_at t3 nathan dest_loc) )
    (oneof (person_at t4 nathan current_loc) (person_at t4 nathan dest_loc) )
    (oneof (person_at t5 nathan current_loc) (person_at t5 nathan dest_loc) )

    (traversable dest_loc current_loc)
    (traversable current_loc dest_loc)
    (traversable dest_loc home)
    (traversable home dest_loc)
    (traversable home current_loc)
    (traversable current_loc home)

    (same_location_constraint)
    ;;(not_same_location_constraint)

    ;;success states

    (message_given_success reminder_1_msg)
    (person_at_success nathan dest_loc)

    ;; specify valid input argument combinations for all actions
    (valid_reminder_message first_reminder reminder_1_msg)

    ;; specify world state constraints for all actions
    (reminder_person_location_constraint first_reminder nathan dest_loc)
    (reminder_robot_location_constraint first_reminder dest_loc)

    (wait_robot_location_constraint t1 home)
    (wait_robot_location_constraint t2 home)
    (wait_robot_location_constraint t3 home)
    (wait_robot_location_constraint t4 home)

)
(:goal (and (success)  ) )

)
