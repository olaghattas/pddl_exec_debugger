(define (problem high_level)
  (:domain high_level_domain)
  (:objects
     am_meds pm_meds - MedicineProtocol
     move_reminder - MoveReminderProtocol
     exercise_reminder - ExerciseReminderProtocol
     internal_check_reminder - InternalCheckReminderProtocol
     practice_reminder - PracticeReminderProtocol
     living_room - Landmark
     home - Landmark
     outside - Landmark
     bedroom - Landmark
     nathan - Person
     t1 - Time ;;t2 t3 t4 t5
  )
  (:init

       ;; for testing
      (time_for_internal_check_reminder internal_check_reminder)
      (already_reminded_internal_check internal_check_reminder)

      (time_for_move_reminder move_reminder)
      (already_reminded_move move_reminder)

      (time_for_exercise_reminder exercise_reminder)
      (already_reminded_exercise exercise_reminder)

      (time_for_practice_reminder practice_reminder)
      (already_reminded_practice practice_reminder)

      (time_to_take_medicine am_meds)
      ;;(time_to_take_medicine pm_meds)
      ;;(already_took_medicine  pm_meds)
      ;;(already_took_medicine  am_meds)


      (priority_1)
      ;;(abort)
      (visible_location living_room)
      (visible_location bedroom)
      ;;(time_for_practice_reminder practice_reminder)
      ;;(time_for_practice_reminder practice_reminder)

      (person_currently_at nathan living_room)
      (robot_at living_room)
      ;;(person_currently_at nathan visible_area)
      ;;(time_for_walk_reminder walking_reminder)

      ;; testing medicine
      ;;(time_to_eat breakfast)
      ;;(already_reminded_eating  breakfast)

      ;; alert walk
      ;;(time_to_alert night_alert)

      ;; testing walk
      ;;(time_for_walk_reminder walking_reminder)
      ;;(already_reminded_walk walking_reminder)

      ;; testing gym
      ;;(time_for_gym_reminder gym_reminder)
      ;;(already_reminded_gym gym_reminder)


      ;; testing sleep
      ;;(time_for_sleep_reminder sleep_reminder)
      ;;(already_reminded_sleep  sleep_reminder)

      ;; testing move
      ;;(time_for_move_reminder move_reminder)
      ;;(already_reminded_move  move_reminder)

      ;; testing internal check
      ;;(time_for_internalcheck_reminder internalcheck_reminder)
      ;;(already_reminded_internalcheck  internalcheck_reminder)

      ;; testing practice
      ;;(time_for_practice_reminder practice_reminder)
      ;;(already_reminded_practice  practice_reminder)
  )
  (:goal (and (success)  ) )
)