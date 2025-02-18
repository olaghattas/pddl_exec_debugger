(define (problem shr_domain_problem)
(:domain shr_domain)
(:objects
	 caregiver_call - CallAction
	 t2 - Time
	 living_room - Landmark
	 w1 - WaitAction
	 bedroom - Landmark
	 t1 - Time
	 outside - Landmark
	 home - Landmark
	 nathan - Person
	 t3 - Time
	 na1 - NoAction
	 t4 - Time
	 second_reminder - ReminderAction
	 call_caregiver_msg - Msg
	 reminder_1_msg - Msg
	 na2 - NoAction
	 t5 - Time
	 first_reminder - ReminderAction
	 w2 - WaitAction
	 w4 - WaitAction
	 w5 - WaitAction
	 w3 - WaitAction
	 reminder_2_msg - Msg
	 na3 - NoAction
)
(:init
	(current_time t4)
	(executed_call caregiver_call)
	(noaction_person_location_constraint na3 nathan outside)
	(noaction_person_location_constraint na2 nathan outside)
	(noaction_person_location_constraint na1 nathan outside)
	(MakeCall_enabled)
	(traversable outside living_room)
	(traversable living_room outside)
	(traversable home living_room)
	(traversable living_room home)
	(traversable outside home)
	(traversable home outside)
	(wait_robot_location_constraint t4 home)
	(wait_robot_location_constraint t3 home)
	(wait_robot_location_constraint t2 home)
	(wait_robot_location_constraint t1 home)
	(person_at t3 nathan living_room)
	(person_at t1 nathan living_room)
	(person_at t2 nathan living_room)
	(GiveReminder_enabled)
	(reminder_person_not_taking_medicine_constraint second_reminder nathan)
	(reminder_person_not_taking_medicine_constraint first_reminder nathan)
	(valid_call_message caregiver_call call_caregiver_msg)
	(same_location_constraint)
	(robot_at living_room)
	(medicine_taken_success)
	(DetectPerson_enabled)
	(next_time t1 t2)
	(next_time t2 t3)
	(next_time t3 t4)
	(next_time t4 t5)
	(message_given_success call_caregiver_msg)
	(reminder_blocks_reminder first_reminder second_reminder)
	(valid_reminder_message second_reminder reminder_2_msg)
	(valid_reminder_message first_reminder reminder_1_msg)
	(reminder_blocks_call second_reminder caregiver_call)
	(wait_not_person_location_constraint t4 nathan outside)
	(wait_not_person_location_constraint t3 nathan outside)
	(wait_not_person_location_constraint t2 nathan outside)
	(wait_not_person_location_constraint t1 nathan outside)
	(wait_not_person_location_constraint t5 nathan outside)
	(message_given call_caregiver_msg)
	(DetectTakingMedicine_enabled)
	(message_given reminder_1_msg)
	(message_given reminder_2_msg)
	(executed_reminder first_reminder)
	(executed_reminder second_reminder)
	(used_reminder t3)
	(used_reminder t1)
	(used_reminder t2)
	(used_move t3 living_room)
	(used_move t1 home)
	(used_move t2 living_room)
	(used_move t2 home)
	(robot_at_time t3 living_room)
	(robot_at_time t2 home)
	(robot_at_time t1 home)
	(robot_at_time t2 living_room)
	(executed_wait t2)
	(executed_wait t1)
	(unknown (person_at t5 nathan outside))
	(unknown (person_at t5 nathan living_room))
	(unknown (person_at t4 nathan outside))
	(unknown (person_at t4 nathan living_room))
	(oneof (person_at t2 nathan living_room) (person_at t2 nathan outside))
	(oneof (person_at t3 nathan living_room) (person_at t3 nathan outside))
	(oneof (person_at t4 nathan living_room) (person_at t4 nathan outside))
	(oneof (person_at t5 nathan living_room) (person_at t5 nathan outside))
)
(:goal
(success))
)