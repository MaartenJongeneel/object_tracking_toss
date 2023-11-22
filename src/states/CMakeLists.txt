include_directories(/home/administrator/devel/packages/openvr)

configure_file(/home/administrator/devel/packages/openvr/lib/linux64/libopenvr_api.so "${CMAKE_CURRENT_BINARY_DIR}/etc/libopenvr_api.so")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/libopenvr_api.so" DESTINATION "${MC_RTC_LIBDIR}/mc_controller/${PROJECT_NAME}/states")

add_fsm_state_simple(ObjectTrackingToss_Initial)

add_fsm_state_simple(MoveToNeutral)
target_link_libraries(MoveToNeutral PUBLIC mc_iam::devices)

add_fsm_state_simple(MoveToStart)
target_link_libraries(MoveToStart PUBLIC mc_iam::devices)

add_fsm_state_simple(GrabBox)
target_link_libraries(GrabBox PUBLIC mc_iam::devices)

add_fsm_state_simple(BoxToss)
target_link_libraries(BoxToss PUBLIC mc_iam::devices)

#add_fsm_state_simple(RecordRobotTeleoperation)
#target_link_libraries(RecordRobotTeleoperation PUBLIC "${MC_RTC_LIBDIR}/mc_controller/${PROJECT_NAME}/states/libopenvr_api.so" mc_iam::devices)

add_fsm_state_simple(TerminateProcess)
target_link_libraries(TerminateProcess PUBLIC mc_iam::devices)

add_fsm_data_directory(data)
