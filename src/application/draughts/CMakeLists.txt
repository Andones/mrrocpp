if(ROBOT_IRP6_POSTUMENT)
add_executable(ecp_draughts
	ecp_g_vis_ib_eih_follower_irp6ot.cc
	ecp_mp_tr_draughtsAI.cc
	ecp_t_draughts.cc
	../../generator/ecp/ecp_g_sleep.cc
)

target_link_libraries(ecp_draughts
	ecp
)

install(TARGETS ecp_draughts DESTINATION bin)
endif(ROBOT_IRP6_POSTUMENT)
