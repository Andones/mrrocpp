add_executable(mp_rubic_cube_hswitals
        mp_rubic_cube_hswitals.cc
)

target_link_libraries(mp_rubic_cube_hswitals mp)

install(TARGETS mp_rubic_cube_hswitals DESTINATION bin)

add_executable(ecp_hswitals_generatore
	ecp_t_hswitals_generatore.cc
        ecp_g_hswitals_rubik_rotate.cc
)

target_link_libraries(ecp_hswitals_generatore ecp_robot)

install(TARGETS ecp_hswitals_generatore DESTINATION bin)
