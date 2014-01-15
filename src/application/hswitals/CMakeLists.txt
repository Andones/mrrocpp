add_executable(ecp_hswitals_generatore
	ecp_g_hswitals_generatore.cc
	ecp_t_hswitals_generatore.cc
)

add_library(ecp_mp_g_hswitals_generatore
        ecp_mp_g_hswitals_generatore.cc
        )

target_link_libraries(ecp_hswitals_generatore ecp_robot ecp_mp_g_hswitals_generatore)

add_executable(mp_rubic_cube_hswitals
        mp_rubic_cube_hswitals.cc
)

target_link_libraries(mp_rubic_cube_hswitals mp ecp_mp_g_hswitals_generatore)

install(TARGETS mp_rubic_cube_hswitals ecp_hswitals_generatore DESTINATION bin)
