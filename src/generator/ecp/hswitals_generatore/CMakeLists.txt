add_library(ecp_g_hswitals_generatore
	ecp_g_hswitals_generatore.cc
)

add_library(ecp_mp_g_hswitals_generatore
	ecp_mp_g_hswitals_generatore.cc
	)
	
target_link_libraries(ecp_g_hswitals_generatore ecp)

install(TARGETS ecp_g_hswitals_generatore ecp_mp_g_hswitals_generatore DESTINATION lib)
