# CMake input file to build ProdExplUnits

#include ose library
INCLUDE_DIRECTORIES( ${ECOS_INCLUDE}/include )
INCLUDE_DIRECTORIES( ${BASICLIBINCLUDE}/LibUtilities/include )
INCLUDE_DIRECTORIES( ${BASICLIBINCLUDE}/LibModules/include )
INCLUDE_DIRECTORIES( ${BASICLIBINCLUDE}/LibData/include )

INCLUDE_DIRECTORIES( ${PROD_EXPL_SOURCE_DIR}/ProdExplLib/include )

# Make sure the linker can find the libraries once they are built.
LINK_DIRECTORIES(${CMAKE_INSTALL_PREFIX}/lib)
#LINK_DIRECTORIES(${BASICLIBLIBRARY}/LibUtilities)
#LINK_DIRECTORIES(${BASICLIBLIBRARY}/LibModules)
LINK_DIRECTORIES(${ECOS_LIBRARY}/include)
LINK_DIRECTORIES(${ECOS_LIBRARY}/lib)

#AUX_SOURCE_DIRECTORY( . PROD_EXPL_UNITS_SRCS )
SET(PROD_EXPL_UNITS_SRCS
	source/alarm_audio_data.cxx
	source/alarm_list_fun.cxx
	source/alarm_pcu_audio.cxx
	source/audio_sample.cxx
	source/bcu_ann.cxx
	source/bcu_cmu_handle.cxx
	source/bcu_config.cxx
	source/bcu_extern_button.cxx
	source/bcu_failure_handle.cxx
	source/bcu_function.cxx
	source/bcu_gd_sync.cxx
	source/bcu_intercom.cxx
	source/bcu_main.cxx
	source/bcu_state_machine.cxx
	source/bcu_update_table_from_ccu.cxx
	source/buffer_read_write_fun.cxx
	source/data_struct_bcu.cxx
	source/function_for_network.cxx
	source/log_record.cxx
	source/tftp_server_apply.cxx
	source/tftp_server_fun.cxx
	source/network/udp_send_fun.cxx
	source/network/zhw_set_into_mul.cxx
	source/network/zhw_shell_function.cxx
	source/network/zhw_middle_forward.cxx
	source/network/zhw_dev_selftest_module.cxx
	source/network/add_ip_to_mul.cxx
	source/network/zhw_dev_config_info.cxx
	source/network/zhw_buffer_init.cxx
	source/network/zhw_sockfd_server_init.cxx
	source/network/zhw_network_main.cxx
	source/network/zhw_socket_init.cxx
	source/network/zhw_use_ioctl_to_set_net.cxx
	source/network/udp_recv_fun.cxx
	source/network/zhw_read_from_db_test.cxx
	source/network/zhw_apply_common_func.cxx
	source/network/zhw_mount_romfs.cxx
	source/network/zhw_bcu_to_cmu.cxx
	source/sound_driver.cxx
	source/thread_comm.cxx
	source/thread_config.cxx
	source/TouchScreen_horizontal.cxx
)

# Strip release build
IF( "${CMAKE_BUILD_TYPE}" STREQUAL "Release" )
  SET( CMAKE_EXE_LINKER_FLAGS  "-Wl,-strip-all")
ENDIF( "${CMAKE_BUILD_TYPE}" STREQUAL "Release" )
#SET(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS)

ADD_EXECUTABLE(ProdTest ${PROD_EXPL_UNITS_SRCS})

# Link the executable to the libraries.
TARGET_LINK_LIBRARIES(ProdTest
#  ${CMAKE_INSTALL_PREFIX}/lib/LibModules.a
)
TARGET_LINK_LIBRARIES(ProdTest  target )



INSTALL( TARGETS ProdTest  RUNTIME DESTINATION ./ )

INSTALL( CODE "
	EXECUTE_PROCESS(COMMAND rm  ProdTest_${BUILD}.srec ProdTest_${BUILD}.bin  ProdTest_${BUILD}.img ProdTest_${BUILD}.elf WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/)	
#	EXECUTE_PROCESS(COMMAND rm ProdTest_${BUILD}.srec ProdTest_${BUILD}.bin  ProdTest_${BUILD}.img ProdTest_${BUILD}.elf   WORKING_DIRECTORY  /home/benty/workspace/Products/${BUILD}/)	

	
	EXECUTE_PROCESS(COMMAND ${CROSSTOOL}/${TOOLS_PREFIX}/bin/${TOOLS_PREFIX}-objcopy -O  binary ProdTest.elf ProdTest_${BUILD}.bin WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/)
	EXECUTE_PROCESS(COMMAND ${CROSSTOOL}/${TOOLS_PREFIX}/bin/${TOOLS_PREFIX}-objcopy -O srec ProdTest.elf ProdTest_${BUILD}.srec WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/)
	EXECUTE_PROCESS(COMMAND cp	ProdTest.elf ./ProdTest_${BUILD}.elf WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/)
#	EXECUTE_PROCESS(COMMAND	dd if=/dev/zero of=ProdTest_${BUILD}.img bs=512 count=2880 WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/)
#	EXECUTE_PROCESS(COMMAND dd if=ProdTest_${BUILD}.bin of=ProdTest_${BUILD}.img bs=512 conv=notrunc WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/)
	EXECUTE_PROCESS(COMMAND cp  ProdTest_${BUILD}.srec ProdTest_${BUILD}.bin  ProdTest_${BUILD}.img ProdTest_${BUILD}.elf /home/benty/workspace/Products/${BUILD}/   WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/)
#	EXECUTE_PROCESS(COMMAND rm ProdTest.elf WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/)	
")

