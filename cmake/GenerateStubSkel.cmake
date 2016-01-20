####
## settings for macro 
##
set(idl_flags -bcxx -Wba -Wbuse_quotes -Wbh=.hh -Wbs=Sk.cpp )
set(package_path OpenHRP)
if(NOT QNXNTO)
  set(JAVAC javac)
  set(JAR jar)
  set(IDLJ idlj)
  set(idlj_flags -fclient -fserver -emitAll -d ORBIT2_IDL -d TYPECODE_CORBA_PREFIX)
  set(javac_flags -target 1.7)
endif()

####
## macro generate_stub_skel()
## 
macro(generate_stub_skel idl_basename)
  set(idl_file ${CMAKE_CURRENT_SOURCE_DIR}/${idl_basename}.idl)
  include_directories(${CMAKE_CURRENT_BINARY_DIR})
  if(NOT QNXNTO)
    set(jarfile ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}.jar)
    add_custom_command(
      OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}.hh ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}Sk.cpp ${jarfile} ${CMAKE_CURRENT_BINARY_DIR}/python/${idl_basename}_idl.py
      COMMAND omniidl ${idl_flags} -C ${CMAKE_CURRENT_BINARY_DIR} ${idl_file}
      COMMAND mkdir -p ${CMAKE_CURRENT_BINARY_DIR}/python
      COMMAND omniidl -bpython -C ${CMAKE_CURRENT_BINARY_DIR}/python -I ${OPENRTM_IDL_DIR} ${idl_file}
      COMMAND ${IDLJ} ${idlj_flags} -td ${CMAKE_CURRENT_BINARY_DIR}/src -I ${OPENRTM_IDL_DIR} ${idl_file}
      COMMAND mkdir -p ${CMAKE_CURRENT_BINARY_DIR}/bin
      COMMAND ${JAVAC} ${javac_flags} -sourcepath ${CMAKE_CURRENT_BINARY_DIR}/src  ${CMAKE_CURRENT_BINARY_DIR}/src/*/*.java -d ${CMAKE_CURRENT_BINARY_DIR}/bin
      COMMAND ${JAR} cf ${jarfile} -C ${CMAKE_CURRENT_BINARY_DIR}/bin .
      DEPENDS ${idl_file}
      )
  else()
    add_custom_command(
      OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}.hh ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}Sk.cpp ${jarfile} 
      COMMAND omniidl ${idl_flags} ${idl_file}
      DEPENDS ${idl_file}
      )
  endif()
endmacro()
