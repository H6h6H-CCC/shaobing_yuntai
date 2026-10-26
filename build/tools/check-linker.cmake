find_program(CMAKE_LINKER NAMES arm-none-eabi-ld ld HINTS "C:/Users/abcd1/AppData/Local/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin" NO_CMAKE_PATH NO_CMAKE_ENVIRONMENT_PATH)
message(STATUS "Linker: ${CMAKE_LINKER}")
