project (sun_ethercat_master)

#add_compile_options(-pthread)

set(${PROJECT_NAME}_SOURCES src/Master.cpp)
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC include)

target_link_libraries(${PROJECT_NAME} soem)
target_link_libraries(${PROJECT_NAME} sun_scheduling)
target_compile_options(${PROJECT_NAME} PUBLIC
    "-I/usr/xenomai/include"
    "-I/usr/xenomai/include/alchemy"
    "-I/usr/xenomai/include/cobalt"
)

# Linka la libreria Xenomai e Alchemy
target_link_libraries(${PROJECT_NAME} 
    "-lalchemy"
    "-lcobalt"
    "-lpthread"
    "-lrt"
    "-L/usr/xenomai/lib"
)

