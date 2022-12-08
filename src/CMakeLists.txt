project(RTracer)

option(USE_ISPC "Compile with ISPC" ON)
option(USE_BVH "Use acceleration structure" ON)
find_package(Threads REQUIRED)

set(SRC_FILES 
    scene/sceneloader.cpp
    main.cpp
)
set(HDR_FILES )

add_executable(Tracer ${SRC_FILES} )

add_custom_command(
    TARGET Tracer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory              
        ${CMAKE_CURRENT_SOURCE_DIR}/models $<TARGET_FILE_DIR:Tracer>/models
    COMMENT "Copying models" VERBATIM
    )
  
# target_include_directories(Tracer PRIVATE ${OpenGL_INCLUDE_DIR})
# target_compile_definitions(Tracer PUBLIC IMGUI_IMPL_OPENGL_LOADER_GLEW)
if(USE_ISPC)

target_compile_definitions(Tracer PUBLIC USE_ISPC)

endif(USE_ISPC)
if(USE_BVH)

target_compile_definitions(Tracer PUBLIC USE_BVH)

endif(USE_BVH)

target_link_directories(Tracer PRIVATE <TARGET_FILE_DIR:lib>)
target_link_libraries(Tracer PUBLIC Threads::Threads)
# jsoncpp_lib
# )
