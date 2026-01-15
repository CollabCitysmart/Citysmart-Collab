cmake_minimum_required(VERSION 3.15)
project(CitySmart VERSION 0.1.0 LANGUAGES C CXX)

# ===============================
# Use C++17
# ===============================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===============================
# Create executable
# ===============================
add_executable(CitySmart main.cpp CityMap.cpp Vehicle.cpp EmergencyVehicle.cpp)
# ===============================
# Platform-specific Raylib setup
# ===============================
if (WIN32)

    if(MSVC)
        message(STATUS "Using MSVC")
        
        # --- PATH FIX: Use the relative path we created (raylib-lib folder in project root) ---
        set(RAYLIB_PATH "${CMAKE_SOURCE_DIR}/raylib-lib") 
        set(RAYLIB_LIB "${RAYLIB_PATH}/lib/raylib.lib")
        set(RAYLIB_DLL "${RAYLIB_PATH}/lib/raylib.dll")

        target_include_directories(CitySmart PRIVATE "${RAYLIB_PATH}/include")
        
        # CRITICAL FIX: Explicitly set C Runtime to Dynamic (MD/MDd)
        # This resolves the LNK2038 mismatch because Raylib uses the DLL runtime.
        target_compile_options(CitySmart PRIVATE 
             $<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>:/MDd>
             $<$<OR:$<CONFIG:Release>,$<CONFIG:MinSizeRel>>:/MD>
        )
        
        # CRITICAL: Link the core raylib.lib along with system libraries AND the missing C-runtime dependencies.
        # The linker will automatically find MSVCPRTD.lib and MSVCRTD.lib now.
        target_link_libraries(CitySmart PRIVATE 
            ${RAYLIB_LIB} 
            
            # Windows System Libraries (Required by Raylib)
            opengl32 gdi32 winmm
            User32.lib kernel32.lib shell32.lib 
            Ws2_32.lib 
            
            # Legacy C Standard Library (Required to resolve imports like __imp_calloc, __imp_fopen)
            legacy_stdio_definitions.lib
        )

        # We remove the /NODEFAULTLIB options because they are preventing the Dynamic Runtime from being linked automatically.
        
        # --- POST-BUILD COMMANDS: Copy DLL and PNG file to the output directory ---

        # 1. Copy the raylib.dll
        add_custom_command(TARGET CitySmart POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${RAYLIB_DLL}
            $<TARGET_FILE_DIR:CitySmart>)
            
   
    
    elseif(MINGW)
        message(STATUS "Using MinGW")
        set(RAYLIB_PATH "${CMAKE_SOURCE_DIR}/raylib-mingw")
        set(RAYLIB_LIB "${RAYLIB_PATH}/lib/libraylib.a")

        target_include_directories(CitySmart PRIVATE "${RAYLIB_PATH}/include")
        target_link_libraries(CitySmart PRIVATE ${RAYLIB_LIB} opengl32 gdi32 winmm)
    endif()

elseif(APPLE)
    find_package(raylib REQUIRED)
    target_link_libraries(CitySmart PRIVATE raylib)

elseif(UNIX)
    find_package(raylib REQUIRED)
    target_link_libraries(CitySmart PRIVATE raylib)

endif()