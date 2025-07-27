# JUCE Integration for Visage
# This optional module provides seamless integration with JUCE applications

option(VISAGE_BUILD_JUCE_INTEGRATION "Build JUCE integration bridge" OFF)

if (VISAGE_BUILD_JUCE_INTEGRATION)
    # Find JUCE - this will need to be configured by the user
    find_package(PkgConfig QUIET)
    
    # Use system JUCE installation
    set(JUCE_INCLUDE_DIR "/usr/share/juce/modules")
    
    if(NOT EXISTS ${JUCE_INCLUDE_DIR})
        message(FATAL_ERROR "JUCE modules not found at ${JUCE_INCLUDE_DIR}. Please install juce-modules-source package.")
    endif()
    
    message(STATUS "Using system JUCE at: ${JUCE_INCLUDE_DIR}")
    
    if (JUCE_INCLUDE_DIR)
        message(STATUS "Found JUCE at: ${JUCE_INCLUDE_DIR}")
        
        # Create the JUCE integration library
        add_library(VisageJuceIntegration STATIC
            juce_visage_bridge.cpp
            juce_visage_bridge.h
        )
        
        target_include_directories(VisageJuceIntegration PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${JUCE_INCLUDE_DIR}
        )
        
        target_link_libraries(VisageJuceIntegration PUBLIC
            visage
        )
        
        # Set up proper C++ standard for JUCE compatibility
        target_compile_features(VisageJuceIntegration PUBLIC cxx_std_17)
        
        # Platform-specific JUCE modules and linking
        if (APPLE)
            target_link_libraries(VisageJuceIntegration PUBLIC
                "-framework Cocoa"
                "-framework OpenGL"
                "-framework IOKit"
                "-framework CoreFoundation"
                "-framework CoreAudio"
                "-framework CoreMIDI"
                "-framework AudioToolbox"
                "-framework AudioUnit"
            )
        elseif (WIN32)
            target_link_libraries(VisageJuceIntegration PUBLIC
                opengl32
                winmm
                ole32
                oleaut32
            )
        elseif (UNIX)
            target_link_libraries(VisageJuceIntegration PUBLIC
                GL
                X11
                Xext
                Xinerama
                asound
                dl
                freetype
                pthread
                rt
            )
        endif()
        
        # Export the target
        set_target_properties(VisageJuceIntegration PROPERTIES
            FOLDER "visage/integrations"
            EXPORT_NAME JuceIntegration
        )
        
        # Install rules
        install(TARGETS VisageJuceIntegration
            EXPORT VisageJuceIntegrationTargets
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
            RUNTIME DESTINATION bin
        )
        
        install(FILES juce_visage_bridge.h
            DESTINATION include/visage/integrations/juce
        )
        
                # Create a GUI test application to demonstrate the fix
        add_executable(VisageJuceTestApp
            test_app.cpp
        )
        
        target_include_directories(VisageJuceTestApp PRIVATE
            ${JUCE_INCLUDE_DIR}
        )
        
        target_link_libraries(VisageJuceTestApp PRIVATE
            visage
        )
        
        # Add JUCE source files directly since we can't link to static libs easily
        target_sources(VisageJuceTestApp PRIVATE
            ${JUCE_INCLUDE_DIR}/juce_core/juce_core.cpp
            ${JUCE_INCLUDE_DIR}/juce_data_structures/juce_data_structures.cpp
            ${JUCE_INCLUDE_DIR}/juce_graphics/juce_graphics.cpp
            ${JUCE_INCLUDE_DIR}/juce_gui_basics/juce_gui_basics.cpp
            ${JUCE_INCLUDE_DIR}/juce_events/juce_events.cpp
            ${JUCE_INCLUDE_DIR}/juce_opengl/juce_opengl.cpp
        )
        
        # Add platform-specific JUCE libraries for the test app
        if (UNIX AND NOT APPLE)
            # Find required system libraries for JUCE
            find_package(PkgConfig REQUIRED)
            find_package(X11 REQUIRED)
            
            # ALSA for audio
            pkg_check_modules(ALSA REQUIRED alsa)
            
            # Freetype for text rendering  
            pkg_check_modules(FREETYPE REQUIRED freetype2)
            
            # CURL for web requests
            find_package(CURL REQUIRED)
            
            # JPEG library 
            find_package(JPEG REQUIRED)
            
            # PNG library
            find_package(PNG REQUIRED)
            
            # ZLIB for compression
            find_package(ZLIB REQUIRED)
            
            target_link_libraries(VisageJuceTestApp PRIVATE
                ${X11_LIBRARIES}
                ${ALSA_LIBRARIES}
                ${FREETYPE_LIBRARIES}
                ${CURL_LIBRARIES}
                ${JPEG_LIBRARIES}
                ${PNG_LIBRARIES}
                ${ZLIB_LIBRARIES}
                GL
                dl
                pthread
                rt
            )
            
            target_include_directories(VisageJuceTestApp PRIVATE
                ${ALSA_INCLUDE_DIRS}
                ${FREETYPE_INCLUDE_DIRS}
                ${CURL_INCLUDE_DIRS}
                ${JPEG_INCLUDE_DIRS}
                ${PNG_INCLUDE_DIRS}
                ${ZLIB_INCLUDE_DIRS}
            )
            
            # Find optional X11 extensions
            if(X11_Xext_FOUND)
                target_link_libraries(VisageJuceTestApp PRIVATE ${X11_Xext_LIB})
            endif()
            if(X11_Xinerama_FOUND)
                target_link_libraries(VisageJuceTestApp PRIVATE ${X11_Xinerama_LIB})
            endif()
            if(X11_Xrandr_FOUND)
                target_link_libraries(VisageJuceTestApp PRIVATE ${X11_Xrandr_LIB})
            endif()
            if(X11_Xcursor_FOUND)
                target_link_libraries(VisageJuceTestApp PRIVATE ${X11_Xcursor_LIB})
            endif()
        endif()
        
        target_compile_definitions(VisageJuceTestApp PRIVATE
            JUCE_STANDALONE_APPLICATION=1
            JUCE_GLOBAL_MODULE_SETTINGS_INCLUDED=1
            JUCE_MODULE_AVAILABLE_juce_core=1
            JUCE_MODULE_AVAILABLE_juce_data_structures=1
            JUCE_MODULE_AVAILABLE_juce_graphics=1
            JUCE_MODULE_AVAILABLE_juce_gui_basics=1
            JUCE_MODULE_AVAILABLE_juce_events=1
            JUCE_MODULE_AVAILABLE_juce_opengl=1
        )
        
        set_target_properties(VisageJuceTestApp PROPERTIES
            FOLDER "visage/integrations/tests"
        )
        
        # Simple integration test that doesn't require full JUCE runtime
        add_executable(VisageJuceSimpleTest
            simple_test.cpp
        )
        
        target_link_libraries(VisageJuceSimpleTest
            PRIVATE
                VisageJuceIntegration
        )
        
        set_target_properties(VisageJuceSimpleTest PROPERTIES
            FOLDER "visage/integrations/tests"
        )
        
    else()
        message(WARNING "JUCE not found. Set JUCE_DIR or JUCE_PATH to enable JUCE integration.")
        message(STATUS "You can download JUCE from: https://juce.com/")
    endif()
endif()
