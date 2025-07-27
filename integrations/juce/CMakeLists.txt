# JUCE Integration for Visage
# This optional module provides seamless integration with JUCE applications

option(VISAGE_BUILD_JUCE_INTEGRATION "Build JUCE integration bridge" OFF)

if (VISAGE_BUILD_JUCE_INTEGRATION)
    # Find JUCE - this will need to be configured by the user
    find_package(PkgConfig QUIET)
    
    # Try to find JUCE via common methods
    find_path(JUCE_INCLUDE_DIR 
        NAMES juce_core/juce_core.h
        PATHS 
            ${JUCE_DIR}/modules
            ${JUCE_PATH}/modules
            /usr/local/include/JUCE/modules
            /opt/JUCE/modules
            ${CMAKE_PREFIX_PATH}/include/JUCE/modules
    )
    
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
        
    else()
        message(WARNING "JUCE not found. Set JUCE_DIR or JUCE_PATH to enable JUCE integration.")
        message(STATUS "You can download JUCE from: https://juce.com/")
    endif()
endif()
