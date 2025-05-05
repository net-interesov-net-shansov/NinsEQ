cmake_minimum_required(VERSION 3.22)

# Version is needed by JUCE.
project(NinsEQ VERSION 0.1.0)

# Adding a directory with the library/application name as a subfolder of the
# include folder is a good practice. It helps avoid name clashes later on.
set(INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/NinsEQ")

# Adds a plugin target (that's basically what the Projucer does).
juce_add_plugin(
  ${PROJECT_NAME}
  COMPANY_NAME
  "NinsCorp" 
  IS_SYNTH
  FALSE
  NEEDS_MIDI_INPUT
  FALSE
  NEEDS_MIDI_OUTPUT
  FALSE
  PLUGIN_MANUFACTURER_CODE
  Nnsc
  PLUGIN_CODE
  Ndeq
  FORMATS
  Standalone
  VST3
  PRODUCT_NAME
  "NinsEQ" 
)

# Sets the source files of the plugin project.
set(SOURCE_FILES source/PluginEditor.cpp source/PluginProcessor.cpp)
# Optional; includes header files in the project file tree in Visual Studio
set(HEADER_FILES ${INCLUDE_DIR}/PluginEditor.h ${INCLUDE_DIR}/PluginProcessor.h)
target_sources(${PROJECT_NAME} PRIVATE ${SOURCE_FILES} ${HEADER_FILES})

# Sets the include directories of the plugin project.
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Links to all necessary dependencies. The present ones are recommended by JUCE.
# For our EQ plugin, we need DSP module for filters and FFT for spectrum analysis
target_link_libraries_system(${PROJECT_NAME} PUBLIC juce::juce_audio_utils juce::juce_dsp)
target_link_libraries(
  ${PROJECT_NAME} PUBLIC juce::juce_recommended_config_flags juce::juce_recommended_lto_flags
)

# These definitions are recommended by JUCE.
target_compile_definitions(${PROJECT_NAME} PUBLIC JUCE_WEB_BROWSER=0 JUCE_USE_CURL=0 JUCE_VST3_CAN_REPLACE_VST2=0)

# In Visual Studio this command provides a nice grouping of source files in "filters".
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/..)
