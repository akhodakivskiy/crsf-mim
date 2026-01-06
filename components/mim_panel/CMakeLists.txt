find_program(NPM_EXECUTABLE npm)

if(NPM_EXECUTABLE)
    # Check if package.json exists at configure time
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/panel/package.json)
        # Create stamp file for tracking
        set(NPM_BUILD_STAMP "${CMAKE_CURRENT_BINARY_DIR}/npm_build.stamp")

        # Glob frontend source files to track dependencies
        file(GLOB_RECURSE PANEL_SOURCES
             "${CMAKE_CURRENT_SOURCE_DIR}/panel/src/*"
             "${CMAKE_CURRENT_SOURCE_DIR}/panel/public/*")

        # Custom command that runs during build
        add_custom_command(
            OUTPUT ${NPM_BUILD_STAMP}
            COMMAND ${NPM_EXECUTABLE} install
            COMMAND ${NPM_EXECUTABLE} run build:prod
            COMMAND ${CMAKE_COMMAND} -E touch ${NPM_BUILD_STAMP}
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/panel
            DEPENDS
                ${CMAKE_CURRENT_SOURCE_DIR}/panel/package.json
                ${CMAKE_CURRENT_SOURCE_DIR}/panel/package-lock.json
                ${PANEL_SOURCES}
            COMMENT "Building frontend with npm..."
            VERBATIM
        )

        # Create a target for the build
        add_custom_target(build_panel ALL DEPENDS ${NPM_BUILD_STAMP})
    else()
        message(WARNING "panel/package.json not found. Skipping frontend build.")
    endif()
else()
    message(WARNING "npm not found. Skipping frontend build.")
endif()

idf_component_register(
  SRCS
    "mim_panel.c"
    "mim_panel_httpd.c"
  INCLUDE_DIRS "."
  REQUIRES
    spiffs
    vfs
    esp_http_server
    cjson
    mim_conn
    mim_nav
)

if(TARGET build_panel)
    # Make SPIFFS image creation depend on frontend build
    spiffs_create_partition_image(www ${CMAKE_CURRENT_SOURCE_DIR}/panel/dist-prod
                                   FLASH_IN_PROJECT
                                   DEPENDS build_panel)
else()
    spiffs_create_partition_image(www ${CMAKE_CURRENT_SOURCE_DIR}/panel/dist-prod
                                   FLASH_IN_PROJECT)
endif()
