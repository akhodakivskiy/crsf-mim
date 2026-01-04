find_program(NPM_EXECUTABLE npm)

find_program(NPM_EXECUTABLE npm)

if(NPM_EXECUTABLE)
    # Check if package.json exists
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/panel/package.json
        AND NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/panel/dist-prod)
        # Run npm build
        execute_process(
            COMMAND ${NPM_EXECUTABLE} install
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/panel
            RESULT_VARIABLE NPM_INSTALL_RESULT
        )

        if(NPM_INSTALL_RESULT EQUAL 0)
            execute_process(
                COMMAND ${NPM_EXECUTABLE} run build:prod
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/panel
                RESULT_VARIABLE NPM_BUILD_RESULT
            )

            if(NOT NPM_BUILD_RESULT EQUAL 0)
                message(FATAL_ERROR "npm build failed")
            endif()
        else()
            message(FATAL_ERROR "npm install failed")
        endif()
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

spiffs_create_partition_image(www ${CMAKE_CURRENT_SOURCE_DIR}/panel/dist-prod FLASH_IN_PROJECT)
