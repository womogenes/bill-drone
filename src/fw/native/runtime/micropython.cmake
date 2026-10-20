# Our code gets its own archive so flight.lf can link it last. ldgen orders
# archives by name, hence "zflight". Being last matters: the Xtensa linker
# shares identical literals with earlier code, so anything after our code
# would change size whenever our constants did.
set(flight_sources
    ${CMAKE_CURRENT_LIST_DIR}/flight_rt.c
    ${CMAKE_CURRENT_LIST_DIR}/radio.c
    ${CMAKE_CURRENT_LIST_DIR}/settings.c
    ${CMAKE_CURRENT_LIST_DIR}/../flight_native.c
)
list(APPEND MICROPY_SOURCE_USERMOD ${flight_sources})
list(APPEND MICROPY_LDFRAGMENTS ${CMAKE_CURRENT_LIST_DIR}/flight.lf)

# Deferred until the main component exists, so the archive gets its IDF flags.
function(flight_library)
    add_library(zflight STATIC ${flight_sources})
    # Keep strings in our own rodata rather than the image-wide merged pool.
    target_compile_options(zflight PRIVATE -fno-merge-constants)
    target_link_libraries(zflight PRIVATE ${MICROPY_TARGET})
    # Link it into the ELF rather than main, so main's generated headers, which
    # these sources include, do not wait on this archive.
    cmake_language(DEFER DIRECTORY ${CMAKE_SOURCE_DIR}
        CALL target_link_libraries ${CMAKE_PROJECT_NAME}.elf PRIVATE zflight)
endfunction()
cmake_language(DEFER CALL flight_library)
