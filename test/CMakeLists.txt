
set (SOURCES
#        main
        TBlock.onetraj
        TBlock.multitraj
        )
aux_source_directory(../ POJ_SRC)

foreach (test ${SOURCES})
    add_executable(test-${test} ./${test}.cpp ${POJ_SRC})
    target_link_libraries(test-${test} TBlock)
#    target_link_libraries(test-${test} sqlite3_lib)
endforeach()