# If Qt+OpenGL dependencies were found, descend into these directories
if(ARGOS_QTOPENGL_FOUND)
  	add_subdirectory(demoC)
  	add_subdirectory(clustering)
    add_subdirectory(BestN)
endif(ARGOS_QTOPENGL_FOUND)
