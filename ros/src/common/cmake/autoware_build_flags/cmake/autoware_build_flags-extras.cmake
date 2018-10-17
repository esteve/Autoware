if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-undefined,error")
endif()

# Enable support for C++11
if(${CMAKE_VERSION} VERSION_LESS "3.1.0")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wall")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Werror -Wall")
else()
  set(CMAKE_CXX_STANDARD 11)
  add_compile_options(-Werror -Wall)
endif()
