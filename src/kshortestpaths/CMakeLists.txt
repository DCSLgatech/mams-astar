#CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
#PROJECT( kshortestpath )

# Set output directories for libraries and executables
#SET( BASE_DIR ${CMAKE_SOURCE_DIR} )
#SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
#SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
#SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASE_DIR}/bin )


ADD_LIBRARY(kshortestpaths DijkstraShortestPathAlg.cpp Graph.cpp YenTopKShortestPathsAlg.cpp)

ADD_EXECUTABLE(test_graph MainP.cpp)
TARGET_LINK_LIBRARIES(test_graph kshortestpaths)