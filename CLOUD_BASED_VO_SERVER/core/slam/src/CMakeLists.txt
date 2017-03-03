FIND_PACKAGE( OpenCV REQUIRED)

INCLUDE_DIRECTORIES(/usr/include/eigen3/)

ADD_LIBRARY(slamBase_server SHARED slamBase_server.cpp)
TARGET_LINK_LIBRARIES(slamBase_server
	${OpenCV_LIBS}
	${PCL_LIBRARIES} )

ADD_LIBRARY( robotJson_server SHARED robotJson_server.cpp )
TARGET_LINK_LIBRARIES( robotJson_server )


ADD_LIBRARY(slamInterface SHARED slamInterface.cpp)
TARGET_LINK_LIBRARIES(slamInterface
	robotJson_server
	slamBase_server
	${OpenCV_LIBS})
