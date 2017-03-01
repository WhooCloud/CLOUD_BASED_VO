FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization filters)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

FIND_PACKAGE( OpenCV REQUIRED)

ADD_DEFINITIONS(${PCL_DEFINITIONS})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_LIBRARIES(${PCL_LIBRARY_DIRS})

ADD_LIBRARY(slamBase SHARED slamBase.cpp)
TARGET_LINK_LIBRARIES(slamBase
	${OpenCV_LIBS}
	${PCL_LIBRARIES} )

ADD_LIBRARY( robotJson SHARED robotJson.cpp )
TARGET_LINK_LIBRARIES( robotJson )

ADD_LIBRARY( robotWS SHARED robotWS.cpp )
TARGET_LINK_LIBRARIES( robotWS
	boost_system
	crypto
	ssl
	cpprest)

ADD_LIBRARY(slamInterface SHARED slamInterface.cpp)
TARGET_LINK_LIBRARIES(slamInterface
	robotJson
	robotWS
	slamBase
	${OpenCV_LIBS})
