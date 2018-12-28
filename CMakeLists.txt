add_library(mv1 SHARED IMPORTED)
set_property(TARGET mv1 PROPERTY IMPORTED_LOCATION $ENV{MV_SDK}/usr/lib/libmv1.so)
set_property(TARGET mv1 PROPERTY INTERFACE_INCLUDE_DIRECTORIES $ENV{MV_SDK}/usr/include/mv)

px4_add_module(
	MODULE examples__snapdragon_mv
	MAIN snapdragon_mv
	STACK_MAIN 2400
	SRCS
		src/camera/SnapdragonCameraManager.cpp
		src/camera/SnapdragonCameraUtil.cpp
		src/vislam/SnapdragonVislamManager.cpp
		src/nodes/SnapdragonVislamPx4Main.cpp
	INCLUDES
		src/camera/
		src/imu/
		src/vislam/
		src/common/
	DEPENDS
		mv1
	)

target_link_libraries(examples__snapdragon_mv PRIVATE camera)
