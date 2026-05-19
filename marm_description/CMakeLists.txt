cmake_minimum_required(VERSION 3.0.2)
project(marm_description)

find_package(catkin REQUIRED)
catkin_package()


find_package(roslaunch REQUIRED)  

# 安装所有资源目录到share/marm_description
foreach(dir config launch meshes urdf rviz)
	if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
		install(DIRECTORY ${dir}/
			DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
	endif()
endforeach()
