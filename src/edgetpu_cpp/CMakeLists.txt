cmake_minimum_required(VERSION 3.10.2)
project(edgetpu_cpp)

find_package(catkin REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(glog 0.4.0 REQUIRED)

catkin_package(
	INCLUDE_DIRS ${PROJECT_NAME}/src
	LIBRARIES classification detection util posenet basic swig)

include_directories(src ${CATKIN_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(flatbuffers /usr/local/include/flatbuffers)
include_directories(benchmark /usr/local/include/benchmark)
include_directories(dep/libedgetpu)

add_subdirectory(dep/abseil-cpp)

add_library(classification
	edgetpu_cpp/src/classification/engine.cc)
target_link_libraries(classification absl::base absl::flags absl::strings)

add_library(detection
	edgetpu_cpp/src/detection/engine.cc)
target_link_libraries(detection absl::base absl::flags absl::strings)

add_library(util STATIC
	edgetpu_cpp/src/examples/label_utils.cc
	edgetpu_cpp/src/examples/model_utils.cc
	edgetpu_cpp/src/fake_op.cc
	edgetpu_cpp/src/version.cc
	edgetpu_cpp/src/version_test.cc
	edgetpu_cpp/src/error_reporter.cc
	edgetpu_cpp/src/test_utils.cc
	edgetpu_cpp/src/utils.cc)
target_link_libraries(util classification detection
	${CMAKE_SOURCE_DIR}/edgetpu_cpp/dep/libedgetpu/tensorflow/lite/tools/make/gen/linux_x86_64/lib/libtensorflow-lite.a
	 glog::glog absl::base absl::flags absl::strings)

add_library(posenet STATIC
	edgetpu_cpp/src/posenet/posenet_decoder.cc
	edgetpu_cpp/src/posenet/posenet_decoder_op.cc
	edgetpu_cpp/src/posenet/posenet_decoder_tflite_plugin.cc)
target_link_libraries(posenet util flatbuffers absl::base absl::flags absl::strings)

add_library(basic STATIC
	edgetpu_cpp/src/basic/edgetpu_resource_manager.cc
	edgetpu_cpp/src/basic/basic_engine.cc
	edgetpu_cpp/src/basic/basic_engine_native.cc)
target_link_libraries(basic 
	${CMAKE_SOURCE_DIR}/edgetpu_cpp/dep/libedgetpu/tensorflow/lite/tools/make/gen/linux_x86_64/lib/libtensorflow-lite.a
	${CMAKE_SOURCE_DIR}/edgetpu_cpp/dep/libedgetpu/direct/k8/libedgetpu.so.1
        util posenet absl::base absl::flags absl::strings)

add_library(swig STATIC
	edgetpu_cpp/src/swig/basic_engine_python_wrapper.cc)
target_link_libraries(swig ${PYTHON_LIBRARIES} absl::base absl::flags absl::strings)

install(TARGETS util basic detection
  	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY ${PROJECT_NAME}/src
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".cc" EXCLUDE
)
