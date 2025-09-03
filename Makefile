ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
BUILD_DIR:=$(ROOT_DIR)/build

# Define a variable for ID
ID ?= 1  # You can provide a default ID if needed
TARGET?=anchor
BAUDRATE?=230400

anchor-upload: anchor
	$(MAKE) TARGET=anchor upload

router-upload: router
	$(MAKE) TARGET=router upload

anchor: 
	rm -fR ${BUILD_DIR}/anchor
	mkdir -p ${BUILD_DIR}/anchor/obj
	cd ${BUILD_DIR}/anchor && cmake -G "Unix Makefiles" -DUSE_ANCHOR=ON -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DANCHOR_ID=${ID} ../.. && make

router:
	rm -fR ${BUILD_DIR}/router
	mkdir -p ${BUILD_DIR}/router/obj
	cd ${BUILD_DIR}/router && cmake -G "Unix Makefiles" -DUSE_ANCHOR=OFF -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DBAUDRATE=${BAUDRATE} ../.. && make

upload:
	st-flash write ${BUILD_DIR}/$(TARGET)/obj/${TARGET}.bin 0x08000000
