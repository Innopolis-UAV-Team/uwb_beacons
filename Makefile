ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
BUILD_DIR:=$(ROOT_DIR)/build

# Define a variable for ID
ID ?= 1  # You can provide a default ID if needed
TARGET?=anchor
BAUDRATE?=230400
export CALIBRATE?=0
export PLATFORM?=stm32

# .DELETE_ON_ERROR:
.PHONY: ubuntu calibrate

calibrate: CALIBRATE = 1
calibrate:
	@echo "Calibrating"

ubuntu: PLATFORM = ubuntu
ubuntu:
	@echo "Building on Ubuntu"

anchor-upload: anchor
	$(MAKE) TARGET=anchor upload

router-upload: router
	$(MAKE) TARGET=router upload

# calibration is not implemented for anchor yet
anchor router:
	@echo "Building $@"
	rm -fR ${BUILD_DIR}/$@
	mkdir -p ${BUILD_DIR}/$@/obj
	cd ${BUILD_DIR}/$@ && cmake -G "Unix Makefiles" -DTYPE=$@ -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DBAUDRATE=${BAUDRATE} -DCALIBRATE=${CALIBRATE} -DID=${ID} -DPLATFORM=${PLATFORM} ../.. && make

upload:
	st-flash write ${BUILD_DIR}/$(TARGET)/obj/${TARGET}.bin 0x08000000
