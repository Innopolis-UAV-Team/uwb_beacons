ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
BUILD_DIR:=$(ROOT_DIR)/build

# Define a variable for ID
ID ?= 1  # You can provide a default ID if needed

anchor:
	rm -fR ${BUILD_DIR}/anchor
	mkdir -p ${BUILD_DIR}/anchor/obj
	cd ${BUILD_DIR}/anchor && cmake -G "Unix Makefiles" -DUSE_ANCHOR=ON -DANCHOR_ID=${ID} ../.. && make

router:
	rm -fR ${BUILD_DIR}/router
	mkdir -p ${BUILD_DIR}/router/obj
	cd ${BUILD_DIR}/router && cmake -G "Unix Makefiles" -DUSE_ANCHOR=OFF ../.. && make
