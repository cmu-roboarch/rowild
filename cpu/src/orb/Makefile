INC_DIRS := ../../include /usr/include/opencv4
SRC_DIRS := ./
OBJ_DIR := ./obj
BIN_DIR := .
EXE_NAME := orb.out

include ../../makefile.rules

LDFLAGS += -L/usr/local/lib -fopenmp -L/usr/local/lib -lopencv_stitching -lopencv_highgui -lopencv_imgcodecs -lopencv_dnn -lopencv_imgproc -lopencv_core -lopencv_features2d -lopencv_calib3d -o orb.out
