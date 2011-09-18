ifeq ($(BUILD_LIB_HTC_ACOUSTIC_WINCE),true)

LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES := libacoustic.c

LOCAL_MODULE := libhtc_acoustic

LOCAL_SHARED_LIBRARIES := libc libcutils libm liblog libdl

LOCAL_ARM_MODE := arm

LOCAL_MODULE_TAGS := optional

include $(BUILD_SHARED_LIBRARY)

endif

