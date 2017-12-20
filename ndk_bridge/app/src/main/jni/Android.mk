LOCAL_PATH := $(call my-dir)
PROJECT_ROOT:= $(call my-dir)/../../../../..

include $(CLEAR_VARS)
LOCAL_MODULE    := chatter_bot
LOCAL_SRC_FILES := src/chatter_bot.cpp
LOCAL_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_LDLIBS := -landroid
LOCAL_STATIC_LIBRARIES := android_native_app_glue roscpp_android_ndk
include $(BUILD_SHARED_LIBRARY)

$(call import-add-path, $(PROJECT_ROOT) )
$(call import-module, android/native_app_glue)
$(call import-module, roscpp_android_ndk)