#pragma once

#include <jni.h>

#ifdef __cplusplus
extern "C" {
#endif

JNIEXPORT jlong JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3DJNI_worldCreate
  (JNIEnv*, jclass);

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3DJNI_worldDestroy
  (JNIEnv*, jclass, jlong);

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3DJNI_worldStep
  (JNIEnv*, jclass, jlong, jfloat);

#ifdef __cplusplus
}
#endif