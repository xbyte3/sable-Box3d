#pragma once
#include <jni.h>

#ifdef __cplusplus
extern "C" {
#endif

JNIEXPORT jlong JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_worldCreate
(JNIEnv*, jclass, jfloat gx, jfloat gy, jfloat gz);

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_worldStep
(JNIEnv*, jclass, jlong, jfloat, jint);

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_worldDestroy
(JNIEnv*, jclass, jlong);

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_worldSetGravity
(JNIEnv*, jclass, jlong, jfloat, jfloat, jfloat);

#ifdef __cplusplus
}
#endif