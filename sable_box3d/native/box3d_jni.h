#pragma once

#include <jni.h>
#include "include/box3d.h"

#ifdef __cplusplus
extern "C" {
#endif

JNIEXPORT jlong JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_worldCreate
    (JNIEnv*, jclass, jfloat gx, jfloat gy, jfloat gz);

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_worldDestroy
    (JNIEnv*, jclass, jlong world);

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_worldStep
    (JNIEnv*, jclass, jlong world, jfloat dt, jint substeps);

// BODY

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_getPose
    (JNIEnv* env, jclass, jlong bodyHandle, jfloatArray outArray);

JNIEXPORT jlong JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_createSublevel
    (JNIEnv*, jclass, jlong world, jfloatArray pose);

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_removeSublevel
    (JNIEnv*, jclass, jlong body);

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_addChunk
    (JNIEnv*, jclass, jlong world, jint x, jint y, jint z, jintArray data, jboolean global, jint object_id);

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_removeChunk
    (JNIEnv*, jclass, jlong world, jint x, jint y, jint z, jboolean global);

// SHAPE
JNIEXPORT jlong JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_createBox
    (JNIEnv*, jclass, jlong world, jfloat mass, jfloat hx, jfloat hy, jfloat hz, jfloatArray pose);

#ifdef __cplusplus
}
#endif