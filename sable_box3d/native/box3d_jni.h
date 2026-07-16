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
    (JNIEnv* env, jclass, jlong bodyHandle, jdoubleArray outArray);

JNIEXPORT jint JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_newVoxelCollider
    (JNIEnv*, jclass, jdouble friction, jdouble volume, jdouble restitution, jboolean isFluid, jboolean dynamic);

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_addVoxelColliderBox
    (JNIEnv*, jclass, jint index, jdoubleArray boxBounds);

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_clearVoxelColliderBoxes
    (JNIEnv*, jclass, jint index);

JNIEXPORT jlong JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_createSubLevel
    (JNIEnv*, jclass, jlong world, jint id, jdoubleArray pose);

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_removeSubLevel
    (JNIEnv*, jclass, jlong world, jint id);

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_getLinearVelocity
    (JNIEnv*, jclass, jlong world, jint id, jdoubleArray out);

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_getAngularVelocity
    (JNIEnv*, jclass, jlong world, jint id, jdoubleArray out);

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_addChunk
    (JNIEnv*, jclass, jlong world, jint x, jint y, jint z, jintArray data, jboolean global, jint object_id);

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_removeChunk
    (JNIEnv*, jclass, jlong world, jint x, jint y, jint z, jboolean global);

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_changeBlock
(JNIEnv*, jclass, jlong world, jint x, jint y, jint z, jint packedBlock);

// SHAPE
JNIEXPORT jlong JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_createBox
    (JNIEnv*, jclass, jlong world, jfloat mass, jfloat hx, jfloat hy, jfloat hz, jfloatArray pose);

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_wakeUpObject(JNIEnv*, jclass, jlong world, jint objectId);

#ifdef __cplusplus
}
#endif