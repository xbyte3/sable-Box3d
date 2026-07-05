#include "box3d_jni.h"
#include "include/box3d.h"

static inline b3WorldId toWorld(jlong handle)
{
    return b3LoadWorldId((uint32_t)handle);
}

JNIEXPORT jlong JNICALL
Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_worldCreate
(JNIEnv*, jclass, jfloat gx, jfloat gy, jfloat gz)
{
    b3WorldDef def = b3DefaultWorldDef();
    def.gravity = { gx, gy, gz };

    b3WorldId world = b3CreateWorld(&def);

    return (jlong)b3StoreWorldId(world);
}

JNIEXPORT void JNICALL
Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_worldStep
(JNIEnv*, jclass, jlong worldHandle, jfloat dt, jint substeps)
{
    b3World_Step(toWorld(worldHandle), dt, substeps);
}

JNIEXPORT void JNICALL
Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_worldDestroy
(JNIEnv*, jclass, jlong worldHandle)
{
    b3DestroyWorld(toWorld(worldHandle));
}

JNIEXPORT void JNICALL
Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_worldSetGravity
(JNIEnv*, jclass, jlong worldHandle, jfloat gx, jfloat gy, jfloat gz)
{
    b3Vec3 gravity = { gx, gy, gz };
    b3World_SetGravity(toWorld(worldHandle), gravity);
}