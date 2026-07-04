#include "box3d_jni.h"
#include "include/box3d.h"

struct WorldHolder {
    b3WorldId world;
};

JNIEXPORT jlong JNICALL
Java_dev_ryanhcode_sable_physics_impl_box3d_Box3DJNI_worldCreate
(JNIEnv*, jclass, jfloat gx, jfloat gy, jfloat gz)
{
    WorldHolder* holder = new WorldHolder();

    b3WorldDef def = b3DefaultWorldDef();
    def.gravity = { gx, gy, gz };

    holder->world = b3CreateWorld(&def);

    return reinterpret_cast<jlong>(holder);
}

JNIEXPORT void JNICALL
Java_dev_ryanhcode_sable_physics_impl_box3d_Box3DJNI_worldStep
(JNIEnv*, jclass, jlong ptr, jfloat dt, jint substeps)
{
    auto* holder = reinterpret_cast<WorldHolder*>(ptr);

    b3World_Step(holder->world, dt, substeps);
}

JNIEXPORT void JNICALL
Java_dev_ryanhcode_sable_physics_impl_box3d_Box3DJNI_worldDestroy
(JNIEnv*, jclass, jlong ptr)
{
    auto* holder = reinterpret_cast<WorldHolder*>(ptr);

    b3DestroyWorld(holder->world);

    delete holder;
}