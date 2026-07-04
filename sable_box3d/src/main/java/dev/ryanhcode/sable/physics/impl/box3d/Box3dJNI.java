package dev.ryanhcode.sable.physics.impl.box3d;

public final class Box3DJNI {

    static {
        System.loadLibrary("box3d_jni");
    }

    // World
    public static native long worldCreate();
    public static native void worldDestroy(long world);

    public static native void worldSetGravity(long world, float gx, float gy, float gz);
    public static native void worldStep(long world, float dt);

    // Body
    public static native long bodyCreateBox(long world, float x, float y, float z);
    public static native void bodyDestroy(long world, long body);

    public static native void bodySetTransform(long body, float x, float y, float z);
    public static native void bodyGetTransform(long body, float[] out16); // mat4 or pos+rot
}