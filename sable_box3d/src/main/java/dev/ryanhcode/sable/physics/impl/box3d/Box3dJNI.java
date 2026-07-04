package dev.ryanhcode.sable.physics.impl.box3d;

public final class Box3dJNI {

    static {
        System.loadLibrary("box3d_jni");
    }

    // World functions
    public static native long worldCreate();
    public static native void worldDestroy(long worldHandle);
    public static native void worldSetGravity(long worldHandle, float x, float y, float z);
    public static native void worldStep(long worldHandle, float timeStep);

    // Body functions
    public static native long bodyCreate(long worldHandle, float x, float y, float z);
    public static native void bodyDestroy(long worldHandle, long bodyHandle);
    public static native void bodyGetPose(long worldHandle, long bodyHandle, double[] poseArray);
    public static native void bodySetPose(long worldHandle, long bodyHandle, double[] poseArray);

    // Shape functions
    public static native void shapeCreateBox(long bodyHandle, float halfX, float halfY, float halfZ, float mass);
}