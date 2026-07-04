package dev.ryanhcode.sable.physics.impl.box3d;

import java.nio.file.*;
import java.io.*;
import java.util.zip.*;

public final class Box3dNative {

    private static final String LIB_NAME = "box3d_jni";

    static {
        loadLibrary();
    }

    private static void loadLibrary() {
        try {
            System.loadLibrary(LIB_NAME);
        } catch (UnsatisfiedLinkError e) {

            // fallback (как у Rapier)
            System.load(
                    new java.io.File("src/main/resources/natives/sable_box3d/windows/box3d_x86_64_windows.dll")
                            .getAbsolutePath()
            );
        }
    }

    public static native long worldCreate();
    public static native void worldStep(long world, float dt);
    public static native void worldDestroy(long world);
}