package dev.ryanhcode.sable.physics.impl.box3d;

import dev.ryanhcode.sable.Sable;
import dev.ryanhcode.sable.platform.SableLoaderPlatform;
import net.jpountz.lz4.LZ4FrameInputStream;

import java.nio.file.*;
import java.io.*;
import java.util.zip.*;

public final class Box3dJNI {

    private static final Path NATIVE_DIR = resolveNativeDir();
    private static final String LIB_ZIP_NAME = "box3d_binaries.zip.l4z";
    public static final String NATIVE_NAME = getNativeName();

    private static String getNativeName() {

        String arch = System.getProperty("os.arch");

        String a;
        if (arch.contains("aarch64") || arch.contains("arm")) {
            a = "aarch64";
        } else {
            a = "x86_64";
        }

        String os = System.getProperty("os.name").toLowerCase();

        if (os.contains("win")) {
            return "box3d_" + a + "_windows.dll";
        }

        if (os.contains("mac")) {
            return "box3d_" + a + "_macos.dylib";
        }

        return "box3d_" + a + "_linux.so";
    }

    private static Path resolveNativeDir() {
        final Path gameDir = SableLoaderPlatform.INSTANCE.getGameDirectory();
        if (gameDir != null) {
            final Path gameDirRelativeDir = gameDir.resolve(".sable").resolve("natives").normalize();
            Sable.LOGGER.info("Using game-dir-relative Box3d native directory {}", gameDirRelativeDir.toAbsolutePath());
            return gameDirRelativeDir;
        }

        final Path fallbackDir = Paths.get(System.getProperty("user.home", System.getProperty("user.dir")), ".sable", "natives");
        Sable.LOGGER.info("Using fallback Box3d native directory {}", fallbackDir.toAbsolutePath());
        return fallbackDir;
    }

    static {
        loadLibrary();
    }
    private static void loadLibrary() {
        try (InputStream is =
                     Box3dJNI.class.getResourceAsStream(
                             "/natives/box3d/" + LIB_ZIP_NAME)) {

            if (is == null)
                throw new FileNotFoundException(LIB_ZIP_NAME);

            Files.createDirectories(NATIVE_DIR);

            try (LZ4FrameInputStream lz4 = new LZ4FrameInputStream(is);
                 ZipInputStream zip = new ZipInputStream(lz4)) {

                ZipEntry entry;

                while ((entry = zip.getNextEntry()) != null) {

                    if (!entry.getName().equals(NATIVE_NAME))
                        continue;

                    Path out = NATIVE_DIR.resolve(NATIVE_NAME);

                    Files.copy(zip, out, StandardCopyOption.REPLACE_EXISTING);

                    System.load(out.toAbsolutePath().toString());
                    return;
                }

                throw new FileNotFoundException(NATIVE_NAME);
            }

        } catch (Exception e) {
            throw new RuntimeException("Failed to load Box3D natives", e);
        }
    }

    // World functions
    public static native long worldCreate(float gx, float gy, float gz);

    public static native long worldCreateDefault();

    public static native void worldDestroy(long worldHandle);

    public static native void worldSetGravity(long worldHandle, float x, float y, float z);

    public static native void worldStep(long worldHandle, float dt, int substeps);

    // Body functions
    public static native long bodyCreate(long worldHandle, float x, float y, float z);
    public static native void bodyDestroy(long worldHandle, long bodyHandle);
    public static native void bodyGetPose(long worldHandle, long bodyHandle, float[] pose);
    public static native void bodySetPose(long worldHandle, long bodyHandle, float[] pose);

    // Shape functions
    public static native void shapeCreateBox(long bodyHandle, float halfX, float halfY, float halfZ, float mass);

}