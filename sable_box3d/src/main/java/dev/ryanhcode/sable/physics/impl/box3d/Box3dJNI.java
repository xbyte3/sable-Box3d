package dev.ryanhcode.sable.physics.impl.box3d;

import dev.ryanhcode.sable.Sable;
import dev.ryanhcode.sable.platform.SableLoaderPlatform;
import net.jpountz.lz4.LZ4FrameInputStream;
import org.jetbrains.annotations.ApiStatus;

import java.nio.file.*;
import java.io.*;
import java.util.zip.*;

public final class Box3dJNI {
    public static final String NATIVE_NAME = getNativeName();
    private static final Path NATIVE_DIR = resolveNativeDir();
    private static final String LIB_ZIP_NAME = "box3d_binaries.zip.l4z";
    private static int countingObjectID = 0;

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

    @ApiStatus.Internal
    public static synchronized int nextBodyID() {
        return countingObjectID++;
    }

    public static native long worldCreate(float gx, float gy, float gz);
    public static native void worldDestroy(long worldHandle);
    public static native void worldStep(long worldHandle, float dt, int substeps);

    public static native long createSubLevel(long worldHandle, int id, double[] pose);
    public static native void removeSubLevel(long worldHandle, int id);

    public static native void addChunk(long worldHandle, int x, int y, int z, int[] data, boolean global, int objectId);
    public static native void removeChunk(long worldHandle, int x, int y, int z, boolean global);

    /**
     * All poses are formatted in a double array as:
     * [x, y, z, qx, qy, qz, qw]
     */
    @ApiStatus.Internal
    public static native long createBox(final long worldHandle, float mass, float halfExtentsX, float halfExtentsY, float halfExtentsZ, float[] pose);

    @ApiStatus.Internal
    public static native void removeBox(final long body);

    /**
     * Gets the pose of an body.
     *
     * @param body    the body pointer
     * @param store The array to store pose of the body in the format [x, y, z, qx, qy, qz, qw]
     */
    @ApiStatus.Internal
    public static native void getPose(final long body, double[] store);

    /**
     * "Wakes up" an object, indicating environmental or other changes have occurred that should resume physics if idled or sleeping
     *
     * @param body the object ID
     */
    @ApiStatus.Internal
    public static native void wakeUpObject(final long body);
}