package dev.ryanhcode.sable.physics.impl.box3d.box;

import dev.ryanhcode.sable.api.physics.object.box.BoxHandle;
import dev.ryanhcode.sable.companion.math.Pose3d;
import dev.ryanhcode.sable.companion.math.Pose3dc;
import dev.ryanhcode.sable.physics.impl.box3d.Box3dJNI;
import org.joml.Quaterniondc;
import org.joml.Vector3dc;

public record Box3dBoxHandle(long world, long body, double[] poseCache) implements BoxHandle {

    public static Box3dBoxHandle create(final long worldHandle, final Pose3dc pose, final Vector3dc halfExtents, final float mass) {
        final Vector3dc pos = pose.position();
        final Quaterniondc rot = pose.orientation();

        final int id = Box3dJNI.nextBodyID();
        final long bodyHandle = Box3dJNI.createBox(worldHandle, mass, (float) halfExtents.x(), (float) halfExtents.y(), (float) halfExtents.z(), new float[]{(float) pos.x(), (float) pos.y(), (float) pos.z(), (float) rot.x(), (float) rot.y(), (float) rot.z(), (float) rot.w()});
        return new Box3dBoxHandle(worldHandle, bodyHandle, new double[7]);
    }

    /**
     * Queries the pose of the box from the physics engine
     */
    @Override
    public void readPose(final Pose3d dest) {
        Box3dJNI.getPose(this.body, this.poseCache);

        dest.position().set(this.poseCache[0], this.poseCache[1], this.poseCache[2]);
        dest.orientation().set(this.poseCache[3], this.poseCache[4], this.poseCache[5], this.poseCache[6]);
    }

    /**
     * Removes the box from the physics pipeline
     */
    @Override
    public void remove() {
        Box3dJNI.removeBox(this.body);
    }

    /**
     * Wakes up the box
     */
    @Override
    public void wakeUp() {
        Box3dJNI.wakeUpObject(this.body);
    }

    /**
     * @return the runtime ID of the box
     */
    @Override
    public int getRuntimeId() {
        return Math.toIntExact(this.body);
    }
}
