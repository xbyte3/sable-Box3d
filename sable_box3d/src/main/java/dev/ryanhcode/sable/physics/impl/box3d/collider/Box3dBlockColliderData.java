package dev.ryanhcode.sable.physics.impl.box3d.collider;

import dev.ryanhcode.sable.physics.impl.box3d.Box3dJNI;
import org.joml.Vector3d;
import org.joml.Vector3dc;


import javax.swing.*;
import java.util.ArrayList;
import java.util.List;

public class Box3dBlockColliderData {

    public static final Box3dBlockColliderData EMPTY =
            new Box3dBlockColliderData(0,0,0,false);

    public final List<Box> boxes = new ArrayList<>();

    public final double friction;
    public final double volume;
    public final double restitution;
    public final boolean liquid;

    // Индекс в нативном реестре Box3D (Box3dJNI.newVoxelCollider). -1, пока не зарегистрирован.
    private int handle = -1;

    public record Box(Vector3d min, Vector3d max) {}

    public Box3dBlockColliderData(double friction, double volume, double restitution, boolean liquid) {
        this.friction = friction;
        this.volume = volume;
        this.restitution = restitution;
        this.liquid = liquid;
    }

    public void addBox(Vector3dc min, Vector3dc max) {
        this.boxes.add(new Box(new Vector3d(min), new Vector3d(max)));
    }

    public int handle() {
        if (this.handle < 0) {
            throw new IllegalStateException("Collider not registered with native side yet");
        }
        return this.handle;
    }

    public void setHandle(int handle) {
        this.handle = handle;
    }

    private Box3dBlockColliderData build(BlockState state) {

        boolean liquid = VoxelNeighborhoodState.isLiquid(state);

        double friction = PhysicsBlockPropertyHelper.getFriction(state);
        double volume = PhysicsBlockPropertyHelper.getVolume(state);
        double restitution = PhysicsBlockPropertyHelper.getRestitution(state);

        Box3dBlockColliderData data = new Box3dBlockColliderData(
                friction,
                volume,
                restitution,
                liquid
        );

        if (liquid) {
            data.addBox(new Vector3d(0,0,0), new Vector3d(1,1,1));
            this.registerCollider(data);
            return data;
        }

        VoxelShape shape;

        this.level.setup(state);

        if (state.getBlock() instanceof BlockSubLevelCollisionShape ext) {
            shape = ext.getSubLevelCollisionShape(this.level, state);
        } else {
            shape = state.getCollisionShape(this.level, BlockPos.ZERO, SableCollisionContext.get());
        }

        this.level.setup(Blocks.AIR.defaultBlockState());

        if (shape.isEmpty()) {
            return Box3dBlockColliderData.EMPTY;
        }

        shape.forAllBoxes((minX, minY, minZ, maxX, maxY, maxZ) -> {
            data.addBox(
                    new Vector3d(Math.max(minX, 0.0), Math.max(minY, 0.0), Math.max(minZ, 0.0)),
                    new Vector3d(Math.min(maxX, 1.0), Math.min(maxY, 1.0), Math.min(maxZ, 1.0))
            );
        });

        this.registerCollider(data);
        return data;
    }

    /**
     * Регистрирует коллайдер в нативном реестре Box3D (аналог Rapier
     * newVoxelCollider/addVoxelColliderBox) и сохраняет полученный handle.
     */
    private void registerCollider(Box3dBlockColliderData data) {
        int handle = Box3dJNI.newVoxelCollider(data.friction, data.volume, data.restitution, data.liquid, false);
        data.setHandle(handle);

        for (Box3dBlockColliderData.Box box : data.boxes) {
            Box3dJNI.addVoxelColliderBox(handle, new double[]{
                    box.min().x(), box.min().y(), box.min().z(),
                    box.max().x(), box.max().y(), box.max().z()
            });
        }
    }
}