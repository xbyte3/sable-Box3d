package dev.ryanhcode.sable.physics.impl.box3d.collider;

import dev.ryanhcode.sable.api.block.BlockSubLevelCollisionShape;
import dev.ryanhcode.sable.api.physics.collider.SableCollisionContext;
import dev.ryanhcode.sable.physics.config.block_properties.PhysicsBlockPropertyHelper;
import dev.ryanhcode.sable.physics.impl.box3d.Box3dJNI;
import net.minecraft.core.BlockPos;
import net.minecraft.world.level.block.Blocks;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.phys.shapes.VoxelShape;
import org.joml.Vector3d;
import org.joml.Vector3dc;
import dev.ryanhcode.sable.physics.chunk.VoxelNeighborhoodState;


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
}