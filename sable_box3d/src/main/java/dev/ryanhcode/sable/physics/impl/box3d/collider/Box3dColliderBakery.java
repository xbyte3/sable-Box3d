package dev.ryanhcode.sable.physics.impl.box3d.collider;

import dev.ryanhcode.sable.api.block.BlockSubLevelCollisionShape;
import dev.ryanhcode.sable.api.physics.collider.SableCollisionContext;
import dev.ryanhcode.sable.physics.chunk.VoxelNeighborhoodState;
import dev.ryanhcode.sable.physics.config.block_properties.PhysicsBlockPropertyHelper;
import net.minecraft.Util;
import net.minecraft.core.BlockPos;
import net.minecraft.world.level.BlockGetter;
import net.minecraft.world.level.block.Blocks;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.phys.shapes.VoxelShape;
import org.joml.Vector3d;

import javax.annotation.Nullable;
import java.util.function.Function;

public class Box3dColliderBakery {

    private final PhysicsColliderBlockGetter level;

    private final Function<BlockState, Box3dBlockColliderData> cache =
            Util.memoize(this::build);

    public Box3dColliderBakery(BlockGetter blockGetter) {
        this.level = new PhysicsColliderBlockGetter(blockGetter);
    }

    public BlockGetter getLevel() {
        return this.level;
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

        return data;
    }

    public @Nullable Box3dBlockColliderData get(BlockState state) {
        final Box3dBlockColliderData data = this.cache.apply(state);
        return data == Box3dBlockColliderData.EMPTY ? null : data;
    }
}