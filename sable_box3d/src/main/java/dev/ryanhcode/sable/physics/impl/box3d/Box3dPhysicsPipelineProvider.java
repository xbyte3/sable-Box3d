package dev.ryanhcode.sable.physics.impl.box3d;

import dev.ryanhcode.sable.api.physics.PhysicsPipeline;
import dev.ryanhcode.sable.api.physics.PhysicsPipelineProvider;
import net.minecraft.server.level.ServerLevel;
import org.jetbrains.annotations.NotNull;

@PhysicsPipelineProvider.LoadPriority(1500)
public final class Box3dPhysicsPipelineProvider implements PhysicsPipelineProvider {

    @Override
    public @NotNull PhysicsPipeline createPipeline(@NotNull final ServerLevel level) {
        return new Box3dPhysicsPipeline(level);
    }
}
