package dev.ryanhcode.sable.physics.impl.box3d;

import dev.ryanhcode.sable.api.physics.PhysicsPipeline;

import dev.ryanhcode.sable.api.physics.PhysicsPipelineBody;
import dev.ryanhcode.sable.api.physics.constraint.PhysicsConstraintConfiguration;
import dev.ryanhcode.sable.api.physics.constraint.PhysicsConstraintHandle;
import dev.ryanhcode.sable.api.physics.object.box.BoxHandle;
import dev.ryanhcode.sable.api.physics.object.box.BoxPhysicsObject;
import dev.ryanhcode.sable.api.physics.object.rope.RopeHandle;
import dev.ryanhcode.sable.api.physics.object.rope.RopePhysicsObject;
import dev.ryanhcode.sable.api.sublevel.KinematicContraption;
import dev.ryanhcode.sable.api.sublevel.SubLevelContainer;
import dev.ryanhcode.sable.companion.math.Pose3d;
import dev.ryanhcode.sable.companion.math.Pose3dc;
import dev.ryanhcode.sable.physics.chunk.VoxelNeighborhoodState;
import dev.ryanhcode.sable.physics.config.PhysicsConfigData;
import dev.ryanhcode.sable.physics.impl.box3d.collider.Box3dBlockColliderData;
import dev.ryanhcode.sable.physics.impl.box3d.collider.Box3dColliderBakery;
import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import dev.ryanhcode.sable.sublevel.plot.LevelPlot;
import dev.ryanhcode.sable.sublevel.system.SubLevelPhysicsSystem;
import dev.ryanhcode.sable.util.LevelAccelerator;
import it.unimi.dsi.fastutil.ints.Int2ObjectArrayMap;
import it.unimi.dsi.fastutil.ints.Int2ObjectMap;
import it.unimi.dsi.fastutil.longs.Long2LongOpenHashMap;
import it.unimi.dsi.fastutil.objects.ReferenceArrayList;
import it.unimi.dsi.fastutil.objects.ReferenceList;
import net.minecraft.core.BlockPos;
import net.minecraft.core.Direction;
import net.minecraft.core.SectionPos;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.level.block.Blocks;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.level.chunk.LevelChunk;
import net.minecraft.world.level.chunk.LevelChunkSection;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.joml.Quaterniondc;
import org.joml.Vector3d;
import org.joml.Vector3dc;

import java.util.HashMap;
import java.util.Map;

public class Box3dPhysicsPipeline implements PhysicsPipeline {
    private static final Logger log = LogManager.getLogger(Box3dPhysicsPipeline.class);
    private final ServerLevel level;
    private final LevelAccelerator accelerator;
    private final Int2ObjectMap<ServerSubLevel> activeSubLevels = new Int2ObjectArrayMap<>();
    private final ReferenceList<PhysicsPipelineBody> queuedWakeUps = new ReferenceArrayList<>();
    private long worldHandle = 0;  // JNI handle
    private final double[] poseCache;
    private final Box3dColliderBakery colliderBakery;
    private static int nextBodyID = 0;
    private final Map<Integer, Long> bodies = new HashMap<>();  // ID -> JNI body handle

    private final Long2LongOpenHashMap recentCollisions = new Long2LongOpenHashMap();

    public Box3dPhysicsPipeline(final ServerLevel level) {
        this.level = level;
        this.accelerator = new LevelAccelerator(level);
        this.colliderBakery = new Box3dColliderBakery(this.accelerator);
        this.recentCollisions.defaultReturnValue(-1);
        this.poseCache = new double[7];
    }

    public long getBodyID(final int id) {
        final Long value = this.bodies.get(id);

        if (value == null) {
            throw new IllegalStateException("Body not found: " + id);
        }

        return value;
    }

    public void addBodyID(final int id, final long body) {
        this.bodies.put(id, body);
    }

    @Override
    public void init(Vector3dc gravity, final double universalDrag) {
        if (gravity == null) {
            gravity = new Vector3d(0, -9.81, 0);
        }

        this.worldHandle = Box3dJNI.worldCreate(
                (float) gravity.x(),
                (float) gravity.y(),
                (float) gravity.z());

        log.info("Hi! {}", this.worldHandle);
    }

    @Override
    public void dispose() {
        if (this.worldHandle == 0) {
            return;
        }

        Box3dJNI.worldDestroy(this.worldHandle);
        this.worldHandle = 0;
    }

    @Override
    public void physicsTick(final double timeStep) {
        Box3dJNI.worldStep(this.worldHandle, (float) timeStep, 1);
    }

    /**
     * Called after all physics substeps have been run, to finalize the physics tick.
     */
    @Override
    public void postPhysicsTicks() {
        //this.processCollisionEffects();
    }

    /**
     * Runs a tick to update any separate sub-level tracking / logic, even if physics is currently paused
     */
    @Override
    public void tick() {
        this.accelerator.clearCache();
    }

    @Override
    public void add(final ServerSubLevel subLevel, final Pose3dc pose) {
        this.assertBodyValid(subLevel);
        final Vector3dc pos = pose.position();
        final Quaterniondc rot = pose.orientation();

        final long body = Box3dJNI.createSubLevel(this.worldHandle, subLevel.getRuntimeId(), new double[]{pos.x(), pos.y(), pos.z(), rot.x(), rot.y(), rot.z(), rot.w()});
        this.addBodyID(subLevel.getRuntimeId(), body);

        subLevel.updateMergedMassData(1.0f);
        final Vector3dc centerOfMass = subLevel.getMassTracker().getCenterOfMass();

        if (centerOfMass != null) {
            subLevel.logicalPose().rotationPoint().set(centerOfMass);

            this.onStatsChanged(subLevel);
        }

        this.activeSubLevels.put(subLevel.getRuntimeId(), subLevel);
    }

    /**
     * Removes a {@link ServerSubLevel} from the physics pipeline.
     */
    @Override
    public void remove(final ServerSubLevel subLevel) {
        Box3dJNI.removeSubLevel(this.worldHandle, subLevel.getRuntimeId());
        this.activeSubLevels.remove(subLevel.getRuntimeId());
    }

    @Override
    public void add(KinematicContraption contraption) {

    }

    @Override
    public void remove(KinematicContraption contraption) {

    }

    private void assertBodyValid(final PhysicsPipelineBody body) {
        if (body.isRemoved()) {
            throw new RuntimeException("Body has been removed");
        }
    }

    @Override
    public Pose3d readPose(ServerSubLevel subLevel, Pose3d dest) {
        this.assertBodyValid(subLevel);
        final long body = this.getBodyID(subLevel.getRuntimeId());

        Box3dJNI.getPose(body, this.poseCache);

        dest.position().set(this.poseCache[0], this.poseCache[1], this.poseCache[2]);
        dest.orientation().set(this.poseCache[3], this.poseCache[4], this.poseCache[5], this.poseCache[6]);

        return dest;
    }

    @Override
    public RopeHandle addRope(RopePhysicsObject rope) {
        return null;
    }

    @Override
    public BoxHandle addBox(final BoxPhysicsObject boxPhysicsObject) {
        return null; //Box3dBoxHandle.create(this.worldHandle, boxPhysicsObject.getPose(), boxPhysicsObject.getHalfExtents(), (float) boxPhysicsObject.getMass());
    }

    /**
     * Handles the addition of a chunk section to the physics context
     */
    @Override
    public void handleChunkSectionAddition(final LevelChunkSection section, final int x, final int y, final int z, final boolean uploadDataIfGlobal) {
        this.accelerator.clearCache();

        final int[] array = new int[LevelChunkSection.SECTION_SIZE];
        final SectionPos sectionPos = SectionPos.of(x, y, z);

        if (!section.hasOnlyAir()) {
            final LevelChunk chunk = this.accelerator.getChunk(x, z);

            for (int bx = 0; bx < 16; bx++) {
                for (int bz = 0; bz < 16; bz++) {
                    for (int by = 0; by < 16; by++) {
                        final BlockPos globalPos = new BlockPos(bx, by, bz).offset(sectionPos.minBlockX(), sectionPos.minBlockY(), sectionPos.minBlockZ());
                        final BlockState blockState = this.accelerator.getBlockState(globalPos);

                        final VoxelNeighborhoodState nState = VoxelNeighborhoodState.getState(this.accelerator, globalPos, chunk);
                        final Box3dBlockColliderData colliderData = this.colliderBakery.get(blockState);

                        final int colliderId = colliderData == null ? 0 : colliderData.handle() + 1;
                        final int index = bx + (bz << 4) + (by << 8);

                        array[index] = packBlockState(nState, colliderId);
                    }
                }
            }
        }

        final LevelPlot plot = SubLevelContainer.getContainer(this.level).getPlot(x, z); // x, z здесь уже координаты чанка
        final boolean global = plot == null;
        int id = -1;

        int sendX = x, sendY = y, sendZ = z;

        if (plot != null) {
            final BlockPos center = plot.getCenterBlock();
            sendX = x - (center.getX() >> 4);
            sendY = y - (center.getY() >> 4);
            sendZ = z - (center.getZ() >> 4);

            id = ((ServerSubLevel) plot.getSubLevel()).getRuntimeId(); // всегда, без if (uploadDataIfGlobal)
        }

        Box3dJNI.addChunk(this.worldHandle, sendX, sendY, sendZ, array, global, id);
    }

    /**
     * Handles the removal of a chunk section from the physics context
     */
    @Override
    public void handleChunkSectionRemoval(final int x, final int y, final int z) {
        Box3dJNI.removeChunk(this.worldHandle, x, y, z, !SubLevelContainer.getContainer(this.level).inBounds(x, z));
    }

    /**
     * Упаковывает данные о блоке в формат int, ожидаемый Box3dJNI.addChunk на
     * C++ стороне: нижние 16 бит — индекс VoxelPhysicsState (см. C++
     * ALL_VOXEL_PHYSICS_STATES: Empty=0, Face=1, Edge=2, Corner=3, Interior=4),
     * верхние 16 бит — collider id (0 = нет коллизии).
     */
    private static int packBlockState(final VoxelNeighborhoodState state, final int colliderID) {
        return ((int) state.byteRepresentation()) | (colliderID << 16);
    }

    @Override
    public void handleBlockChange(final SectionPos sectionPos, final LevelChunkSection chunk, int x, int y, int z, final BlockState oldState, final BlockState newState) {
        x = (sectionPos.x() << 4) + x;
        y = (sectionPos.y() << 4) + y;
        z = (sectionPos.z() << 4) + z;

        final BlockPos globalBlockPos = new BlockPos(x, y, z);

        // Соседние блоки могут сменить VoxelNeighborhoodState (Face/Edge/Corner/
        // Interior) при изменении этого блока — их коллизию тоже нужно обновить,
        // как и в Rapier-версии.
        for (final Direction dir : Direction.values()) {
            this.updateBlockPhysics(globalBlockPos.relative(dir));
        }

        this.updateBlockPhysics(globalBlockPos);
    }

    @Override
    public void onStatsChanged(@NotNull ServerSubLevel serverSubLevel) {
        //PhysicsPipeline.super.onStatsChanged(serverSubLevel);
    }

    private void updateBlockPhysics(final BlockPos pos) {
        final VoxelNeighborhoodState state = VoxelNeighborhoodState.getState(this.accelerator, pos, null);
        final BlockState blockState = this.accelerator.getBlockState(pos);
        final Box3dBlockColliderData colliderData = this.colliderBakery.get(blockState);
        final int colliderId = colliderData == null ? 0 : colliderData.handle() + 1;

        final LevelPlot plot = SubLevelContainer.getContainer(this.level).getPlot(pos.getX() >> 4, pos.getZ() >> 4);

        int sendX = pos.getX(), sendY = pos.getY(), sendZ = pos.getZ();

        if (plot != null) {
            final BlockPos center = plot.getCenterBlock();
            sendX -= center.getX();
            sendY -= center.getY();
            sendZ -= center.getZ();
        }

        Box3dJNI.changeBlock(this.worldHandle, sendX, sendY, sendZ, packBlockState(state, colliderId));
    }

    @Override
    public void teleport(PhysicsPipelineBody body, Vector3dc position, Quaterniondc orientation) {

    }

    @Override
    public void applyImpulse(PhysicsPipelineBody body, Vector3dc position, Vector3dc force) {

    }

    @Override
    public void applyLinearAndAngularImpulse(PhysicsPipelineBody body, Vector3dc force, Vector3dc torque, boolean wakeUp) {

    }

    @Override
    public void addLinearAndAngularVelocity(PhysicsPipelineBody body, Vector3dc linearVelocity, Vector3dc angularVelocity) {
        //PhysicsPipeline.super.addLinearAndAngularVelocity(body, linearVelocity, angularVelocity);
    }

    @Override
    public void resetVelocity(PhysicsPipelineBody body) {
        //PhysicsPipeline.super.resetVelocity(body);
    }

    @Override
    public Vector3d getLinearVelocity(final PhysicsPipelineBody body, final Vector3d dest) {
        this.assertBodyValid(body);
        Box3dJNI.getLinearVelocity(this.worldHandle, body.getRuntimeId(), this.poseCache);
        return dest.set(this.poseCache);
    }

    @Override
    public Vector3d getAngularVelocity(final PhysicsPipelineBody body, final Vector3d dest) {
        this.assertBodyValid(body);
        Box3dJNI.getAngularVelocity(this.worldHandle, body.getRuntimeId(), this.poseCache);
        return dest.set(this.poseCache);
    }

    /**
     * "Wakes up" a sub-level, indicating environmental or other changes have occurred that should resume physics for idled or sleeping sub-levels.
     *
     * @param body the sub-level to wake up
     */
    @Override
    public void wakeUp(final PhysicsPipelineBody body) {
        this.assertBodyValid(body);

        if (!SubLevelPhysicsSystem.IN_PHYSICS_STEP) {
            Box3dJNI.wakeUpObject(this.worldHandle, body.getRuntimeId());
        } else {
            this.queuedWakeUps.add(body);
        }
    }

    @Override
    public @Nullable <T extends PhysicsConstraintHandle> T addConstraint(@Nullable PhysicsPipelineBody bodyA, @Nullable PhysicsPipelineBody bodyB, @NotNull PhysicsConstraintConfiguration<T> configuration) {
        return PhysicsPipeline.super.addConstraint(bodyA, bodyB, configuration);
    }

    @Override
    public void updateConfigFrom(PhysicsConfigData data) {
        //PhysicsPipeline.super.updateConfigFrom(data);
    }

    @Override
    public int getNextRuntimeID() {
        return nextBodyID++;
    }

    @Override
    public void prePhysicsTicks() {

    }
}