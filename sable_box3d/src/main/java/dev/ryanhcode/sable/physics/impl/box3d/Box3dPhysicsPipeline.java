package dev.ryanhcode.sable.physics.impl.box3d;

import dev.ryanhcode.sable.api.physics.PhysicsPipeline;

import dev.ryanhcode.sable.api.physics.PhysicsPipelineBody;
import dev.ryanhcode.sable.api.physics.object.box.BoxHandle;
import dev.ryanhcode.sable.api.physics.object.box.BoxPhysicsObject;
import dev.ryanhcode.sable.api.physics.object.rope.RopeHandle;
import dev.ryanhcode.sable.api.physics.object.rope.RopePhysicsObject;
import dev.ryanhcode.sable.api.sublevel.KinematicContraption;
import dev.ryanhcode.sable.companion.math.Pose3d;
import dev.ryanhcode.sable.companion.math.Pose3dc;
import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import dev.ryanhcode.sable.util.LevelAccelerator;
import it.unimi.dsi.fastutil.ints.Int2ObjectArrayMap;
import it.unimi.dsi.fastutil.ints.Int2ObjectMap;
import it.unimi.dsi.fastutil.longs.Long2LongOpenHashMap;
import it.unimi.dsi.fastutil.objects.ReferenceArrayList;
import it.unimi.dsi.fastutil.objects.ReferenceList;
import net.minecraft.core.SectionPos;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.level.chunk.LevelChunkSection;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
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
    private static int nextBodyID = 0;
    private final Map<Integer, Long> bodyHandles = new HashMap<>();  // ID -> JNI body handle

    private final Long2LongOpenHashMap recentCollisions = new Long2LongOpenHashMap();

    public Box3dPhysicsPipeline(final ServerLevel level) {
        this.level = level;
        this.accelerator = new LevelAccelerator(level);
        //this.colliderBakery = new Box3dVoxelColliderBakery(this.accelerator);
        this.recentCollisions.defaultReturnValue(-1);
        this.poseCache = new double[7];
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

        log.info("Hi! {}", Long.toString(this.worldHandle));
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
    public void physicsTick(double timeStep) {
        Box3dJNI.worldStep(this.worldHandle, (float) timeStep, 1);
    }

    @Override
    public void postPhysicsTicks() {

    }

    @Override
    public void tick() {

    }

    @Override
    public void add(ServerSubLevel subLevel, Pose3dc pose) {

    }

    @Override
    public void remove(ServerSubLevel subLevel) {

    }

    @Override
    public void add(KinematicContraption contraption) {

    }

    @Override
    public void remove(KinematicContraption contraption) {

    }

    @Override
    public Pose3d readPose(ServerSubLevel subLevel, Pose3d dest) {
        return null;
    }

    @Override
    public RopeHandle addRope(RopePhysicsObject rope) {
        return null;
    }

    @Override
    public BoxHandle addBox(BoxPhysicsObject boxPhysicsObject) {
        return null;
    }

    @Override
    public void handleChunkSectionAddition(LevelChunkSection chunk, int x, int y, int z, boolean uploadDataIfGlobal) {

    }

    @Override
    public void handleChunkSectionRemoval(int x, int y, int z) {

    }

    @Override
    public void handleBlockChange(SectionPos sectionPos, LevelChunkSection chunk, int x, int y, int z, BlockState oldState, BlockState newState) {

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
    public void wakeUp(PhysicsPipelineBody body) {

    }

    @Override
    public int getNextRuntimeID() {
        return nextBodyID++;
    }

    @Override
    public void prePhysicsTicks() {

    }
}