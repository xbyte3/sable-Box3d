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
import it.unimi.dsi.fastutil.objects.Object2ObjectOpenHashMap;
import it.unimi.dsi.fastutil.objects.ReferenceArrayList;
import it.unimi.dsi.fastutil.objects.ReferenceList;
import net.minecraft.core.SectionPos;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.level.chunk.LevelChunkSection;
import org.jetbrains.annotations.NotNull;
import org.joml.Quaterniondc;
import org.joml.Vector3d;
import org.joml.Vector3dc;

import java.util.HashMap;

public class Box3dPhysicsPipeline implements PhysicsPipeline {
    /**
     * Distance threshold for uploading sub-contraptions to the physics pipeline
     */
    private static final double DISTANCE_THRESHOLD = 1e-7;

    /**
     * Angle threshold for uploading sub-contraptions to the physics pipeline
     */
    private static final double ANGULAR_THRESHOLD = 1e-7;

    private final ServerLevel level;
    private final LevelAccelerator accelerator;
    // private final RapierVoxelColliderBakery colliderBakery;
    private final Int2ObjectMap<ServerSubLevel> activeSubLevels = new Int2ObjectArrayMap<>();
    private final Object2ObjectMap<KinematicContraption, TrackedKinematicContraption> activeContraptions = new Object2ObjectOpenHashMap<>();
    private final Long2LongOpenHashMap recentCollisions = new Long2LongOpenHashMap();
    private final ReferenceList<PhysicsPipelineBody> queuedWakeUps = new ReferenceArrayList<>();
    private final double[] poseCache;
    private Box3dPhysicsScene scene;

    private static int nextBodyID = 0;
    private final Map<Integer, Body> bodyMap = new HashMap<>(); // ID -> Box3d Body
    private final World world;

    private void trackBody(int id, Body body) {
        bodyMap.put(id, body);
    }

    public Box3dPhysicsPipeline(final @NotNull ServerLevel level) {
        this.level = level;
        this.accelerator = new LevelAccelerator(level);
        //this.colliderBakery = new RapierVoxelColliderBakery(this.accelerator);
        this.recentCollisions.defaultReturnValue(-1);
        this.poseCache = new double[7];
    }
    private long world;

    @Override
    public void init(Vector3dc gravity, double universalDrag) {
        if (gravity == null) {
            gravity = new Vector3d(0, -9.81, 0);
        }

        world = Box3DJNI.worldCreate();

        Box3DJNI.worldSetGravity(
                world,
                (float) gravity.x(),
                (float) gravity.y(),
                (float) gravity.z()
        );
    }

    @Override
    public void physicsTick(double timeStep) {
        Box3DJNI.worldStep(world, (float) timeStep);
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
    public void dispose() {
        Box3DJNI.worldDestroy(world);
    }

    @Override
    public void prePhysicsTicks() {

    }
}