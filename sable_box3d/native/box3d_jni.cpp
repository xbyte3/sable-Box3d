
#include "box3d_jni.h"
#include <malloc.h>
#include <vector>
#include <stdexcept>
#include <cassert>
#include <unordered_map>
#include "sublevel_octree.cpp"

typedef enum {
    EMPTY,
    FACE,
    EDGE,
    CORNER,
    INTERIOR
} VoxelPhysicsState;

static const VoxelPhysicsState ALL_VOXEL_PHYSICS_STATES[5] = {
    EMPTY,
    FACE,
    EDGE,
    CORNER,
    INTERIOR
};

struct BlockState {
    uint32_t block_collider_id;
    VoxelPhysicsState voxel_state;
};

static constexpr uint8_t CHUNK_SHIFT = 4;
static constexpr uint8_t CHUNK_SIZE = 1 << CHUNK_SHIFT;
static constexpr int32_t CHUNK_MASK = (CHUNK_SIZE - 1);

struct ChunkSection {
    std::vector<BlockState> blocks;

    ChunkSection(std::vector<BlockState> blocks)
        : blocks(std::move(blocks))
    {
        if (this->blocks.size() != CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE) {
            throw std::runtime_error("Invalid block count");
        }
    }

    inline size_t get_index(int x, int y, int z) const {
        return (x + (z << 4) + (y << 8));
    }

    void set_block(int x, int y, int z, BlockState state) {
        size_t index = get_index(x, y, z);
        blocks[index] = state;
    }

    BlockState get_block(int x, int y, int z) const {
        size_t index = get_index(x, y, z);
        return blocks[index];
    }
};

constexpr int32_t OCTREE_CHUNK_SHIFT = 6;
constexpr int32_t OCTREE_CHUNK_SIZE = 1 << OCTREE_CHUNK_SHIFT;


struct OctreeChunkSection
{
    SubLevelOctree octree;
    SubLevelOctree liquidOctree;


    OctreeChunkSection()
        : octree(OCTREE_CHUNK_SHIFT),
        liquidOctree(OCTREE_CHUNK_SHIFT)
    {
    }
};

using LevelColliderID = size_t;

struct WorldData {
    std::unordered_map<int64_t, ChunkSection> mainLevelChunks;
    std::unordered_map<LevelColliderID, b3BodyId> bodies;
    std::unordered_map<int64_t, OctreeChunkSection> octreeChunks;

    //SableJointSet joint_set;

    //RopeMap rope_map;

    //std::unordered_map<LevelColliderID, ActiveLevelColliderInfo> levelColliders;
    
};

std::unordered_map<uint32_t, WorldData> worldData;

inline int64_t packSectionPosition(int32_t i, int32_t j, int32_t k)
{
    constexpr int64_t XZ_MASK = 0x3FFFFF; // 22 bit
    constexpr int64_t Y_MASK = 0xFFFFF;  // 20 bit

    return ((static_cast<int64_t>(i) & XZ_MASK) << 42) |
        ((static_cast<int64_t>(k) & XZ_MASK) << 20) |
        (static_cast<int64_t>(j) & Y_MASK);
}

struct VoxelColliderMap
{
private:

    std::vector<std::optional<VoxelColliderData>> voxelColliders;


    std::unordered_map<
        IVec3,
        std::optional<VoxelColliderData>,
        IVec3Hash
    > dynamicColliders;



public:

    VoxelColliderMap() = default;



    const VoxelColliderData* get(
        size_t index,
        const IVec3& blockPos
    ) const
    {
        const auto& collider =
            voxelColliders.at(index);


        if (collider.has_value() &&
            collider->dynamic)
        {
            auto it =
                dynamicColliders.find(blockPos);


            if (it != dynamicColliders.end())
            {
                if (it->second.has_value())
                    return &it->second.value();
            }
        }


        if (collider.has_value())
            return &collider.value();


        return nullptr;
    }
};

void insertBlockOctree(
    const VoxelColliderMap& colliderMap,
    SubLevelOctree& octree,
    const BlockState& state,
    bool remove,
    int32_t x,
    int32_t y,
    int32_t z
)
{
    int32_t blockColliderId = state.first; // state.0

    const VoxelCollider* blockCollider = nullptr;

    if (blockColliderId > 0)
    {
        blockCollider =
            &colliderMap.voxelColliders
            .at(static_cast<size_t>(blockColliderId - 1));
    }


    VoxelPhysicsState voxelState = state.second; // state.1


    bool solid =
        voxelState != VoxelPhysicsState::Interior &&
        voxelState != VoxelPhysicsState::Empty &&
        blockColliderId > 0 &&
        !blockCollider->collisionBoxes.empty();



    if (remove && !solid)
    {
        octree.insert(
            x,
            y,
            z,
            -1
        );
    }


    if (solid)
    {
        octree.insert(
            x,
            y,
            z,
            blockColliderId
        );
    }
}

// =======================
// WORLD
// =======================

static inline b3WorldId toWorld(jlong h)
{
    assert(h != 0 && "null world handle");

    return b3LoadWorldId((uint32_t)h);
}

static inline jlong fromWorld(b3WorldId id)
{
    return (jlong)b3StoreWorldId(id);
}

// =======================
// BODY 64bit!!
// =======================

static inline b3BodyId toBody(jlong h)
{
    return b3LoadBodyId((uint64_t)h);
}

static inline jlong fromBody(b3BodyId id)
{
    return (jlong)b3StoreBodyId(id);
}

// =======================
// WORLD
// =======================

JNIEXPORT jlong JNICALL
Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_worldCreate
(JNIEnv*, jclass, jfloat gx, jfloat gy, jfloat gz)
{
    b3WorldDef def = b3DefaultWorldDef();
    def.gravity = { gx, gy, gz };
    jlong worldHandle = fromWorld(b3CreateWorldDoublePrecision(&def));
    worldData[worldHandle] = WorldData{};

    return worldHandle;
}

JNIEXPORT void JNICALL
Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_worldDestroy
(JNIEnv*, jclass, jlong world)
{
    b3DestroyWorld(toWorld(world)); // look at this later
}

JNIEXPORT void JNICALL
Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_worldStep
(JNIEnv*, jclass, jlong world, jfloat dt, jint substeps)
{
    b3World_Step(toWorld(world), dt, substeps);
}

// =======================
// BODY
// =======================

JNIEXPORT void JNICALL
Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_getPose
(JNIEnv* env, jclass, jlong bodyHandle, jdoubleArray outPose)
{
    b3BodyId body = toBody(bodyHandle);

    b3Pos p = b3Body_GetPosition(body);
    b3Quat q = b3Body_GetRotation(body);

    jdouble tmp[7]{};
    tmp[0] = p.x;
    tmp[1] = p.y;
    tmp[2] = p.z;

    tmp[3] = q.v.x;
    tmp[4] = q.v.y;
    tmp[5] = q.v.z;
    tmp[6] = q.s;

    env->SetDoubleArrayRegion(outPose, 0, 7, tmp);
}

JNIEXPORT jlong JNICALL
Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_createSublevel
(JNIEnv* env, jclass, jlong world, jdoubleArray pose)
{
    b3BodyDef def = b3DefaultBodyDef();
    def.type = b3_dynamicBody;

    jdouble* elements = env->GetDoubleArrayElements(pose, nullptr);

    def.position = { elements[0], elements[1], elements[2] };
    def.rotation = { (float)elements[3], (float)elements[4], (float)elements[5], (float)elements[6] };

    env->ReleaseDoubleArrayElements(pose, elements, JNI_ABORT);

    b3BodyId body = b3CreateBody(toWorld(world), &def);

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3BoxHull box = b3MakeBoxHull(1, 1, 1);

    b3CreateHullShape( body, &shapeDef, &box.base );

    b3MassData myMassData;
    myMassData.mass = 1.0f;
    myMassData.center = {0.0f, 0.0f, 0.0f};
    myMassData.inertia = b3Mat3_identity;
    b3Body_SetMassData(body, myMassData);

    return fromBody(body);
}

JNIEXPORT void JNICALL
Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_removeSublevel
(JNIEnv* env, jclass, jlong body)
{
    b3DestroyBody(toBody(body));
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_addChunk(JNIEnv* env, jclass, jlong worldHandle, jint x, jint y, jint z, jintArray intData, jboolean global, jint object_id)
{
    jint* ints = env->GetIntArrayElements(intData, nullptr);

    std::vector<BlockState> blocks;
    blocks.reserve(4096);

    for (int i = 0; i < 4096; i++) {
        uint32_t block = (uint32_t)ints[i];

        uint16_t block_collider_id = block >> 16;
        uint16_t voxel_state_id = block & 0xFFFF;

        if (voxel_state_id < 5) {
            blocks.push_back({
                (uint32_t)block_collider_id,
                ALL_VOXEL_PHYSICS_STATES[voxel_state_id]
                });
        }
    }


    ChunkSection chunk(std::move(blocks));

    b3WorldId world = toWorld(worldHandle);
    
    auto& data = worldData.at(worldHandle);

    if (global == 0) {
        if (object_id != -1) {
            //b3BodyId& body = levelColliders.at(objectId);
        }
    }

    data.mainLevelChunks.emplace(
        packSectionPosition(x, y, z),
        chunk
    );

    env->ReleaseIntArrayElements(intData, ints, JNI_ABORT);
}

// =======================
// SHAPE
// =======================

JNIEXPORT jlong JNICALL
Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_createBox
(JNIEnv* env, jclass, jlong world, jfloat mass, jfloat hx, jfloat hy, jfloat hz, jfloatArray pose)
{
    b3BodyDef def = b3DefaultBodyDef();
    def.type = b3_dynamicBody;

    jfloat* elements = env->GetFloatArrayElements(pose, nullptr);

    def.position = { elements[0], elements[1], elements[2] };
    def.rotation = { elements[3], elements[4], elements[5], elements[6] };

    env->ReleaseFloatArrayElements(pose, elements, JNI_ABORT);

    b3BodyId body = b3CreateBody(toWorld(world), &def);

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3BoxHull box = b3MakeBoxHull(hx, hy, hz);

    b3CreateHullShape( body, &shapeDef, &box.base );

    b3MassData myMassData;
    myMassData.mass = mass;
    myMassData.center = {0.0f, 0.0f, 0.0f};
    myMassData.inertia = b3Mat3_identity; // b3Matrix3 inertia tensor
    b3Body_SetMassData(body, myMassData);

    return fromBody(body);
}

JNIEXPORT void JNICALL
Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_removeBox
(JNIEnv*, jclass, jlong body)
{
    b3DestroyBody(toBody(body));
}

JNIEXPORT void JNICALL
Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_wakeUpObject
(JNIEnv*, jclass, jlong body)
{
    b3Body_SetAwake(toBody(body), true);
}