
#include "box3d_jni.h"
#include <malloc.h>
#include <vector>
#include <stdexcept>
#include <cassert>
#include <unordered_map>

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

using LevelColliderID = size_t;

struct WorldData {
    std::unordered_map<int64_t, ChunkSection> mainLevelChunks;
    std::unordered_map<LevelColliderID, b3BodyId> bodies;

    // Статическое тело, на которое навешиваются шейпы глобального (world) террейна.
    b3BodyId levelBody;

    // Шейпы, созданные для каждого чанка глобального уровня, чтобы их можно
    // было удалить/пересоздать в addChunk/removeChunk без утечек.
    std::unordered_map<int64_t, std::vector<b3ShapeId>> globalChunkShapes;

    // То же самое, но для чанков, принадлежащих конкретному sub-level объекту
    // (object_id). При удалении самого объекта (removeSublevel) тело
    // уничтожается вместе со всеми шейпами автоматически, эта карта нужна
    // только чтобы addChunk не плодил дубликаты при повторном вызове на тот
    // же чанк.
    std::unordered_map<LevelColliderID, std::unordered_map<int64_t, std::vector<b3ShapeId>>> objectChunkShapes;
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

// Единичный box hull (полублоки 0.5 по каждой оси), переиспользуется для
// каждого твёрдого вокселя через b3CreateTransformedHullShape, чтобы не
// пересобирать геометрию хала на каждый блок.
static const b3BoxHull& unitVoxelHull()
{
    static const b3BoxHull hull = b3MakeBoxHull(0.5f, 0.5f, 0.5f);
    return hull;
}

// Считаем воксель "твёрдым", если у него есть collider id и он не пустой/не
// внутренний (Interior блоки не генерируют коллизию, как и в Rapier-версии).
static inline bool isSolidBlock(const BlockState& state)
{
    return state.block_collider_id > 0
        && state.voxel_state != VoxelPhysicsState::EMPTY
        && state.voxel_state != VoxelPhysicsState::INTERIOR;
}

// Создаёт box-шейпы для всех твёрдых блоков чанка на заданном теле.
// chunkOriginBlockX/Y/Z — координата блока (0,0,0) чанка в локальном
// пространстве тела.
static std::vector<b3ShapeId> createChunkShapes(
    b3BodyId body,
    const ChunkSection& chunk,
    int32_t chunkOriginBlockX,
    int32_t chunkOriginBlockY,
    int32_t chunkOriginBlockZ)
{
    std::vector<b3ShapeId> shapes;
    shapes.reserve(64); // обычно чанк не полностью сплошной

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.baseMaterial.friction = 0.6f;   // TODO: брать из VoxelColliderData, когда её портируем
    shapeDef.baseMaterial.restitution = 0.0f;
    shapeDef.density = 0.0f;        // геометрия террейна не должна влиять на массу тела
    shapeDef.updateBodyMass = false;

    for (int bx = 0; bx < CHUNK_SIZE; bx++) {
        for (int by = 0; by < CHUNK_SIZE; by++) {
            for (int bz = 0; bz < CHUNK_SIZE; bz++) {
                BlockState state = chunk.get_block(bx, by, bz);
                if (!isSolidBlock(state)) {
                    continue;
                }

                b3Transform transform = { 
                    { 
                        (float)(chunkOriginBlockX + bx) + 0.5f,
                        (float)(chunkOriginBlockY + by) + 0.5f,
                        (float)(chunkOriginBlockZ + bz) + 0.5f
                    },
                    b3Quat_identity 
                };

                b3ShapeId shape = b3CreateTransformedHullShape(
                    body, &shapeDef, &unitVoxelHull().base, transform, { 1.0f, 1.0f, 1.0f });

                shapes.push_back(shape);
            }
        }
    }

    return shapes;
}

static void destroyChunkShapes(std::vector<b3ShapeId>& shapes)
{
    for (b3ShapeId shape : shapes) {
        if (b3Shape_IsValid(shape)) {
            b3DestroyShape(shape, /*updateBodyMass*/ false);
        }
    }
    shapes.clear();
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

// Возвращает WorldData для мира по его jlong-хэндлу. Централизует явное
// сужение jlong -> uint32_t (ключ worldData совпадает по типу с b3WorldId's
// index+generation, упакованными в b3StoreWorldId), чтобы не плодить неявные
// narrowing-конверсии (C4244) по всему файлу.
static inline WorldData& getWorldData(jlong worldHandle)
{
    return worldData.at((uint32_t)worldHandle);
}

static inline WorldData& getOrCreateWorldData(jlong worldHandle)
{
    return worldData.try_emplace((uint32_t)worldHandle).first->second;
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
(JNIEnv* env, jclass, jfloat gx, jfloat gy, jfloat gz)
{
    b3WorldDef def = b3DefaultWorldDef();
    def.gravity = { gx, gy, gz };
    jlong worldHandle = fromWorld(b3CreateWorldDoublePrecision(&def));
    try
    {
        WorldData& data = getOrCreateWorldData(worldHandle);

        b3BodyDef levelBodyDef = b3DefaultBodyDef();
        data.levelBody = b3CreateBody(toWorld(worldHandle), &levelBodyDef);
    }
    catch (const std::exception& e)
    {
        printf("%s\n", e.what());
        fflush(stdout);

        jclass cls = env->FindClass("java/lang/RuntimeException");
        env->ThrowNew(cls, e.what());
        return 0;
    }
    

    return worldHandle;
}

JNIEXPORT void JNICALL
Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_worldDestroy
(JNIEnv*, jclass, jlong world)
{
    auto it = worldData.find((uint32_t)world);
    if (it != worldData.end()) {
        worldData.erase(it);
    }

    b3DestroyWorld(toWorld(world));
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
Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_createSubLevel
(JNIEnv* env, jclass, jlong world, jint id, jdoubleArray pose)
{
    try
    {
        printf("[createSubLevel] begin world=%lld id=%d pose=%p\n",
            (long long)world,
            id,
            pose);

        fflush(stdout);


        // =========================
        // WORLD CHECK
        // =========================

        if (world == 0)
        {
            throw std::runtime_error("createSubLevel: world handle is zero");
        }

        auto worldIt = worldData.find((uint32_t)world);
        if (worldIt == worldData.end())
        {
            throw std::runtime_error("createSubLevel: worldData not found");
        }

        b3WorldId worldId = toWorld(world);

        if (!b3World_IsValid(worldId))
        {
            throw std::runtime_error("createSubLevel: invalid b3WorldId");
        }


        // =========================
        // POSE CHECK
        // =========================

        if (pose == nullptr)
        {
            throw std::runtime_error("createSubLevel: pose is null");
        }

        jsize poseLength = env->GetArrayLength(pose);

        printf("[createSubLevel] pose length=%d\n", poseLength);

        if (poseLength < 7)
        {
            throw std::runtime_error("createSubLevel: pose length < 7");
        }


        jdouble* elements = env->GetDoubleArrayElements(pose, nullptr);

        if (elements == nullptr)
        {
            throw std::runtime_error("createSubLevel: GetDoubleArrayElements failed");
        }


        printf(
            "[createSubLevel] pose: %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
            elements[0],
            elements[1],
            elements[2],
            elements[3],
            elements[4],
            elements[5],
            elements[6]
        );


        // =========================
        // CREATE BODY
        // =========================

        b3BodyDef def = b3DefaultBodyDef();

        def.type = b3_dynamicBody;

        def.position = {
            (float)elements[0],
            (float)elements[1],
            (float)elements[2]
        };

        def.rotation = {
            (float)elements[3],
            (float)elements[4],
            (float)elements[5],
            (float)elements[6]
        };


        env->ReleaseDoubleArrayElements(
            pose,
            elements,
            JNI_ABORT
        );


        printf("[createSubLevel] creating body\n");
        fflush(stdout);


        b3BodyId body = b3CreateBody(worldId, &def);


        printf("[createSubLevel] body index=%u generation=%u\n",
            body.index1,
            body.generation);

        fflush(stdout);


        if (!b3Body_IsValid(body))
        {
            throw std::runtime_error("createSubLevel: created body is invalid");
        }


        // =========================
        // CREATE SHAPE
        // =========================

        printf("[createSubLevel] creating shape\n");
        fflush(stdout);


        b3ShapeDef shapeDef = b3DefaultShapeDef();

        b3BoxHull box = b3MakeBoxHull(
            1.0f,
            1.0f,
            1.0f
        );


        b3ShapeId shape = b3CreateHullShape(
            body,
            &shapeDef,
            &box.base
        );


        printf("[createSubLevel] shape index=%u generation=%u\n",
            shape.index1,
            shape.generation);

        fflush(stdout);


        if (!b3Shape_IsValid(shape))
        {
            throw std::runtime_error("createSubLevel: created shape is invalid");
        }


        // =========================
        // MASS
        // =========================

        printf("[createSubLevel] setting mass\n");
        fflush(stdout);


        b3MassData massData;

        massData.mass = 1.0f;
        massData.center = {
            0.0f,
            0.0f,
            0.0f
        };
        massData.inertia = b3Mat3_identity;


        b3Body_SetMassData(
            body,
            massData
        );


        // =========================
        // STORE
        // =========================

        WorldData& data = worldIt->second;


        auto existing = data.bodies.find((LevelColliderID)id);

        if (existing != data.bodies.end())
        {
            printf(
                "[createSubLevel] WARNING: replacing existing body id=%d\n",
                id
            );

            if (b3Body_IsValid(existing->second))
            {
                b3DestroyBody(existing->second);
            }
        }


        data.bodies[(LevelColliderID)id] = body;


        printf(
            "[createSubLevel] success id=%d handle=%lld\n",
            id,
            (long long)fromBody(body)
        );

        fflush(stdout);


        return fromBody(body);
    }
    catch (const std::exception& e)
    {
        printf(
            "[createSubLevel] ERROR: %s\n",
            e.what()
        );

        fflush(stdout);


        jclass cls = env->FindClass(
            "java/lang/RuntimeException"
        );

        if (cls)
        {
            env->ThrowNew(
                cls,
                e.what()
            );
        }

        return 0;
    }
}

JNIEXPORT void JNICALL
Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_removeSubLevel
(JNIEnv* env, jclass, jlong world, jint id)
{
    WorldData& data = getWorldData(world);
        
    auto it = data.bodies.find((LevelColliderID)id);
    if (it == data.bodies.end()) {
        return;
    }

    b3DestroyBody(it->second); // уничтожает и все чанк-шейпы объекта
    data.bodies.erase(it);
    data.objectChunkShapes.erase((LevelColliderID)id);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_addChunk(
    JNIEnv* env, jclass, jlong worldHandle, jint x, jint y, jint z,
    jintArray intData, jboolean global, jint object_id)
{
    std::vector<jint> ints(4096);
    env->GetIntArrayRegion(intData, 0, 4096, ints.data());

    std::vector<BlockState> blocks;
    blocks.reserve(4096);

    for (int i = 0; i < 4096; i++) {
        uint32_t block = (uint32_t)ints[i];

        uint16_t blockColliderId = block >> 16;
        uint16_t voxelStateId = block & 0xFFFF;

        if (voxelStateId >= 5) {
            voxelStateId = 0; // EMPTY
        }

        blocks.push_back({ (uint32_t)blockColliderId, ALL_VOXEL_PHYSICS_STATES[voxelStateId] });
    }

    ChunkSection chunk(std::move(blocks));

    WorldData& data = getWorldData(worldHandle);
    int64_t packedPos = packSectionPosition(x, y, z);

    data.mainLevelChunks.insert_or_assign(packedPos, chunk);
    const ChunkSection& storedChunk = data.mainLevelChunks.at(packedPos);

    if (global != 0) {
        std::vector<b3ShapeId>& shapes = data.globalChunkShapes[packedPos];
        destroyChunkShapes(shapes);

        shapes = createChunkShapes(
            data.levelBody, storedChunk,
            x << CHUNK_SHIFT, y << CHUNK_SHIFT, z << CHUNK_SHIFT);

        return;
    }

    if (object_id == -1) {
        return;
    }

    b3BodyId objectBody = data.bodies.at((LevelColliderID)object_id);

    std::vector<b3ShapeId>& shapes = data.objectChunkShapes[(LevelColliderID)object_id][packedPos];
    destroyChunkShapes(shapes);

    shapes = createChunkShapes(
        objectBody, storedChunk,
        x << CHUNK_SHIFT, y << CHUNK_SHIFT, z << CHUNK_SHIFT);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_removeChunk(
    JNIEnv*, jclass, jlong worldHandle, jint x, jint y, jint z, jboolean global)
{
    WorldData& data = getWorldData(worldHandle);
    int64_t packedPos = packSectionPosition(x, y, z);

    data.mainLevelChunks.erase(packedPos);

    if (global != 0) {
        auto it = data.globalChunkShapes.find(packedPos);
        if (it != data.globalChunkShapes.end()) {
            destroyChunkShapes(it->second);
            data.globalChunkShapes.erase(it);
        }
        return;
    }

    // Как и в Rust-версии, для sub-level объектов removeChunk по конкретному
    // object_id не поддерживается (сигнатура не передаёт object_id) —
    // объектные чанки чистятся целиком при уничтожении самого объекта
    // (removeSublevel), где b3DestroyBody сам уберёт все шейпы.
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
(JNIEnv*, jclass, jlong worldHandle, jint objectId)
{
    WorldData& data = getWorldData(worldHandle);
    b3BodyId objectBody = data.bodies.at((LevelColliderID)objectId);
    b3Body_SetAwake(objectBody, true);
}