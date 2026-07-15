
#include "box3d_jni.h"
#include <windows.h>
#include <malloc.h>
#include <vector>
#include <stdexcept>
#include <cassert>
#include <unordered_map>
#include <mutex>
#include <array>

static std::recursive_mutex g_physicsMutex;

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

struct BlockKey
{
    int32_t x, y, z;
    bool operator==(const BlockKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct BlockKeyHash
{
    size_t operator()(const BlockKey& k) const noexcept {
        size_t h = std::hash<int32_t>()(k.x);
        h ^= std::hash<int32_t>()(k.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= std::hash<int32_t>()(k.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
};

struct VoxelColliderBox
{
    float minX, minY, minZ, maxX, maxY, maxZ;
};

struct VoxelColliderData
{
    std::vector<VoxelColliderBox> collisionBoxes;
    bool isFluid = false;
    float friction = 1.0f;
    float volume = 1.0f;
    float restitution = 0.0f;
    bool dynamic = false;
    // contact_events/contact_method (Rapier hooks.rs) сознательно не портированы —
    // у Box3D пока нет аналога SablePhysicsHooks/dispatcher.
};

struct VoxelColliderRegistry
{
    std::vector<VoxelColliderData> colliders;
    std::mutex mutex; // регистрация может идти с потоков worldgen
};

static VoxelColliderRegistry g_voxelColliderRegistry;

static inline const VoxelColliderData* getVoxelColliderData(uint32_t blockColliderId)
{
    if (blockColliderId == 0) {
        return nullptr;
    }
    size_t index = (size_t)blockColliderId - 1; // colliderId = registry index + 1, 0 = "нет коллайдера"
    if (index >= g_voxelColliderRegistry.colliders.size()) {
        return nullptr;
    }
    return &g_voxelColliderRegistry.colliders[index];
}

static constexpr uint8_t CHUNK_SHIFT = 4;
static constexpr uint8_t CHUNK_SIZE = 1 << CHUNK_SHIFT;
static constexpr int32_t CHUNK_MASK = (CHUNK_SIZE - 1);

struct ChunkSection {
    std::vector<BlockState> blocks;
    b3BodyId body = b3_nullBodyId;
    std::vector<std::vector<b3ShapeId>> shapes;

    ChunkSection(std::vector<BlockState> blocks)
        : blocks(std::move(blocks)),
        shapes(CHUNK_SIZE* CHUNK_SIZE* CHUNK_SIZE)
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

static void dbg(const char* fmt, ...)
{
    char buf[512];

    va_list args;
    va_start(args, fmt);
    vsnprintf_s(buf, sizeof(buf), _TRUNCATE, fmt, args);
    va_end(args);

    OutputDebugStringA(buf);
    OutputDebugStringA("\n");
}


inline int getBlockIndex(int x, int y, int z)
{
    return x + y * 16 + z * 16 * 16;
}

using LevelColliderID = size_t;

struct WorldData {
    std::unordered_map<int64_t, ChunkSection> mainLevelChunks;
    std::unordered_map<LevelColliderID, b3BodyId> bodies;

    // Статическое тело, на которое навешиваются шейпы глобального (world) террейна.
    b3BodyId levelBody = b3_nullBodyId;

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
    if (state.voxel_state == VoxelPhysicsState::EMPTY || state.voxel_state == VoxelPhysicsState::INTERIOR) {
        return false;
    }

    const VoxelColliderData* data = getVoxelColliderData(state.block_collider_id);
    return data != nullptr && !data->isFluid && !data->collisionBoxes.empty();
}

// Создаёт шейп(ы) для одного солидного блока по его локальным блок-координатам в чанке.
static std::vector<b3ShapeId> createBlockShapes(
    b3BodyId body,
    const VoxelColliderData& colliderData,
    int32_t blockX, int32_t blockY, int32_t blockZ)
{
    std::vector<b3ShapeId> shapes;
    shapes.reserve(colliderData.collisionBoxes.size());

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.baseMaterial.friction = colliderData.friction;
    shapeDef.baseMaterial.restitution = colliderData.restitution;
    shapeDef.density = 0.0f;
    shapeDef.updateBodyMass = false;

    for (const VoxelColliderBox& box : colliderData.collisionBoxes) {
        float hx = (box.maxX - box.minX) * 0.5f;
        float hy = (box.maxY - box.minY) * 0.5f;
        float hz = (box.maxZ - box.minZ) * 0.5f;

        // Защита от вырожденных боксов (источник прошлого "area > 0.0f" assert'а).
        if (hx <= 0.0f || hy <= 0.0f || hz <= 0.0f) {
            continue;
        }

        b3BoxHull hull = b3MakeBoxHull(hx, hy, hz);

        b3Transform transform;
        transform.q = b3Quat_identity;
        transform.p = {
            blockX + (box.minX + box.maxX) * 0.5f,
            blockY + (box.minY + box.maxY) * 0.5f,
            blockZ + (box.minZ + box.maxZ) * 0.5f,
        };

        shapes.push_back(b3CreateTransformedHullShape(body, &shapeDef, &hull.base, transform, { 1.0f, 1.0f, 1.0f }));
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
    std::lock_guard<std::recursive_mutex> lock(g_physicsMutex);

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
        b3DestroyWorld(toWorld(worldHandle));

        dbg("%s\n", e.what());

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
    std::lock_guard<std::recursive_mutex> lock(g_physicsMutex);

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
JNIEXPORT jint JNICALL
Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_newVoxelCollider
(JNIEnv*, jclass, jdouble friction, jdouble volume, jdouble restitution, jboolean isFluid, jboolean dynamic)
{
    std::lock_guard<std::mutex> lock(g_voxelColliderRegistry.mutex);

    VoxelColliderData data;
    data.friction = (float)friction;
    data.volume = (float)volume;
    data.restitution = (float)restitution;
    data.isFluid = isFluid != 0;
    data.dynamic = dynamic != 0;

    jint index = (jint)g_voxelColliderRegistry.colliders.size();
    g_voxelColliderRegistry.colliders.push_back(std::move(data));
    return index;
}

JNIEXPORT void JNICALL
Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_addVoxelColliderBox
(JNIEnv* env, jclass, jint index, jdoubleArray boxBounds)
{
    jdouble bounds[6];
    env->GetDoubleArrayRegion(boxBounds, 0, 6, bounds);

    std::lock_guard<std::mutex> lock(g_voxelColliderRegistry.mutex);
    if (index < 0 || (size_t)index >= g_voxelColliderRegistry.colliders.size()) {
        return;
    }

    g_voxelColliderRegistry.colliders[index].collisionBoxes.push_back({
        (float)bounds[0], (float)bounds[1], (float)bounds[2],
        (float)bounds[3], (float)bounds[4], (float)bounds[5]
        });
}

JNIEXPORT void JNICALL
Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_clearVoxelColliderBoxes
(JNIEnv*, jclass, jint index)
{
    std::lock_guard<std::mutex> lock(g_voxelColliderRegistry.mutex);
    if (index < 0 || (size_t)index >= g_voxelColliderRegistry.colliders.size()) {
        return;
    }
    g_voxelColliderRegistry.colliders[index].collisionBoxes.clear();
}

JNIEXPORT jlong JNICALL
Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_createSubLevel
(JNIEnv* env, jclass, jlong world, jint id, jdoubleArray pose)
{
    std::lock_guard<std::recursive_mutex> lock(g_physicsMutex);

    auto worldIt = worldData.find((uint32_t)world);

    b3WorldId worldId = toWorld(world);

    jdouble* elements = env->GetDoubleArrayElements(pose, nullptr);

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

    b3BodyId body = b3CreateBody(worldId, &def);

    b3ShapeDef shapeDef = b3DefaultShapeDef();

    b3BoxHull box = b3MakeBoxHull(
        0.5f,
        0.5f,
        0.5f
    );

    b3ShapeId shape = b3CreateHullShape(
        body,
        &shapeDef,
        &box.base
    );

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

    WorldData& data = worldIt->second;

    auto existing = data.bodies.find((LevelColliderID)id);

    if (existing != data.bodies.end())
    {
        if (b3Body_IsValid(existing->second))
        {
            b3DestroyBody(existing->second);
        }
    }

    data.bodies[(LevelColliderID)id] = body;

    return fromBody(body);
}

JNIEXPORT void JNICALL
Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_removeSubLevel
(JNIEnv* env, jclass, jlong world, jint id)
{
    std::lock_guard<std::recursive_mutex> lock(g_physicsMutex);
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
    std::lock_guard<std::recursive_mutex> lock(g_physicsMutex);
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

    // =========================
        // CREATE BODY
    // =========================

    b3BodyDef def = b3DefaultBodyDef();
    def.position = {
        static_cast<double>(x << CHUNK_SHIFT),
        static_cast<double>(y << CHUNK_SHIFT),
        static_cast<double>(z << CHUNK_SHIFT)
    };
    def.rotation = b3Quat_identity;

    WorldData& data = getWorldData(worldHandle);
    int64_t packedPos = packSectionPosition(x, y, z);

    auto [it, inserted] = data.mainLevelChunks.insert_or_assign(
        packedPos,
        ChunkSection(std::move(blocks))
    );

    ChunkSection& storedChunk = it->second;
    storedChunk.body = b3CreateBody(toWorld(worldHandle), &def);

    if (global != 0) {
        for (int bx = 0; bx < CHUNK_SIZE; bx++) {
            for (int by = 0; by < CHUNK_SIZE; by++) {
                for (int bz = 0; bz < CHUNK_SIZE; bz++) {
                    BlockState blockState = storedChunk.get_block(bx, by, bz);

                    if (!isSolidBlock(blockState)) {
                        continue;
                    }

                    const VoxelColliderData* colliderData = getVoxelColliderData(blockState.block_collider_id);

                    if (!colliderData) {
                        continue;
                    }

                    int index = getBlockIndex(bx, by, bz);

                    storedChunk.shapes[index] =
                        createBlockShapes(
                            storedChunk.body,
                            *colliderData,
                            bx,
                            by,
                            bz
                        );
                }
            }
        }
        return;
    }

    /*if (object_id == -1) { TODO: Fix it with using new function createChunkShapes.
        return;
    }

    b3BodyId objectBody = data.bodies.at((LevelColliderID)object_id);

    std::vector<b3ShapeId>& shapes = data.objectChunkShapes[(LevelColliderID)object_id][packedPos];
    destroyChunkShapes(shapes);

    shapes = createChunkShapes(
        objectBody, storedChunk,
        x << CHUNK_SHIFT, y << CHUNK_SHIFT, z << CHUNK_SHIFT);*/
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_removeChunk(
    JNIEnv*, jclass, jlong worldHandle, jint x, jint y, jint z, jboolean global)
{
     std::lock_guard<std::recursive_mutex> lock(g_physicsMutex);

    WorldData& data = getWorldData(worldHandle);

    int64_t packedPos = packSectionPosition(x, y, z);

    auto it = data.mainLevelChunks.find(packedPos);

    if (it == data.mainLevelChunks.end()) {
        return;
    }

    ChunkSection& chunk = it->second;

    // Удаляем все shape'ы секции
    for (auto& shapes : chunk.shapes) {
        destroyChunkShapes(shapes);
    }

    // Удаляем тело секции
    if (b3Body_IsValid(chunk.body)) {
        b3DestroyBody(chunk.body);
    }

    data.mainLevelChunks.erase(it);

    // Как и в Rust-версии, для sub-level объектов removeChunk по конкретному
    // object_id не поддерживается (сигнатура не передаёт object_id) —
    // объектные чанки чистятся целиком при уничтожении самого объекта
    // (removeSublevel), где b3DestroyBody сам уберёт все шейпы.
}

JNIEXPORT void JNICALL
Java_dev_ryanhcode_sable_physics_impl_box3d_Box3dJNI_changeBlock
(JNIEnv*, jclass, jlong worldHandle, jint x, jint y, jint z, jint packedBlock)
{
    std::lock_guard<std::recursive_mutex> lock(g_physicsMutex);

    WorldData& data = getWorldData(worldHandle);

    uint32_t packed = (uint32_t)packedBlock;

    uint16_t blockColliderId = packed >> 16;
    uint16_t voxelStateId = packed & 0xFFFF;

    if (voxelStateId >= 5) {
        voxelStateId = 0;
    }

    BlockState blockState{
        (uint32_t)blockColliderId,
        ALL_VOXEL_PHYSICS_STATES[voxelStateId]
    };


    // Координаты секции
    int32_t sectionX = x >> CHUNK_SHIFT;
    int32_t sectionY = y >> CHUNK_SHIFT;
    int32_t sectionZ = z >> CHUNK_SHIFT;


    int64_t sectionPos =
        packSectionPosition(sectionX, sectionY, sectionZ);


    auto chunkIt = data.mainLevelChunks.find(sectionPos);

    if (chunkIt == data.mainLevelChunks.end()) {
        return;
    }


    ChunkSection& chunk = chunkIt->second;


    // Локальные координаты блока в секции
    int bx = x & (CHUNK_SIZE - 1);
    int by = y & (CHUNK_SIZE - 1);
    int bz = z & (CHUNK_SIZE - 1);


    int index = getBlockIndex(bx, by, bz);


    // Удаляем старую коллизию блока

    dbg("index=%zu size=%zu", index, chunk.shapes.size());
    destroyChunkShapes(chunk.shapes[index]);


    // Обновляем состояние блока
    chunk.set_block(bx, by, bz, blockState);


    if (!isSolidBlock(blockState)) {
        return;
    }


    const VoxelColliderData* colliderData =
        getVoxelColliderData(blockState.block_collider_id);


    if (!colliderData) {
        return;
    }


    chunk.shapes[index] =
        createBlockShapes(
            chunk.body,
            *colliderData,
            bx,
            by,
            bz
        );
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
    std::lock_guard<std::recursive_mutex> lock(g_physicsMutex);
    WorldData& data = getWorldData(worldHandle);
    b3BodyId objectBody = data.bodies.at((LevelColliderID)objectId);
    b3Body_SetAwake(objectBody, true);
}