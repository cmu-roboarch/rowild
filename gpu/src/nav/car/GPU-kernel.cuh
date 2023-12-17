/*
 * This file is a part of uAstar (https://github.com/zhou13/uastar) with minor
 * modifications
 */

#ifndef __GPU_KERNEL_CUH_IUGANILK
#define __GPU_KERNEL_CUH_IUGANILK

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_profiler_api.h>

#include <moderngpu.cuh>

// Suppose we only use x dimension
#define THREAD_ID (threadIdx.x)
#define GLOBAL_ID (THREAD_ID + NT * blockIdx.x)
#define BLOCK_ID  (blockIdx.x)

#define cudaAssert(X) \
    if ( !(X) ) { \
        printf( "Thread %d:%d failed assert at %s:%d!\n", \
                blockIdx.x, threadIdx.x, __FILE__, __LINE__ ); \
        return; \
    }

#define NUM_THETA_INDICES (8)
#define TWO_PI (6.283185308)

using namespace mgpu;

struct heap_t {
    float fValue;
    uint32_t addr;
};

__host__ __device__ bool operator<(const heap_t &a, const heap_t &b)
{
    return a.fValue < b.fValue;
}
__host__ __device__ bool operator>(const heap_t &a, const heap_t &b)
{
    return a.fValue > b.fValue;
}

struct node_t {
    uint32_t prev;
    float fValue;
    float gValue;
    uint32_t nodeID;
};

struct sort_t {
    uint32_t nodeID;
    float gValue;
};

__host__ __device__ bool operator<(const sort_t &a, const sort_t &b)
{
    if (a.nodeID != b.nodeID)
        return a.nodeID < b.nodeID;
    return a.gValue < b.gValue;
}

inline __host__ __device__ uint32_t flipFloat(float fl)
{
    union {
        float fl;
        uint32_t  u;
    } un;
    un.fl = fl;
    return un.u ^ ((un.u >> 31) | 0x80000000);
}

inline __host__ __device__ float reverseFlipFloat(uint32_t u)
{
    union {
        float f;
        uint32_t u;
    } un;
    un.u = u ^ ((~u >> 31) | 0x80000000);
    return un.f;
}

__constant__ int d_mapX;
__constant__ int d_mapY;
__constant__ int d_targetX;
__constant__ int d_targetY;
__constant__ int d_targetThetaIdx;
__constant__ uint32_t d_targetID;
__constant__ uint32_t d_modules[10];
__constant__ double d_hweight;
__host__ __device__ double sinThetas[NUM_THETA_INDICES];
__host__ __device__ double cosThetas[NUM_THETA_INDICES];
__constant__ int d_robotLength;
__constant__ int d_robotWidth;

inline __device__ void idToXYTheta(uint32_t nodeID, int *x, int *y, int *thetaIdx)
{
    *thetaIdx = nodeID % NUM_THETA_INDICES;
    *x = (nodeID / NUM_THETA_INDICES) / d_mapY;
    *y = (nodeID / NUM_THETA_INDICES) % d_mapY;
}

inline __device__ int xythetaToID(int x, int y, int thetaIdx)
{
    return (x * d_mapY + y) * NUM_THETA_INDICES + thetaIdx;
}

inline __device__ float computeHValue(int x, int y)
{
    const float SQRT2 = 1.4142135623731f;
    int dx = abs(d_targetX - x);
    int dy = abs(d_targetY - y);
    return d_hweight * (min(dx, dy)*SQRT2 + abs(dx-dy));
}

inline __device__ float computeHValue(uint32_t nodeID)
{
    int x, y, theta;
    idToXYTheta(nodeID, &x, &y, &theta);
    return computeHValue(x, y);
}

inline __device__ float isInRange(int x, int y)
{
    return 0 <= x && x < d_mapX && 0 <= y && y < d_mapY;
}

inline cudaError_t initializeCUDAConstantMemory(
    int mapX,
    int mapY,
    int targetX,
    int targetY,
    int targetThetaIdx,
    uint32_t targetID,
    double heuristicWeight,
    int robotLength,
    int robotWidth
)
{
    cudaError_t ret = cudaSuccess;
    ret = cudaMemcpyToSymbol(d_mapX, &mapX, sizeof(int));
    ret = cudaMemcpyToSymbol(d_mapY, &mapY, sizeof(int));
    ret = cudaMemcpyToSymbol(d_targetX, &targetX, sizeof(int));
    ret = cudaMemcpyToSymbol(d_targetY, &targetY, sizeof(int));
    ret = cudaMemcpyToSymbol(d_targetThetaIdx, &targetThetaIdx, sizeof(int));
    ret = cudaMemcpyToSymbol(d_targetID, &targetID, sizeof(uint32_t));
    ret = cudaMemcpyToSymbol(d_hweight, &heuristicWeight, sizeof(double));
    ret = cudaMemcpyToSymbol(d_robotLength, &robotLength, sizeof(int));
    ret = cudaMemcpyToSymbol(d_robotWidth, &robotWidth, sizeof(int));
    return ret;
}

inline cudaError_t updateModules(const vector<uint32_t> &mvec)
{
    return cudaMemcpyToSymbol(
        d_modules, mvec.data(), sizeof(uint32_t) * mvec.size());
}


__global__ void kInitialize(
    node_t g_nodes[],
    uint32_t g_hash[],
    heap_t g_openList[],
    int g_heapSize[],
    int startX,
    int startY,
    int startThetaIdx
)
{
    node_t node;
    node.fValue = computeHValue(startX, startY);
    node.gValue = 0;
    node.prev = UINT32_MAX;
    node.nodeID = xythetaToID(startX, startY, startThetaIdx);

    heap_t heap;
    heap.fValue = node.fValue;
    heap.addr = 0;

    g_nodes[0] = node;
    g_openList[0] = heap;
    g_heapSize[0] = 1;
    g_hash[node.nodeID] = 0;

    for (int i = 0; i < NUM_THETA_INDICES; i++) {
        sinThetas[i] = sin(i * TWO_PI / NUM_THETA_INDICES);
        cosThetas[i] = cos(i * TWO_PI / NUM_THETA_INDICES);
    }
}

// NB: number of CUDA block
// NT: number of CUDA thread per CUDA block
// VT: value handled per thread
template<int NB, int NT, int VT, int HEAP_CAPACITY>
__global__ void kExtractExpand(
    // global nodes
    node_t g_nodes[],

    uint8_t g_graph[],

    // open list
    heap_t g_openList[],
    int g_heapSize[],

    // solution
    uint32_t *g_optimalDistance,
    heap_t g_optimalNodes[],
    int *g_optimalNodesSize,

    // output buffer
    sort_t g_sortList[],
    uint32_t g_prevList[],
    int *g_sortListSize,

    // cleanup
    int *g_heapBeginIndex,
    int *g_heapInsertSize
)
{
    __shared__ uint32_t s_optimalDistance;
    __shared__ int s_sortListSize;
    __shared__ int s_sortListBase;

    int gid = GLOBAL_ID;
    int tid = THREAD_ID;
    if (tid == 0) {
        s_optimalDistance = UINT32_MAX;
        s_sortListSize = 0;
        s_sortListBase = 0;
    }

    __syncthreads();

    heap_t *heap = g_openList + HEAP_CAPACITY * gid - 1;

    heap_t extracted[VT];
    int popCount = 0;
    int heapSize = g_heapSize[gid];

#pragma unroll
    for (int k = 0; k < VT; ++k) {
        if (heapSize == 0)
            break;

        extracted[k] = heap[1];
        popCount++;

        heap_t nowValue = heap[heapSize--];

        int now = 1;
        int next;
        while ((next = now*2) <= heapSize) {
            heap_t nextValue = heap[next];
            heap_t nextValue2 = heap[next+1];
            bool inc = (next+1 <= heapSize) && (nextValue2 < nextValue);
            if (inc) {
                ++next;
                nextValue = nextValue2;
            }

            if (nextValue < nowValue) {
                heap[now] = nextValue;
                now = next;
            } else
                break;
        }
        heap[now] = nowValue;

    }
    g_heapSize[gid] = heapSize;

    int sortListCount = 0;
    sort_t sortList[VT*4];
    int prevList[VT*4];
    bool valid[VT*4];

#pragma unroll
    for (int k = 0; k < VT; ++k) {
#pragma unroll
        for (int i = 0; i < 4; ++i)
            valid[k*4 + i] = false;

        if (k >= popCount)
            continue;
        atomicMin(&s_optimalDistance, flipFloat(extracted[k].fValue));
        node_t node = g_nodes[extracted[k].addr];
        if (extracted[k].fValue != node.fValue)
            continue;

        int currX, currY, currTheta;
        idToXYTheta(node.nodeID, &currX, &currY, &currTheta);
        if (currX == d_targetX && currY == d_targetY) {
            int index = atomicAdd(g_optimalNodesSize, 1);
            g_optimalNodes[index] = extracted[k];
            continue;
        }

        int x, y, thetaIdx;
        idToXYTheta(node.nodeID, &x, &y, &thetaIdx);

        int dX = round(cosThetas[thetaIdx]);
        int dY = round(sinThetas[thetaIdx]);

        const int NX[4] = { x, x, x + dX, x - dX };
        const int NY[4] = { y, y, y + dY, y - dY };

        const int NTHETA_IDX[4] = {
            (thetaIdx + 1) % NUM_THETA_INDICES,
            (thetaIdx - 1 + NUM_THETA_INDICES) % NUM_THETA_INDICES,
            thetaIdx,
            thetaIdx
        };

        const float COST[4] = { 0, 0, 1, 1 };

#pragma unroll
        for (int i = 0; i < 4; ++i) {

            int nx = NX[i];
            int ny = NY[i];
            int nthetaIdx = NTHETA_IDX[i];

            // >>> Collision detection
            bool isStateFree = true;
            double sine = sinThetas[nthetaIdx];
            double cosine = cosThetas[nthetaIdx];

            for (int __i = 0; __i <= d_robotLength; __i++) {

                for (int __j = 0; __j <= d_robotWidth; __j++) {
                    int xbar = nx + round(__i * cosine);
                    int ybar = ny + round(__j * sine);

                    if (!isInRange(xbar, ybar)) {
                        isStateFree = false;
                        break;
                    }

                    int gIdx = xythetaToID(xbar, ybar, nthetaIdx);
                    if (g_graph[gIdx] == 0) {
                        isStateFree = false;
                        break;
                    }

                }

                if (!isStateFree) break;
            }
            // <<< Collision detection


            if (isStateFree) {
                int index = k*4 + i;
                uint32_t nodeID = xythetaToID(nx, ny, nthetaIdx);
                sortList[index].nodeID = nodeID;
                sortList[index].gValue = node.gValue + COST[i];
                prevList[index] = extracted[k].addr;
                valid[index] = true;
                ++sortListCount;
            }
        }
    }

    int sortListIndex = atomicAdd(&s_sortListSize, sortListCount);
    __syncthreads();
    if (tid == 0) {
        s_sortListBase = atomicAdd(g_sortListSize, s_sortListSize);
    }
    __syncthreads();
    sortListIndex += s_sortListBase;

#pragma unroll
    for (int k = 0; k < VT*4; ++k)
        if (valid[k]) {
            g_sortList[sortListIndex] = sortList[k];
            g_prevList[sortListIndex] = prevList[k];
            sortListIndex++;
        }
    if (tid == 0)
        atomicMin(g_optimalDistance, s_optimalDistance);
    if (gid == 0) {
        int newHeapBeginIndex = *g_heapBeginIndex + *g_heapInsertSize;

        *g_heapBeginIndex = newHeapBeginIndex % (NB*NT);
        *g_heapInsertSize = 0;
    }
}

// Assume g_sortList is sorted
template<int NT>
__global__ void kAssign(
    sort_t g_sortList[],
    uint32_t g_prevList[],
    int sortListSize,

    sort_t g_sortList2[],
    uint32_t g_prevList2[],
    int *g_sortListSize2
)
{
    __shared__ uint32_t s_nodeIDList[NT+1];
    __shared__ uint32_t s_sortListCount2;
    __shared__ uint32_t s_sortListBase2;

    int tid = THREAD_ID;
    int gid = GLOBAL_ID;

    bool working = false;
    sort_t sort;
    uint32_t prev;

    if (tid == 0)
        s_sortListCount2 = 0;

    if (tid == 0 && gid != 0)
        s_nodeIDList[0] = g_sortList[gid - 1].nodeID;

    if (gid < sortListSize) {
        working = true;
        sort = g_sortList[gid];
        prev = g_prevList[gid];
        s_nodeIDList[tid+1] = sort.nodeID;
    }
    __syncthreads();

    working &= (gid == 0 || s_nodeIDList[tid] != s_nodeIDList[tid+1]);

    int index;
    if (working) {
        index = atomicAdd(&s_sortListCount2, 1);
    }

    __syncthreads();
    if (tid == 0) {
         s_sortListBase2 = atomicAdd(g_sortListSize2, s_sortListCount2);
    }
    __syncthreads();

    if (working) {
        g_sortList2[s_sortListBase2 + index] = sort;
        g_prevList2[s_sortListBase2 + index] = prev;

    }
}

template<int NT>
__global__ void kDeduplicate(
    // global nodes
    node_t g_nodes[],
    int *g_nodeSize,

    // hash table
    uint32_t g_hash[],

    sort_t g_sortList[],
    uint32_t g_prevList[],
    int sortListSize,

    heap_t g_heapInsertList[],
    int *g_heapInsertSize
)
{
    int tid = THREAD_ID;
    int gid = GLOBAL_ID;
    bool working = gid < sortListSize;

    __shared__ int s_nodeInsertCount;
    __shared__ int s_nodeInsertBase;

    __shared__ int s_heapInsertCount;
    __shared__ int s_heapInsertBase;

    if (tid == 0) {
        s_nodeInsertCount = 0;
        s_heapInsertCount = 0;
    }
    __syncthreads();

    node_t node;
    bool insert = true;
    bool found = true;
    uint32_t nodeIndex;
    uint32_t heapIndex;
    uint32_t addr;

    if (working) {
        node.nodeID = g_sortList[gid].nodeID;
        node.gValue = g_sortList[gid].gValue;
        node.prev   = g_prevList[gid];
        node.fValue = node.gValue + computeHValue(node.nodeID);

        // cudaAssert((int)node.nodeID >= 0);
        addr = g_hash[node.nodeID];
        found = (addr != UINT32_MAX);

        if (found) {
            if (node.fValue < g_nodes[addr].fValue) {
                g_nodes[addr] = node;
            } else {
                insert = false;
            }
        }

        if (!found) {
            nodeIndex = atomicAdd(&s_nodeInsertCount, 1);
        }
        if (insert) {
            heapIndex = atomicAdd(&s_heapInsertCount, 1);
        }
    }

    __syncthreads();
    if (tid == 0) {
        s_nodeInsertBase = atomicAdd(g_nodeSize, s_nodeInsertCount);
        s_heapInsertBase = atomicAdd(g_heapInsertSize, s_heapInsertCount);
    }
    __syncthreads();

    if (working && !found) {
        addr = s_nodeInsertBase + nodeIndex;
        g_hash[node.nodeID] = addr;
        g_nodes[addr] = node;
    }
    if (working && insert) {
        uint32_t index = s_heapInsertBase + heapIndex;
        g_heapInsertList[index].fValue = node.fValue;
        g_heapInsertList[index].addr = addr;
    }
}

template<int NB, int NT, int HEAP_CAPACITY>
__global__ void kHeapInsert(
    // open list
    heap_t g_openList[],
    int g_heapSize[],
    int *g_heapBeginIndex,

    heap_t g_heapInsertList[],
    int *g_heapInsertSize,

    // cleanup variable
    int *sortListSize,
    int *sortListSize2,
    uint32_t *optimalDistance,
    int *optimalNodesSize
)
{
    int gid = GLOBAL_ID;

    int heapInsertSize = *g_heapInsertSize;
    int heapIndex = *g_heapBeginIndex + gid;
    if (heapIndex >= NB*NT)
        heapIndex -= NB*NT;

    int heapSize = g_heapSize[heapIndex];
    heap_t *heap = g_openList + HEAP_CAPACITY * heapIndex - 1;

    for (int i = gid; i < heapInsertSize; i += NB*NT) {
        heap_t value = g_heapInsertList[i];
        int now = ++heapSize;

        while (now > 1) {
            int next = now / 2;
            heap_t nextValue = heap[next];
            if (value < nextValue) {
                heap[now] = nextValue;
                now = next;
            } else
                break;
        }
        heap[now] = value;
    }

    g_heapSize[heapIndex] = heapSize;
    if (gid == 0) {
        *sortListSize = 0;
        *sortListSize2 = 0;
        *optimalDistance = UINT32_MAX;
        *optimalNodesSize = 0;
    }
}

__global__ void kFetchAnswer(
    node_t *g_nodes,

    uint32_t *lastAddr,

    uint32_t answerList[],
    int *g_answerSize
)
{
    int count = 0;
    int addr = *lastAddr;

    while (addr != UINT32_MAX) {
        answerList[count++] = g_nodes[addr].nodeID;
        addr = g_nodes[addr].prev;
    }

    *g_answerSize = count;
}

#endif /* end of include guard: __GPU_KERNEL_CUH_IUGANILK */
