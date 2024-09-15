---
title: 'Optimizing CUDA Kernels'
date: 2024-09-04
permalink: /posts/2024/cuda-kernels/
---

Notes from "Professional CUDA C Programming" by John Cheng, Max Grossman, and Ty McKercher. Since this book is old, information is only accurate for Fermi / Kepler generation devices.

---


### Shared Memory Bank Conflicts
Each SM contains small latency memory pool accessible by all threads in threadblock known as shared memory. Shared memory is divided into 32 equally-sized memory banks. Each bank can service one 32-bit word per clock cycle. If multiple threads in the same warp access the same bank, the bank must serialize the requests. This is known as a bank conflict.

On Kepler, the size of each bank entry can be 32 or 64 bit.

```c++
# 64-bit mode
bank index = (bbyte address / 8 bytes/bank) % 32 banks
```

To mitigate bank conflicts, we can use padding to ensure that each thread accesses a different bank.

You typically use `__syncthreads()` to ensure that all threads in the block have finished writing to shared memory before reading from it.

### Global Memory
We want our reads/writes to be:
- aligned
- coalesced

L1 cache line is 128-bytes.

Uncached loads that do not pass through L1 cache are performaed at the granularity of memory segments (32-bytes) and not cache lines (128-bytes).

Writes do not get cached in L1. They can be cached in L2. Memory transactions are 32-byte granularity and can be one, two, or four segments at a time.


### Fast Matrix Transpose

Matrix transpose is hard because reading/writing to global memory column-wise results in a lot of uncoalesced memory accesses. This will hurt your global load/store efficiency. 

Assume 2D matrix. Each threadblock is responsible for a 2D tile. Each thread is responsible for 1 element in tile.

1. Each thread reads from global memory (row-wise) and writes to shared memory (row-wise). Adjacent threads (based on `threadIdx.x`) read adjacent elements in the same row. These reads are likely coalesced and aligned (good). These writes to shared memory are also row-wise and do not result in bank conflicts.
2. Use `__syncthreads()` to ensure all threads have finished writing to shared memory.
3. Each thread reads from shared memory (column-wise) and writes to global memory (row-wise). Adjacent threads (based on `threadIdx.x`) read adjacent elements in the same column. These reads from smem may result in bank conflicts but we can fix this. These writes to global memory are row-wise so they are coalesced and aligned.

Optimizations
- unroll the grid. now each threadblock is responsible for `k` tiles. each thread is responsible for `k` elements in tile.
- pad the shared memory to avoid bank conflicts:
```c++
// not padded
__shared__ float tile[BDIM_Y][BDIM_X];
// padded
__shared__ float tile[BDIM_Y][BDIM_X+2];
```
- try various grid dimensions. more threadblocks can mean more device parallelism.

### Fast Reduce
- warp shuffle trick is insane. this forgoes need for block level sync like `__syncthreads()`. This is because threads in a warp can communicate with each other without needing to sync with other warps.
```c++
__inline__ __device__ int warpReduce(int mySum) {
    mySum += __shfl_xor(mySum, 16);
    mySum += __shfl_xor(mySum, 8);
    mySum += __shfl_xor(mySum, 4);
    mySum += __shfl_xor(mySum, 2);
    mySum += __shfl_xor(mySum, 1);
}
```

```c++
__global__ void reduceShfl(int *g_idata, int *g_odata, unsigned int n) {
    // shared memory for each warp sum
    __shared__ int smem[SMEMDIM];
    
    // boundary check
    unsigned int idx = blockIdx.x*blockDim.x + threadIdx.x;
    if (idx >= n) return;

    // read from global memory
    int mySum = g_idata[idx];

    // calculate lane index and warp index
    int laneIdx = threadIdx.x % warpSize;
    int warpIdx = threadIdx.x / warpSize;

    // block-wide warp reduce
    mySum = warpReduce(mySum);

    // save warp sum to shared memory
    if (laneIdx == 0) smem[warpIdx] = mySum;

    // block-wide sync
    __syncthreads();

    // last warp reduce
    mySum = (threadIdx.x < SMEMDIM) ? smem[laneIdx] : 0;
    if (warpIdx == 0) mySum = warpReduce(mySum);

    // write to global memory
    if (threadIdx.x == 0) g_odata[blockIdx.x] = mySum;
}
```