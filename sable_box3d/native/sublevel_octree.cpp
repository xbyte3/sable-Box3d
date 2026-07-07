#pragma once

#include <vector>
#include <cstdint>
#include <algorithm>
#include <stdexcept>
#include <cmath>


class SubLevelOctree
{
private:
    static constexpr int32_t MAX_SIZE = INT32_MAX - 8 * 2;
    static constexpr float RESIZE_FRACTION = 1.5f;


public:

    // log2 стороны дерева
    int32_t logSize;


    // Плоский буфер узлов
    std::vector<int32_t> buffer;


private:

    // занято узлов
    int32_t size;


    // голова списка свободного места
    int32_t freeSpaceIndexHead;


public:

    explicit SubLevelOctree(int32_t logSize)
        :
        logSize(logSize),
        buffer(256, 0),
        size(1),
        freeSpaceIndexHead(-1)
    {
    }


private:

    static inline int32_t getOctantIndex(
        int32_t x,
        int32_t y,
        int32_t z
    )
    {
        return (x & 1)
            | ((y & 1) << 1)
            | ((z & 1) << 2);
    }



    int32_t allocateBranch()
    {
        // есть свободное место
        if (freeSpaceIndexHead != -1)
        {
            int32_t index = freeSpaceIndexHead - 1;

            freeSpaceIndexHead = buffer[index];

            return index;
        }


        // расширение
        if (size + 8 > static_cast<int32_t>(buffer.size()))
        {
            int32_t newSize =
                static_cast<int32_t>(
                    std::ceil(buffer.size() * RESIZE_FRACTION)
                    );


            if (newSize > MAX_SIZE)
                throw std::runtime_error(
                    "Octree buffer is full"
                );


            buffer.resize(newSize, 0);
        }


        int32_t index = size;

        size += 8;

        return index;
    }



    void split(int32_t index)
    {
        int32_t node = buffer[index];

        int32_t branchStartIndex =
            allocateBranch();


        buffer[index] = branchStartIndex;


        for (int i = 0; i < 8; i++)
        {
            buffer[branchStartIndex + i] = node;
        }
    }



public:

    void merge(int32_t index)
    {
        int32_t branchStartIndex =
            buffer[index];


        int32_t node =
            buffer[branchStartIndex];


        for (int i = 1; i < 8; i++)
        {
            if (buffer[branchStartIndex + i] != node)
                return;
        }


        if (node == 0)
        {
            buffer[index] = node;

            deleteChildren(branchStartIndex);

            freeSpaceIndexHead =
                branchStartIndex + 1;
        }
    }



private:

    void deleteChildren(int32_t branchStartIndex)
    {
        buffer[branchStartIndex] =
            freeSpaceIndexHead;


        freeSpaceIndexHead =
            branchStartIndex;


        for (int i = 1; i < 8; i++)
        {
            buffer[branchStartIndex + i] = 0;
        }
    }



public:

    bool insert(
        int32_t x,
        int32_t y,
        int32_t z,
        int32_t block
    )
    {
        int32_t shift = logSize - 1;

        int32_t index = 0;

        int32_t node = buffer[index];


        int32_t branchIndex = 0;


        std::vector<int32_t> branchesVisited(
            logSize,
            0
        );


        while (shift >= 0)
        {
            if (node == blockIdToNode(block))
            {
                return false;
            }


            int32_t octantIndex =
                getOctantIndex(
                    x >> shift,
                    y >> shift,
                    z >> shift
                );


            if (node > 0)
            {
                branchesVisited[branchIndex] = index;
                branchIndex++;
            }
            else
            {
                split(index);
            }


            int32_t branchStartIndex =
                buffer[index];


            index =
                branchStartIndex + octantIndex;


            node =
                buffer[index];


            shift--;
        }


        buffer[index] =
            blockIdToNode(block);



        // reduce
        for (int i = logSize - 1; i >= 0; i--)
        {
            merge(branchesVisited[i]);
        }


        return true;
    }




    int32_t query(
        int32_t x,
        int32_t y,
        int32_t z,
        int32_t logSizeOfTarget
    ) const
    {
        int32_t treeSize =
            1 << logSize;


        if (x < 0 || y < 0 || z < 0 ||
            x >= treeSize ||
            y >= treeSize ||
            z >= treeSize)
        {
            return -2;
        }


        int32_t shift =
            logSize - 1;


        int32_t index = 0;


        int32_t node =
            buffer[index];



        while (shift >= logSizeOfTarget)
        {
            if (node < 0)
            {
                return nodeToBlockId(node);
            }
            else if (node == 0)
            {
                return -2;
            }


            int32_t octantIndex =
                getOctantIndex(
                    x >> shift,
                    y >> shift,
                    z >> shift
                );


            index =
                node + octantIndex;


            node =
                buffer[index];


            shift--;
        }


        if (node < 0)
            return nodeToBlockId(node);

        if (node == 0)
            return -2;


        return -1;
    }



private:

    inline int32_t blockIdToNode(
        int32_t block
    ) const
    {
        return -block - 1;
    }



    inline int32_t nodeToBlockId(
        int32_t node
    ) const
    {
        return -node - 1;
    }



public:

    bool isEmpty() const
    {
        return buffer[0] == 0;
    }
};