#ifndef CHUNK_H
#define CHUNK_H

#include <ros/ros.h>
#include<map>
#include<unordered_map>
#include<string>
#include<array>

using namespace std;

class Chunk {
  public:
    int size = 16;
    array<array<array<bool, 16>, 16>, 16> chunkData;

    Chunk()
    {
        for (int z = 0; z < size; z++)
        {
            for (int y = 0; y < size; y++)
            {
                for (int x = 0; x < size; x++)
                {
                    chunkData[x][y][z] = false;
                }
            }

        }
    }
};


class World {
public:
    int chunkSize;
    float offset;
    float precision;
    unordered_map<string, Chunk> chunkDict;

    World(int cSize, float off, float pre)
    {
        chunkSize = cSize;
        offset = off;
        precision = pre;

        ROS_INFO("WORLD: %d, %f, %f", chunkSize, offset, precision);
    }

    string GetChunkKey(int x, int y, int z)
    {
        return to_string((x/chunkSize) * chunkSize) + "-" + to_string((y/chunkSize) * chunkSize) + "-" + to_string((z / chunkSize) * chunkSize);

    }

    bool CheckBlock(int x, int y, int z)
    {

        //ROS_INFO("%d %d %d", ix, iy, iz);
        int chunk_x = (x < 0) ?  ( x / chunkSize) -1 : x / chunkSize;
        int chunk_y = (y < 0) ?  ( y / chunkSize) -1 : y / chunkSize;
        int chunk_z = (z < 0) ?  ( z / chunkSize) -1 : z / chunkSize;

        string chunk_key = GetChunkKey(chunk_x, chunk_y, chunk_z);
        //ROS_INFO("%s", chunk_key.c_str());
        int index_x = (x < 0) ? x & chunkSize - 1 : x % chunkSize;
        int index_y = (y < 0) ? y & chunkSize - 1 : y % chunkSize;
        int index_z = (z < 0) ? z & chunkSize - 1 : z % chunkSize;

        if (chunkDict.find(chunk_key) != chunkDict.end())  // La chiave c'è
        {

            Chunk c = chunkDict[chunk_key];

            if (c.chunkData[index_x][index_y][index_z]) return false;
            else c.chunkData[index_x][index_y][index_z] = true;

        } else
        {
            //ROS_INFO("Chunk Creato %s", chunk_key.c_str());
            Chunk c;
            c.chunkData[index_x][index_y][index_z] = true;
            chunkDict[chunk_key] = c;
        }

        return true;
    }



};


#endif
