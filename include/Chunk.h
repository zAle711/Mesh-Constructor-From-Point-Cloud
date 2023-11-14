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
        for (int x = 0; x < size; x ++)
        {
            for (int y = 0; y < size; y++)
            {
                for (int z = 0; z < size; z++)
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
        string key = to_string((x/chunkSize) * chunkSize) + "-" + to_string((y/chunkSize) * chunkSize) + "-" + to_string((z / chunkSize) * chunkSize);
        return key;
    }

    /*
    string GetChunkKey(int x, int y, int z)
    {

        int kx = (x/chunkSize) * chunkSize;
        int ky = (y/chunkSize) * chunkSize;
        int kz = (z/chunkSize) * chunkSize;

        ROS_INFO("%d %d %d", kx, ky, kz);

        string key = to_string(kx) + "-" + to_string(ky) + "-" + to_string(kz);
        ROS_INFO("CHUNK KEY: %s", key.c_str());
        return key;
    } */

    bool CheckBlock(int x, int y, int z)
    {

        //ROS_INFO("%d %d %d", ix, iy, iz);

        string chunk_key = GetChunkKey(x, y, z);
        //ROS_INFO("%s", chunk_key.c_str());
        int index_x = x % chunkSize;
        int index_y = y % chunkSize;
        int index_z = z % chunkSize;
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
