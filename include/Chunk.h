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
    int size;
    bool* chunkData;
    Chunk(int chunkSize) {
        size = chunkSize;
        chunkData = new bool[size * size * size];
    }

    int get_index(int x, int y, int z)
    {
        return x + y * size + z * size * size;
    }

    void set_data(int x, int y, int z, bool value)
    {
        int index = get_index(x,y,z);
        chunkData[index] = value;
    }

    bool get_data(int x, int y, int z)
    {
        int index = get_index(x,y,z);
        return chunkData[index];
    }
};


class World {
public:
    int chunkSize;
    unordered_map<string, Chunk> chunkDict;

    World(int cSize)
    {
        chunkSize = cSize;

        ROS_INFO("WORLD: %d", chunkSize);
    }

    string GetChunkKey(int x, int y, int z)
    {
        return to_string(x) + "_" + to_string(y) + "_" + to_string(z);

    }

    bool CheckBlock(int chunk_x, int chunk_y, int chunk_z, int x, int y, int z)
    {

        string chunk_key = GetChunkKey(chunk_x, chunk_y, chunk_z);


        if (chunkDict.find(chunk_key) != chunkDict.end())  // La chiave c'è
        {

            Chunk c = chunkDict.at(chunk_key.c_str());
            if (c.get_data(x, y, z)) return false;
            else c.set_data(x, y, z, true);

        } else
        {
            //ROS_INFO("Chunk Creato %s", chunk_key.c_str());
            Chunk c(chunkSize);
            c.set_data(x, y, z, true);
            chunkDict.insert(make_pair(chunk_key, c));
            //chunkDict[chunk_key] = c;
        }

        return true;
    }



};


#endif
