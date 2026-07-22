// Offline batch entry point for evaluation: runs detectObstacles() directly
// over a directory of .pcd files (no ROS, no rosbag replay, no PCL viewer),
// writing one JSON detection file per input frame using the input frame's
// own filename. This guarantees an exact 1:1 frame correspondence with
// ground-truth label files named the same way (SUSTechPOINTS convention),
// unlike the live-ROS/rosbag-replay path where message drops, `--clock`
// sync, and nearest-timestamp pairing can desync predicted frames from GT.
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

#include <boost/filesystem.hpp>

#include "city_block.h"

namespace fs = boost::filesystem;

static void writeBoxesJson(const std::vector<Box>& boxes, const std::string& outPath)
{
    std::ofstream out(outPath);
    if (!out)
    {
        std::cerr << "failed to open output file " << outPath << std::endl;
        return;
    }

    out << "[\n";
    for (size_t i = 0; i < boxes.size(); ++i)
    {
        const Box& b = boxes[i];
        float cx = 0.5f * (b.x_min + b.x_max);
        float cy = 0.5f * (b.y_min + b.y_max);
        float cz = 0.5f * (b.z_min + b.z_max);
        float sx = b.x_max - b.x_min;
        float sy = b.y_max - b.y_min;
        float sz = b.z_max - b.z_min;

        out << "  {\n"
            << "    \"obj_id\": \"" << i << "\",\n"
            << "    \"obj_type\": \"Unknown\",\n"
            << "    \"psr\": {\n"
            << "      \"position\": {\"x\": " << cx << ", \"y\": " << cy << ", \"z\": " << cz << "},\n"
            << "      \"rotation\": {\"x\": 0, \"y\": 0, \"z\": 0},\n"
            << "      \"scale\": {\"x\": " << sx << ", \"y\": " << sy << ", \"z\": " << sz << "}\n"
            << "    }\n"
            << "  }" << (i + 1 < boxes.size() ? "," : "") << "\n";
    }
    out << "]\n";
}

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cerr << "usage: " << argv[0] << " <input_pcd_dir> <output_json_dir>" << std::endl;
        return 1;
    }
    std::string inputDir = argv[1];
    std::string outputDir = argv[2];
    fs::create_directories(outputDir);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<fs::path> stream = pointProcessorI->streamPcd(inputDir);

    int count = 0;
    for (const fs::path& p : stream)
    {
        if (p.extension() != ".pcd")
            continue;

        pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI = pointProcessorI->loadPcd(p.string());
        DetectionResult result = detectObstacles(pointProcessorI, inputCloudI);

        std::string stem = p.stem().string(); // e.g. "1783928982.182113409"
        std::string outPath = outputDir + "/" + stem + ".json";
        writeBoxesJson(result.boxes, outPath);
        ++count;
    }

    std::cout << "wrote " << count << " detection files to " << outputDir << std::endl;

    delete pointProcessorI;
    return 0;
}
