#include <iostream>
#include <vector>
#include <cmath>
#include <random>

struct Object {
    float x, y; // Object's position
};

struct Detection {
    float distance;  // Distance to the object
    float angle;     // Angle to the object
};

class RadarSensor {
private:
    float range;
    float fov;   // In degrees
    float noise_std_dev;
    
    std::default_random_engine generator;
    std::normal_distribution<float> noise_distribution;

public:
    RadarSensor(float range, float fov, float noise_std_dev = 0.1)
        : range(range), fov(fov), noise_std_dev(noise_std_dev),
          noise_distribution(0.0, noise_std_dev) {}

    std::vector<Detection> detectObjects(const std::vector<Object>& objects, float radar_x, float radar_y) {
        std::vector<Detection> detections;

        for (const auto& obj : objects) {
            float dx = obj.x - radar_x;
            float dy = obj.y - radar_y;
            float distance = std::sqrt(dx * dx + dy * dy);
            float angle = std::atan2(dy, dx) * 180.0 / M_PI;

            if (distance <= range && std::abs(angle) <= fov / 2) {
                // Add noise to the distance
                distance += noise_distribution(generator);
                detections.push_back({distance, angle});
            }
        }
        return detections;
    }
};

int main() {
    // Define a radar with a 50m range and 90-degree FOV
    RadarSensor radar(50.0f, 90.0f);

    // Environment with some objects
    std::vector<Object> objects = {
        {30.0f, 10.0f}, // Object at (30, 10)
        {60.0f, 20.0f}, // Out of range
        {20.0f, -5.0f}  // Within range and FOV
    };

    // Simulate radar detection
    auto detections = radar.detectObjects(objects, 0.0f, 0.0f);

    // Print detections
    for (const auto& detection : detections) {
        std::cout << "Detected object at distance: " << detection.distance
                  << "m, angle: " << detection.angle << "°\n";
    }

    return 0;
}
