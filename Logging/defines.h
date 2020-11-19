#include <mutex>

class v_pid {
    public:
        // target for pid
        float setpoint;
        // current value for pid
        float value;
        // p constant
        float kp;
        // i constant
        float ki;
        // d constant
        float kd;
        // integrating factor
        float i;
        // last errors
        float lastError;
};

class vector3 {
    public:
        float x, y, z;
};

class gps_data {
    public:
        double lat, lng, alt, vel, hdg;
        bool valid;
        int numSats;
        int hz500_misses = 0;
        int hz100_misses = 0;
        int hz50_misses = 0;
};

class vehicle_info {
    public:
        vector3 rotation;
        vector3 gyro;
        vector3 acceleration;
        std::mutex mutex;
        gps_data gps;
        float motorCommands[6] = {0.0f};
        float motorAngleCommands[6] = {0.0f};
        long controlTicks;
        int iter_time;
};
