#include "robot_model_config.h"

#include "json/json.h"

#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace
{
std::vector<std::string> readStrings(const Json::Value &value)
{
    std::vector<std::string> result;
    result.reserve(value.size());
    for (const auto &entry : value)
        result.push_back(entry.asString());
    return result;
}

std::vector<double> readDoubles(const Json::Value &value)
{
    std::vector<double> result;
    result.reserve(value.size());
    for (const auto &entry : value)
        result.push_back(entry.asDouble());
    return result;
}

std::string resolvePath(const std::filesystem::path &base, const Json::Value &root, const char *key)
{
    const auto path = std::filesystem::path(root[key].asString());
    return (path.is_absolute() ? path : base / path).lexically_normal().string();
}
}

RobotModelConfig loadRobotModelConfig(const std::string &configPath)
{
    std::ifstream input(configPath, std::ios::binary);
    if (!input)
        throw std::runtime_error("Cannot open robot config: " + configPath);

    Json::Value root;
    Json::Reader reader;
    if (!reader.parse(input, root))
        throw std::runtime_error("Cannot parse robot config: " + configPath + "\n" + reader.getFormattedErrorMessages());

    RobotModelConfig config;
    config.configPath = std::filesystem::absolute(configPath).lexically_normal().string();
    const auto base = std::filesystem::path(config.configPath).parent_path();
    config.name = root["name"].asString();
    config.urdfPath = resolvePath(base, root, "urdf");
    config.scenePath = resolvePath(base, root, "scene");
    config.jointControlPath = resolvePath(base, root, "joint_control");

    const auto &frames = root["frames"];
    config.baseBody = frames["base_body"].asString();
    config.leftFootFrame = frames["left_foot"].asString();
    config.rightFootFrame = frames["right_foot"].asString();
    config.leftHandFrame = frames["left_hand"].asString();
    config.rightHandFrame = frames["right_hand"].asString();
    config.leftHipFrame = frames["left_hip"].asString();
    config.rightHipFrame = frames["right_hip"].asString();
    config.hipReferenceFrame = frames["hip_reference"].asString();

    const auto &sensors = root["sensors"];
    config.orientationSensor = sensors["orientation"].asString();
    config.velocitySensor = sensors["velocity"].asString();
    config.gyroSensor = sensors["gyro"].asString();
    config.accelerationSensor = sensors["acceleration"].asString();
    config.leftFootContactSensor = sensors.get("left_foot_contact", "").asString();
    config.rightFootContactSensor = sensors.get("right_foot_contact", "").asString();

    const auto &joints = root["joints"];
    config.joints.actuated = readStrings(joints["actuated"]);
    config.joints.leftArm = readStrings(joints["left_arm"]);
    config.joints.rightArm = readStrings(joints["right_arm"]);
    config.joints.leftLeg = readStrings(joints["left_leg"]);
    config.joints.rightLeg = readStrings(joints["right_leg"]);
    config.joints.waist = readStrings(joints["waist"]);
    config.joints.head = readStrings(joints["head"]);

    config.initialJointPositions = readDoubles(root["initial_joint_positions"]);
    if (config.initialJointPositions.size() != config.joints.actuated.size())
        throw std::runtime_error("initial_joint_positions size does not match actuated joint count");

    const auto &gait = root["gait"];
    config.nominalBaseHeight = gait["nominal_base_height"].asDouble();
    config.hipWidth = gait["hip_width"].asDouble();
    config.footHeight = gait["foot_height"].asDouble();
    config.swingTime = gait["swing_time"].asDouble();
    config.walkingSpeed = gait["walking_speed"].asDouble();
    if (root.isMember("simulation"))
    {
        const auto &simulation = root["simulation"];
        config.simulationTimeStep = simulation.get("timestep", config.simulationTimeStep).asDouble();
        config.simulationJointDamping = simulation.get("joint_damping", config.simulationJointDamping).asDouble();
        config.simulationJointFrictionLoss = simulation.get("joint_friction_loss", config.simulationJointFrictionLoss).asDouble();
        config.simulationJointArmature = simulation.get("joint_armature", config.simulationJointArmature).asDouble();
        config.simulationFloorFriction = simulation.get("floor_friction", config.simulationFloorFriction).asDouble();
        config.touchdownForce = simulation.get("touchdown_force", config.touchdownForce).asDouble();
    }
    return config;
}

RobotModelConfig azureLoongDefaultConfig()
{
    return loadRobotModelConfig("../common/robot_configs/azureloong.json");
}
