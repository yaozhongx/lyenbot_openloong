#pragma once

#include <string>
#include <vector>

struct JointLayout
{
    std::vector<std::string> actuated;
    std::vector<std::string> leftArm;
    std::vector<std::string> rightArm;
    std::vector<std::string> leftLeg;
    std::vector<std::string> rightLeg;
    std::vector<std::string> waist;
    std::vector<std::string> head;
};

struct RobotModelConfig
{
    std::string name;
    std::string configPath;
    std::string urdfPath;
    std::string scenePath;
    std::string jointControlPath;

    std::string baseBody;
    std::string leftFootFrame;
    std::string rightFootFrame;
    std::string leftHandFrame;
    std::string rightHandFrame;
    std::string leftHipFrame;
    std::string rightHipFrame;
    std::string hipReferenceFrame;

    std::string orientationSensor;
    std::string velocitySensor;
    std::string gyroSensor;
    std::string accelerationSensor;
    std::string leftFootContactSensor;
    std::string rightFootContactSensor;

    JointLayout joints;
    std::vector<double> initialJointPositions;
    double nominalBaseHeight{1.0};
    double hipWidth{0.2};
    double footHeight{0.05};
    double swingTime{0.25};
    double walkingSpeed{0.15};
    double simulationTimeStep{0.001};
    double simulationJointDamping{0.1};
    double simulationJointFrictionLoss{0.02};
    double simulationJointArmature{0.01};
    double simulationFloorFriction{1.0};
    double touchdownForce{20.0};
};

RobotModelConfig loadRobotModelConfig(const std::string &configPath);
RobotModelConfig azureLoongDefaultConfig();
