#pragma once

#include <string>
#include <vector>
#include <limits>

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
    std::string leftFootCollisionGeom;
    std::string rightFootCollisionGeom;

    JointLayout joints;
    std::vector<double> initialJointPositions;
    double nominalBaseHeight{1.0};
    double hipWidth{0.2};
    double footHeight{0.05};
    double swingTime{0.25};
    double walkingSpeed{0.15};
    double initialDoubleSupportTime{0.0};
    double doubleSupportTransferTime{0.40};
    double supportBaseHeight{1.0};
    double supportBaseRoll{0.0};
    double supportComInwardOffset{0.0};
    double supportBaseOutwardOffset{0.0};
    double swingStepHeight{0.025};
    double swingPeakPitch{0.0};
    double swingPitchRisePhase{0.20};
    double swingPitchReturnPhase{0.70};
    bool preventOutwardSwingExpansion{false};
    double swingLandingWorldHeight{std::numeric_limits<double>::quiet_NaN()};
    double contactHalfLength{0.0};
    double contactHalfWidth{0.0};
    double simulationTimeStep{0.001};
    double simulationJointDamping{0.1};
    double simulationJointFrictionLoss{0.02};
    double simulationJointArmature{0.01};
    double simulationFloorFriction{1.0};
    double touchdownForce{20.0};
    double stateEstimatorFootFrameGroundHeight{0.07};
    bool stateEstimatorCompensateAccelerometerGravity{false};
    double stateEstimatorAngularVelocityMeasurementNoiseScale{1.0};
};

RobotModelConfig loadRobotModelConfig(const std::string &configPath);
RobotModelConfig azureLoongDefaultConfig();
