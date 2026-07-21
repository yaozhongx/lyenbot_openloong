#include "MJ_interface.h"
#include "PVT_ctrl.h"
#include "data_bus.h"
#include "robot_model_config.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
constexpr double kSettleDuration = 1.0;
constexpr double kExcitationDuration = 0.25;
constexpr double kPositionIncrement = 0.02;
constexpr double kMinimumTargetMotion = 0.003;
constexpr double kMaximumOtherMotion = 0.01;

struct TrialResult
{
    double expectedMotion{0.0};
    double oppositeMotion{0.0};
    double maximumOtherMotion{0.0};
    std::string maximumOtherJoint;
    bool passed{false};
};

void require(bool condition, const std::string &message)
{
    if (!condition)
        throw std::runtime_error(message);
}

std::vector<int> jointQposAddresses(const mjModel *model, const RobotModelConfig &config)
{
    std::vector<int> addresses;
    addresses.reserve(config.joints.actuated.size());
    for (const auto &name : config.joints.actuated)
    {
        const int jointId = mj_name2id(model, mjOBJ_JOINT, name.c_str());
        require(jointId >= 0, "MuJoCo joint missing: " + name);
        require(model->jnt_type[jointId] == mjJNT_HINGE, "Expected hinge joint: " + name);
        addresses.push_back(model->jnt_qposadr[jointId]);
    }
    return addresses;
}

void fillLimits(const mjModel *model, const RobotModelConfig &config,
                Eigen::VectorXd &maximumTorque, Eigen::VectorXd &maximumSpeed,
                Eigen::VectorXd &maximumPosition, Eigen::VectorXd &minimumPosition)
{
    const int count = static_cast<int>(config.joints.actuated.size());
    maximumTorque.resize(count);
    maximumSpeed = Eigen::VectorXd::Constant(count, 100.0);
    maximumPosition.resize(count);
    minimumPosition.resize(count);
    for (int index = 0; index < count; ++index)
    {
        const auto &name = config.joints.actuated[index];
        const int jointId = mj_name2id(model, mjOBJ_JOINT, name.c_str());
        int actuatorId = -1;
        for (int candidate = 0; candidate < model->nu; ++candidate)
            if (model->actuator_trnid[2 * candidate] == jointId)
            {
                actuatorId = candidate;
                break;
            }
        require(actuatorId >= 0, "MuJoCo actuator missing: " + name);
        maximumTorque[index] = std::max(std::abs(model->actuator_ctrlrange[2 * actuatorId]),
                                        std::abs(model->actuator_ctrlrange[2 * actuatorId + 1]));
        minimumPosition[index] = model->jnt_range[2 * jointId];
        maximumPosition[index] = model->jnt_range[2 * jointId + 1];
    }
}

void runControllerFor(mjModel *model, mjData *data, MJ_Interface &interface, PVT_Ctr &pvt,
                      DataBus &state, int stepCount)
{
    for (int step = 0; step < stepCount; ++step)
    {
        interface.updateSensorValues();
        interface.dataBusWrite(state);
        pvt.dataBusRead(state);
        pvt.calMotorsPVT();
        pvt.dataBusWrite(state);
        interface.setMotorsTorque(state.motors_tor_out);
        mj_step(model, data);
    }
    interface.updateSensorValues();
    interface.dataBusWrite(state);
}

std::vector<std::vector<double>> runTrajectory(
    mjModel *model, mjData *data, const RobotModelConfig &config,
    const Eigen::VectorXd &maximumTorque, const Eigen::VectorXd &maximumSpeed,
    const Eigen::VectorXd &maximumPosition, const Eigen::VectorXd &minimumPosition,
    int targetIndex, double direction)
{
    // LYENBOT MODIFY: every joint trial starts from the same generated home keyframe.
    require(model->nkey > 0, "Lyenbot dynamic mapping test requires the home keyframe");
    mj_resetDataKeyframe(model, data, 0);
    mj_forward(model, data);

    MJ_Interface interface(model, data, config);
    PVT_Ctr pvt(model->opt.timestep, config.jointControlPath.c_str(), config.joints.actuated,
                maximumTorque, maximumSpeed, maximumPosition, minimumPosition);
    DataBus state(static_cast<int>(config.joints.actuated.size()) + 6);
    state.motors_pos_des = config.initialJointPositions;
    state.motors_vel_des.assign(config.joints.actuated.size(), 0.0);
    state.motors_tor_des.assign(config.joints.actuated.size(), 0.0);

    const int settleSteps = static_cast<int>(std::ceil(kSettleDuration / model->opt.timestep));
    const int excitationSteps = static_cast<int>(std::ceil(kExcitationDuration / model->opt.timestep));
    runControllerFor(model, data, interface, pvt, state, settleSteps);
    if (targetIndex >= 0)
        state.motors_pos_des[targetIndex] += direction * kPositionIncrement;

    std::vector<std::vector<double>> trajectory;
    trajectory.reserve(excitationSteps);
    for (int step = 0; step < excitationSteps; ++step)
    {
        interface.updateSensorValues();
        interface.dataBusWrite(state);
        pvt.dataBusRead(state);
        pvt.calMotorsPVT();
        pvt.dataBusWrite(state);
        interface.setMotorsTorque(state.motors_tor_out);
        mj_step(model, data);

        interface.updateSensorValues();
        trajectory.push_back(interface.motor_pos);
    }
    return trajectory;
}

TrialResult compareTrajectory(const RobotModelConfig &config,
                              const std::vector<std::vector<double>> &baseline,
                              const std::vector<std::vector<double>> &excited,
                              int targetIndex, double direction)
{
    require(baseline.size() == excited.size(), "Baseline and excited trajectory sizes differ");
    TrialResult result;
    for (std::size_t step = 0; step < baseline.size(); ++step)
    {
        const double signedMotion = direction * (excited[step][targetIndex] - baseline[step][targetIndex]);
        result.expectedMotion = std::max(result.expectedMotion, signedMotion);
        result.oppositeMotion = std::max(result.oppositeMotion, -signedMotion);
        for (std::size_t index = 0; index < baseline[step].size(); ++index)
        {
            if (static_cast<int>(index) == targetIndex)
                continue;
            const double motion = std::abs(excited[step][index] - baseline[step][index]);
            if (motion > result.maximumOtherMotion)
            {
                result.maximumOtherMotion = motion;
                result.maximumOtherJoint = config.joints.actuated[index];
            }
        }
    }
    result.passed = result.expectedMotion >= kMinimumTargetMotion
                    && result.expectedMotion > result.oppositeMotion
                    && result.expectedMotion > result.maximumOtherMotion
                    && result.maximumOtherMotion <= kMaximumOtherMotion;
    return result;
}
}

int main(int argc, char **argv)
{
    mjModel *model = nullptr;
    mjData *data = nullptr;
    try
    {
        const std::string configPath = argc > 1 ? argv[1] : "../common/robot_configs/lyenbot.json";
        const RobotModelConfig config = loadRobotModelConfig(configPath);
        char error[2048] = {};
        model = mj_loadXML(config.scenePath.c_str(), nullptr, error, sizeof(error));
        require(model != nullptr, "Cannot load Lyenbot scene: " + std::string(error));
        data = mj_makeData(model);
        require(data != nullptr, "Cannot allocate MuJoCo data");
        require(model->nu == static_cast<int>(config.joints.actuated.size()),
                "Actuator count does not match configured joint count");
        jointQposAddresses(model, config);

        Eigen::VectorXd maximumTorque;
        Eigen::VectorXd maximumSpeed;
        Eigen::VectorXd maximumPosition;
        Eigen::VectorXd minimumPosition;
        fillLimits(model, config, maximumTorque, maximumSpeed, maximumPosition, minimumPosition);

        const auto baseline = runTrajectory(model, data, config, maximumTorque, maximumSpeed,
                                            maximumPosition, minimumPosition, -1, 1.0);

        int passedCount = 0;
        std::cout << "id  result  joint  target_motion  opposite_motion  max_other_motion  max_other_joint\n";
        for (int index = 0; index < static_cast<int>(config.joints.actuated.size()); ++index)
        {
            const double positiveRoom = maximumPosition[index] - config.initialJointPositions[index];
            const double negativeRoom = config.initialJointPositions[index] - minimumPosition[index];
            const double direction = positiveRoom >= kPositionIncrement ? 1.0 : -1.0;
            require(direction > 0.0 || negativeRoom >= kPositionIncrement,
                    "No safe position increment available for " + config.joints.actuated[index]);
            const auto excited = runTrajectory(model, data, config, maximumTorque, maximumSpeed,
                                               maximumPosition, minimumPosition, index, direction);
            const TrialResult result = compareTrajectory(config, baseline, excited, index, direction);
            passedCount += result.passed ? 1 : 0;
            std::cout << std::setw(2) << index << "  " << (result.passed ? "PASS" : "FAIL") << "  "
                      << config.joints.actuated[index] << "  " << std::scientific << std::setprecision(6)
                      << result.expectedMotion << "  " << result.oppositeMotion << "  "
                      << result.maximumOtherMotion << "  "
                      << (result.maximumOtherJoint.empty() ? "-" : result.maximumOtherJoint) << '\n';
        }
        std::cout << "JOINT_MAPPING_SUMMARY passed=" << passedCount
                  << " total=" << config.joints.actuated.size() << '\n';
        const bool allPassed = passedCount == static_cast<int>(config.joints.actuated.size());
        std::cout << (allPassed ? "LYENBOT_JOINT_MAPPING_OK" : "LYENBOT_JOINT_MAPPING_FAILED") << '\n';
        mj_deleteData(data);
        mj_deleteModel(model);
        return allPassed ? 0 : 1;
    }
    catch (const std::exception &exception)
    {
        if (data)
            mj_deleteData(data);
        if (model)
            mj_deleteModel(model);
        std::cerr << "LYENBOT_JOINT_MAPPING_FAILED: " << exception.what() << '\n';
        return 1;
    }
}
