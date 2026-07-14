#include "robot_model_config.h"
#include "urdf_model_loader.h"

#include <Eigen/Eigenvalues>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <mujoco/mujoco.h>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <set>
#include <stdexcept>

namespace
{
void require(bool condition, const std::string &message)
{
    if (!condition)
        throw std::runtime_error(message);
}

void requireFrame(const pinocchio::Model &model, const std::string &name)
{
    require(model.existFrame(name), "Missing URDF frame: " + name);
}
}

int main(int argc, char **argv)
{
    try
    {
        const std::string configPath = argc > 1 ? argv[1] : "../common/robot_configs/lyenbot.json";
        const RobotModelConfig config = loadRobotModelConfig(configPath);
        require(std::filesystem::exists(config.urdfPath), "URDF does not exist: " + config.urdfPath);

        pinocchio::Model model;
        int trimmedAttributes = 0;
        buildFloatingBaseModelFromUrdf(config.urdfPath, model, &trimmedAttributes);
        require(model.nv == static_cast<int>(config.joints.actuated.size()) + 6,
                "nv does not match configured actuated joint count");
        require(model.nq == model.nv + 1, "Expected a floating-base model with nq=nv+1");

        requireFrame(model, config.baseBody);
        requireFrame(model, config.leftFootFrame);
        requireFrame(model, config.rightFootFrame);
        requireFrame(model, config.leftHandFrame);
        requireFrame(model, config.rightHandFrame);

        std::set<std::string> uniqueNames;
        std::cout << "robot=" << config.name << " nq=" << model.nq << " nv=" << model.nv
                  << " actuated=" << config.joints.actuated.size() << '\n';
        std::cout << "normalized_attribute_whitespace=" << trimmedAttributes << '\n';
        std::cout << "id  joint  idx_q  idx_v  lower  upper  velocity  effort\n";
        for (std::size_t i = 0; i < config.joints.actuated.size(); ++i)
        {
            const auto &name = config.joints.actuated[i];
            require(uniqueNames.insert(name).second, "Duplicate configured joint: " + name);
            require(model.existJointName(name), "Missing URDF joint: " + name);
            const auto jointId = model.getJointId(name);
            const auto idxQ = model.idx_qs[jointId];
            const auto idxV = model.idx_vs[jointId];
            require(model.nqs[jointId] == 1 && model.nvs[jointId] == 1, "Only 1-DoF actuated joints are supported: " + name);
            require(model.lowerPositionLimit[idxQ] <= model.upperPositionLimit[idxQ], "Invalid position limit: " + name);
            require(model.velocityLimit[idxV] > 0, "Invalid velocity limit: " + name);
            require(model.effortLimit[idxV] > 0, "Invalid effort limit: " + name);
            std::cout << std::setw(2) << i << "  " << name << "  " << idxQ << "  " << idxV << "  "
                      << model.lowerPositionLimit[idxQ] << "  " << model.upperPositionLimit[idxQ] << "  "
                      << model.velocityLimit[idxV] << "  " << model.effortLimit[idxV] << '\n';
        }

        require(static_cast<int>(uniqueNames.size()) == model.nv - 6,
                "Config does not name every actuated URDF joint exactly once");
        pinocchio::Data data(model);
        Eigen::VectorXd q = pinocchio::neutral(model);
        q(2) = config.nominalBaseHeight;
        for (std::size_t i = 0; i < config.joints.actuated.size(); ++i)
        {
            const auto jointId = model.getJointId(config.joints.actuated[i]);
            q(model.idx_qs[jointId]) = config.initialJointPositions[i];
        }
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
        const auto centerOfMass = pinocchio::centerOfMass(model, data, q);
        const auto leftFootId = model.getFrameId(config.leftFootFrame, pinocchio::BODY);
        const auto rightFootId = model.getFrameId(config.rightFootFrame, pinocchio::BODY);
        std::cout << "initial_left_foot=" << data.oMf[leftFootId].translation().transpose() << '\n'
                  << "initial_right_foot=" << data.oMf[rightFootId].translation().transpose() << '\n'
                  << "initial_com=" << centerOfMass.transpose() << '\n';

        if (std::filesystem::exists(config.scenePath))
        {
            char error[2048] = {};
            mjModel *mujocoModel = mj_loadXML(config.scenePath.c_str(), nullptr, error, sizeof(error));
            require(mujocoModel != nullptr, "Cannot load generated MuJoCo scene: " + std::string(error));
            mjData *mujocoData = mj_makeData(mujocoModel);
            require(mujocoData != nullptr, "Cannot allocate MuJoCo data");
            if (mujocoModel->nkey > 0)
                mj_resetDataKeyframe(mujocoModel, mujocoData, 0);
            mj_forward(mujocoModel, mujocoData);
            require(mujocoModel->nq == model.nq && mujocoModel->nv == model.nv,
                    "MuJoCo and Pinocchio generalized dimensions differ");
            require(mujocoModel->nu == static_cast<int>(config.joints.actuated.size()),
                    "MuJoCo actuator count differs from configuration");
            for (const auto &name : config.joints.actuated)
            {
                const int mujocoJoint = mj_name2id(mujocoModel, mjOBJ_JOINT, name.c_str());
                require(mujocoJoint >= 0, "MuJoCo joint missing: " + name);
                const auto pinJoint = model.getJointId(name);
                const auto pinQ = model.idx_qs[pinJoint];
                const auto pinV = model.idx_vs[pinJoint];
                require(std::abs(mujocoModel->jnt_range[2 * mujocoJoint] - model.lowerPositionLimit[pinQ]) < 1e-6,
                        "Lower limit mismatch: " + name);
                require(std::abs(mujocoModel->jnt_range[2 * mujocoJoint + 1] - model.upperPositionLimit[pinQ]) < 1e-6,
                        "Upper limit mismatch: " + name);
                int actuator = -1;
                for (int candidate = 0; candidate < mujocoModel->nu; ++candidate)
                    if (mujocoModel->actuator_trnid[2 * candidate] == mujocoJoint)
                        actuator = candidate;
                require(actuator >= 0, "MuJoCo actuator missing: " + name);
                require(std::abs(mujocoModel->actuator_ctrlrange[2 * actuator] + model.effortLimit[pinV]) < 1e-6
                        && std::abs(mujocoModel->actuator_ctrlrange[2 * actuator + 1] - model.effortLimit[pinV]) < 1e-6,
                        "Effort/ctrlrange mismatch: " + name);
            }
            const int leftSite = mj_name2id(mujocoModel, mjOBJ_SITE, "lf-tc");
            const int rightSite = mj_name2id(mujocoModel, mjOBJ_SITE, "rf-tc");
            require(leftSite >= 0 && rightSite >= 0, "Generated foot contact sites are missing");
            const Eigen::Map<const Eigen::Vector3d> leftSitePosition(mujocoData->site_xpos + 3 * leftSite);
            const Eigen::Map<const Eigen::Vector3d> rightSitePosition(mujocoData->site_xpos + 3 * rightSite);
            const double leftError = (leftSitePosition - data.oMf[leftFootId].translation()).norm();
            const double rightError = (rightSitePosition - data.oMf[rightFootId].translation()).norm();
            require(leftError < 1e-6 && rightError < 1e-6, "Pinocchio/MuJoCo foot pose mismatch");
            std::cout << "mujoco_nq=" << mujocoModel->nq << " mujoco_nv=" << mujocoModel->nv
                      << " mujoco_nu=" << mujocoModel->nu << " foot_pose_error=" << leftError << ' ' << rightError << '\n';
            mj_deleteData(mujocoData);
            mj_deleteModel(mujocoModel);
        }
        std::cout << "MODEL_CHECK_OK\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "MODEL_CHECK_FAILED: " << error.what() << '\n';
        return 1;
    }
}
