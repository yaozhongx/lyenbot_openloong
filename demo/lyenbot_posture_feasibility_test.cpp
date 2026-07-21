// LYENBOT MODIFY: offline quasi-static posture feasibility scan.
#include "pino_kin_dyn.h"
#include "robot_model_config.h"
#include "urdf_model_loader.h"

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
struct Candidate
{
    bool valid{false};
    double score{-std::numeric_limits<double>::infinity()};
    double baseY{0.0};
    double baseHeight{0.0};
    double baseRoll{0.0};
    double comRelativeY{0.0};
    double minimumLegMargin{0.0};
    double leftAnkleRoll{0.0};
    double rightAnkleRoll{0.0};
    Eigen::VectorXd q;
};

struct SwingCandidate
{
    bool valid{false};
    double score{-std::numeric_limits<double>::infinity()};
    double height{0.0};
    double forward{0.0};
    double outward{0.0};
    double pitch{0.0};
    double knee{0.0};
    double anklePitch{0.0};
    double anklePitchMargin{0.0};
    double minimumLegMargin{0.0};
};

Eigen::VectorXd floatingConfiguration(const RobotModelConfig &config,
                                      const pinocchio::Model &floatingModel,
                                      const pinocchio::Model &fixedModel,
                                      const Eigen::VectorXd &fixedQ,
                                      const Eigen::Vector3d &basePosition,
                                      const Eigen::Matrix3d &baseRotation)
{
    Eigen::VectorXd q = pinocchio::neutral(floatingModel);
    q.head<3>() = basePosition;
    const Eigen::Quaterniond quaternion(baseRotation);
    q[3] = quaternion.x();
    q[4] = quaternion.y();
    q[5] = quaternion.z();
    q[6] = quaternion.w();
    for (const auto &name : config.joints.actuated)
    {
        const auto floatingJoint = floatingModel.getJointId(name);
        const auto fixedJoint = fixedModel.getJointId(name);
        q[floatingModel.idx_qs[floatingJoint]] = fixedQ[fixedModel.idx_qs[fixedJoint]];
    }
    return q;
}

double jointPosition(const pinocchio::Model &model, const Eigen::VectorXd &q,
                     const std::string &name)
{
    return q[model.idx_qs[model.getJointId(name)]];
}

double jointPositionMargin(const pinocchio::Model &model, const Eigen::VectorXd &q,
                           const std::string &name)
{
    const auto joint = model.getJointId(name);
    const int index = model.idx_qs[joint];
    return std::min(q[index] - model.lowerPositionLimit[index],
                    model.upperPositionLimit[index] - q[index]);
}

double minimumLegPositionMargin(const RobotModelConfig &config,
                                const pinocchio::Model &model,
                                const Eigen::VectorXd &q)
{
    double margin = std::numeric_limits<double>::infinity();
    for (const auto &name : config.joints.leftLeg)
        margin = std::min(margin, jointPositionMargin(model, q, name));
    for (const auto &name : config.joints.rightLeg)
        margin = std::min(margin, jointPositionMargin(model, q, name));
    return margin;
}

void scanSupport(const RobotModelConfig &config, Pin_KinDyn &kinDyn,
                 const pinocchio::Model &fixedModel,
                 const pinocchio::SE3 &leftFootAnchor,
                 const pinocchio::SE3 &rightFootAnchor,
                 bool supportLeft)
{
    pinocchio::Data data(kinDyn.model_biped);
    const double direction = supportLeft ? 1.0 : -1.0;
    const double stanceY = supportLeft ? leftFootAnchor.translation().y()
                                       : rightFootAnchor.translation().y();
    const double desiredComRelativeY = supportLeft ? -0.010 : 0.010;
    constexpr double supportHalfWidthWithMargin = 0.035;
    constexpr double minimumKneeFlexion = 0.03;

    std::size_t samples = 0;
    std::size_t ikSuccess = 0;
    std::size_t jointValid = 0;
    std::size_t supportValid = 0;
    Candidate best;
    Candidate leastRoll;
    Candidate closestCom;
    std::vector<Candidate> rollProfile(41);

    for (int heightIndex = 0; heightIndex <= 12; ++heightIndex)
    {
        const double baseHeight = 0.73 + 0.01 * heightIndex;
        for (int lateralIndex = 0; lateralIndex <= 28; ++lateralIndex)
        {
            const double baseY = direction * 0.005 * lateralIndex;
            for (int rollIndex = 0; rollIndex <= 40; ++rollIndex)
            {
                ++samples;
                const double baseRoll = -direction * 0.01 * rollIndex;
                const Eigen::Matrix3d baseRotation =
                    Eigen::AngleAxisd(baseRoll, Eigen::Vector3d::UnitX()).toRotationMatrix();
                const Eigen::Vector3d basePosition(0.0, baseY, baseHeight);
                const Eigen::Matrix3d leftRotationInBase =
                    baseRotation.transpose() * leftFootAnchor.rotation();
                const Eigen::Matrix3d rightRotationInBase =
                    baseRotation.transpose() * rightFootAnchor.rotation();
                const Eigen::Vector3d leftPositionInBase =
                    baseRotation.transpose() * (leftFootAnchor.translation() - basePosition);
                const Eigen::Vector3d rightPositionInBase =
                    baseRotation.transpose() * (rightFootAnchor.translation() - basePosition);

                const auto ik = kinDyn.computeInK_Leg(leftRotationInBase, leftPositionInBase,
                                                       rightRotationInBase, rightPositionInBase);
                if (ik.status != 0)
                    continue;
                ++ikSuccess;
                const Eigen::VectorXd q = floatingConfiguration(config, kinDyn.model_biped,
                                                                 fixedModel, ik.jointPosRes,
                                                                 basePosition, baseRotation);

                bool withinLimits = true;
                double minimumLegMargin = std::numeric_limits<double>::infinity();
                std::vector<std::string> legNames = config.joints.leftLeg;
                legNames.insert(legNames.end(), config.joints.rightLeg.begin(),
                                config.joints.rightLeg.end());
                for (const auto &name : legNames)
                {
                    const auto joint = kinDyn.model_biped.getJointId(name);
                    const int index = kinDyn.model_biped.idx_qs[joint];
                    const double lowerMargin = q[index] - kinDyn.model_biped.lowerPositionLimit[index];
                    const double upperMargin = kinDyn.model_biped.upperPositionLimit[index] - q[index];
                    minimumLegMargin = std::min(minimumLegMargin,
                                                std::min(lowerMargin, upperMargin));
                    withinLimits = withinLimits && lowerMargin >= -1e-8 && upperMargin >= -1e-8;
                }
                withinLimits = withinLimits
                    && jointPosition(kinDyn.model_biped, q, "left_knee_pitch_joint")
                           >= minimumKneeFlexion
                    && jointPosition(kinDyn.model_biped, q, "right_knee_pitch_joint")
                           >= minimumKneeFlexion;
                if (!withinLimits)
                    continue;
                ++jointValid;

                const Eigen::Vector3d com = pinocchio::centerOfMass(kinDyn.model_biped, data, q);
                const double comRelativeY = com.y() - stanceY;
                if (std::abs(comRelativeY) > supportHalfWidthWithMargin)
                    continue;
                ++supportValid;

                const double score = minimumLegMargin
                    - 0.5 * std::abs(comRelativeY - desiredComRelativeY)
                    - 0.02 * std::abs(baseRoll)
                    - 0.10 * std::abs(baseHeight - config.nominalBaseHeight);
                Candidate candidate;
                candidate.valid = true;
                candidate.score = score;
                candidate.baseY = baseY;
                candidate.baseHeight = baseHeight;
                candidate.baseRoll = baseRoll;
                candidate.comRelativeY = comRelativeY;
                candidate.minimumLegMargin = minimumLegMargin;
                candidate.leftAnkleRoll = jointPosition(kinDyn.model_biped, q,
                                                         "left_ankle_roll_joint");
                candidate.rightAnkleRoll = jointPosition(kinDyn.model_biped, q,
                                                          "right_ankle_roll_joint");
                candidate.q = q;
                if (!best.valid || candidate.score > best.score)
                    best = candidate;
                if (!leastRoll.valid
                    || std::abs(candidate.baseRoll) < std::abs(leastRoll.baseRoll) - 1e-12
                    || (std::abs(std::abs(candidate.baseRoll) - std::abs(leastRoll.baseRoll))
                            <= 1e-12
                        && candidate.score > leastRoll.score))
                    leastRoll = candidate;
                if (!closestCom.valid
                    || std::abs(candidate.comRelativeY)
                           < std::abs(closestCom.comRelativeY) - 1e-12
                    || (std::abs(std::abs(candidate.comRelativeY)
                                 - std::abs(closestCom.comRelativeY)) <= 1e-12
                        && candidate.minimumLegMargin > closestCom.minimumLegMargin))
                    closestCom = candidate;
                const int absoluteRollIndex = static_cast<int>(
                    std::lround(std::abs(candidate.baseRoll) / 0.01));
                if (!rollProfile[absoluteRollIndex].valid
                    || candidate.minimumLegMargin
                           > rollProfile[absoluteRollIndex].minimumLegMargin)
                    rollProfile[absoluteRollIndex] = candidate;
            }
        }
    }

    std::cout << "support=" << (supportLeft ? "left" : "right")
              << " samples=" << samples << " ik_success=" << ikSuccess
              << " joint_valid=" << jointValid << " support_valid=" << supportValid << '\n';
    if (!best.valid)
    {
        std::cout << "best=none\n";
        return;
    }
    std::cout << std::setprecision(8)
              << "best base_y=" << best.baseY << " base_z=" << best.baseHeight
              << " roll=" << best.baseRoll << " com_relative_y=" << best.comRelativeY
              << " min_leg_margin=" << best.minimumLegMargin
              << " left_ankle_roll=" << best.leftAnkleRoll
              << " right_ankle_roll=" << best.rightAnkleRoll << '\n';
    std::cout << "least_roll base_y=" << leastRoll.baseY
              << " base_z=" << leastRoll.baseHeight << " roll=" << leastRoll.baseRoll
              << " com_relative_y=" << leastRoll.comRelativeY
              << " min_leg_margin=" << leastRoll.minimumLegMargin
              << " left_ankle_roll=" << leastRoll.leftAnkleRoll
              << " right_ankle_roll=" << leastRoll.rightAnkleRoll << '\n';
    std::cout << "closest_com base_y=" << closestCom.baseY
              << " base_z=" << closestCom.baseHeight << " roll=" << closestCom.baseRoll
              << " com_relative_y=" << closestCom.comRelativeY
              << " min_leg_margin=" << closestCom.minimumLegMargin
              << " left_ankle_roll=" << closestCom.leftAnkleRoll
              << " right_ankle_roll=" << closestCom.rightAnkleRoll << '\n';
    for (const int rollIndex : {5, 10, 15, 20, 23, 25, 30, 35, 40})
    {
        const Candidate &profile = rollProfile[rollIndex];
        std::cout << "roll_profile abs_roll=" << 0.01 * rollIndex
                  << " feasible=" << profile.valid;
        if (profile.valid)
            std::cout << " base_y=" << profile.baseY << " base_z=" << profile.baseHeight
                      << " com_relative_y=" << profile.comRelativeY
                      << " min_leg_margin=" << profile.minimumLegMargin
                      << " left_ankle_roll=" << profile.leftAnkleRoll
                      << " right_ankle_roll=" << profile.rightAnkleRoll;
        std::cout << '\n';
    }
    for (const auto &name : config.joints.leftLeg)
        std::cout << name << '=' << jointPosition(kinDyn.model_biped, best.q, name) << ' ';
    for (const auto &name : config.joints.rightLeg)
        std::cout << name << '=' << jointPosition(kinDyn.model_biped, best.q, name) << ' ';
    std::cout << '\n';
}

void printSwingCandidate(const std::string &prefix, const SwingCandidate &candidate)
{
    std::cout << prefix << " feasible=" << candidate.valid;
    if (candidate.valid)
    {
        std::cout << " height=" << candidate.height
                  << " forward=" << candidate.forward
                  << " outward=" << candidate.outward
                  << " pitch=" << candidate.pitch
                  << " knee=" << candidate.knee
                  << " ankle_pitch=" << candidate.anklePitch
                  << " ankle_pitch_margin=" << candidate.anklePitchMargin
                  << " min_leg_margin=" << candidate.minimumLegMargin;
    }
    std::cout << '\n';
}

// LYENBOT MODIFY: scan complete 6D swing-foot targets before changing the
// online WBC path. A leg has no spare knee DoF once base and foot pose are set.
void scanSwingFamily(const RobotModelConfig &config, Pin_KinDyn &kinDyn,
                     const pinocchio::Model &fixedModel,
                     const pinocchio::SE3 &leftFootAnchor,
                     const pinocchio::SE3 &rightFootAnchor,
                     bool supportLeft, const std::string &family,
                     double baseHeight, double baseRollMagnitude,
                     double baseYMagnitude)
{
    constexpr double requiredAnklePitchMargin = 0.060;
    constexpr double requiredMinimumLegMargin = 0.015;
    const double side = supportLeft ? 1.0 : -1.0;
    const Eigen::Vector3d basePosition(0.0, side * baseYMagnitude, baseHeight);
    const double baseRoll = -side * baseRollMagnitude;
    const Eigen::Matrix3d baseRotation =
        Eigen::AngleAxisd(baseRoll, Eigen::Vector3d::UnitX()).toRotationMatrix();
    const pinocchio::SE3 &stanceAnchor = supportLeft ? leftFootAnchor : rightFootAnchor;
    const pinocchio::SE3 &swingAnchor = supportLeft ? rightFootAnchor : leftFootAnchor;
    const std::string swingKneeName = supportLeft ? "right_knee_pitch_joint"
                                                   : "left_knee_pitch_joint";
    const std::string swingAnklePitchName = supportLeft ? "right_ankle_pitch_joint"
                                                         : "left_ankle_pitch_joint";

    std::size_t samples = 0;
    std::size_t ikSuccess = 0;
    std::size_t jointValid = 0;
    std::size_t clearanceValid = 0;
    SwingCandidate best;
    SwingCandidate maximumKnee;
    SwingCandidate baselineHeight;
    std::vector<SwingCandidate> heightProfile(13);
    std::vector<SwingCandidate> pitchProfile(13);

    for (int heightIndex = 0; heightIndex <= 12; ++heightIndex)
    {
        const double height = 0.005 * heightIndex;
        for (int forwardIndex = 0; forwardIndex <= 8; ++forwardIndex)
        {
            const double forward = 0.010 * forwardIndex;
            for (int outwardIndex = 0; outwardIndex <= 4; ++outwardIndex)
            {
                const double outward = 0.010 * outwardIndex;
                for (int pitchIndex = -6; pitchIndex <= 6; ++pitchIndex)
                {
                    ++samples;
                    const double pitch = 0.050 * pitchIndex;
                    pinocchio::SE3 leftTarget = leftFootAnchor;
                    pinocchio::SE3 rightTarget = rightFootAnchor;
                    pinocchio::SE3 &swingTarget = supportLeft ? rightTarget : leftTarget;
                    swingTarget.translation() = swingAnchor.translation();
                    swingTarget.translation().x() += forward;
                    swingTarget.translation().y() -= side * outward;
                    swingTarget.translation().z() += height;
                    swingTarget.rotation() = Eigen::AngleAxisd(
                        pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
                    pinocchio::SE3 &stanceTarget = supportLeft ? leftTarget : rightTarget;
                    stanceTarget = stanceAnchor;

                    const Eigen::Matrix3d leftRotationInBase =
                        baseRotation.transpose() * leftTarget.rotation();
                    const Eigen::Matrix3d rightRotationInBase =
                        baseRotation.transpose() * rightTarget.rotation();
                    const Eigen::Vector3d leftPositionInBase =
                        baseRotation.transpose() * (leftTarget.translation() - basePosition);
                    const Eigen::Vector3d rightPositionInBase =
                        baseRotation.transpose() * (rightTarget.translation() - basePosition);
                    const auto ik = kinDyn.computeInK_Leg(leftRotationInBase, leftPositionInBase,
                                                           rightRotationInBase, rightPositionInBase);
                    if (ik.status != 0)
                        continue;
                    ++ikSuccess;
                    const Eigen::VectorXd q = floatingConfiguration(
                        config, kinDyn.model_biped, fixedModel, ik.jointPosRes,
                        basePosition, baseRotation);
                    const double minimumLegMargin = minimumLegPositionMargin(
                        config, kinDyn.model_biped, q);
                    if (minimumLegMargin < -1e-8)
                        continue;
                    ++jointValid;

                    SwingCandidate candidate;
                    candidate.valid = true;
                    candidate.height = height;
                    candidate.forward = forward;
                    candidate.outward = outward;
                    candidate.pitch = pitch;
                    candidate.knee = jointPosition(kinDyn.model_biped, q, swingKneeName);
                    candidate.anklePitch = jointPosition(
                        kinDyn.model_biped, q, swingAnklePitchName);
                    candidate.anklePitchMargin = jointPositionMargin(
                        kinDyn.model_biped, q, swingAnklePitchName);
                    candidate.minimumLegMargin = minimumLegMargin;
                    if (candidate.anklePitchMargin < requiredAnklePitchMargin
                        || candidate.minimumLegMargin < requiredMinimumLegMargin)
                        continue;
                    ++clearanceValid;

                    // Prefer visible knee flexion, then retain joint clearance and
                    // avoid relying on a large foot pitch or an extreme step.
                    candidate.score = candidate.knee
                        + 0.50 * candidate.anklePitchMargin
                        + 0.25 * candidate.minimumLegMargin
                        - 0.08 * std::abs(candidate.pitch)
                        - 0.10 * std::abs(candidate.forward - 0.030)
                        - 0.05 * std::abs(candidate.outward - 0.020);
                    if (!best.valid || candidate.score > best.score)
                        best = candidate;
                    if (!maximumKnee.valid || candidate.knee > maximumKnee.knee)
                        maximumKnee = candidate;
                    if (heightIndex == 5
                        && (!baselineHeight.valid || candidate.score > baselineHeight.score))
                        baselineHeight = candidate;
                    if (!heightProfile[heightIndex].valid
                        || candidate.score > heightProfile[heightIndex].score)
                        heightProfile[heightIndex] = candidate;
                    const int pitchProfileIndex = pitchIndex + 6;
                    if (!pitchProfile[pitchProfileIndex].valid
                        || candidate.score > pitchProfile[pitchProfileIndex].score)
                        pitchProfile[pitchProfileIndex] = candidate;
                }
            }
        }
    }

    std::cout << std::setprecision(8)
              << "swing_scan family=" << family
              << " support=" << (supportLeft ? "left" : "right")
              << " base_y=" << basePosition.y()
              << " base_z=" << baseHeight << " roll=" << baseRoll
              << " samples=" << samples << " ik_success=" << ikSuccess
              << " joint_valid=" << jointValid
              << " clearance_valid=" << clearanceValid << '\n';
    printSwingCandidate("swing_best", best);
    printSwingCandidate("swing_max_knee", maximumKnee);
    printSwingCandidate("swing_25mm", baselineHeight);
    for (const int heightIndex : {5, 7, 8, 10, 12})
        printSwingCandidate("swing_height_profile requested_height="
                                + std::to_string(0.005 * heightIndex),
                            heightProfile[heightIndex]);
    for (const int pitchIndex : {2, 4, 6, 8, 10})
        printSwingCandidate("swing_pitch_profile requested_pitch="
                                + std::to_string(0.050 * (pitchIndex - 6)),
                            pitchProfile[pitchIndex]);
}
}

int main(int argc, char **argv)
{
    try
    {
        const std::string configPath = argc > 1 ? argv[1]
                                                 : "../common/robot_configs/lyenbot.json";
        const RobotModelConfig config = loadRobotModelConfig(configPath);
        Pin_KinDyn kinDyn(config);
        pinocchio::Model fixedModel;
        buildFixedBaseModelFromUrdf(config.urdfPath, fixedModel);

        pinocchio::Data initialData(kinDyn.model_biped);
        Eigen::VectorXd initialQ = pinocchio::neutral(kinDyn.model_biped);
        initialQ[2] = config.nominalBaseHeight;
        for (std::size_t index = 0; index < config.joints.actuated.size(); ++index)
        {
            const auto joint = kinDyn.model_biped.getJointId(config.joints.actuated[index]);
            initialQ[kinDyn.model_biped.idx_qs[joint]] = config.initialJointPositions[index];
        }
        pinocchio::forwardKinematics(kinDyn.model_biped, initialData, initialQ);
        pinocchio::updateFramePlacements(kinDyn.model_biped, initialData);
        const auto leftFrame = kinDyn.model_biped.getFrameId(config.leftFootFrame,
                                                              pinocchio::BODY);
        const auto rightFrame = kinDyn.model_biped.getFrameId(config.rightFootFrame,
                                                               pinocchio::BODY);
        const pinocchio::SE3 leftFootAnchor = initialData.oMf[leftFrame];
        const pinocchio::SE3 rightFootAnchor = initialData.oMf[rightFrame];
        std::cout << "left_anchor=" << leftFootAnchor.translation().transpose()
                  << " right_anchor=" << rightFootAnchor.translation().transpose() << '\n';

        scanSupport(config, kinDyn, fixedModel, leftFootAnchor, rightFootAnchor, true);
        scanSupport(config, kinDyn, fixedModel, leftFootAnchor, rightFootAnchor, false);
        scanSwingFamily(config, kinDyn, fixedModel, leftFootAnchor, rightFootAnchor,
                        true, "accepted", 0.80, 0.40, 0.125);
        scanSwingFamily(config, kinDyn, fixedModel, leftFootAnchor, rightFootAnchor,
                        false, "accepted", 0.80, 0.40, 0.125);
        scanSwingFamily(config, kinDyn, fixedModel, leftFootAnchor, rightFootAnchor,
                        true, "reduced_roll", 0.82, 0.30, 0.090);
        scanSwingFamily(config, kinDyn, fixedModel, leftFootAnchor, rightFootAnchor,
                        false, "reduced_roll", 0.82, 0.30, 0.090);
        std::cout << "LYENBOT_POSTURE_FEASIBILITY_SCAN_OK\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "LYENBOT_POSTURE_FEASIBILITY_SCAN_FAILED: " << error.what() << '\n';
        return 1;
    }
}
