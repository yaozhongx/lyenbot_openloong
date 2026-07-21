/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024-2025 Humanoid Robot (Shanghai) Co., Ltd.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/

#include "GLFW_callbacks.h"
#include "MJ_interface.h"
#include "PVT_ctrl.h"
#include "StateEst.h"
#include "foot_placement.h"
#include "gait_scheduler.h"
#include "joystick_interpreter.h"
#include "pino_kin_dyn.h"
#include "robot_model_config.h"
#include "useful_math.h"
#include "wbc_priority.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
bool hasArgument(int argc, char **argv, const std::string &expected)
{
    for (int index = 1; index < argc; ++index)
        if (argv[index] == expected)
            return true;
    return false;
}

double argumentDouble(int argc, char **argv, const std::string &prefix, double fallback)
{
    for (int index = 1; index < argc; ++index)
    {
        const std::string argument = argv[index];
        if (argument.rfind(prefix, 0) == 0)
            return std::stod(argument.substr(prefix.size()));
    }
    return fallback;
}

std::string argumentString(int argc, char **argv, const std::string &prefix)
{
    for (int index = 1; index < argc; ++index)
    {
        const std::string argument = argv[index];
        if (argument.rfind(prefix, 0) == 0)
            return argument.substr(prefix.size());
    }
    return {};
}

template<typename Derived>
void writeCsvVector(std::ostream &stream, const Eigen::MatrixBase<Derived> &value)
{
    for (Eigen::Index index = 0; index < value.size(); ++index)
        stream << ',' << value(index);
}

void writeCsvColumns(std::ostream &stream, const std::string &prefix, int count)
{
    for (int index = 0; index < count; ++index)
        stream << ',' << prefix << index;
}

void writeCsvNaNs(std::ostream &stream, int count)
{
    for (int index = 0; index < count; ++index)
        stream << ",nan";
}

void writeCsvVectorPadded(std::ostream &stream, const Eigen::VectorXd &value, int count)
{
    const int available = std::min(count, static_cast<int>(value.size()));
    for (int index = 0; index < available; ++index)
        stream << ',' << value(index);
    writeCsvNaNs(stream, count - available);
}

std::pair<double, double> singularValueBounds(const Eigen::MatrixXd &matrix)
{
    if (matrix.rows() == 0 || matrix.cols() == 0)
        return {std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN()};
    const Eigen::VectorXd values = matrix.jacobiSvd().singularValues();
    return {values.minCoeff(), values.maxCoeff()};
}

double smoothBlend(double phase)
{
    const double clamped = std::clamp(phase, 0.0, 1.0);
    return 0.5 - 0.5 * std::cos(3.14159265358979323846 * clamped);
}

// LYENBOT MODIFY: aggregate every contact on each compound-foot body instead
// of relying on a single touch site that can miss mesh or edge contacts.
Eigen::Matrix<double, 12, 1> measuredFootWrenches(const mjModel *model, const mjData *data,
                                                  int leftFootGeom, int rightFootGeom,
                                                  const DataBus &state)
{
    Eigen::Matrix<double, 12, 1> result = Eigen::Matrix<double, 12, 1>::Zero();
    const int leftFootBody = model->geom_bodyid[leftFootGeom];
    const int rightFootBody = model->geom_bodyid[rightFootGeom];
    for (int contactId = 0; contactId < data->ncon; ++contactId)
    {
        const mjContact &contact = data->contact[contactId];
        const int body0 = model->geom_bodyid[contact.geom[0]];
        const int body1 = model->geom_bodyid[contact.geom[1]];
        int footOffset = -1;
        int footBody = -1;
        const Eigen::Vector3d *footPosition = nullptr;
        if (body0 == leftFootBody || body1 == leftFootBody)
        {
            footOffset = 0;
            footBody = leftFootBody;
            footPosition = &state.fe_l_pos_W;
        }
        else if (body0 == rightFootBody || body1 == rightFootBody)
        {
            footOffset = 6;
            footBody = rightFootBody;
            footPosition = &state.fe_r_pos_W;
        }
        if (footOffset < 0)
            continue;

        mjtNum contactWrench[6] = {};
        mj_contactForce(model, data, contactId, contactWrench);
        const Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>> contactFrame(
            contact.frame);
        const Eigen::Vector3d forceLocal(contactWrench[0], contactWrench[1], contactWrench[2]);
        const Eigen::Vector3d torqueLocal(contactWrench[3], contactWrench[4], contactWrench[5]);
        const double footSign = body1 == footBody ? 1.0 : -1.0;
        const Eigen::Vector3d forceWorld = footSign * contactFrame.transpose() * forceLocal;
        const Eigen::Vector3d torqueWorld = footSign * contactFrame.transpose() * torqueLocal;
        const Eigen::Vector3d contactPosition(contact.pos[0], contact.pos[1], contact.pos[2]);
        result.segment<3>(footOffset) += forceWorld;
        result.segment<3>(footOffset + 3) += torqueWorld
            + (contactPosition - *footPosition).cross(forceWorld);
    }
    return result;
}

std::string lyenbotConfigPath()
{
    const std::filesystem::path fromRoot = "common/robot_configs/lyenbot.json";
    if (std::filesystem::exists(fromRoot))
        return fromRoot.string();
    return "../common/robot_configs/lyenbot.json";
}

std::vector<double> tailToStd(const Eigen::VectorXd &value, int count)
{
    return eigen2std(value.tail(count));
}

int actuatedIndex(const RobotModelConfig &config, const std::string &name)
{
    const auto iterator = std::find(config.joints.actuated.begin(), config.joints.actuated.end(), name);
    if (iterator == config.joints.actuated.end())
        throw std::runtime_error("Missing actuated joint: " + name);
    return static_cast<int>(std::distance(config.joints.actuated.begin(), iterator));
}
}

int main(int argc, char **argv)
{
    mjModel *model = nullptr;
    mjData *data = nullptr;
    std::unique_ptr<UIctr> ui;
    try
    {
        const bool enableGui = hasArgument(argc, argv, "--gui");
        const bool standOnly = hasArgument(argc, argv, "--stand-only");
        const bool useMeasuredState = hasArgument(argc, argv, "--measured-state");
        const double duration = argumentDouble(argc, argv, "--duration=", standOnly ? 13.0 : 30.0);
        const std::string stateEstDiagnosticPath = argumentString(
            argc, argv, "--state-est-diagnostic=");
        const double stateEstDiagnosticPeriod = argumentDouble(
            argc, argv, "--state-est-diagnostic-period=", 0.002);
        if (!(duration > 0.0))
            throw std::runtime_error("--duration must be positive");
        if (!(stateEstDiagnosticPeriod > 0.0))
            throw std::runtime_error("--state-est-diagnostic-period must be positive");

        // LYENBOT MODIFY: all robot-specific paths and semantic names come from one config.
        const RobotModelConfig config = loadRobotModelConfig(lyenbotConfigPath());
        char error[2048] = {};
        model = mj_loadXML(config.scenePath.c_str(), nullptr, error, sizeof(error));
        if (!model)
            throw std::runtime_error(std::string("Cannot load Lyenbot scene: ") + error);
        data = mj_makeData(model);
        if (!data)
            throw std::runtime_error("Cannot allocate MuJoCo data");
        const int leftFootCollisionGeom = mj_name2id(
            model, mjOBJ_GEOM, config.leftFootCollisionGeom.c_str());
        const int rightFootCollisionGeom = mj_name2id(
            model, mjOBJ_GEOM, config.rightFootCollisionGeom.c_str());
        if (leftFootCollisionGeom < 0 || rightFootCollisionGeom < 0)
            throw std::runtime_error("Cannot find configured Lyenbot foot collision geoms");
        if (model->nkey > 0)
            mj_resetDataKeyframe(model, data, 0);
        mj_forward(model, data);

        if (enableGui)
        {
            ui = std::make_unique<UIctr>(model, data);
            ui->iniGLFW();
            ui->enableTracking();
            ui->createWindow("Lyenbot walk_wbc", false);
            ui->updateScene();
        }

        // The construction and control-module order mirrors demo/walk_wbc.cpp.
        MJ_Interface mjInterface(model, data, config);
        Pin_KinDyn kinDyn(config);
        DataBus state(kinDyn.model_nv);
        WBC_priority wbc(kinDyn.model_nv, 18, 26, 0.6, model->opt.timestep,
                         &config, &kinDyn.model_biped);
        GaitScheduler gait(config.swingTime, model->opt.timestep);
        // LYENBOT MODIFY: use an opt-in double-support preparation before the
        // original scheduler releases the first swing foot.
        gait.enableInitialDoubleSupportTransfer = config.initialDoubleSupportTime > 0.0;
        gait.enableDoubleSupportTransfer = gait.enableInitialDoubleSupportTransfer;
        if (gait.enableInitialDoubleSupportTransfer)
        {
            gait.initialDoubleSupportTime = config.initialDoubleSupportTime;
            gait.doubleSupportTime = config.doubleSupportTransferTime;
        }
        gait.useMeasuredContact = true;
        gait.FzThrehold = config.touchdownForce;
        gait.minimumTouchdownPhase = 0.75;
        gait.minimumTransferContactForce = 5.0;
        PVT_Ctr pvt(model->opt.timestep, config.jointControlPath.c_str(), config.joints.actuated,
                    kinDyn.motorMaxTorque, kinDyn.motorMaxSpeed,
                    kinDyn.motorMaxPos, kinDyn.motorMinPos);
        FootPlacement footPlacement;
        JoyStickInterpreter joystick(model->opt.timestep);
        // LYENBOT MODIFY: the configured Pinocchio foot frame is the sole
        // contact point, so its estimator ground height is independent of the
        // ankle-to-sole geometry used by gait planning.
        const Eigen::Vector3d worldGravity(model->opt.gravity[0], model->opt.gravity[1],
                                           model->opt.gravity[2]);
        StateEst stateEstimator(model->opt.timestep,
                                config.stateEstimatorFootFrameGroundHeight,
                                config.stateEstimatorCompensateAccelerometerGravity
                                    ? worldGravity : Eigen::Vector3d::Zero());
        // LYENBOT MODIFY: retain the original filter and tune only its gyro
        // measurement covariance through the robot adapter configuration.
        stateEstimator.eul_w_filter.R.block<3, 3>(3, 3)
            *= config.stateEstimatorAngularVelocityMeasurementNoiseScale;

        // LYENBOT MODIFY: opt-in evidence capture for the original StateEst ->
        // Pin_KinDyn boundary. It does not participate in the control path.
        std::ofstream stateEstDiagnostic;
        if (!stateEstDiagnosticPath.empty())
        {
            stateEstDiagnostic.open(stateEstDiagnosticPath);
            if (!stateEstDiagnostic)
                throw std::runtime_error("Cannot open StateEst diagnostic: "
                                         + stateEstDiagnosticPath);
            stateEstDiagnostic << std::setprecision(17)
                << "time,state_source,estimator_leg,estimator_phi,current_leg,current_phi"
                << ",next_leg,wbc_leg,wbc_next_leg,wbc_contact_release"
                << ",transfer_phi,fL,fR,ncon";
            writeCsvColumns(stateEstDiagnostic, "measured_pos_", 3);
            writeCsvColumns(stateEstDiagnostic, "measured_world_vel_", 3);
            writeCsvColumns(stateEstDiagnostic, "measured_world_omega_", 3);
            writeCsvColumns(stateEstDiagnostic, "selected_pos_", 3);
            writeCsvColumns(stateEstDiagnostic, "selected_world_vel_", 3);
            writeCsvColumns(stateEstDiagnostic, "selected_world_omega_", 3);
            writeCsvColumns(stateEstDiagnostic, "pin_local_vel_", 3);
            writeCsvColumns(stateEstDiagnostic, "pin_reconstructed_world_vel_", 3);
            writeCsvColumns(stateEstDiagnostic, "pin_local_omega_", 3);
            writeCsvColumns(stateEstDiagnostic, "pin_reconstructed_world_omega_", 3);
            writeCsvColumns(stateEstDiagnostic, "measured_rpy_", 3);
            writeCsvColumns(stateEstDiagnostic, "estimated_rpy_", 3);
            writeCsvColumns(stateEstDiagnostic, "free_acc_", 3);
            writeCsvColumns(stateEstDiagnostic, "delta_acc_", 3);
            writeCsvColumns(stateEstDiagnostic, "x_", 15);
            writeCsvColumns(stateEstDiagnostic, "p_diag_", 15);
            writeCsvColumns(stateEstDiagnostic, "innovation_", 14);
            for (int row = 0; row < 6; ++row)
                for (int column = 0; column < 14; ++column)
                    stateEstDiagnostic << ",k_" << row << '_' << column;
            writeCsvColumns(stateEstDiagnostic, "base_pos_des_", 3);
            writeCsvColumns(stateEstDiagnostic, "com_pos_", 3);
            writeCsvColumns(stateEstDiagnostic, "com_pos_des_", 3);
            writeCsvColumns(stateEstDiagnostic, "wbc_ddq_qp_", 6);
            writeCsvColumns(stateEstDiagnostic, "wbc_delta_q_", 6);
            writeCsvColumns(stateEstDiagnostic, "fr_ff_", 12);
            writeCsvColumns(stateEstDiagnostic, "measured_wrench_", 12);
            writeCsvColumns(stateEstDiagnostic, "wbc_fr_res_", 12);
            writeCsvColumns(stateEstDiagnostic, "wbc_kin_ddq_", 6);
            writeCsvColumns(stateEstDiagnostic, "wbc_qp_delta_ddq_", 6);
            stateEstDiagnostic << ",qp_equality_residual_inf,qp_inequality_violation_max";
            writeCsvColumns(stateEstDiagnostic, "contact_err_", 12);
            writeCsvColumns(stateEstDiagnostic, "contact_derr_", 12);
            writeCsvColumns(stateEstDiagnostic, "contact_task_ddq_", 6);
            writeCsvColumns(stateEstDiagnostic, "posrot_err_", 6);
            writeCsvColumns(stateEstDiagnostic, "swing_err_", 6);
            stateEstDiagnostic << ",contact_j_smin,contact_j_smax"
                << ",contact_jpre_smin,contact_jpre_smax";
            stateEstDiagnostic << '\n';
        }

        const int motorCount = static_cast<int>(config.joints.actuated.size());
        const int leftKnee = actuatedIndex(config, "left_knee_pitch_joint");
        const int rightKnee = actuatedIndex(config, "right_knee_pitch_joint");
        const int leftAnklePitch = actuatedIndex(config, "left_ankle_pitch_joint");
        const int rightAnklePitch = actuatedIndex(config, "right_ankle_pitch_joint");
        const double robotWeight = -mj_getTotalmass(model) * model->opt.gravity[2];
        const double standLegLength = config.nominalBaseHeight - config.footHeight;
        constexpr double walkingSpeed = 0.03;
        constexpr double startSteppingTime = 3.0;
        constexpr double startWalkingTime = 5.0;

        state.width_hips = config.hipWidth;
        state.motors_pos_des = config.initialJointPositions;
        state.motors_vel_des.assign(motorCount, 0.0);
        state.motors_tor_des.assign(motorCount, 0.0);
        state.base_pos_des << 0.0, 0.0, config.nominalBaseHeight;
        state.base_rpy_des.setZero();

        footPlacement.kp_vx = 0.03;
        footPlacement.kp_vy = 0.03;
        footPlacement.kp_wz = 0.03;
        footPlacement.stepHeight = config.swingStepHeight;
        footPlacement.legLength = standLegLength;
        // LYENBOT MODIFY: use the already validated Lyenbot FootPlacement adapter values.
        footPlacement.forwardOffset = 0.0;
        footPlacement.inwardOffset = -0.04;
        footPlacement.landingHeightOffset = -0.010;
        footPlacement.fixedLandingWorldHeight = config.swingLandingWorldHeight;
        footPlacement.lateTouchdownStretchStep = 0.0;
        footPlacement.maxStepLength = 0.10;
        footPlacement.maxStepWidthChange = 0.03;

        bool initialized = false;
        bool wbcReferenceInitialized = false;
        bool initialTransferReferenceInitialized = false;
        Eigen::Vector3d initialTransferBase = Eigen::Vector3d::Zero();
        Eigen::Vector3d initialTransferCom = Eigen::Vector3d::Zero();
        double initialTransferRoll = 0.0;
        double initialTransferYaw = 0.0;
        int supportTransitions = 0;
        int nextDiagnosticSecond = 1;
        double nextRenderTime = 0.0;
        double nextStateEstDiagnosticTime = 0.0;
        DataBus::LegState previousLeg = DataBus::DSt;
        long jointPositionViolations = 0;
        long jointVelocityViolations = 0;
        long jointEffortViolations = 0;
        double minimumJointPositionMargin = std::numeric_limits<double>::infinity();
        double minimumJointVelocityMargin = std::numeric_limits<double>::infinity();
        double minimumJointEffortMargin = std::numeric_limits<double>::infinity();
        std::string worstPositionJoint;
        double worstPositionTime = 0.0;
        double worstPositionActual = 0.0;
        double worstPositionDesired = 0.0;
        double worstPositionMeasuredEffort = 0.0;
        std::cout << "LYENBOT_WALK_WBC_START mode=" << (standOnly ? "stand" : "walk")
                  << " state_source=" << (useMeasuredState ? "mujoco" : "StateEst")
                  << " pause_during_dst=1"
                  << " duration=" << duration << " model_nv=" << kinDyn.model_nv
                  << " motors=" << motorCount << " robot_weight=" << robotWeight << '\n';

        while (data->time < duration && (!ui || !glfwWindowShouldClose(ui->window)))
        {
            if (ui && !ui->runSim)
            {
                ui->updateScene();
                continue;
            }

            // Preserve the original walk_wbc control-loop order: simulate, sense,
            // estimate, update dynamics, plan gait/foot, solve WBC, run PVT, write torque.
            mj_step(model, data);
            const double simTime = data->time;
            mjInterface.updateSensorValues();
            mjInterface.dataBusWrite(state);
            const Eigen::Matrix<double, 12, 1> measuredWrench = measuredFootWrenches(
                model, data, leftFootCollisionGeom, rightFootCollisionGeom, state);
            // LYENBOT MODIFY: feed the full compound-foot force into StateEst
            // before its touchdown-confirmation update.
            state.fL[2] = measuredWrench[2];
            state.fR[2] = measuredWrench[8];

            const DataBus::LegState estimatorLegState = state.legState;
            const double estimatorPhi = state.phi;
            const Eigen::Vector3d measuredPosition(state.basePos[0], state.basePos[1],
                                                   state.basePos[2]);
            const Eigen::Vector3d measuredWorldVelocity(state.baseLinVel[0],
                                                        state.baseLinVel[1],
                                                        state.baseLinVel[2]);
            const Eigen::Vector3d measuredWorldAngularVelocity = state.base_omega_W;

            if (!useMeasuredState)
            {
                if (simTime > 1.0 && stateEstimator.flag_init)
                    stateEstimator.init(state);
                stateEstimator.set(state);
                stateEstimator.update();
                stateEstimator.get(state);
            }
            const Eigen::Vector3d selectedPosition = state.q.head<3>();
            const Eigen::Vector3d selectedWorldVelocity = state.dq.head<3>();
            const Eigen::Vector3d selectedWorldAngularVelocity = state.dq.segment<3>(3);
            const Eigen::Matrix3d selectedBaseRotation = state.base_rot;

            kinDyn.dataBusRead(state);
            kinDyn.computeJ_dJ();
            kinDyn.computeDyn();
            kinDyn.dataBusWrite(state);
            const Eigen::Vector3d pinLocalVelocity = state.dq.head<3>();
            const Eigen::Vector3d pinReconstructedWorldVelocity
                = selectedBaseRotation * pinLocalVelocity;
            const Eigen::Vector3d pinLocalAngularVelocity = state.dq.segment<3>(3);
            const Eigen::Vector3d pinReconstructedWorldAngularVelocity
                = selectedBaseRotation * pinLocalAngularVelocity;

            if (!useMeasuredState)
            {
                stateEstimator.setF(state);
                stateEstimator.updateF();
                stateEstimator.getF(state);
            }

            if (!initialized)
            {
                initialized = true;
                state.base_pos_des << state.q(0), state.q(1), config.nominalBaseHeight;
                state.swing_fe_pos_des_W = state.fe_r_pos_W;
                state.swing_fe_rpy_des_W.setZero();
                state.stance_fe_pos_cur_W = state.fe_l_pos_W;
                state.stance_fe_rot_cur_W = state.fe_l_rot_W;
                state.stanceDesPos_W = state.fe_l_pos_W;
                joystick.setIniPos(state.q(0), state.q(1), config.nominalBaseHeight, state.base_rpy(2));
            }

            // LYENBOT MODIFY: latch all full-foot WBC references in the same
            // initialized StateEst frame before gait planning changes them.
            const bool wbcActive = simTime > startSteppingTime;
            if (wbcActive)
            {
                // LYENBOT MODIFY: acceptance-only audit of all 23 measured
                // actuator states; these counters do not alter control output.
                for (int index = 0; index < motorCount; ++index)
                {
                    const double positionMargin = std::min(
                        state.motors_pos_cur[index] - kinDyn.motorMinPos[index],
                        kinDyn.motorMaxPos[index] - state.motors_pos_cur[index]);
                    const double velocityMargin = kinDyn.motorMaxSpeed[index]
                                                  - std::abs(state.motors_vel_cur[index]);
                    const double effortMargin = kinDyn.motorMaxTorque[index]
                                                - std::abs(state.motors_tor_cur[index]);
                    if (positionMargin < minimumJointPositionMargin)
                    {
                        minimumJointPositionMargin = positionMargin;
                        worstPositionJoint = config.joints.actuated[index];
                        worstPositionTime = simTime;
                        worstPositionActual = state.motors_pos_cur[index];
                        worstPositionDesired = state.motors_pos_des[index];
                        worstPositionMeasuredEffort = state.motors_tor_cur[index];
                    }
                    minimumJointVelocityMargin = std::min(minimumJointVelocityMargin,
                                                          velocityMargin);
                    minimumJointEffortMargin = std::min(minimumJointEffortMargin,
                                                        effortMargin);
                    if (positionMargin < -1e-9) ++jointPositionViolations;
                    if (velocityMargin < -1e-9) ++jointVelocityViolations;
                    if (effortMargin < -1e-9) ++jointEffortViolations;
                }
            }
            if (wbcActive && !wbcReferenceInitialized)
            {
                wbcReferenceInitialized = true;
                wbc.setQini(state.q, state.q);
                wbc.pCoMDes = state.pCoM_W;
                state.base_pos_des = state.q.head<3>();
                state.swing_fe_pos_des_W = state.fe_r_pos_W;
                state.stance_fe_pos_cur_W = state.fe_l_pos_W;
                state.stance_fe_rot_cur_W = state.fe_l_rot_W;
                state.stanceDesPos_W = state.fe_l_pos_W;
                joystick.setIniPos(state.q(0), state.q(1), state.q(2), state.base_rpy(2));
            }

            const bool initialTransferCompleted = gait.initialDoubleSupportCompleted
                                                  || !gait.enableInitialDoubleSupportTransfer;
            if (!standOnly && simTime > startWalkingTime && initialTransferCompleted)
            {
                // LYENBOT MODIFY: keep the original motion seed in single
                // support, but stop advancing base_pos_des.x while both feet
                // are rigidly constrained during regular DSt transfer.
                const bool pauseDuringDoubleSupport = gait.initialDoubleSupportCompleted
                    && state.legState == DataBus::DSt;
                joystick.setWzDesLPara(0.0, 1.0);
                joystick.setVxDesLPara(pauseDuringDoubleSupport ? 0.0 : walkingSpeed,
                                       pauseDuringDoubleSupport ? 0.2 : 2.0);
                state.motionState = DataBus::Walk;
            }
            else
            {
                if (!wbcReferenceInitialized)
                    joystick.setIniPos(state.q(0), state.q(1), config.nominalBaseHeight,
                                       state.base_rpy(2));
                state.motionState = DataBus::Stand;
            }

            if (!standOnly && simTime > startSteppingTime)
            {
                joystick.step();
                joystick.dataBusWrite(state);
                gait.start();
                state.motionState = DataBus::Walk;
                gait.dataBusRead(state);
                gait.step();
                gait.dataBusWrite(state);

                const double leftBaseTarget = state.fe_l_pos_W.y()
                                              + config.supportBaseOutwardOffset;
                const double rightBaseTarget = state.fe_r_pos_W.y()
                                               - config.supportBaseOutwardOffset;
                const double leftComTarget = state.fe_l_pos_W.y()
                                             - config.supportComInwardOffset;
                const double rightComTarget = state.fe_r_pos_W.y()
                                              + config.supportComInwardOffset;
                state.base_pos_des.z() = config.supportBaseHeight;
                if (state.legState == DataBus::DSt
                    && !gait.initialDoubleSupportCompleted)
                {
                    if (!initialTransferReferenceInitialized)
                    {
                        initialTransferReferenceInitialized = true;
                        initialTransferBase = state.base_pos_des;
                        initialTransferCom = wbc.pCoMDes;
                        initialTransferRoll = state.base_rpy_des.x();
                        initialTransferYaw = state.base_rpy_des.z();
                    }
                    // LYENBOT MODIFY: coordinate base posture, observable CoM
                    // reference and nominal contact load during the first transfer.
                    const double blend = smoothBlend(gait.transferPhi);
                    const bool leftFirst = gait.firstleg == DataBus::LSt;
                    const double baseYTarget = leftFirst ? leftBaseTarget : rightBaseTarget;
                    const double comYTarget = leftFirst ? leftComTarget : rightComTarget;
                    const double rollTarget = leftFirst ? -config.supportBaseRoll
                                                        : config.supportBaseRoll;
                    state.base_pos_des.x() = initialTransferBase.x();
                    state.base_pos_des.y() = (1.0 - blend) * initialTransferBase.y()
                                             + blend * baseYTarget;
                    state.base_pos_des.z() = (1.0 - blend) * initialTransferBase.z()
                                             + blend * config.supportBaseHeight;
                    state.base_rpy_des.x() = (1.0 - blend) * initialTransferRoll
                                             + blend * rollTarget;
                    state.base_rpy_des.z() = initialTransferYaw;
                    wbc.pCoMDes.y() = (1.0 - blend) * initialTransferCom.y()
                                      + blend * comYTarget;
                }
                else if (state.legState == DataBus::LSt)
                {
                    state.base_pos_des.y() = leftBaseTarget;
                    state.base_rpy_des.x() = -config.supportBaseRoll;
                    wbc.pCoMDes.y() = leftComTarget;
                }
                else if (state.legState == DataBus::RSt)
                {
                    state.base_pos_des.y() = rightBaseTarget;
                    state.base_rpy_des.x() = config.supportBaseRoll;
                    wbc.pCoMDes.y() = rightComTarget;
                }
                else if (state.legStateNext == DataBus::RSt)
                {
                    const double blend = smoothBlend(gait.transferPhi);
                    state.base_pos_des.y() = (1.0 - blend) * leftBaseTarget
                                             + blend * rightBaseTarget;
                    state.base_rpy_des.x() = (2.0 * blend - 1.0)
                                              * config.supportBaseRoll;
                    wbc.pCoMDes.y() = (1.0 - blend) * leftComTarget
                                      + blend * rightComTarget;
                }
                else
                {
                    const double blend = smoothBlend(gait.transferPhi);
                    state.base_pos_des.y() = (1.0 - blend) * rightBaseTarget
                                             + blend * leftBaseTarget;
                    state.base_rpy_des.x() = (1.0 - 2.0 * blend)
                                              * config.supportBaseRoll;
                    wbc.pCoMDes.y() = (1.0 - blend) * rightComTarget
                                      + blend * leftComTarget;
                }

                // FootPlacement only has single-support semantics. In double
                // support WBC keeps both feet constrained, so retain the last target.
                if (state.legState != DataBus::DSt)
                {
                    footPlacement.dataBusRead(state);
                    footPlacement.getSwingPos();
                    footPlacement.dataBusWrite(state);
                    if (config.swingPeakPitch > 0.0)
                    {
                        double pitchBlend = 0.0;
                        if (state.phi < config.swingPitchRisePhase)
                            pitchBlend = smoothBlend(state.phi / config.swingPitchRisePhase);
                        else if (state.phi < config.swingPitchReturnPhase)
                            pitchBlend = 1.0 - smoothBlend(
                                (state.phi - config.swingPitchRisePhase)
                                / (config.swingPitchReturnPhase
                                   - config.swingPitchRisePhase));
                        state.swing_fe_rpy_des_W.y() = config.swingPeakPitch * pitchBlend;
                    }
                    if (config.preventOutwardSwingExpansion)
                    {
                        // LYENBOT MODIFY: allow inward recovery but prevent the
                        // live hip/base feedback from widening each touchdown.
                        if (state.legState == DataBus::LSt)
                            state.swing_fe_pos_des_W.y() = std::max(
                                state.swing_fe_pos_des_W.y(), state.swingStartPos_W.y());
                        else
                            state.swing_fe_pos_des_W.y() = std::min(
                                state.swing_fe_pos_des_W.y(), state.swingStartPos_W.y());
                    }
                }
            }

            const bool initialTransferActive = !standOnly && wbcActive
                && state.legState == DataBus::DSt
                && !gait.initialDoubleSupportCompleted;
            state.des_ddq.setZero();
            state.des_dq.setZero();
            state.des_delta_q.setZero();
            state.Fr_ff.setZero();
            if (standOnly || state.motionState == DataBus::Stand || initialTransferActive)
            {
                state.Fr_ff[2] = 0.5 * robotWeight;
                state.Fr_ff[8] = 0.5 * robotWeight;
            }
            else if (state.legState == DataBus::LSt)
                state.Fr_ff[2] = robotWeight;
            else if (state.legState == DataBus::RSt)
                state.Fr_ff[8] = robotWeight;
            else
            {
                const double blend = smoothBlend(gait.transferPhi);
                constexpr double pendingSupportPreloadRatio = 0.10;
                const double pendingRatio = gait.initialDoubleSupportCompleted
                    ? pendingSupportPreloadRatio
                        + (1.0 - pendingSupportPreloadRatio) * blend
                    : 0.5 + 0.5 * blend;
                if (state.legStateNext == DataBus::LSt)
                {
                    state.Fr_ff[2] = pendingRatio * robotWeight;
                    state.Fr_ff[8] = (1.0 - pendingRatio) * robotWeight;
                }
                else
                {
                    state.Fr_ff[2] = (1.0 - pendingRatio) * robotWeight;
                    state.Fr_ff[8] = pendingRatio * robotWeight;
                }
            }
            if (!standOnly && simTime > startWalkingTime + 1.0
                && gait.initialDoubleSupportCompleted && state.legState != DataBus::DSt)
            {
                state.des_delta_q.head<2>() << joystick.vx_W * model->opt.timestep,
                    joystick.vy_W * model->opt.timestep;
                state.des_delta_q(5) = joystick.wz_L * model->opt.timestep;
                state.des_dq.head<2>() << joystick.vx_W, joystick.vy_W;
                state.des_dq(5) = joystick.wz_L;
                state.des_ddq.head<2>() << 5.0 * (joystick.vx_W - state.dq(0)),
                    5.0 * (joystick.vy_W - state.dq(1));
                state.des_ddq(5) = 5.0 * (joystick.wz_L - state.dq(5));
            }

            // LYENBOT MODIFY: full-foot WBC latches world contact anchors on
            // its first solve. Delay that solve until StateEst has initialized
            // and the result is actually handed to PVT.
            if (wbcActive)
            {
                // LYENBOT MODIFY: the initial scheduler-owned DSt phase uses
                // the original Stand WBC task stack so pCoMDes is controlled.
                const DataBus::MotionState plannedMotionState = state.motionState;
                const DataBus::LegState plannedLegState = state.legState;
                const Eigen::VectorXd plannedFeedforwardWrench = state.Fr_ff;
                if (initialTransferActive)
                    state.motionState = DataBus::Stand;
                wbc.dataBusRead(state);
                wbc.computeDdq(kinDyn);
                wbc.computeTau();
                wbc.dataBusWrite(state);
                state.motionState = plannedMotionState;
                state.legState = plannedLegState;
                state.Fr_ff = plannedFeedforwardWrench;
            }

            if (simTime <= startSteppingTime)
            {
                state.motors_pos_des = config.initialJointPositions;
                state.motors_vel_des.assign(motorCount, 0.0);
                state.motors_tor_des.assign(motorCount, 0.0);
            }
            else
            {
                const Eigen::VectorXd desiredQ = kinDyn.integrateDIY(state.q, state.wbc_delta_q_final);
                state.motors_pos_des = tailToStd(desiredQ, motorCount);
                state.motors_vel_des = tailToStd(state.wbc_dq_final, motorCount);
                state.motors_tor_des = eigen2std(state.wbc_tauJointRes);
                // LYENBOT MODIFY: retain the staged URDF-limit guard at the actuator adapter.
                for (int index = 0; index < motorCount; ++index)
                {
                    const bool anklePitch = index == leftAnklePitch || index == rightAnklePitch;
                    const double requestedMargin = anklePitch ? 0.060 : 0.030;
                    const double margin = std::min(requestedMargin,
                        0.25 * (kinDyn.motorMaxPos[index] - kinDyn.motorMinPos[index]));
                    state.motors_pos_des[index] = std::clamp(state.motors_pos_des[index],
                        kinDyn.motorMinPos[index] + margin, kinDyn.motorMaxPos[index] - margin);
                }
                state.motors_pos_des[leftKnee] = std::max(state.motors_pos_des[leftKnee], 0.03);
                state.motors_pos_des[rightKnee] = std::max(state.motors_pos_des[rightKnee], 0.03);
            }

            pvt.dataBusRead(state);
            if (simTime <= startSteppingTime)
                pvt.calMotorsPVT(0.002);
            else
                pvt.calMotorsPVT();
            pvt.dataBusWrite(state);
            mjInterface.setMotorsTorque(state.motors_tor_out);

            if (stateEstDiagnostic && simTime >= nextStateEstDiagnosticTime)
            {
                stateEstDiagnostic << simTime << ',' << (useMeasuredState ? 1 : 0)
                    << ',' << estimatorLegState << ',' << estimatorPhi
                    << ',' << state.legState << ',' << state.phi
                    << ',' << state.legStateNext << ','
                    << (wbcActive ? static_cast<int>(wbc.legStateCur) : -1)
                    << ',' << (wbcActive ? static_cast<int>(wbc.legStateNextCur) : -1)
                    << ',' << (wbcActive && wbc.contactReleaseCur ? 1 : 0)
                    << ',' << gait.transferPhi
                    << ',' << state.fL[2]
                    << ',' << state.fR[2] << ',' << data->ncon;
                writeCsvVector(stateEstDiagnostic, measuredPosition);
                writeCsvVector(stateEstDiagnostic, measuredWorldVelocity);
                writeCsvVector(stateEstDiagnostic, measuredWorldAngularVelocity);
                writeCsvVector(stateEstDiagnostic, selectedPosition);
                writeCsvVector(stateEstDiagnostic, selectedWorldVelocity);
                writeCsvVector(stateEstDiagnostic, selectedWorldAngularVelocity);
                writeCsvVector(stateEstDiagnostic, pinLocalVelocity);
                writeCsvVector(stateEstDiagnostic, pinReconstructedWorldVelocity);
                writeCsvVector(stateEstDiagnostic, pinLocalAngularVelocity);
                writeCsvVector(stateEstDiagnostic, pinReconstructedWorldAngularVelocity);
                writeCsvVector(stateEstDiagnostic,
                    Eigen::Vector3d(state.rpy[0], state.rpy[1], state.rpy[2]));
                if (useMeasuredState || stateEstimator.flag_init)
                {
                    writeCsvNaNs(stateEstDiagnostic, 3 + 3 + 3 + 15 + 15 + 14 + 6 * 14);
                }
                else
                {
                    writeCsvVector(stateEstDiagnostic, stateEstimator.eul_woOff);
                    writeCsvVector(stateEstDiagnostic, stateEstimator.freeAcc);
                    writeCsvVector(stateEstDiagnostic, stateEstimator.delta_acc);
                    writeCsvVector(stateEstDiagnostic, stateEstimator.X);
                    writeCsvVector(stateEstDiagnostic, stateEstimator.P.diagonal());
                    writeCsvVector(stateEstDiagnostic, stateEstimator.innovation);
                    for (int row = 0; row < 6; ++row)
                        for (int column = 0; column < 14; ++column)
                            stateEstDiagnostic << ',' << stateEstimator.K(row, column);
                }
                writeCsvVector(stateEstDiagnostic, state.base_pos_des);
                writeCsvVector(stateEstDiagnostic, state.pCoM_W);
                writeCsvVector(stateEstDiagnostic, wbc.pCoMDes);
                writeCsvVector(stateEstDiagnostic, state.wbc_ddq_qp.head<6>());
                if (state.wbc_delta_q_final.size() >= 6)
                    writeCsvVector(stateEstDiagnostic, state.wbc_delta_q_final.head<6>());
                else
                    writeCsvNaNs(stateEstDiagnostic, 6);
                writeCsvVector(stateEstDiagnostic, state.Fr_ff);
                writeCsvVector(stateEstDiagnostic, measuredWrench);
                if (state.wbc_FrRes.size() >= 12)
                    writeCsvVector(stateEstDiagnostic, state.wbc_FrRes.head<12>());
                else
                    writeCsvNaNs(stateEstDiagnostic, 12);
                if (wbcActive)
                {
                    writeCsvVector(stateEstDiagnostic, wbc.ddq_final_kin.head<6>());
                    writeCsvVector(stateEstDiagnostic, wbc.eigen_xOpt.head<6>());
                    stateEstDiagnostic << ',' << state.wbc_qp_equality_residual_inf
                        << ',' << state.wbc_qp_inequality_violation_max;
                    const Task &contactTask = wbc.kin_tasks_walk.taskLib[
                        wbc.kin_tasks_walk.getId("static_Contact")];
                    const Task &posRotTask = wbc.kin_tasks_walk.taskLib[
                        wbc.kin_tasks_walk.getId("PosRot")];
                    const Task &swingTask = wbc.kin_tasks_walk.taskLib[
                        wbc.kin_tasks_walk.getId("SwingLeg")];
                    writeCsvVectorPadded(stateEstDiagnostic, contactTask.errX, 12);
                    writeCsvVectorPadded(stateEstDiagnostic, contactTask.derrX, 12);
                    writeCsvVectorPadded(stateEstDiagnostic, contactTask.ddq, 6);
                    writeCsvVectorPadded(stateEstDiagnostic, posRotTask.errX, 6);
                    writeCsvVectorPadded(stateEstDiagnostic, swingTask.errX, 6);
                    const auto contactBounds = singularValueBounds(contactTask.J);
                    const auto projectedBounds = singularValueBounds(contactTask.Jpre);
                    stateEstDiagnostic << ',' << contactBounds.first << ',' << contactBounds.second
                        << ',' << projectedBounds.first << ',' << projectedBounds.second;
                }
                else
                {
                    writeCsvNaNs(stateEstDiagnostic,
                        6 + 6 + 2 + 12 + 12 + 6 + 6 + 6 + 4);
                }
                stateEstDiagnostic << '\n';
                nextStateEstDiagnosticTime = simTime + stateEstDiagnosticPeriod;
            }

            if (state.legState != DataBus::DSt && state.legState != previousLeg)
            {
                ++supportTransitions;
                previousLeg = state.legState;
            }
            if (simTime >= nextDiagnosticSecond)
            {
                std::cout << "t=" << simTime << " z=" << state.basePos[2]
                          << " state_z=" << state.q(2)
                          << " x=" << state.basePos[0] << " state_x=" << state.q(0)
                          << " com_x=" << state.pCoM_W.x() << " com_des_x=" << wbc.pCoMDes.x()
                          << " measured_v=" << state.baseLinVel[0] << ' '
                          << state.baseLinVel[1] << ' ' << state.baseLinVel[2]
                          << " est_world_v=" << stateEstimator.base_vel.transpose()
                          << " pin_local_v=" << state.dq.head<3>().transpose()
                          << " foot_v_l=" << stateEstimator.fe_l_vel_L.transpose()
                          << " foot_v_r=" << stateEstimator.fe_r_vel_L.transpose()
                          << " kf_y_v_l=" << stateEstimator.Y.segment<3>(6).transpose()
                          << " kf_y_v_r=" << stateEstimator.Y.segment<3>(9).transpose()
                          << " free_acc=" << stateEstimator.freeAcc.transpose()
                          << " est_rpy=" << stateEstimator.eul_woOff.transpose()
                          << " rpy=" << state.rpy[0] << ' ' << state.rpy[1] << ' ' << state.rpy[2]
                          << " leg=" << state.legState << " phi=" << state.phi
                          << " qp=" << state.qp_status << " transitions=" << supportTransitions << '\n';
                ++nextDiagnosticSecond;
            }

            const bool finite = std::isfinite(state.q.norm()) && std::isfinite(state.dq.norm());
            const double qpBaseAcceleration = state.wbc_ddq_qp.head<6>().lpNorm<Eigen::Infinity>();
            // LYENBOT MODIFY: the original demo computes WBC during the initial
            // PD ramp but does not execute its result until the 3 s handover.
            const bool wbcSafe = !wbcActive || (state.qp_status == 0
                && std::isfinite(qpBaseAcceleration) && qpBaseAcceleration <= 100.0);
            if (!finite || !wbcSafe || state.basePos[2] < 0.35
                || std::abs(state.rpy[0]) > 1.0 || std::abs(state.rpy[1]) > 1.0)
            {
                throw std::runtime_error("Safety stop at t=" + std::to_string(simTime)
                    + " qp=" + std::to_string(state.qp_status)
                    + " qp_base_ddq_max=" + std::to_string(qpBaseAcceleration)
                    + " base_z=" + std::to_string(state.basePos[2])
                    + " state_z=" + std::to_string(state.q(2))
                    + " measured_vx=" + std::to_string(state.baseLinVel[0])
                    + " pin_local_vx=" + std::to_string(state.dq(0))
                    + " delta_acc_x=" + std::to_string(stateEstimator.delta_acc.x())
                    + " est_pitch=" + std::to_string(stateEstimator.eul_woOff.y())
                    + " leg=" + std::to_string(state.legState)
                    + " phi=" + std::to_string(state.phi)
                    + " fL=" + std::to_string(state.fL[2])
                    + " fR=" + std::to_string(state.fR[2])
                    + " left_foot_z=" + std::to_string(state.fe_l_pos_W.z())
                    + " right_foot_z=" + std::to_string(state.fe_r_pos_W.z())
                    + " swing_des_z=" + std::to_string(state.swing_fe_pos_des_W.z())
                    + " roll=" + std::to_string(state.rpy[0])
                    + " pitch=" + std::to_string(state.rpy[1]));
            }

            if (ui && simTime >= nextRenderTime)
            {
                ui->updateScene();
                nextRenderTime = simTime + 1.0 / 60.0;
            }
        }

        const bool closedEarly = ui && glfwWindowShouldClose(ui->window) && data->time < duration;
        if (closedEarly)
            std::cout << "LYENBOT_WALK_WBC_GUI_CLOSED time=" << data->time << '\n';
        else
            std::cout << "LYENBOT_WALK_WBC_OK time=" << data->time
                      << " base_z=" << data->qpos[2]
                      << " transitions=" << supportTransitions
                      << " joint_position_violations=" << jointPositionViolations
                      << " joint_velocity_violations=" << jointVelocityViolations
                      << " joint_effort_violations=" << jointEffortViolations
                      << " min_joint_position_margin=" << minimumJointPositionMargin
                      << " min_joint_velocity_margin=" << minimumJointVelocityMargin
                      << " min_joint_effort_margin=" << minimumJointEffortMargin
                      << " worst_position_joint=" << worstPositionJoint
                      << " worst_position_time=" << worstPositionTime
                      << " worst_position_actual=" << worstPositionActual
                      << " worst_position_desired=" << worstPositionDesired
                      << " worst_position_effort=" << worstPositionMeasuredEffort << '\n';
        ui.reset();
        mj_deleteData(data);
        mj_deleteModel(model);
        return 0;
    }
    catch (const std::exception &exception)
    {
        std::cerr << "LYENBOT_WALK_WBC_FAILED: " << exception.what() << '\n';
        ui.reset();
        if (data) mj_deleteData(data);
        if (model) mj_deleteModel(model);
        return 1;
    }
}
