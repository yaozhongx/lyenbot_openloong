#include "MJ_interface.h"
#include "GLFW_callbacks.h"
#include "PVT_ctrl.h"
#include "data_bus.h"
#include "foot_placement.h"
#include "gait_scheduler.h"
#include "joystick_interpreter.h"
#include "mpc.h"
#include "pino_kin_dyn.h"
#include "robot_model_config.h"
#include "useful_math.h"
#include "wbc_priority.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace
{
enum class Stage { Pd, Wbc, Walk, StepLeft, StepRight };

Stage parseStage(int argc, char **argv)
{
    const std::string value = argc > 1 ? argv[1] : "pd";
    if (value == "pd") return Stage::Pd;
    if (value == "wbc") return Stage::Wbc;
    if (value == "walk") return Stage::Walk;
    if (value == "step-left") return Stage::StepLeft;
    if (value == "step-right") return Stage::StepRight;
    throw std::runtime_error("stage must be one of: pd, wbc, walk, step-left, step-right");
}

bool isSingleStep(Stage stage)
{
    return stage == Stage::StepLeft || stage == Stage::StepRight;
}

double smoothBlend(double value)
{
    const double clamped = std::clamp(value, 0.0, 1.0);
    return 0.5 - 0.5 * std::cos(3.14159265358979323846 * clamped);
}

bool hasArgument(int argc, char **argv, const std::string &expected)
{
    for (int index = 2; index < argc; ++index)
        if (argv[index] == expected)
            return true;
    return false;
}

std::string argumentValue(int argc, char **argv, const std::string &prefix, const std::string &fallback)
{
    for (int index = 2; index < argc; ++index)
    {
        const std::string argument = argv[index];
        if (argument.rfind(prefix, 0) == 0)
            return argument.substr(prefix.size());
    }
    return fallback;
}

std::vector<double> tailToStd(const Eigen::VectorXd &value, int count)
{
    return eigen2std(value.tail(count));
}

// LYENBOT MODIFY: aggregate MuJoCo contact forces into world-frame foot wrenches.
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
        const Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>> contactFrame(contact.frame);
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

// LYENBOT MODIFY: staged walk/single-step CSV diagnostics; this class does not modify controller state.
class WalkDiagnostics
{
public:
    WalkDiagnostics(const std::string &path, const RobotModelConfig &config,
                    const Eigen::VectorXd &effortLimits, double samplePeriod, double startTime)
        : output_(path), config_(config), effortLimits_(effortLimits), samplePeriod_(samplePeriod),
          nextSampleTime_(startTime)
    {
        if (!output_)
            throw std::runtime_error("Cannot open walk diagnostic file: " + path);
        for (std::size_t index = 0; index < config_.joints.actuated.size(); ++index)
            if (config_.joints.actuated[index].find("ankle") != std::string::npos)
                ankleIndices_.push_back(static_cast<int>(index));
        writeHeader();
    }

    void writeIfDue(double time, const DataBus &state, const Eigen::Vector3d &comDesired,
                    const Eigen::Matrix<double, 12, 1> &measuredWrench, double transferPhi,
                    int stepTransitions, int contactCount, bool mpcEnabled)
    {
        if (time + 1e-12 < nextSampleTime_)
            return;
        nextSampleTime_ = time + samplePeriod_;
        updateSupportAnchor(state);

        const Eigen::Vector3d leftVelocity = (state.J_l * state.dq).head<3>();
        const Eigen::Vector3d rightVelocity = (state.J_r * state.dq).head<3>();
        const Eigen::Vector3d stanceVelocity = state.legState == DataBus::LSt ? leftVelocity
                                                    : state.legState == DataBus::RSt ? rightVelocity
                                                                                     : Eigen::Vector3d::Zero();
        const double stanceSlip = state.legState == DataBus::LSt && leftAnchorValid_
                                      ? (state.fe_l_pos_W.head<2>() - leftAnchor_.head<2>()).norm()
                                  : state.legState == DataBus::RSt && rightAnchorValid_
                                      ? (state.fe_r_pos_W.head<2>() - rightAnchor_.head<2>()).norm()
                                      : 0.0;

        output_ << time << ',' << mpcEnabled << ',' << static_cast<int>(state.motionState) << ','
                << static_cast<int>(state.legState) << ',' << static_cast<int>(state.legStateNext) << ','
                << state.phi << ',' << transferPhi << ',' << state.wbc_contact_release << ','
                << stepTransitions;
        appendArray(state.basePos, 3);
        appendArray(state.baseLinVel, 3);
        appendArray(state.baseAcc, 3);
        appendArray(state.rpy, 3);
        appendArray(state.baseAngVel, 3);
        appendVector(state.base_pos_des);
        output_ << ',' << state.des_dq[0] << ',' << state.des_dq[1];
        for (int index = 0; index < 6; ++index)
            output_ << ',' << state.wbc_ddq_final[index];
        for (int index = 0; index < 6; ++index)
            output_ << ',' << state.wbc_ddq_qp[index];
        for (int index = 0; index < 6; ++index)
            output_ << ',' << state.wbc_ddq_qp[index] - state.wbc_ddq_final[index];
        appendVector(state.pCoM_W);
        appendVector(comDesired);
        appendVector(state.fe_l_pos_W);
        appendRotation(state.fe_l_rot_W);
        appendVector(leftVelocity);
        appendVector(state.fe_r_pos_W);
        appendRotation(state.fe_r_rot_W);
        appendVector(rightVelocity);
        appendVector(state.swing_fe_pos_des_W);
        appendVector(state.swing_fe_rpy_des_W);
        output_ << ',' << (state.fL[2] >= config_.touchdownForce) << ','
                << (state.fR[2] >= config_.touchdownForce) << ',' << state.fL[2] << ',' << state.fR[2]
                << ',' << contactCount << ',' << state.qp_status << ',' << state.qp_nWSR << ','
                << state.qp_cpuTime << ',' << state.wbc_qp_equality_residual_inf << ','
                << state.wbc_qp_inequality_violation_max << ',' << stanceVelocity.norm() << ',' << stanceSlip;
        for (int index = 0; index < state.wbc_FrRes.size(); ++index)
            output_ << ',' << state.wbc_FrRes[index];
        for (int index = 0; index < state.Fr_ff.size(); ++index)
            output_ << ',' << state.Fr_ff[index];
        for (int index = 0; index < measuredWrench.size(); ++index)
            output_ << ',' << measuredWrench[index];
        const auto appendMeasuredCop = [&](int offset)
        {
            const double fz = measuredWrench[offset + 2];
            output_ << ',' << (std::abs(fz) > 1e-6 ? -measuredWrench[offset + 4] / fz : 0.0)
                    << ',' << (std::abs(fz) > 1e-6 ? measuredWrench[offset + 3] / fz : 0.0);
        };
        appendMeasuredCop(0);
        appendMeasuredCop(6);
        for (const int index : ankleIndices_)
            output_ << ',' << effortLimits_[index] - std::abs(state.motors_tor_out[index]);
        for (std::size_t index = 0; index < config_.joints.actuated.size(); ++index)
            output_ << ',' << state.motors_pos_des[index] << ',' << state.motors_pos_cur[index]
                    << ',' << state.motors_vel_des[index] << ',' << state.motors_vel_cur[index]
                    << ',' << state.motors_tor_des[index] << ',' << state.motors_tor_out[index];
        output_ << '\n';
        output_.flush();
    }

private:
    void writeHeader()
    {
        output_ << "time,mpc_enabled,motion_state,leg_state,leg_state_next,phi,transfer_phi"
                   ",contact_release,step_transitions"
                   ",base_x,base_y,base_z,base_vx,base_vy,base_vz,base_ax,base_ay,base_az,roll,pitch,yaw"
                   ",base_wx,base_wy,base_wz,base_des_x,base_des_y,base_des_z,des_dq_x,des_dq_y"
                   ",wbc_kin_ddq_x,wbc_kin_ddq_y,wbc_kin_ddq_z,wbc_kin_ddq_roll,wbc_kin_ddq_pitch,wbc_kin_ddq_yaw"
                   ",wbc_qp_ddq_x,wbc_qp_ddq_y,wbc_qp_ddq_z,wbc_qp_ddq_roll,wbc_qp_ddq_pitch,wbc_qp_ddq_yaw"
                   ",wbc_qp_delta_ddq_x,wbc_qp_delta_ddq_y,wbc_qp_delta_ddq_z"
                   ",wbc_qp_delta_ddq_roll,wbc_qp_delta_ddq_pitch,wbc_qp_delta_ddq_yaw"
                   ",com_x,com_y,com_z,wbc_com_des_x,wbc_com_des_y,wbc_com_des_z"
                   ",left_foot_x,left_foot_y,left_foot_z,left_foot_roll,left_foot_pitch,left_foot_yaw"
                   ",left_foot_vx,left_foot_vy,left_foot_vz"
                   ",right_foot_x,right_foot_y,right_foot_z,right_foot_roll,right_foot_pitch,right_foot_yaw"
                   ",right_foot_vx,right_foot_vy,right_foot_vz"
                   ",swing_foot_des_x,swing_foot_des_y,swing_foot_des_z"
                   ",swing_foot_des_roll,swing_foot_des_pitch,swing_foot_des_yaw"
                   ",left_touch,right_touch,left_fz,right_fz,contact_count,qp_status,qp_nwsr,qp_cpu_time"
                   ",qp_equality_residual_inf,qp_inequality_violation_max,stance_foot_velocity_norm"
                   ",stance_foot_slip_xy,wbc_left_fx,wbc_left_fy,wbc_left_fz,wbc_left_tx,wbc_left_ty,wbc_left_tz"
                   ",wbc_right_fx,wbc_right_fy,wbc_right_fz,wbc_right_tx,wbc_right_ty,wbc_right_tz"
                   ",wbc_ref_left_fx,wbc_ref_left_fy,wbc_ref_left_fz"
                   ",wbc_ref_left_tx,wbc_ref_left_ty,wbc_ref_left_tz"
                   ",wbc_ref_right_fx,wbc_ref_right_fy,wbc_ref_right_fz"
                   ",wbc_ref_right_tx,wbc_ref_right_ty,wbc_ref_right_tz"
                   ",measured_left_fx,measured_left_fy,measured_left_fz"
                   ",measured_left_tx,measured_left_ty,measured_left_tz"
                   ",measured_right_fx,measured_right_fy,measured_right_fz"
                   ",measured_right_tx,measured_right_ty,measured_right_tz"
                   ",measured_left_cop_x,measured_left_cop_y"
                   ",measured_right_cop_x,measured_right_cop_y";
        for (const int index : ankleIndices_)
            output_ << ',' << config_.joints.actuated[index] << "_effort_margin";
        for (const auto &name : config_.joints.actuated)
            output_ << ',' << name << "_q_des," << name << "_q," << name << "_dq_des," << name
                    << "_dq," << name << "_tau_ff," << name << "_tau_out";
        output_ << '\n';
    }

    void updateSupportAnchor(const DataBus &state)
    {
        if (state.legState == lastSupportLeg_)
            return;
        if (state.legState == DataBus::LSt)
        {
            leftAnchor_ = state.fe_l_pos_W;
            leftAnchorValid_ = true;
        }
        else if (state.legState == DataBus::RSt)
        {
            rightAnchor_ = state.fe_r_pos_W;
            rightAnchorValid_ = true;
        }
        lastSupportLeg_ = state.legState;
    }

    void appendArray(const double *values, int count)
    {
        for (int index = 0; index < count; ++index)
            output_ << ',' << values[index];
    }

    void appendVector(const Eigen::Vector3d &value)
    {
        for (int index = 0; index < 3; ++index)
            output_ << ',' << value[index];
    }

    void appendRotation(const Eigen::Matrix3d &rotation)
    {
        const Eigen::Vector3d yawPitchRoll = rotation.eulerAngles(2, 1, 0);
        output_ << ',' << yawPitchRoll[2] << ',' << yawPitchRoll[1] << ',' << yawPitchRoll[0];
    }

    std::ofstream output_;
    const RobotModelConfig &config_;
    const Eigen::VectorXd &effortLimits_;
    std::vector<int> ankleIndices_;
    double samplePeriod_{0.01};
    double nextSampleTime_{0.0};
    DataBus::LegState lastSupportLeg_{DataBus::DSt};
    Eigen::Vector3d leftAnchor_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d rightAnchor_{Eigen::Vector3d::Zero()};
    bool leftAnchorValid_{false};
    bool rightAnchorValid_{false};
};

// LYENBOT MODIFY: keep a full-rate in-memory trace and write it only on a safety stop.
class SafetyTrace
{
public:
    SafetyTrace(std::string path, std::size_t capacity, const DataBus &state,
                const RobotModelConfig &config, const WBC_priority &wbc)
        : path_(std::move(path)), capacity_(capacity)
    {
        std::ostringstream header;
        header << "time,motion_state,leg_state,leg_state_next,contact_release,contact_count"
                  ",qp_status,qp_nwsr,qp_cpu_time,qp_equality_residual_inf"
                  ",qp_inequality_violation_max,swing_foot_fz_max";
        appendIndexedHeader(header, "q", state.q.size());
        appendIndexedHeader(header, "dq", state.dq.size());
        appendIndexedHeader(header, "kin_ddq", state.dq.size());
        appendIndexedHeader(header, "qp_ddq", state.wbc_ddq_qp.size());
        appendIndexedHeader(header, "wbc_wrench", 12);
        appendIndexedHeader(header, "wbc_ref_wrench", state.Fr_ff.size());
        appendIndexedHeader(header, "measured_wrench", 12);
        for (const auto &task : wbc.kin_tasks_walk.taskLib)
            header << ",task_" << task.taskName << "_ddq_9";
        for (const auto &name : config.joints.actuated)
            header << ',' << name << "_tau_des," << name << "_tau_out";
        header_ = header.str();
        expectedColumns_ = 1 + std::count(header_.begin(), header_.end(), ',');
    }

    void capture(double time, const DataBus &state,
                 const Eigen::Matrix<double, 12, 1> &measuredWrench, int contactCount,
                 const WBC_priority &wbc)
    {
        std::ostringstream row;
        row << time << ',' << static_cast<int>(state.motionState) << ','
            << static_cast<int>(state.legState) << ',' << static_cast<int>(state.legStateNext) << ','
            << state.wbc_contact_release << ',' << contactCount << ',' << state.qp_status << ','
            << state.qp_nWSR << ',' << state.qp_cpuTime << ','
            << state.wbc_qp_equality_residual_inf << ','
            << state.wbc_qp_inequality_violation_max << ',' << state.wbc_swing_foot_fz_max;
        appendEigen(row, state.q);
        appendEigen(row, state.dq);
        appendEigen(row, state.wbc_ddq_final);
        appendEigen(row, state.wbc_ddq_qp);
        appendEigen(row, state.wbc_FrRes);
        appendEigen(row, state.Fr_ff);
        appendEigen(row, measuredWrench);
        for (const auto &task : wbc.kin_tasks_walk.taskLib)
            row << ',' << (task.ddq.size() > 9 ? task.ddq[9] : 0.0);
        for (std::size_t index = 0; index < state.motors_tor_out.size(); ++index)
        {
            const double desired = index < state.motors_tor_des.size()
                                       ? state.motors_tor_des[index] : 0.0;
            row << ',' << desired << ',' << state.motors_tor_out[index];
        }
        const std::string rowText = row.str();
        const std::size_t rowColumns = 1 + std::count(rowText.begin(), rowText.end(), ',');
        if (rowColumns != expectedColumns_)
            throw std::runtime_error("Safety trace column mismatch");
        rows_.push_back(rowText);
        if (rows_.size() > capacity_)
            rows_.pop_front();
    }

    bool dump() const
    {
        std::ofstream output(path_);
        if (!output)
            return false;
        output << header_ << '\n';
        for (const auto &row : rows_)
            output << row << '\n';
        return static_cast<bool>(output);
    }

    const std::string &path() const { return path_; }

private:
    static void appendIndexedHeader(std::ostringstream &header, const std::string &prefix, int size)
    {
        for (int index = 0; index < size; ++index)
            header << ',' << prefix << '_' << index;
    }

    template<typename Derived>
    static void appendEigen(std::ostringstream &row, const Eigen::MatrixBase<Derived> &value)
    {
        for (int index = 0; index < value.size(); ++index)
            row << ',' << value.derived()(index);
    }

    std::string path_;
    std::string header_;
    std::size_t capacity_;
    std::size_t expectedColumns_{0};
    std::deque<std::string> rows_;
};
}

int main(int argc, char **argv)
{
    mjModel *mjModelPtr = nullptr;
    mjData *mjDataPtr = nullptr;
    std::unique_ptr<UIctr> uiController;
    try
    {
        const Stage stage = parseStage(argc, argv);
        const bool enableMpc = stage == Stage::Walk && hasArgument(argc, argv, "mpc");
        const bool useUpstreamWbcInputs = stage == Stage::Walk
                                          && hasArgument(argc, argv, "--upstream-wbc-inputs");
        const bool useCoordinatedTransfer = stage == Stage::Walk
                                             && hasArgument(argc, argv, "--coordinated-transfer");
        const bool useSwingQualityCandidate = stage == Stage::Walk
                                               && hasArgument(argc, argv, "--swing-quality-candidate");
        const bool useModerateRollCenteredSwing = stage == Stage::Walk
            && hasArgument(argc, argv, "--moderate-roll-centered-swing-candidate");
        constexpr double coordinatedTransferTime = 2.25;
        if (useCoordinatedTransfer && !useUpstreamWbcInputs)
            throw std::runtime_error("--coordinated-transfer requires --upstream-wbc-inputs");
        if (useSwingQualityCandidate && !useCoordinatedTransfer)
            throw std::runtime_error(
                "--swing-quality-candidate requires --coordinated-transfer");
        if (useModerateRollCenteredSwing && !useSwingQualityCandidate)
            throw std::runtime_error(
                "--moderate-roll-centered-swing-candidate requires --swing-quality-candidate");
        if (enableMpc && useUpstreamWbcInputs)
            throw std::runtime_error("mpc and --upstream-wbc-inputs cannot be enabled together");
        const bool enableGui = hasArgument(argc, argv, "--gui") || hasArgument(argc, argv, "gui");
        const std::string defaultDiagnosticPath = stage == Stage::StepLeft
                                                      ? "../record/lyenbot_step_left_diagnostic.csv"
                                                  : stage == Stage::StepRight
                                                      ? "../record/lyenbot_step_right_diagnostic.csv"
                                                  : useModerateRollCenteredSwing
                                                      ? "../record/lyenbot_walk_moderate_roll_centered_swing.csv"
                                                  : useSwingQualityCandidate
                                                      ? "../record/lyenbot_walk_swing_quality_candidate.csv"
                                                  : useCoordinatedTransfer
                                                      ? "../record/lyenbot_walk_coordinated_transfer_diagnostic.csv"
                                                  : useUpstreamWbcInputs
                                                      ? "../record/lyenbot_walk_upstream_inputs_diagnostic.csv"
                                                      : "../record/lyenbot_walk_diagnostic.csv";
        const std::string diagnosticPath = argumentValue(argc, argv, "--diagnostic=",
                                                          defaultDiagnosticPath);
        const auto config = loadRobotModelConfig("../common/robot_configs/lyenbot.json");
        char error[2048] = {};
        mjModelPtr = mj_loadXML(config.scenePath.c_str(), nullptr, error, sizeof(error));
        if (!mjModelPtr)
            throw std::runtime_error(std::string("Cannot load Lyenbot scene: ") + error);
        const double robotWeight = -mj_getTotalmass(mjModelPtr) * mjModelPtr->opt.gravity[2];
        if (useUpstreamWbcInputs)
            std::cout << "walk_wbc_input_mode=upstream-compatible robot_weight=" << robotWeight
                      << " nominal_per_foot_fz=" << 0.5 * robotWeight << '\n';
        if (useCoordinatedTransfer)
            std::cout << "coordinated_transfer_time=" << coordinatedTransferTime
                      << " pending_support_preload_ratio=0.10\n";
        if (useSwingQualityCandidate)
            std::cout << "swing_quality_candidate step_height=0.035"
                         " peak_pitch=0.10 pitch_return_phase=0.70\n";
        if (useModerateRollCenteredSwing)
            std::cout << "moderate_roll_centered_swing base_height=0.805 base_roll=0.375"
                         " base_outward_offset=0.00025 com_inward_offset=0.0147\n";
        mjDataPtr = mj_makeData(mjModelPtr);
        if (!mjDataPtr)
            throw std::runtime_error("Cannot allocate MuJoCo data");
        const int leftFootCollisionGeom = mj_name2id(mjModelPtr, mjOBJ_GEOM, "lf-tc-collision");
        const int rightFootCollisionGeom = mj_name2id(mjModelPtr, mjOBJ_GEOM, "rf-tc-collision");
        if (leftFootCollisionGeom < 0 || rightFootCollisionGeom < 0)
            throw std::runtime_error("Cannot find Lyenbot foot collision geoms");
        if (mjModelPtr->nkey > 0)
            mj_resetDataKeyframe(mjModelPtr, mjDataPtr, 0);
        mj_forward(mjModelPtr, mjDataPtr);
        if (enableGui)
        {
            uiController = std::make_unique<UIctr>(mjModelPtr, mjDataPtr);
            uiController->iniGLFW();
            uiController->enableTracking();
            uiController->createWindow("Lyenbot staged simulation", false);
            uiController->updateScene();
        }
        double minimumContactDistance = 0.0;
        for (int contactId = 0; contactId < mjDataPtr->ncon; ++contactId)
            minimumContactDistance = std::min(minimumContactDistance, mjDataPtr->contact[contactId].dist);
        std::cout << "initial_contacts=" << mjDataPtr->ncon
                  << " minimum_contact_distance=" << minimumContactDistance << '\n';

        Pin_KinDyn kinDyn(config);
        DataBus state(kinDyn.model_nv);
        MJ_Interface mjInterface(mjModelPtr, mjDataPtr, config);
        PVT_Ctr pvt(mjModelPtr->opt.timestep, config.jointControlPath.c_str(), config.joints.actuated,
                    kinDyn.motorMaxTorque, kinDyn.motorMaxSpeed, kinDyn.motorMaxPos, kinDyn.motorMinPos);
        WBC_priority wbc(kinDyn.model_nv, 18, 26, 0.6, mjModelPtr->opt.timestep, &config, &kinDyn.model_biped);
        MPC mpc(0.005);
        GaitScheduler gait(config.swingTime, mjModelPtr->opt.timestep);
        gait.useMeasuredContact = true;
        gait.FzThrehold = config.touchdownForce;
        gait.minimumTouchdownPhase = 0.75;
        // LYENBOT MODIFY: pause lateral load transfer while the newly landed
        // compound foot is no longer carrying a measurable contact force.
        gait.minimumTransferContactForce = 5.0;
        gait.enableDoubleSupportTransfer = true;
        // LYENBOT MODIFY: allow the coordinated support posture to move between
        // mirrored feasible regions while both feet remain constrained.
        gait.doubleSupportTime = useCoordinatedTransfer ? coordinatedTransferTime : 3.0;
        FootPlacement footPlacement;
        JoyStickInterpreter joystick(mjModelPtr->opt.timestep);
        std::unique_ptr<WalkDiagnostics> walkDiagnostics;
        std::unique_ptr<SafetyTrace> safetyTrace;
        if (stage == Stage::Walk || isSingleStep(stage))
        {
            walkDiagnostics = std::make_unique<WalkDiagnostics>(diagnosticPath, config,
                                                                 kinDyn.motorMaxTorque, 0.01,
                                                                 stage == Stage::Walk ? 9.0 : 7.0);
            std::cout << "walk_diagnostic=" << diagnosticPath << " sample_hz=100\n";
        }
        if (isSingleStep(stage))
            safetyTrace = std::make_unique<SafetyTrace>(diagnosticPath + ".safety.csv",
                static_cast<std::size_t>(std::ceil(0.1 / mjModelPtr->opt.timestep)), state, config, wbc);

        const int motorCount = config.joints.actuated.size();
        const auto actuatedIndex = [&](const std::string &name)
        {
            const auto iterator = std::find(config.joints.actuated.begin(),
                                            config.joints.actuated.end(), name);
            if (iterator == config.joints.actuated.end())
                throw std::runtime_error("Missing actuated joint: " + name);
            return static_cast<int>(std::distance(config.joints.actuated.begin(), iterator));
        };
        const int leftKneeIndex = actuatedIndex("left_knee_pitch_joint");
        const int rightKneeIndex = actuatedIndex("right_knee_pitch_joint");
        const int leftAnklePitchIndex = actuatedIndex("left_ankle_pitch_joint");
        const int rightAnklePitchIndex = actuatedIndex("right_ankle_pitch_joint");
        state.motors_pos_des = config.initialJointPositions;
        state.motors_vel_des.assign(motorCount, 0.0);
        state.motors_tor_des.assign(motorCount, 0.0);
        state.width_hips = config.hipWidth;
        footPlacement.legLength = config.nominalBaseHeight - config.footHeight;
        footPlacement.stepHeight = useSwingQualityCandidate ? 0.035 : 0.025;
        // LYENBOT MODIFY: remove the AzureLoong backward-step bias and preserve
        // Lyenbot's wider nominal stance with a mirrored outward foot offset.
        footPlacement.forwardOffset = 0.0;
        footPlacement.inwardOffset = -0.04;
        // LYENBOT MODIFY: the Lyenbot foot frame lands near world z=0; retain
        // only the small penetration already validated by the single-step path.
        footPlacement.landingHeightOffset = -0.010;
        footPlacement.lateTouchdownStretchStep = 0.0;
        footPlacement.maxStepLength = 0.10;
        footPlacement.maxStepWidthChange = 0.03;
        footPlacement.kp_vx = 0.03;
        footPlacement.kp_vy = 0.03;
        footPlacement.kp_wz = 0.03;

        state.base_pos_des << 0, 0, config.nominalBaseHeight;
        state.base_rpy_des.setZero();
        state.des_ddq.setZero();
        state.des_dq.setZero();
        state.des_delta_q.setZero();
        state.motionState = DataBus::Stand;

        bool initialized = false;
        bool wbcInitialized = false;
        int mpcCounter = 0;
        int stepTransitions = 0;
        int nextDiagnosticSecond = 1;
        Eigen::Vector3d walkPreparationCom = Eigen::Vector3d::Zero();
        Eigen::Vector3d stepTransferCom = Eigen::Vector3d::Zero();
        Eigen::Vector3d stepSwingStart = Eigen::Vector3d::Zero();
        Eigen::Vector3d stepStanceAnchor = Eigen::Vector3d::Zero();
        Eigen::Vector3d stepBaseTarget = Eigen::Vector3d::Zero();
        Eigen::Vector3d stepTouchdownCom = Eigen::Vector3d::Zero();
        Eigen::VectorXd stepWrenchStart = Eigen::VectorXd::Zero(12);
        Eigen::VectorXd stepWrenchTarget = Eigen::VectorXd::Zero(12);
        bool stepReferenceInitialized = false;
        bool stepLoadTransferInitialized = false;
        bool stepSingleSupportInitialized = false;
        bool stepReleaseConfirmed = false;
        bool stepAirborne = false;
        bool stepTouchdown = false;
        double stepReleaseStableDuration = 0.0;
        double stepTouchdownContactDuration = 0.0;
        double stepReleaseConfirmedTime = 0.0;
        double stepTouchdownTime = 0.0;
        double stepRecoveryCenterY = 0.0;
        double stepStableDuration = 0.0;
        double stepMaximumStanceSlip = 0.0;
        double stepMaximumRoll = 0.0;
        double stepMaximumPitch = 0.0;
        double nextRenderTime = mjDataPtr->time;
        DataBus::LegState previousLeg = DataBus::DSt;
        // LYENBOT MODIFY: leave enough time for the larger CoM shift to the
        // stance-foot center before unloading the future swing foot.
        constexpr double stepLateralTransferStart = 8.0;
        constexpr double stepLoadTransferStart = 11.0;
        constexpr double stepSingleSupportStart = 13.0;
        // LYENBOT MODIFY: continuous walk starts from the same feasible-posture
        // family validated by the independent single-step acceptance path.
        constexpr double walkPreparationStart = 10.0;
        constexpr double walkStart = 13.0;
        const double walkBaseHeight = useModerateRollCenteredSwing ? 0.805 : 0.80;
        const double walkBaseRoll = useModerateRollCenteredSwing ? 0.375 : 0.40;
        const double walkComInwardOffset = useModerateRollCenteredSwing ? 0.0147 : 0.004;
        const double walkBaseOutwardOffset = useModerateRollCenteredSwing ? 0.00025 : 0.013;
        const double duration = stage == Stage::Pd ? 10.0
                                : stage == Stage::Wbc ? 13.0
                                : isSingleStep(stage) ? 24.0
                                                     : 65.0;

        while (mjDataPtr->time < duration
               && (!uiController || !glfwWindowShouldClose(uiController->window)))
        {
            if (uiController && !uiController->runSim)
            {
                uiController->updateScene();
                continue;
            }
            mjInterface.updateSensorValues();
            mjInterface.dataBusWrite(state);
            kinDyn.dataBusRead(state);
            kinDyn.computeJ_dJ();
            kinDyn.computeDyn();
            kinDyn.dataBusWrite(state);
            // LYENBOT MODIFY: sample the complete compound-foot contact wrench
            // before the release state machine and controller update.
            const Eigen::Matrix<double, 12, 1> measuredWrench = measuredFootWrenches(
                mjModelPtr, mjDataPtr, leftFootCollisionGeom, rightFootCollisionGeom, state);

            if (!initialized)
            {
                initialized = true;
                state.base_pos_des << state.q(0), state.q(1), config.nominalBaseHeight;
                state.swing_fe_pos_des_W = state.fe_r_pos_W;
                state.swing_fe_rpy_des_W.setZero();
                state.stance_fe_pos_cur_W = state.fe_l_pos_W;
                state.stance_fe_rot_cur_W = state.fe_l_rot_W;
                state.stanceDesPos_W = state.fe_l_pos_W;
                state.js_pos_des << state.q(0), state.q(1), config.nominalBaseHeight;
                joystick.setIniPos(state.q(0), state.q(1), config.nominalBaseHeight, state.rpy[2]);
            }

            const bool useWbc = stage != Stage::Pd && mjDataPtr->time >= 3.0;
            const bool prepareWalk = stage == Stage::Walk
                                     && mjDataPtr->time >= walkPreparationStart
                                     && mjDataPtr->time < walkStart;
            const bool useWalk = stage == Stage::Walk && mjDataPtr->time >= walkStart;
            if (useWbc && !wbcInitialized)
            {
                wbcInitialized = true;
                wbc.pCoMDes = state.pCoM_W;
                state.base_pos_des << state.q(0), state.q(1), state.q(2);
                walkPreparationCom = state.pCoM_W;
            }
            // LYENBOT MODIFY: opt-in comparison path for restoring the complete
            // upstream walk_wbc seed inputs without changing the accepted baseline.
            if (useUpstreamWbcInputs && useWbc)
            {
                state.Fr_ff.setZero();
                state.Fr_ff[2] = 0.5 * robotWeight;
                state.Fr_ff[8] = 0.5 * robotWeight;
            }
            if (prepareWalk)
            {
                const double blend = smoothBlend(
                    (mjDataPtr->time - walkPreparationStart) / (walkStart - walkPreparationStart));
                const double leftSupportComTarget = state.fe_l_pos_W.y() - walkComInwardOffset;
                wbc.pCoMDes = walkPreparationCom;
                wbc.pCoMDes.y() = (1.0 - blend) * walkPreparationCom.y()
                                  + blend * leftSupportComTarget;
                state.base_rpy_des.x() = -blend * walkBaseRoll;
                state.base_pos_des.z() = (1.0 - blend) * config.nominalBaseHeight
                                         + blend * walkBaseHeight;
            }
            if (useWalk)
            {
                if (state.motionState != DataBus::Walk)
                {
                    if (enableMpc)
                        mpc.enable();
                    joystick.setIniPos(state.q(0), state.q(1), walkBaseHeight, state.rpy[2]);
                }
                state.motionState = DataBus::Walk;
                // LYENBOT MODIFY: establish the pure-WBC acceptance baseline at
                // low speed before restoring the configured MPC target speed.
                constexpr double pureWbcWalkingSpeed = 0.03;
                const bool pauseForDoubleSupport = !enableMpc && stepTransitions > 0
                                                   && state.legState == DataBus::DSt;
                const double requestedWalkingSpeed = enableMpc
                    ? config.walkingSpeed
                    : pauseForDoubleSupport ? 0.0
                    : std::min(config.walkingSpeed, pureWbcWalkingSpeed);
                joystick.setVxDesLPara(requestedWalkingSpeed,
                                       pauseForDoubleSupport ? 0.2 : 2.0);
                joystick.setWzDesLPara(0, 1.0);
                joystick.step();
                joystick.dataBusWrite(state);
                state.js_pos_des(2) = walkBaseHeight;
                if (useUpstreamWbcInputs)
                {
                    state.des_delta_q.setZero();
                    state.des_dq.setZero();
                    state.des_ddq.setZero();
                    // LYENBOT MODIFY: reproduce the original walk_wbc motion
                    // seed as one coordinated set after the velocity ramp starts.
                    if (mjDataPtr->time > walkStart + 1.0)
                    {
                        state.des_delta_q.head<2>() = state.js_vel_des.head<2>()
                                                      * mjModelPtr->opt.timestep;
                        state.des_delta_q[5] = state.js_omega_des[2]
                                               * mjModelPtr->opt.timestep;
                        state.des_dq.head<2>() = state.js_vel_des.head<2>();
                        state.des_dq[5] = state.js_omega_des[2];
                        constexpr double upstreamAccelerationGain = 5.0;
                        state.des_ddq.head<2>() = upstreamAccelerationGain
                            * (state.js_vel_des.head<2>() - state.dq.head<2>());
                        state.des_ddq[5] = upstreamAccelerationGain
                            * (state.js_omega_des[2] - state.dq[5]);
                    }
                }

                // LYENBOT MODIFY: drive gait touchdown from the complete
                // compound-foot contact force instead of the contact-box-only sensor.
                state.fL[2] = measuredWrench[2];
                state.fR[2] = measuredWrench[8];
                gait.dataBusRead(state);
                gait.step();
                gait.dataBusWrite(state);
                const double leftSupportTarget = state.fe_l_pos_W.y() + walkBaseOutwardOffset;
                const double rightSupportTarget = state.fe_r_pos_W.y() - walkBaseOutwardOffset;
                const double leftSupportComTarget = state.fe_l_pos_W.y() - walkComInwardOffset;
                const double rightSupportComTarget = state.fe_r_pos_W.y() + walkComInwardOffset;
                // LYENBOT MODIFY: Walk currently regulates base position rather
                // than pCoMDes; keep the coordinated CoM target observable for
                // the next upstream planner migration without adding a new task.
                state.base_pos_des.z() = walkBaseHeight;
                if (stepTransitions == 0)
                {
                    state.base_pos_des.y() = leftSupportTarget;
                    state.base_rpy_des.x() = -walkBaseRoll;
                    if (useCoordinatedTransfer)
                        wbc.pCoMDes.y() = leftSupportComTarget;
                }
                else if (state.legState == DataBus::LSt)
                {
                    state.base_pos_des.y() = leftSupportTarget;
                    state.base_rpy_des.x() = -walkBaseRoll;
                    if (useCoordinatedTransfer)
                        wbc.pCoMDes.y() = leftSupportComTarget;
                }
                else if (state.legState == DataBus::RSt)
                {
                    state.base_pos_des.y() = rightSupportTarget;
                    state.base_rpy_des.x() = walkBaseRoll;
                    if (useCoordinatedTransfer)
                        wbc.pCoMDes.y() = rightSupportComTarget;
                }
                else if (state.legStateNext == DataBus::RSt)
                {
                    const double transferBlend = useCoordinatedTransfer
                        ? smoothBlend(gait.transferPhi) : gait.transferPhi;
                    state.base_pos_des.y() = (1.0 - transferBlend) * leftSupportTarget
                                             + transferBlend * rightSupportTarget;
                    state.base_rpy_des.x() = (2.0 * transferBlend - 1.0) * walkBaseRoll;
                    if (useCoordinatedTransfer)
                        wbc.pCoMDes.y() = (1.0 - transferBlend) * leftSupportComTarget
                                          + transferBlend * rightSupportComTarget;
                }
                else if (state.legStateNext == DataBus::LSt)
                {
                    const double transferBlend = useCoordinatedTransfer
                        ? smoothBlend(gait.transferPhi) : gait.transferPhi;
                    state.base_pos_des.y() = (1.0 - transferBlend) * rightSupportTarget
                                             + transferBlend * leftSupportTarget;
                    state.base_rpy_des.x() = (1.0 - 2.0 * transferBlend) * walkBaseRoll;
                    if (useCoordinatedTransfer)
                        wbc.pCoMDes.y() = (1.0 - transferBlend) * rightSupportComTarget
                                          + transferBlend * leftSupportComTarget;
                }
                if (useCoordinatedTransfer)
                {
                    // LYENBOT MODIFY: migrate the nominal wrench with the same
                    // phase as posture while leaving actual load sharing to QP.
                    state.Fr_ff.setZero();
                    state.wbc_swing_foot_fz_max = 1e10;
                    if (state.legState == DataBus::LSt)
                        state.Fr_ff[2] = robotWeight;
                    else if (state.legState == DataBus::RSt)
                        state.Fr_ff[8] = robotWeight;
                    else
                    {
                        const double transferBlend = smoothBlend(gait.transferPhi);
                        // LYENBOT MODIFY: preload the newly landed foot so the
                        // measured-contact gate cannot deadlock at zero phase.
                        constexpr double pendingSupportPreloadRatio = 0.10;
                        const double pendingSupportRatio = pendingSupportPreloadRatio
                            + (1.0 - pendingSupportPreloadRatio) * transferBlend;
                        if (state.legStateNext == DataBus::RSt)
                        {
                            state.Fr_ff[2] = (1.0 - pendingSupportRatio) * robotWeight;
                            state.Fr_ff[8] = pendingSupportRatio * robotWeight;
                        }
                        else
                        {
                            state.Fr_ff[2] = pendingSupportRatio * robotWeight;
                            state.Fr_ff[8] = (1.0 - pendingSupportRatio) * robotWeight;
                        }
                    }
                }
                footPlacement.dataBusRead(state);
                footPlacement.getSwingPos();
                footPlacement.dataBusWrite(state);
                if (useSwingQualityCandidate)
                {
                    // LYENBOT MODIFY: use the offline-feasible foot pitch only
                    // during flight and return it smoothly before the measured
                    // touchdown window. This changes no accepted default path.
                    constexpr double pitchRiseEnd = 0.20;
                    constexpr double pitchReturnEnd = 0.70;
                    constexpr double peakSwingPitch = 0.10;
                    double pitchBlend = 0.0;
                    if (state.phi < pitchRiseEnd)
                        pitchBlend = smoothBlend(state.phi / pitchRiseEnd);
                    else if (state.phi < pitchReturnEnd)
                        pitchBlend = 1.0 - smoothBlend(
                            (state.phi - pitchRiseEnd) / (pitchReturnEnd - pitchRiseEnd));
                    state.swing_fe_rpy_des_W.y() = peakSwingPitch * pitchBlend;
                }
                if (useModerateRollCenteredSwing)
                {
                    // LYENBOT MODIFY: keep the low-roll candidate inside the
                    // fixed-anchor lateral family used by the offline scan.
                    // Inward swing remains allowed; only outward positive
                    // feedback from the live hip/base position is removed.
                    if (state.legState == DataBus::LSt)
                        state.swing_fe_pos_des_W.y() = std::max(
                            state.swing_fe_pos_des_W.y(), state.swingStartPos_W.y());
                    else if (state.legState == DataBus::RSt)
                        state.swing_fe_pos_des_W.y() = std::min(
                            state.swing_fe_pos_des_W.y(), state.swingStartPos_W.y());
                }
                if (previousLeg != DataBus::DSt && state.legState != previousLeg)
                    ++stepTransitions;
                previousLeg = state.legState;

                // LYENBOT MODIFY: a disabled MPC must not overwrite the pure
                // WBC walk references produced by the joystick and gait path.
                if (enableMpc && ++mpcCounter >= 5)
                {
                    mpc.dataBusRead(state);
                    mpc.cal();
                    mpc.dataBusWrite(state);
                    mpcCounter = 0;
                }
            }

            // LYENBOT MODIFY: quasi-static in-place single-step acceptance path.
            if (isSingleStep(stage) && mjDataPtr->time >= stepLateralTransferStart)
            {
                state.wbc_contact_release = false;
                const bool swingLeft = stage == Stage::StepLeft;
                const DataBus::LegState supportLeg = swingLeft ? DataBus::RSt : DataBus::LSt;
                const Eigen::Vector3d &swingPosition = swingLeft ? state.fe_l_pos_W : state.fe_r_pos_W;
                const Eigen::Vector3d &stancePosition = swingLeft ? state.fe_r_pos_W : state.fe_l_pos_W;
                const double measuredSwingForce = measuredWrench[swingLeft ? 2 : 8];
                const bool leftTouch = measuredWrench[2] >= config.touchdownForce;
                const bool rightTouch = measuredWrench[8] >= config.touchdownForce;
                if (!stepReferenceInitialized)
                {
                    stepReferenceInitialized = true;
                    stepTransferCom = state.pCoM_W;
                    stepSwingStart = swingPosition;
                    stepStanceAnchor = stancePosition;
                }

                // LYENBOT MODIFY: use the independently scanned feasible posture
                // instead of translating an upright, nearly straight-leg stance.
                constexpr double supportTargetInwardOffset = 0.004;
                constexpr double feasibleBaseHeight = 0.80;
                constexpr double feasibleBaseRoll = 0.40;
                const double supportTargetY = swingLeft
                                                  ? stepStanceAnchor.y() + supportTargetInwardOffset
                                                  : stepStanceAnchor.y() - supportTargetInwardOffset;
                const double supportRollTarget = swingLeft ? feasibleBaseRoll : -feasibleBaseRoll;
                const double postureBlend = smoothBlend(std::min(1.0,
                    (mjDataPtr->time - stepLateralTransferStart)
                    / (stepLoadTransferStart - stepLateralTransferStart)));
                state.base_rpy_des.x() = postureBlend * supportRollTarget;
                state.base_pos_des.z() = (1.0 - postureBlend) * config.nominalBaseHeight
                                         + postureBlend * feasibleBaseHeight;
                if (mjDataPtr->time >= stepLoadTransferStart && !stepLoadTransferInitialized)
                {
                    stepLoadTransferInitialized = true;
                    stepWrenchStart = state.wbc_FrRes;
                    stepBaseTarget << state.q(0), state.q(1), feasibleBaseHeight;
                    // LYENBOT MODIFY: preserve the lateral CoM target when switching
                    // from the Stand CoM task to the Walk base task.
                    stepBaseTarget.y() += supportTargetY - state.pCoM_W.y();
                    const int stanceOffset = swingLeft ? 6 : 0;
                    const Eigen::Vector3d totalForce = stepWrenchStart.segment<3>(0)
                                                       + stepWrenchStart.segment<3>(6);
                    const Eigen::Vector3d totalTorqueAtStance = stepWrenchStart.segment<3>(3)
                        + (state.fe_l_pos_W - stancePosition).cross(stepWrenchStart.segment<3>(0))
                        + stepWrenchStart.segment<3>(9)
                        + (state.fe_r_pos_W - stancePosition).cross(stepWrenchStart.segment<3>(6));
                    stepWrenchTarget.segment<3>(stanceOffset) = totalForce;
                    stepWrenchTarget.segment<3>(stanceOffset + 3) = totalTorqueAtStance;
                }
                if (stepLoadTransferInitialized && !stepTouchdown)
                {
                    const double loadBlend = smoothBlend(
                        (mjDataPtr->time - stepLoadTransferStart) / 2.0);
                    state.Fr_ff = (1.0 - loadBlend) * stepWrenchStart + loadBlend * stepWrenchTarget;
                    const int swingOffset = swingLeft ? 0 : 6;
                    state.wbc_swing_foot_fz_max = std::max(0.0,
                        (1.0 - loadBlend) * stepWrenchStart[swingOffset + 2]);
                }
                if (mjDataPtr->time < stepLoadTransferStart)
                {
                    const double blend = smoothBlend(std::min(1.0,
                        (mjDataPtr->time - stepLateralTransferStart)
                        / (stepLoadTransferStart - stepLateralTransferStart)));
                    state.motionState = DataBus::Stand;
                    wbc.pCoMDes = stepTransferCom;
                    wbc.pCoMDes.y() = (1.0 - blend) * stepTransferCom.y() + blend * supportTargetY;
                }
                else if (mjDataPtr->time < stepSingleSupportStart)
                {
                    state.motionState = DataBus::Walk;
                    state.legState = DataBus::DSt;
                    state.legStateNext = supportLeg;
                    state.stance_fe_pos_cur_W = stancePosition;
                    state.stance_fe_rot_cur_W = swingLeft ? state.fe_r_rot_W : state.fe_l_rot_W;
                    state.base_pos_des = stepBaseTarget;
                    state.des_dq.setZero();
                    state.des_ddq.setZero();
                    state.des_delta_q.setZero();
                }
                else if (!stepTouchdown)
                {
                    if (!stepSingleSupportInitialized)
                    {
                        stepSingleSupportInitialized = true;
                        stepSwingStart = swingPosition;
                        stepStanceAnchor = stancePosition;
                    }
                    // LYENBOT MODIFY: retain double-foot wrench constraints while
                    // freeing the swing kinematics, then declare single support
                    // only after the complete measured contact force is released.
                    if (!stepReleaseConfirmed)
                    {
                        if (measuredSwingForce < 5.0)
                            stepReleaseStableDuration += mjModelPtr->opt.timestep;
                        else
                            stepReleaseStableDuration = 0.0;
                        if (stepReleaseStableDuration >= 0.020)
                        {
                            stepReleaseConfirmed = true;
                            stepReleaseConfirmedTime = mjDataPtr->time;
                        }
                    }
                    state.motionState = DataBus::Walk;
                    state.legState = stepReleaseConfirmed ? supportLeg : DataBus::DSt;
                    state.legStateNext = stepReleaseConfirmed ? DataBus::DSt : supportLeg;
                    state.wbc_contact_release = !stepReleaseConfirmed;
                    state.stance_fe_pos_cur_W = stancePosition;
                    state.stance_fe_rot_cur_W = swingLeft ? state.fe_r_rot_W : state.fe_l_rot_W;
                    state.base_pos_des = stepBaseTarget;
                    const double stepTime = mjDataPtr->time - stepSingleSupportStart;
                    state.des_dq.setZero();
                    state.des_ddq.setZero();
                    state.des_delta_q.setZero();
                    double height = 0.0;
                    if (stepTime < 1.0)
                        height = 0.025 * smoothBlend(stepTime);
                    else if (stepTime < 1.5)
                        height = 0.025;
                    else if (stepTime < 2.5)
                        height = 0.025 * (1.0 - smoothBlend(stepTime - 1.5));
                    else
                        height = -0.003;
                    state.phi = std::min(1.0, stepTime / 3.0);
                    state.swing_fe_pos_des_W = stepSwingStart;
                    state.swing_fe_pos_des_W.z() += height;
                    state.swing_fe_rpy_des_W.setZero();
                    if (stepReleaseConfirmed && measuredSwingForce < 5.0
                        && swingPosition.z() > stepSwingStart.z() + 0.008)
                        stepAirborne = true;
                    if (stepAirborne && mjDataPtr->time - stepReleaseConfirmedTime >= 2.0)
                    {
                        if (leftTouch && rightTouch)
                            stepTouchdownContactDuration += mjModelPtr->opt.timestep;
                        else
                            stepTouchdownContactDuration = 0.0;
                        // LYENBOT MODIFY: reject the first impact bounce; only a
                        // continuously loaded compound foot can end single support.
                        if (stepTouchdownContactDuration >= 0.050)
                        {
                            stepTouchdown = true;
                            stepTouchdownTime = mjDataPtr->time;
                            stepTouchdownCom = state.pCoM_W;
                            stepRecoveryCenterY = 0.5
                                * (state.fe_l_pos_W.y() + state.fe_r_pos_W.y());
                            wbc.pCoMDes = stepTouchdownCom;
                        }
                    }
                    stepMaximumStanceSlip = std::max(stepMaximumStanceSlip,
                        (stancePosition.head<2>() - stepStanceAnchor.head<2>()).norm());
                }
                else
                {
                    state.Fr_ff.setZero();
                    state.wbc_swing_foot_fz_max = 1e10;
                    state.motionState = DataBus::Stand;
                    state.legState = DataBus::DSt;
                    state.legStateNext = DataBus::DSt;
                    constexpr double touchdownHoldDuration = 0.5;
                    constexpr double recoveryDuration = 3.0;
                    const double recoveryTime = mjDataPtr->time - stepTouchdownTime;
                    const double blend = smoothBlend(
                        (recoveryTime - touchdownHoldDuration) / recoveryDuration);
                    state.base_rpy_des.x() = (1.0 - blend) * supportRollTarget;
                    state.base_pos_des.z() = (1.0 - blend) * feasibleBaseHeight
                                             + blend * config.nominalBaseHeight;
                    wbc.pCoMDes = stepTouchdownCom;
                    wbc.pCoMDes.y() = (1.0 - blend) * stepTouchdownCom.y()
                                      + blend * stepRecoveryCenterY;
                    if (blend >= 1.0 && leftTouch && rightTouch)
                        stepStableDuration += mjModelPtr->opt.timestep;
                    else
                        stepStableDuration = 0.0;
                }
                stepMaximumRoll = std::max(stepMaximumRoll, std::abs(state.rpy[0]));
                stepMaximumPitch = std::max(stepMaximumPitch, std::abs(state.rpy[1]));
            }

            if (useWbc)
            {
                wbc.dataBusRead(state);
                wbc.computeDdq(kinDyn);
                wbc.computeTau();
                wbc.dataBusWrite(state);
                const auto desiredQ = kinDyn.integrateDIY(state.q, state.wbc_delta_q_final);
                state.motors_pos_des = tailToStd(desiredQ, motorCount);
                // LYENBOT MODIFY: the MuJoCo joint limit is compliant, so keep
                // the PVT position reference inside the URDF limits explicitly.
                for (int index = 0; index < motorCount; ++index)
                {
                    constexpr double walkJointPositionMargin = 0.030;
                    constexpr double walkAnklePitchPositionMargin = 0.060;
                    const bool anklePitch = index == leftAnklePitchIndex
                                            || index == rightAnklePitchIndex;
                    const double requestedMargin = anklePitch
                        ? walkAnklePitchPositionMargin : walkJointPositionMargin;
                    const double positionMargin = stage == Stage::Walk
                        ? std::min(requestedMargin,
                                   0.25 * (kinDyn.motorMaxPos[index] - kinDyn.motorMinPos[index]))
                        : 0.0;
                    state.motors_pos_des[index] = std::clamp(
                        state.motors_pos_des[index],
                        kinDyn.motorMinPos[index] + positionMargin,
                        kinDyn.motorMaxPos[index] - positionMargin);
                }
                if (isSingleStep(stage) || stage == Stage::Walk)
                {
                    // LYENBOT MODIFY: keep both knees away from the 0 rad
                    // straight-leg singularity in Lyenbot walking paths.
                    constexpr double minimumKneeFlexion = 0.03;
                    state.motors_pos_des[leftKneeIndex] = std::max(
                        state.motors_pos_des[leftKneeIndex], minimumKneeFlexion);
                    state.motors_pos_des[rightKneeIndex] = std::max(
                        state.motors_pos_des[rightKneeIndex], minimumKneeFlexion);
                }
                state.motors_vel_des = tailToStd(state.wbc_dq_final, motorCount);
                state.motors_tor_des = eigen2std(state.wbc_tauJointRes);
                // LYENBOT MODIFY: fade only the feedforward component that would
                // push a compliant MuJoCo joint farther outside its URDF range.
                constexpr double jointLimitTorqueFadeMargin = 0.02;
                for (int index = 0; index < motorCount; ++index)
                {
                    const double position = state.motors_pos_cur[index];
                    double &feedforwardTorque = state.motors_tor_des[index];
                    if (feedforwardTorque < 0.0)
                    {
                        const double scale = std::clamp(
                            (position - kinDyn.motorMinPos[index]) / jointLimitTorqueFadeMargin,
                            0.0, 1.0);
                        feedforwardTorque *= scale;
                    }
                    else if (feedforwardTorque > 0.0)
                    {
                        const double scale = std::clamp(
                            (kinDyn.motorMaxPos[index] - position) / jointLimitTorqueFadeMargin,
                            0.0, 1.0);
                        feedforwardTorque *= scale;
                    }
                }
            }

            pvt.dataBusRead(state);
            if (mjDataPtr->time < 3.0)
                pvt.calMotorsPVT(0.002);
            else
                pvt.calMotorsPVT();
            pvt.dataBusWrite(state);
            mjInterface.setMotorsTorque(state.motors_tor_out);
            if (walkDiagnostics)
                walkDiagnostics->writeIfDue(mjDataPtr->time, state, wbc.pCoMDes, measuredWrench,
                    gait.transferPhi, stepTransitions, mjDataPtr->ncon, enableMpc);
            if (safetyTrace && useWbc)
                safetyTrace->capture(mjDataPtr->time, state, measuredWrench, mjDataPtr->ncon, wbc);
            mj_step(mjModelPtr, mjDataPtr);
            if (uiController && mjDataPtr->time >= nextRenderTime)
            {
                uiController->updateScene();
                nextRenderTime = mjDataPtr->time + 1.0 / 60.0;
            }

            if (mjDataPtr->time >= nextDiagnosticSecond)
            {
                const double maxTorque = state.motors_tor_out.empty() ? 0.0
                    : *std::max_element(state.motors_tor_out.begin(), state.motors_tor_out.end(),
                        [](double left, double right) { return std::abs(left) < std::abs(right); });
                std::cout << "t=" << mjDataPtr->time << " z=" << state.basePos[2]
                          << " rpy=" << state.rpy[0] << ' ' << state.rpy[1] << ' ' << state.rpy[2]
                          << " xy=" << state.q.head<2>().transpose() << " des_y=" << state.base_pos_des.y()
                          << " com_xy=" << state.pCoM_W.head<2>().transpose()
                          << " feet_x=" << state.fe_l_pos_W.x() << ' ' << state.fe_r_pos_W.x()
                          << " swing_des=" << state.swing_fe_pos_des_W.transpose()
                          << " leg=" << state.legState << " phi=" << state.phi
                          << " qp=" << state.qp_status << " max_tau=" << maxTorque << '\n';
                ++nextDiagnosticSecond;
            }

            const bool finite = std::isfinite(state.q.norm()) && std::isfinite(state.dq.norm());
            // LYENBOT MODIFY: never continue the acceptance demo with a stale QP solution.
            const bool qpSolved = !useWbc || state.qp_status == 0;
            // LYENBOT MODIFY: a mathematically feasible QP can still produce an
            // unsafe base correction when the declared and measured contacts disagree.
            const double qpBaseAccelerationMax = useWbc
                ? state.wbc_ddq_qp.head<6>().lpNorm<Eigen::Infinity>() : 0.0;
            constexpr double qpBaseAccelerationLimit = 100.0;
            const bool qpOutputSafe = std::isfinite(qpBaseAccelerationMax)
                                      && qpBaseAccelerationMax <= qpBaseAccelerationLimit;
            if (!finite || !qpSolved || !qpOutputSafe || state.basePos[2] < 0.35
                || std::abs(state.rpy[0]) > 1.0 || std::abs(state.rpy[1]) > 1.0)
            {
                std::ostringstream detail;
                detail << "Safety stop at t=" << mjDataPtr->time << ": finite=" << finite
                       << " qp_status=" << state.qp_status
                       << " qp_base_ddq_max=" << qpBaseAccelerationMax
                       << " base_z=" << state.basePos[2] << " rpy="
                       << state.rpy[0] << ' ' << state.rpy[1] << ' ' << state.rpy[2];
                if (isSingleStep(stage))
                    detail << " release_confirmed=" << stepReleaseConfirmed
                           << " release_time=" << stepReleaseConfirmedTime
                           << " airborne=" << stepAirborne << " touchdown=" << stepTouchdown
                           << " stable_duration=" << stepStableDuration
                           << " max_stance_slip=" << stepMaximumStanceSlip
                           << " max_roll=" << stepMaximumRoll << " max_pitch=" << stepMaximumPitch;
                if (safetyTrace && safetyTrace->dump())
                    detail << " safety_trace=" << safetyTrace->path();
                throw std::runtime_error(detail.str());
            }
        }

        const bool guiClosedEarly = uiController && glfwWindowShouldClose(uiController->window)
                                    && mjDataPtr->time < duration;
        if (guiClosedEarly)
            std::cout << "LYENBOT_GUI_CLOSED time=" << mjDataPtr->time << '\n';
        else if (isSingleStep(stage))
        {
            const bool passed = stepReleaseConfirmed && stepAirborne && stepTouchdown
                                && stepStableDuration >= 3.0
                                && stepMaximumStanceSlip <= 0.01
                                && stepMaximumRoll < 1.0 && stepMaximumPitch < 1.0;
            std::cout << "LYENBOT_SINGLE_STEP_SUMMARY swing="
                      << (stage == Stage::StepLeft ? "left" : "right")
                      << " release_confirmed=" << stepReleaseConfirmed
                      << " release_time=" << stepReleaseConfirmedTime
                      << " airborne=" << stepAirborne << " touchdown=" << stepTouchdown
                      << " stable_duration=" << stepStableDuration
                      << " max_stance_slip=" << stepMaximumStanceSlip
                      << " max_roll=" << stepMaximumRoll << " max_pitch=" << stepMaximumPitch << '\n';
            if (!passed)
                throw std::runtime_error("Quasi-static single-step acceptance failed");
            std::cout << "LYENBOT_SINGLE_STEP_OK\n";
        }
        else if (stage == Stage::Walk && stepTransitions < 10)
            throw std::runtime_error("Walk acceptance failed: only " + std::to_string(stepTransitions) + " support-leg transitions");
        else
            std::cout << "LYENBOT_STAGE_OK time=" << mjDataPtr->time << " base_z=" << state.basePos[2]
                      << " steps=" << stepTransitions << '\n';
        uiController.reset();
        mj_deleteData(mjDataPtr);
        mj_deleteModel(mjModelPtr);
        return 0;
    }
    catch (const std::exception &exception)
    {
        std::cerr << "LYENBOT_STAGE_FAILED: " << exception.what() << '\n';
        uiController.reset();
        if (mjDataPtr) mj_deleteData(mjDataPtr);
        if (mjModelPtr) mj_deleteModel(mjModelPtr);
        return 1;
    }
}
