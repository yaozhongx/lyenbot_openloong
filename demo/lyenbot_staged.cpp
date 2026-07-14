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
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace
{
enum class Stage { Pd, Wbc, Walk };

Stage parseStage(int argc, char **argv)
{
    const std::string value = argc > 1 ? argv[1] : "pd";
    if (value == "pd") return Stage::Pd;
    if (value == "wbc") return Stage::Wbc;
    if (value == "walk") return Stage::Walk;
    throw std::runtime_error("stage must be one of: pd, wbc, walk");
}

bool hasArgument(int argc, char **argv, const std::string &expected)
{
    for (int index = 2; index < argc; ++index)
        if (argv[index] == expected)
            return true;
    return false;
}

std::vector<double> tailToStd(const Eigen::VectorXd &value, int count)
{
    return eigen2std(value.tail(count));
}
}

int main(int argc, char **argv)
{
    mjModel *mjModelPtr = nullptr;
    mjData *mjDataPtr = nullptr;
    std::unique_ptr<UIctr> uiController;
    try
    {
        const Stage stage = parseStage(argc, argv);
        const bool enableMpc = hasArgument(argc, argv, "mpc");
        const bool enableGui = hasArgument(argc, argv, "--gui") || hasArgument(argc, argv, "gui");
        const auto config = loadRobotModelConfig("../common/robot_configs/lyenbot.json");
        char error[2048] = {};
        mjModelPtr = mj_loadXML(config.scenePath.c_str(), nullptr, error, sizeof(error));
        if (!mjModelPtr)
            throw std::runtime_error(std::string("Cannot load Lyenbot scene: ") + error);
        mjDataPtr = mj_makeData(mjModelPtr);
        if (!mjDataPtr)
            throw std::runtime_error("Cannot allocate MuJoCo data");
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
        WBC_priority wbc(kinDyn.model_nv, 18, 22, 0.6, mjModelPtr->opt.timestep, &config, &kinDyn.model_biped);
        MPC mpc(0.005);
        GaitScheduler gait(config.swingTime, mjModelPtr->opt.timestep);
        gait.useMeasuredContact = true;
        gait.FzThrehold = config.touchdownForce;
        gait.minimumTouchdownPhase = 0.85;
        gait.enableDoubleSupportTransfer = true;
        FootPlacement footPlacement;
        JoyStickInterpreter joystick(mjModelPtr->opt.timestep);

        const int motorCount = config.joints.actuated.size();
        state.motors_pos_des = config.initialJointPositions;
        state.motors_vel_des.assign(motorCount, 0.0);
        state.motors_tor_des.assign(motorCount, 0.0);
        state.width_hips = config.hipWidth;
        footPlacement.legLength = config.nominalBaseHeight - config.footHeight;
        footPlacement.stepHeight = 0.025;
        footPlacement.maxStepLength = 0.06;
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
        double nextRenderTime = mjDataPtr->time;
        DataBus::LegState previousLeg = DataBus::DSt;
        const double duration = stage == Stage::Pd ? 10.0 : (stage == Stage::Wbc ? 13.0 : 40.0);

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
            const bool prepareWalk = stage == Stage::Walk && mjDataPtr->time >= 10.0 && mjDataPtr->time < 12.0;
            const bool useWalk = stage == Stage::Walk && mjDataPtr->time >= 12.0;
            if (useWbc && !wbcInitialized)
            {
                wbcInitialized = true;
                wbc.pCoMDes = state.pCoM_W;
                state.base_pos_des << state.q(0), state.q(1), state.q(2);
                walkPreparationCom = state.pCoM_W;
            }
            if (prepareWalk)
            {
                const double blend = std::min(1.0, (mjDataPtr->time - 10.0) / 2.0);
                wbc.pCoMDes = walkPreparationCom;
                wbc.pCoMDes.y() = (1.0 - blend) * walkPreparationCom.y()
                                  + blend * (state.fe_l_pos_W.y() - 0.060);
            }
            if (useWalk)
            {
                if (state.motionState != DataBus::Walk)
                {
                    if (enableMpc)
                        mpc.enable();
                    joystick.setIniPos(state.q(0), state.q(1), config.nominalBaseHeight, state.rpy[2]);
                }
                state.motionState = DataBus::Walk;
                joystick.setVxDesLPara(config.walkingSpeed, 2.0);
                joystick.setWzDesLPara(0, 1.0);
                joystick.step();
                joystick.dataBusWrite(state);
                state.js_pos_des(2) = config.nominalBaseHeight;

                gait.dataBusRead(state);
                gait.step();
                gait.dataBusWrite(state);
                const double leftSupportTarget = state.fe_l_pos_W.y() - 0.060;
                const double rightSupportTarget = state.fe_r_pos_W.y() + 0.060;
                if (stepTransitions == 0)
                    state.base_pos_des.y() = leftSupportTarget;
                else if (state.legState == DataBus::LSt)
                    state.base_pos_des.y() = (1.0 - state.phi) * rightSupportTarget + state.phi * leftSupportTarget;
                else if (state.legState == DataBus::RSt)
                    state.base_pos_des.y() = (1.0 - state.phi) * leftSupportTarget + state.phi * rightSupportTarget;
                else if (state.legStateNext == DataBus::RSt)
                    state.base_pos_des.y() = (1.0 - gait.transferPhi) * leftSupportTarget
                                             + gait.transferPhi * rightSupportTarget;
                else if (state.legStateNext == DataBus::LSt)
                    state.base_pos_des.y() = (1.0 - gait.transferPhi) * rightSupportTarget
                                             + gait.transferPhi * leftSupportTarget;
                footPlacement.dataBusRead(state);
                footPlacement.getSwingPos();
                footPlacement.dataBusWrite(state);
                if (previousLeg != DataBus::DSt && state.legState != previousLeg)
                    ++stepTransitions;
                previousLeg = state.legState;

                if (++mpcCounter >= 5)
                {
                    mpc.dataBusRead(state);
                    mpc.cal();
                    mpc.dataBusWrite(state);
                    mpcCounter = 0;
                }
            }

            if (useWbc)
            {
                wbc.dataBusRead(state);
                wbc.computeDdq(kinDyn);
                wbc.computeTau();
                wbc.dataBusWrite(state);
                const auto desiredQ = kinDyn.integrateDIY(state.q, state.wbc_delta_q_final);
                state.motors_pos_des = tailToStd(desiredQ, motorCount);
                state.motors_vel_des = tailToStd(state.wbc_dq_final, motorCount);
                state.motors_tor_des = eigen2std(state.wbc_tauJointRes);
            }

            pvt.dataBusRead(state);
            if (mjDataPtr->time < 3.0)
                pvt.calMotorsPVT(0.002);
            else
                pvt.calMotorsPVT();
            pvt.dataBusWrite(state);
            mjInterface.setMotorsTorque(state.motors_tor_out);
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
            if (!finite || state.basePos[2] < 0.35 || std::abs(state.rpy[0]) > 1.0 || std::abs(state.rpy[1]) > 1.0)
            {
                std::ostringstream detail;
                detail << "Safety stop at t=" << mjDataPtr->time << ": finite=" << finite
                       << " base_z=" << state.basePos[2] << " rpy="
                       << state.rpy[0] << ' ' << state.rpy[1] << ' ' << state.rpy[2];
                throw std::runtime_error(detail.str());
            }
        }

        const bool guiClosedEarly = uiController && glfwWindowShouldClose(uiController->window)
                                    && mjDataPtr->time < duration;
        if (guiClosedEarly)
            std::cout << "LYENBOT_GUI_CLOSED time=" << mjDataPtr->time << '\n';
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
