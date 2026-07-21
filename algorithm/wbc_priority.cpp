/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024-2025 Humanoid Robot (Shanghai) Co., Ltd.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/

#include "wbc_priority.h"
#include "iostream"

Eigen::VectorXd WBC_priority::gather(const Eigen::VectorXd &source, const std::vector<int> &indices)
{
    Eigen::VectorXd result(indices.size());
    for (std::size_t i = 0; i < indices.size(); ++i)
        result[i] = source[indices[i]];
    return result;
}

void WBC_priority::zeroColumns(Eigen::MatrixXd &matrix, const std::vector<int> &indices)
{
    for (const int column : indices)
        matrix.col(column).setZero();
}

// QP_nvIn=18, QP_ncIn=22
WBC_priority::WBC_priority(int model_nv_In, int QP_nvIn, int QP_ncIn, double miu_In, double dt,
                           const RobotModelConfig *config, const pinocchio::Model *model) : QP_prob(QP_nvIn,
                                                                                                          QP_ncIn)
{
    timeStep = dt;
    model_nv = model_nv_In;
    miu = miu_In;
    QP_nc = QP_ncIn;
    QP_nv = QP_nvIn;
    Sf = Eigen::MatrixXd::Zero(6, model_nv);
    Sf.block<6, 6>(0, 0) = Eigen::MatrixXd::Identity(6, 6);
    St_qpV2 = Eigen::MatrixXd::Zero(model_nv, model_nv - 6); // 6 means the dims of floating base
    St_qpV2.block(6, 0, model_nv - 6, model_nv - 6) = Eigen::MatrixXd::Identity(model_nv - 6, model_nv - 6);

    St_qpV1 = Eigen::MatrixXd::Zero(model_nv, 6); // 6 means the dims of delta_b
    St_qpV1.block<6, 6>(0, 0) = Eigen::MatrixXd::Identity(6, 6);

    // defined in body frame
    f_z_low = 10;
    f_z_upp = 1400;

    tau_upp_stand_L << 15, 30, 40; // foot end contact torque limit for stand state, in body frame
    tau_low_stand_L << -15, -30, -40;

    tau_upp_walk_L << 15, 40, 40; // foot end contact torque limit for walk state, in body frame
    tau_low_walk_L << -15, -40, -40;

    qpOASES::Options options;
    options.setToMPC();
    // options.setToReliable();
    options.printLevel = qpOASES::PL_LOW;
    QP_prob.setOptions(options);

    eigen_xOpt = Eigen::VectorXd::Zero(QP_nv);
    eigen_ddq_Opt = Eigen::VectorXd::Zero(model_nv);
    eigen_fr_Opt = Eigen::VectorXd::Zero(12);
    eigen_tau_Opt = Eigen::VectorXd::Zero(model_nv - 6);

    delta_q_final_kin = Eigen::VectorXd::Zero(model_nv);
    dq_final_kin = Eigen::VectorXd::Zero(model_nv);
    ;
    ddq_final_kin = Eigen::VectorXd::Zero(model_nv);

    base_rpy_cur = Eigen::VectorXd::Zero(3);

    if (config && model)
    {
        useFullFootContact = true;
        contactHalfLength = config->contactHalfLength;
        contactHalfWidth = config->contactHalfWidth;
        useCoupledCopConstraints = contactHalfLength > 0.0 && contactHalfWidth > 0.0;
        if (useCoupledCopConstraints && QP_nc != 26)
            throw std::runtime_error("coupled CoP constraints require QP_nc=26");
        auto appendIndices = [&](const std::vector<std::string> &names, std::vector<int> &qIndices, std::vector<int> &vIndices)
        {
            for (const auto &name : names)
            {
                const auto jointId = model->getJointId(name);
                qIndices.push_back(model->idx_qs[jointId]);
                vIndices.push_back(model->idx_vs[jointId]);
            }
        };
        appendIndices(config->joints.leftArm, armQIndices, armVIndices);
        appendIndices(config->joints.rightArm, armQIndices, armVIndices);
        appendIndices(config->joints.waist, waistQIndices, waistVIndices);
        appendIndices(config->joints.head, headQIndices, headVIndices);
        for (const auto &name : config->joints.leftLeg)
            if (name.find("hip") != std::string::npos && name.find("pitch") != std::string::npos)
                leftHipPitchQ = model->idx_qs[model->getJointId(name)];
        for (const auto &name : config->joints.rightLeg)
            if (name.find("hip") != std::string::npos && name.find("pitch") != std::string::npos)
                rightHipPitchQ = model->idx_qs[model->getJointId(name)];
        targetArmQ = Eigen::VectorXd::Zero(armQIndices.size());
        for (std::size_t i = 0; i < armQIndices.size(); ++i)
        {
            for (std::size_t motor = 0; motor < config->joints.actuated.size(); ++motor)
                if (model->idx_qs[model->getJointId(config->joints.actuated[motor])] == armQIndices[i])
                    targetArmQ[i] = config->initialJointPositions[motor];
        }
    }
    else
    {
        for (int i = 0; i < 14; ++i)
        {
            armQIndices.push_back(7 + i);
            armVIndices.push_back(6 + i);
        }
        for (int i = 0; i < 3; ++i)
        {
            waistQIndices.push_back(23 + i);
            waistVIndices.push_back(22 + i);
        }
        headQIndices = {21, 22};
        headVIndices = {20, 21};
        leftHipPitchQ = 28;
        rightHipPitchQ = 34;
        targetArmQ.resize(14);
        targetArmQ << 0.475, -1.12, 1.9, 0.86, -0.356, 0, 0, -0.475, -1.12, -1.9, 0.86, 0.356, 0, 0;
    }

    //  WBC task defined and order build
    ///------------ walk --------------
    kin_tasks_walk.addTask("static_Contact");
    kin_tasks_walk.addTask("Roll_Pitch_Yaw_Pz");
    kin_tasks_walk.addTask("RedundantJoints");
    kin_tasks_walk.addTask("PxPy");
    kin_tasks_walk.addTask("SwingLeg");
    kin_tasks_walk.addTask("HandTrackJoints");
    kin_tasks_walk.addTask("PosRot");

    std::vector<std::string> taskOrder_walk;

    taskOrder_walk.emplace_back("static_Contact");
    taskOrder_walk.emplace_back("PosRot");
    taskOrder_walk.emplace_back("SwingLeg");
    taskOrder_walk.emplace_back("RedundantJoints");
    taskOrder_walk.emplace_back("HandTrackJoints");

    kin_tasks_walk.buildPriority(taskOrder_walk);

    ///-------- stand ------------
    kin_tasks_stand.addTask("static_Contact");
    kin_tasks_stand.addTask("CoMTrack");
    kin_tasks_stand.addTask("HandTrackJoints");
    kin_tasks_stand.addTask("HipRPY");
    kin_tasks_stand.addTask("HeadRP");
    kin_tasks_stand.addTask("Pz");
    kin_tasks_stand.addTask("CoMXY_HipRPY");
    kin_tasks_stand.addTask("Roll_Pitch_Yaw");
    kin_tasks_stand.addTask("fixedWaist");

    std::vector<std::string> taskOrder_stand;

    taskOrder_stand.emplace_back("static_Contact");
    taskOrder_stand.emplace_back("CoMXY_HipRPY");
    taskOrder_stand.emplace_back("Pz");
    taskOrder_stand.emplace_back("HandTrackJoints");
    if (!headQIndices.empty())
        taskOrder_stand.emplace_back("HeadRP");

    kin_tasks_stand.buildPriority(taskOrder_stand);
}

void WBC_priority::dataBusRead(const DataBus &robotState)
{
    // foot-end offset posture
    fe_L_rot_L_off = robotState.fe_L_rot_L_off;
    fe_R_rot_L_off = robotState.fe_R_rot_L_off;

    // deisred values
    base_rpy_des = robotState.base_rpy_des;
    base_rpy_cur << robotState.rpy[0], robotState.rpy[1], robotState.rpy[2];
    base_pos_des = robotState.base_pos_des;
    swing_fe_pos_des_W = robotState.swing_fe_pos_des_W;
    swing_fe_rpy_des_W = robotState.swing_fe_rpy_des_W;
    stance_fe_pos_cur_W = robotState.stance_fe_pos_cur_W;
    stance_fe_rot_cur_W = robotState.stance_fe_rot_cur_W;
    stanceDesPos_W = robotState.stanceDesPos_W;
    hd_l_pos_cur_W = robotState.hd_l_pos_W;
    hd_r_pos_cur_W = robotState.hd_r_pos_W;
    hd_l_rot_cur_W = robotState.hd_l_rot_W;
    hd_r_rot_cur_W = robotState.hd_r_rot_W;
    fe_l_pos_cur_W = robotState.fe_l_pos_W;
    fe_r_pos_cur_W = robotState.fe_r_pos_W;
    fe_l_rot_cur_W = robotState.fe_l_rot_W;
    fe_r_rot_cur_W = robotState.fe_r_rot_W;
    des_ddq = robotState.des_ddq;
    des_dq = robotState.des_dq;
    des_delta_q = robotState.des_delta_q;
    des_q = robotState.des_q;

    // state update
    J_base = robotState.J_base;
    dJ_base = robotState.dJ_base;
    base_rot = robotState.base_rot;
    base_pos = robotState.base_pos;
    hip_link_pos = robotState.hip_link_pos;
    hip_link_rot = robotState.hip_link_rot;
    J_hip_link = robotState.J_hip_link;

    Jfe = Eigen::MatrixXd::Zero(12, model_nv);
    Jfe.block(0, 0, 6, model_nv) = robotState.J_l;
    Jfe.block(6, 0, 6, model_nv) = robotState.J_r;
    dJfe = Eigen::MatrixXd::Zero(12, model_nv);
    dJfe.block(0, 0, 6, model_nv) = robotState.dJ_l;
    dJfe.block(6, 0, 6, model_nv) = robotState.dJ_r;
    J_hd_l = robotState.J_hd_l;
    J_hd_r = robotState.J_hd_r;
    dJ_hd_l = robotState.J_hd_l;
    dJ_hd_r = robotState.J_hd_r;
    Fr_ff = robotState.Fr_ff;
    swingFootFzMax = robotState.wbc_swing_foot_fz_max;
    dyn_M = robotState.dyn_M;
    dyn_M_inv = robotState.dyn_M_inv;
    dyn_Ag = robotState.dyn_Ag;
    dyn_dAg = robotState.dyn_dAg;
    dyn_Non = robotState.dyn_Non;
    dq = robotState.dq;
    q = robotState.q;
    legStateCur = robotState.legState;
    legStateNextCur = robotState.legStateNext;
    motionStateCur = robotState.motionState;
    contactReleaseCur = robotState.wbc_contact_release;

    // LYENBOT MODIFY: DSt contact release uses legStateNext as the retained stance foot.
    const DataBus::LegState contactLeg = contactReleaseCur && legStateCur == DataBus::DSt
                                             ? legStateNextCur : legStateCur;
    if (contactLeg == DataBus::LSt)
    {
        Jc = robotState.J_l;
        dJc = robotState.dJ_l;
        Jsw = robotState.J_r;
        dJsw = robotState.dJ_r;
        fe_pos_sw_W = robotState.fe_r_pos_W;
        fe_rot_sw_W = robotState.fe_r_rot_W;
    }
    else
    {
        Jc = robotState.J_r;
        dJc = robotState.dJ_r;
        Jsw = robotState.J_l;
        dJsw = robotState.dJ_l;
        fe_pos_sw_W = robotState.fe_l_pos_W;
        fe_rot_sw_W = robotState.fe_l_rot_W;
    }

    Jcom = robotState.Jcom_W;
    pCoMCur = robotState.pCoM_W;
}

void WBC_priority::dataBusWrite(DataBus &robotState)
{
    robotState.wbc_tauJointRes = tauJointRes;
    robotState.wbc_FrRes = eigen_fr_Opt;
    robotState.qp_cpuTime = cpu_time;
    robotState.qp_nWSR = nWSR;
    robotState.qp_status = qpStatus;

    robotState.wbc_delta_q_final = delta_q_final_kin;
    robotState.wbc_dq_final = dq_final_kin;
    robotState.wbc_ddq_final = ddq_final_kin;
    robotState.wbc_ddq_qp = eigen_ddq_Opt;

    robotState.qp_status = qpStatus;
    robotState.qp_nWSR = nWSR;
    robotState.qp_cpuTime = cpu_time;
    robotState.wbc_qp_equality_residual_inf = qpEqualityResidualInf;
    robotState.wbc_qp_inequality_violation_max = qpInequalityViolationMax;
}

// QP problem contains joint torque, QP_nv=6+12, QP_nc=22;
void WBC_priority::computeTau()
{
    // constust the QP problem, refer to the md file for more details
    Eigen::MatrixXd eigen_qp_A1 = Eigen::MatrixXd::Zero(6, QP_nv); // 18 means the sum of dims of delta_r and delta_Fr
    eigen_qp_A1.block<6, 6>(0, 0) = Sf * dyn_M * St_qpV1;

    eigen_qp_A1.block<6, 12>(0, 6) = -Sf * Jfe.transpose();

    Eigen::VectorXd eqRes = Eigen::VectorXd::Zero(6);
    eqRes = -Sf * dyn_M * ddq_final_kin - Sf * dyn_Non + Sf * Jfe.transpose() * Fr_ff;

    Eigen::Matrix3d Rfe;
    if (motionStateCur == DataBus::Stand)
    {
        Rfe = fe_l_rot_cur_W;
    }
    else
    {
        Rfe = stance_fe_rot_cur_W;
    }

    Eigen::Matrix<double, 12, 12> Mw2b;
    Mw2b.setZero();
    Mw2b.block(0, 0, 3, 3) = Rfe.transpose();
    Mw2b.block(3, 3, 3, 3) = Rfe.transpose();
    Mw2b.block(6, 6, 3, 3) = Rfe.transpose();
    Mw2b.block(9, 9, 3, 3) = Rfe.transpose();

    const int constraintsPerFoot = useCoupledCopConstraints ? 10 : 8;
    const int inequalityCount = 2 * constraintsPerFoot;
    Eigen::MatrixXd W = Eigen::MatrixXd::Zero(inequalityCount, 12);
    Eigen::VectorXd f_low = Eigen::VectorXd::Zero(inequalityCount);
    Eigen::VectorXd f_upp = Eigen::VectorXd::Zero(inequalityCount);
    Eigen::Vector3d tau_upp_fe, tau_low_fe;
    if (motionStateCur == DataBus::Stand)
    {
        tau_upp_fe = tau_upp_stand_L;
        tau_low_fe = tau_low_stand_L;
    }
    else
    {
        tau_upp_fe = tau_upp_walk_L;
        tau_low_fe = tau_low_walk_L;
    }
    double leftNormalForceLow = f_z_low;
    double rightNormalForceLow = f_z_low;
    double leftNormalForceUpper = f_z_upp;
    double rightNormalForceUpper = f_z_upp;
    if (useCoupledCopConstraints && motionStateCur == DataBus::Walk && legStateCur == DataBus::DSt)
    {
        if (legStateNextCur == DataBus::LSt)
        {
            rightNormalForceLow = 0.0;
            rightNormalForceUpper = std::min(f_z_upp, swingFootFzMax);
        }
        else if (legStateNextCur == DataBus::RSt)
        {
            leftNormalForceLow = 0.0;
            leftNormalForceUpper = std::min(f_z_upp, swingFootFzMax);
        }
    }
    auto buildFootConstraints = [&](int row, int column, double normalForceLow, double normalForceUpper)
    {
        W(row, column) = 1;
        W(row, column + 2) = sqrt(2) / 2.0 * miu;
        W(row + 1, column) = -1;
        W(row + 1, column + 2) = sqrt(2) / 2.0 * miu;
        W(row + 2, column + 1) = 1;
        W(row + 2, column + 2) = sqrt(2) / 2.0 * miu;
        W(row + 3, column + 1) = -1;
        W(row + 3, column + 2) = sqrt(2) / 2.0 * miu;
        if (useCoupledCopConstraints)
        {
            W(row + 4, column + 2) = 1;
            W(row + 5, column + 2) = contactHalfWidth;
            W(row + 5, column + 3) = 1;
            W(row + 6, column + 2) = contactHalfWidth;
            W(row + 6, column + 3) = -1;
            W(row + 7, column + 2) = contactHalfLength;
            W(row + 7, column + 4) = 1;
            W(row + 8, column + 2) = contactHalfLength;
            W(row + 8, column + 4) = -1;
            W(row + 9, column + 5) = 1;
            f_low.segment<10>(row) << 0, 0, 0, 0, normalForceLow, 0, 0, 0, 0, tau_low_fe(2);
            f_upp.segment<10>(row) << 1e10, 1e10, 1e10, 1e10, normalForceUpper,
                1e10, 1e10, 1e10, 1e10, tau_upp_fe(2);
        }
        else
        {
            W.block<4, 4>(row + 4, column + 2) = Eigen::MatrixXd::Identity(4, 4);
            f_low.segment<8>(row) << 0, 0, 0, 0, normalForceLow,
                tau_low_fe(0), tau_low_fe(1), tau_low_fe(2);
            f_upp.segment<8>(row) << 1e10, 1e10, 1e10, 1e10, normalForceUpper,
                tau_upp_fe(0), tau_upp_fe(1), tau_upp_fe(2);
        }
    };
    buildFootConstraints(0, 0, leftNormalForceLow, leftNormalForceUpper);
    buildFootConstraints(constraintsPerFoot, 6, rightNormalForceLow, rightNormalForceUpper);
    W = W * Mw2b;

    if (motionStateCur == DataBus::Walk || motionStateCur == DataBus::Walk2Stand)
    {
        if (legStateCur == DataBus::LSt)
        {
            f_low.segment(constraintsPerFoot, constraintsPerFoot).setZero();
            f_upp.segment(constraintsPerFoot, constraintsPerFoot).setZero();
        }
        else if (legStateCur == DataBus::RSt)
        {
            f_low.head(constraintsPerFoot).setZero();
            f_upp.head(constraintsPerFoot).setZero();
        }
    }

    Eigen::MatrixXd eigen_qp_A2 = Eigen::MatrixXd::Zero(inequalityCount, QP_nv);
    eigen_qp_A2.block(0, 6, inequalityCount, 12) = W;
    Eigen::VectorXd neqRes_low = Eigen::VectorXd::Zero(inequalityCount);
    Eigen::VectorXd neqRes_upp = Eigen::VectorXd::Zero(inequalityCount);

    neqRes_low = f_low - W * Fr_ff;
    neqRes_upp = f_upp - W * Fr_ff;

    Eigen::MatrixXd eigen_qp_A_final = Eigen::MatrixXd::Zero(QP_nc, QP_nv);
    eigen_qp_A_final.block<6, 18>(0, 0) = eigen_qp_A1;
    eigen_qp_A_final.block(6, 0, inequalityCount, QP_nv) = eigen_qp_A2;

    Eigen::VectorXd eigen_qp_lbA = Eigen::VectorXd::Zero(QP_nc);
    Eigen::VectorXd eigen_qp_ubA = Eigen::VectorXd::Zero(QP_nc);

    eigen_qp_lbA.block<6, 1>(0, 0) = eqRes;
    eigen_qp_lbA.segment(6, inequalityCount) = neqRes_low;
    eigen_qp_ubA.block<6, 1>(0, 0) = eqRes;
    eigen_qp_ubA.segment(6, inequalityCount) = neqRes_upp;

    Eigen::MatrixXd eigen_qp_H = Eigen::MatrixXd::Zero(QP_nv, QP_nv);
    Q2 = Eigen::MatrixXd::Identity(6, 6);
    Q1 = Eigen::MatrixXd::Identity(12, 12);
	if (motionStateCur == DataBus::Stand){
		eigen_qp_H.block<6, 6>(0, 0) = Q2 * 2.0 * 1e7;
		eigen_qp_H.block<12, 12>(6, 6) = Q1 * 2.0 * 1e1;
		eigen_qp_H(9,9) *= 100;
		eigen_qp_H(10,10) *= 100;
		eigen_qp_H(15,15) *= 100;
		eigen_qp_H(16,16) *= 100;
	}
	else{
		eigen_qp_H.block<6, 6>(0, 0) = Q2 * 2.0 * 1e7;
		eigen_qp_H.block<12, 12>(6, 6) = Q1 * 2.0 * 1e1;
	}

    // obj: (1/2)x'Hx+x'g
    // s.t. lbA<=Ax<=ubA
    //       lb<=x<=ub
    //    qpOASES::real_t qp_H[QP_nv*QP_nv];
    //    qpOASES::real_t qp_A[QP_nc*QP_nv];
    //    qpOASES::real_t qp_g[QP_nv];
    //    qpOASES::real_t qp_lbA[QP_nc];
    //    qpOASES::real_t qp_ubA[QP_nc];
    //    qpOASES::real_t xOpt_iniGuess[QP_nv];

    copy_Eigen_to_real_t(qp_H, eigen_qp_H, eigen_qp_H.rows(), eigen_qp_H.cols());
    copy_Eigen_to_real_t(qp_A, eigen_qp_A_final, eigen_qp_A_final.rows(), eigen_qp_A_final.cols());
    copy_Eigen_to_real_t(qp_lbA, eigen_qp_lbA, eigen_qp_lbA.rows(), eigen_qp_lbA.cols());
    copy_Eigen_to_real_t(qp_ubA, eigen_qp_ubA, eigen_qp_ubA.rows(), eigen_qp_ubA.cols());

    qpOASES::returnValue res;
    for (int i = 0; i < QP_nv; i++)
    {
        xOpt_iniGuess[i] = useFullFootContact ? eigen_xOpt(i) : 0;
        qp_g[i] = 0;
    }
    nWSR = useFullFootContact ? 500 : 200;
    cpu_time = useFullFootContact ? 0.01 : timeStep;
    if (useFullFootContact)
        QP_prob.reset();
    res = QP_prob.init(qp_H, qp_g, qp_A, NULL, NULL, qp_lbA, qp_ubA, nWSR, &cpu_time, xOpt_iniGuess);
    qpStatus = qpOASES::getSimpleStatus(res);
    //    if (res==qpOASES::SUCCESSFUL_RETURN)
    //        printf("WBC-QP: successful_return\n");
    //    else if (res==qpOASES::RET_MAX_NWSR_REACHED)
    //        printf("WBC-QP: max_nwsr\n");
    //    else if (res==qpOASES::RET_INIT_FAILED)
    //        printf("WBC-QP: init_failed\n");

    qpOASES::real_t xOpt[QP_nv];
    QP_prob.getPrimalSolution(xOpt);
    if (res == qpOASES::SUCCESSFUL_RETURN)
        for (int i = 0; i < QP_nv; i++)
            eigen_xOpt(i) = xOpt[i];

    const Eigen::VectorXd qpValue = eigen_qp_A_final * eigen_xOpt;
    qpEqualityResidualInf = (qpValue.head(6) - eqRes).lpNorm<Eigen::Infinity>();
    const double lowerViolation = (eigen_qp_lbA.tail(inequalityCount) - qpValue.tail(inequalityCount)).maxCoeff();
    const double upperViolation = (qpValue.tail(inequalityCount) - eigen_qp_ubA.tail(inequalityCount)).maxCoeff();
    qpInequalityViolationMax = std::max(0.0, std::max(lowerViolation, upperViolation));

    eigen_ddq_Opt = ddq_final_kin;
    eigen_ddq_Opt.block<6, 1>(0, 0) += eigen_xOpt.block<6, 1>(0, 0);
    eigen_fr_Opt = Fr_ff + eigen_xOpt.block<12, 1>(6, 0);

    if (qpStatus != 0)
    {
        Eigen::MatrixXd A_x;
        Eigen::VectorXd xOpt_iniGuess_m(QP_nv, 1);
        for (int i = 0; i < QP_nv; i++)
            xOpt_iniGuess_m(i) = xOpt_iniGuess[i];
    }

    Eigen::VectorXd tauRes;
    tauRes = dyn_M * eigen_ddq_Opt + dyn_Non - Jfe.transpose() * eigen_fr_Opt;

    tauJointRes = tauRes.block(6, 0, model_nv - 6, 1);

    last_nWSR = nWSR;
    last_cpu_time = cpu_time;
}

void WBC_priority::computeDdq(Pin_KinDyn &pinKinDynIn)
{
    // task definition
    /// -------- walk -------------
    {
        int id = kin_tasks_walk.getId("static_Contact");
        // LYENBOT MODIFY: during contact release the QP still carries both foot
        // wrenches, while the kinematic contact task retains only the stance foot.
        const bool doubleSupportWalk = useFullFootContact && legStateCur == DataBus::DSt
                                       && !contactReleaseCur;
        const DataBus::LegState contactLeg = contactReleaseCur && legStateCur == DataBus::DSt
                                                 ? legStateNextCur : legStateCur;
        const int contactSize = doubleSupportWalk ? 12 : 6;
        kin_tasks_walk.taskLib[id].errX = Eigen::VectorXd::Zero(contactSize);
        if (useFullFootContact && doubleSupportWalk)
        {
            if (!walkDoubleSupportInitialized || walkContactLeg != DataBus::DSt)
            {
                walkLeftFootPosition = fe_l_pos_cur_W;
                walkRightFootPosition = fe_r_pos_cur_W;
                walkLeftFootRotation = fe_l_rot_cur_W;
                walkRightFootRotation = fe_r_rot_cur_W;
            }
            walkDoubleSupportInitialized = true;
            kin_tasks_walk.taskLib[id].errX.segment<3>(0) = walkLeftFootPosition - fe_l_pos_cur_W;
            kin_tasks_walk.taskLib[id].errX.segment<3>(3) = diffRot(fe_l_rot_cur_W, walkLeftFootRotation);
            kin_tasks_walk.taskLib[id].errX.segment<3>(6) = walkRightFootPosition - fe_r_pos_cur_W;
            kin_tasks_walk.taskLib[id].errX.segment<3>(9) = diffRot(fe_r_rot_cur_W, walkRightFootRotation);
        }
        else if (useFullFootContact)
        {
            if (walkDoubleSupportInitialized || walkContactLeg != contactLeg)
            {
                walkStancePosition = contactLeg == DataBus::LSt ? fe_l_pos_cur_W : fe_r_pos_cur_W;
                walkStanceRotation = contactLeg == DataBus::LSt ? fe_l_rot_cur_W : fe_r_rot_cur_W;
            }
            walkDoubleSupportInitialized = false;
            const auto &currentPosition = contactLeg == DataBus::LSt ? fe_l_pos_cur_W : fe_r_pos_cur_W;
            const auto &currentRotation = contactLeg == DataBus::LSt ? fe_l_rot_cur_W : fe_r_rot_cur_W;
            kin_tasks_walk.taskLib[id].errX.segment<3>(0) = walkStancePosition - currentPosition;
            kin_tasks_walk.taskLib[id].errX.segment<3>(3) = diffRot(currentRotation, walkStanceRotation);
        }
        if (useFullFootContact)
            walkContactLeg = doubleSupportWalk ? DataBus::DSt : contactLeg;
        kin_tasks_walk.taskLib[id].derrX = Eigen::VectorXd::Zero(contactSize);
        if (useFullFootContact)
            kin_tasks_walk.taskLib[id].derrX = -(doubleSupportWalk ? Jfe : Jc) * dq;
        kin_tasks_walk.taskLib[id].ddxDes = Eigen::VectorXd::Zero(contactSize);
        kin_tasks_walk.taskLib[id].dxDes = Eigen::VectorXd::Zero(contactSize);
        kin_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Identity(contactSize, contactSize) * (useFullFootContact ? 100 : 0);
        kin_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Identity(contactSize, contactSize) * (useFullFootContact ? 20 : 0);
        kin_tasks_walk.taskLib[id].J = doubleSupportWalk ? Jfe : Jc;
        kin_tasks_walk.taskLib[id].dJ = doubleSupportWalk ? dJfe : dJc;
        kin_tasks_walk.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_walk.getId("RedundantJoints");
        std::vector<int> redundantQ = headQIndices;
        redundantQ.insert(redundantQ.end(), waistQIndices.begin(), waistQIndices.end());
        std::vector<int> redundantV = headVIndices;
        redundantV.insert(redundantV.end(), waistVIndices.begin(), waistVIndices.end());
        const int redundantSize = redundantQ.size();
        kin_tasks_walk.taskLib[id].errX = -gather(q, redundantQ);
        kin_tasks_walk.taskLib[id].derrX = Eigen::VectorXd::Zero(redundantSize);
        kin_tasks_walk.taskLib[id].ddxDes = Eigen::VectorXd::Zero(redundantSize);
        kin_tasks_walk.taskLib[id].dxDes = Eigen::VectorXd::Zero(redundantSize);
        kin_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Identity(redundantSize, redundantSize) * 100;
        kin_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Identity(redundantSize, redundantSize) * 20;
        kin_tasks_walk.taskLib[id].J = Eigen::MatrixXd::Zero(redundantSize, model_nv);
        for (int row = 0; row < redundantSize; ++row)
            kin_tasks_walk.taskLib[id].J(row, redundantV[row]) = 1;
        kin_tasks_walk.taskLib[id].dJ = Eigen::MatrixXd::Zero(redundantSize, model_nv);
        kin_tasks_walk.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_walk.getId("Roll_Pitch_Yaw_Pz");
        kin_tasks_walk.taskLib[id].errX = Eigen::VectorXd::Zero(4);
        Eigen::Matrix3d desRot = eul2Rot(base_rpy_des(0), base_rpy_des(1), base_rpy_des(2));
        kin_tasks_walk.taskLib[id].errX.block<3, 1>(0, 0) = diffRot(base_rot, desRot);
        kin_tasks_walk.taskLib[id].errX(3) = base_pos_des(2) - q(2);
        kin_tasks_walk.taskLib[id].derrX = Eigen::VectorXd::Zero(4);
        kin_tasks_walk.taskLib[id].derrX.block<3, 1>(0, 0) = -dq.block<3, 1>(3, 0);
        kin_tasks_walk.taskLib[id].derrX(3) = 0 - dq(2);
        kin_tasks_walk.taskLib[id].ddxDes = Eigen::VectorXd::Zero(4);
        kin_tasks_walk.taskLib[id].dxDes = Eigen::VectorXd::Zero(4);
        kin_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Identity(4, 4) * 100;
        kin_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Identity(4, 4) * 10;
        Eigen::MatrixXd taskMap = Eigen::MatrixXd::Zero(4, 6);
        taskMap(0, 3) = 1;
        taskMap(1, 4) = 1;
        taskMap(2, 5) = 1;
        taskMap(3, 2) = 1;
        kin_tasks_walk.taskLib[id].J = taskMap * J_base;
        kin_tasks_walk.taskLib[id].dJ = taskMap * dJ_base;
        kin_tasks_walk.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_walk.getId("PxPy");
        kin_tasks_walk.taskLib[id].errX = Eigen::VectorXd::Zero(2);
        kin_tasks_walk.taskLib[id].errX = des_dq.block(0, 0, 2, 1) * timeStep;
        kin_tasks_walk.taskLib[id].derrX = Eigen::VectorXd::Zero(2);
        kin_tasks_walk.taskLib[id].ddxDes = Eigen::VectorXd::Zero(2);
        kin_tasks_walk.taskLib[id].dxDes = Eigen::VectorXd::Zero(2);
        kin_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Identity(2, 2) * 100; // 100
        kin_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Identity(2, 2) * 50;
        taskMap = Eigen::MatrixXd::Zero(2, 6);
        taskMap(0, 0) = 1;
        taskMap(1, 1) = 1;
        kin_tasks_walk.taskLib[id].J = taskMap * J_base;
        kin_tasks_walk.taskLib[id].dJ = taskMap * dJ_base;
        kin_tasks_walk.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_walk.getId("PosRot");
        kin_tasks_walk.taskLib[id].errX = Eigen::VectorXd::Zero(6);
        kin_tasks_walk.taskLib[id].errX.block(0, 0, 3, 1) = base_pos_des - q.block(0, 0, 3, 1);
        if (fabs(kin_tasks_walk.taskLib[id].errX(0)) >= 0.02)
            kin_tasks_walk.taskLib[id].errX(0) = 0.02 * sign(kin_tasks_walk.taskLib[id].errX(0));
        if (fabs(kin_tasks_walk.taskLib[id].errX(1)) >= 0.02)
            kin_tasks_walk.taskLib[id].errX(1) = 0.02 * sign(kin_tasks_walk.taskLib[id].errX(1));
        if (kin_tasks_walk.taskLib[id].errX(2)>0.005){
            kin_tasks_walk.taskLib[id].errX(2) = 0.005;
        }
        desRot = eul2Rot(base_rpy_des(0), base_rpy_des(1), base_rpy_des(2));
        kin_tasks_walk.taskLib[id].errX.block<3, 1>(3, 0) = diffRot(base_rot, desRot);
        kin_tasks_walk.taskLib[id].errX(4) -= 0.05 * dq(4);
        kin_tasks_walk.taskLib[id].derrX = Eigen::VectorXd::Zero(6);
        // kin_tasks_walk.taskLib[id].derrX = des_dq.block(0, 0, 6, 1) - dq.block(0, 0, 6, 1);
        kin_tasks_walk.taskLib[id].ddxDes = Eigen::VectorXd::Zero(6);
        kin_tasks_walk.taskLib[id].dxDes = Eigen::VectorXd::Zero(6);
        kin_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Identity(6, 6) * (useFullFootContact ? 100 : 500);
        kin_tasks_walk.taskLib[id].kp(2, 2) = useFullFootContact ? 300 : 500;
        kin_tasks_walk.taskLib[id].kp.block(3, 3, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * (useFullFootContact ? 300 : 500);
        kin_tasks_walk.taskLib[id].kp(0,0) = 100;
        kin_tasks_walk.taskLib[id].kp(4,4) = useFullFootContact ? 300 : 800;
        // kin_tasks_walk.taskLib[id].kp(3,3) = 800;
        kin_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Identity(6, 6) * (useFullFootContact ? 50 : 10);
        kin_tasks_walk.taskLib[id].kd(4,4) = useFullFootContact ? 50 : 10;
        kin_tasks_walk.taskLib[id].J = J_base;
        kin_tasks_walk.taskLib[id].dJ = dJ_base;
        kin_tasks_walk.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_walk.getId("SwingLeg");
        kin_tasks_walk.taskLib[id].errX = Eigen::VectorXd::Zero(6);
        kin_tasks_walk.taskLib[id].errX.block<3, 1>(0, 0) = swing_fe_pos_des_W - fe_pos_sw_W;
        desRot = eul2Rot(swing_fe_rpy_des_W(0), swing_fe_rpy_des_W(1), swing_fe_rpy_des_W(2));
        kin_tasks_walk.taskLib[id].errX.block<3, 1>(3, 0) = diffRot(fe_rot_sw_W, desRot);      
        kin_tasks_walk.taskLib[id].errX(4) *= 2;
        kin_tasks_walk.taskLib[id].derrX = Eigen::VectorXd::Zero(6);
        //        kin_tasks_walk.taskLib[id].derrX=-Jsw*dq;
        kin_tasks_walk.taskLib[id].ddxDes = Eigen::VectorXd::Zero(6);
        kin_tasks_walk.taskLib[id].dxDes = Eigen::VectorXd::Zero(6);
        kin_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Identity(6, 6) * (useFullFootContact ? 80 : 500);
        kin_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Identity(6, 6) * (useFullFootContact ? 25 : 20);
        kin_tasks_walk.taskLib[id].J = Jsw;
        if (useFullFootContact && doubleSupportWalk)
        {
            kin_tasks_walk.taskLib[id].errX.setZero();
            kin_tasks_walk.taskLib[id].J.setZero();
        }
        zeroColumns(kin_tasks_walk.taskLib[id].J, waistVIndices);
        kin_tasks_walk.taskLib[id].dJ = dJsw;
        zeroColumns(kin_tasks_walk.taskLib[id].dJ, waistVIndices);
        kin_tasks_walk.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        // task 6: hand track
        Eigen::VectorXd target_arm_q = targetArmQ;
        if (target_arm_q.size() == 14 && leftHipPitchQ >= 0 && rightHipPitchQ >= 0)
        {
            const double l_hip_pitch = q(leftHipPitchQ) - q(rightHipPitchQ);
            const double r_hip_pitch = -l_hip_pitch;
            target_arm_q(0) -= 0.75 * r_hip_pitch;
            target_arm_q(7) += 0.75 * l_hip_pitch;
        }

        id = kin_tasks_walk.getId("HandTrackJoints");
        const int armSize = armQIndices.size();
        kin_tasks_walk.taskLib[id].errX = target_arm_q - gather(q, armQIndices);
        kin_tasks_walk.taskLib[id].derrX = Eigen::VectorXd::Zero(armSize);
        kin_tasks_walk.taskLib[id].ddxDes = Eigen::VectorXd::Zero(armSize);
        kin_tasks_walk.taskLib[id].dxDes = Eigen::VectorXd::Zero(armSize);
        kin_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Identity(armSize, armSize) * 200;
        kin_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Identity(armSize, armSize) * 10;
        kin_tasks_walk.taskLib[id].J = Eigen::MatrixXd::Zero(armSize, model_nv);
        for (int row = 0; row < armSize; ++row)
            kin_tasks_walk.taskLib[id].J(row, armVIndices[row]) = 1;
        kin_tasks_walk.taskLib[id].dJ = Eigen::MatrixXd::Zero(armSize, model_nv);
        kin_tasks_walk.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);
    }

    /// -------- stand -------------
    {
        int id = kin_tasks_stand.getId("static_Contact");
        kin_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(12);
        if (useFullFootContact)
        {
            if (!standContactInitialized)
            {
                standContactInitialized = true;
                standLeftFootPosition = fe_l_pos_cur_W;
                standRightFootPosition = fe_r_pos_cur_W;
                standLeftFootRotation = fe_l_rot_cur_W;
                standRightFootRotation = fe_r_rot_cur_W;
            }
            kin_tasks_stand.taskLib[id].errX.segment<3>(0) = standLeftFootPosition - fe_l_pos_cur_W;
            kin_tasks_stand.taskLib[id].errX.segment<3>(3) = diffRot(fe_l_rot_cur_W, standLeftFootRotation);
            kin_tasks_stand.taskLib[id].errX.segment<3>(6) = standRightFootPosition - fe_r_pos_cur_W;
            kin_tasks_stand.taskLib[id].errX.segment<3>(9) = diffRot(fe_r_rot_cur_W, standRightFootRotation);
        }
        kin_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(12);
        if (useFullFootContact)
            kin_tasks_stand.taskLib[id].derrX = -Jfe * dq;
        kin_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(12);
        kin_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(12);
        kin_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(12, 12) * (useFullFootContact ? 100 : 0);
        kin_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(12, 12) * (useFullFootContact ? 20 : 0);
        kin_tasks_stand.taskLib[id].J = Eigen::MatrixXd::Zero(12, model_nv);
        Eigen::MatrixXd taskCtMap = Eigen::MatrixXd::Zero(3, 3);
        taskCtMap(0, 0) = 0;
        taskCtMap(1, 1) = 1;
        taskCtMap(2, 2) = 1;
        taskCtMap = fe_l_rot_cur_W * taskCtMap * fe_l_rot_cur_W.transpose(); // disable ankle roll joint
        kin_tasks_stand.taskLib[id].J = Jfe;
        if (!useFullFootContact)
        {
            kin_tasks_stand.taskLib[id].J.block(3, 0, 3, model_nv) = taskCtMap * kin_tasks_stand.taskLib[id].J.block(3, 0, 3, model_nv);
            kin_tasks_stand.taskLib[id].J.block(9, 0, 3, model_nv) = taskCtMap * kin_tasks_stand.taskLib[id].J.block(9, 0, 3, model_nv);
        }
        zeroColumns(kin_tasks_stand.taskLib[id].J, waistVIndices);
        kin_tasks_stand.taskLib[id].dJ = useFullFootContact ? dJfe : Eigen::MatrixXd::Zero(12, model_nv);
        zeroColumns(kin_tasks_stand.taskLib[id].dJ, waistVIndices);
        kin_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_stand.getId("HipRPY");
        kin_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(3);
        Eigen::Matrix3d desRot = eul2Rot(0, 0, 0);
        kin_tasks_stand.taskLib[id].errX.block<3, 1>(0, 0) = diffRot(hip_link_rot, desRot);
        kin_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(3);
        kin_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(3);
        kin_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(3);
        kin_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(3, 3) * 1000;
        kin_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(3, 3) * 50;
        Eigen::MatrixXd taskMapRPY = Eigen::MatrixXd::Zero(3, 6);
        taskMapRPY(0, 3) = 1;
        taskMapRPY(1, 4) = 1;
        taskMapRPY(2, 5) = 1;
        kin_tasks_stand.taskLib[id].J = taskMapRPY * J_hip_link;
        zeroColumns(kin_tasks_stand.taskLib[id].J, waistVIndices);
        zeroColumns(kin_tasks_stand.taskLib[id].J, armVIndices);
        kin_tasks_stand.taskLib[id].dJ = Eigen::MatrixXd::Zero(3, model_nv);
        kin_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_stand.getId("Pz");
        kin_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(1);
        kin_tasks_stand.taskLib[id].errX(0) = base_pos_des(2) - q(2);
        kin_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(1);
        kin_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(1);
        kin_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(1);
        kin_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(1, 1) * (useFullFootContact ? 500 : 2000);
        kin_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(1, 1) * (useFullFootContact ? 50 : 10);
        Eigen::MatrixXd taskMap = Eigen::MatrixXd::Zero(1, 6);
        taskMap(0, 2) = 1;
        kin_tasks_stand.taskLib[id].J = taskMap * J_base;
        zeroColumns(kin_tasks_stand.taskLib[id].J, waistVIndices);
        kin_tasks_stand.taskLib[id].dJ = taskMap * dJ_base;
        zeroColumns(kin_tasks_stand.taskLib[id].dJ, waistVIndices);
        kin_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_stand.getId("CoMTrack");
        kin_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(2);
        kin_tasks_stand.taskLib[id].errX = pCoMDes.block(0, 0, 2, 1) - pCoMCur.block(0, 0, 2, 1);
        kin_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(2);
        kin_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(2);
        kin_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(2);
        kin_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(2, 2) * 2000; // 100
        kin_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(2, 2) * 100;
        kin_tasks_stand.taskLib[id].J = Jcom.block(0, 0, 2, model_nv);
        zeroColumns(kin_tasks_stand.taskLib[id].J, armVIndices);
        kin_tasks_stand.taskLib[id].dJ = Eigen::MatrixXd::Zero(2, model_nv);
        kin_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_stand.getId("CoMXY_HipRPY");
        taskMapRPY = Eigen::MatrixXd::Zero(3, 6);
        taskMapRPY(0, 3) = 1;
        taskMapRPY(1, 4) = 1;
        taskMapRPY(2, 5) = 1;
        kin_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(5);
        	kin_tasks_stand.taskLib[id].errX.block(0, 0, 2, 1) = pCoMDes.block(0, 0, 2, 1) - pCoMCur.block(0, 0, 2, 1);
			desRot = eul2Rot(base_rpy_des(0), base_rpy_des(1), base_rpy_des(2));
        	kin_tasks_stand.taskLib[id].errX.block<3, 1>(2, 0) = diffRot(hip_link_rot, desRot);
        kin_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(5);
//                    kin_tasks_stand.taskLib[id].derrX.block(0,0,2,1)=-(Jcom*dq).block(0,0,2,1);
//                    kin_tasks_stand.taskLib[id].derrX.block(2,0,3,1)=-taskMapRPY*J_hip_link*dq;
        kin_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(5);
        kin_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(5);
        kin_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(5, 5) * (useFullFootContact ? 100 : 250);
		kin_tasks_stand.taskLib[id].kp.block(2, 2, 3, 3) = Eigen::MatrixXd::Identity(3,3)*(useFullFootContact ? 300 : 1000);
		kin_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(5, 5) * (useFullFootContact ? 30 : 10);
        kin_tasks_stand.taskLib[id].kd.block(2, 2, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * (useFullFootContact ? 50 : 10);
        kin_tasks_stand.taskLib[id].J = Eigen::MatrixXd::Zero(5, model_nv);
        kin_tasks_stand.taskLib[id].J.block(0, 0, 2, model_nv) = Jcom.block(0, 0, 2, model_nv);
        kin_tasks_stand.taskLib[id].J.block(2, 0, 3, model_nv) = taskMapRPY * J_hip_link;
        zeroColumns(kin_tasks_stand.taskLib[id].J, waistVIndices);
        zeroColumns(kin_tasks_stand.taskLib[id].J, armVIndices);
        kin_tasks_stand.taskLib[id].dJ = Eigen::MatrixXd::Zero(5, model_nv);
        kin_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);
        for (const int column : waistVIndices)
            kin_tasks_stand.taskLib[id].W.diagonal()(column) = 200;

        // define swing arm motion
        Eigen::VectorXd target_arm_q = targetArmQ;

        id = kin_tasks_stand.getId("HandTrackJoints");
        const int standArmSize = armQIndices.size();
        kin_tasks_stand.taskLib[id].errX = target_arm_q - gather(q, armQIndices);
        kin_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(standArmSize);
        kin_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(standArmSize);
        kin_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(standArmSize);
        kin_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(standArmSize, standArmSize) * (useFullFootContact ? 200 : 2000);
        kin_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(standArmSize, standArmSize) * (useFullFootContact ? 30 : 100);
        kin_tasks_stand.taskLib[id].J = Eigen::MatrixXd::Zero(standArmSize, model_nv);
        for (int row = 0; row < standArmSize; ++row)
            kin_tasks_stand.taskLib[id].J(row, armVIndices[row]) = 1;
        kin_tasks_stand.taskLib[id].dJ = Eigen::MatrixXd::Zero(standArmSize, model_nv);
        kin_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        // Enter here functions to send actuator commands, like:
        // arm-l: 0-6, arm-r: 7-13, head: 14,15, waist: 16-18, leg-l: 19-24, leg-r: 25-30

        if (!headQIndices.empty())
        {
            id = kin_tasks_stand.getId("HeadRP");
            const int headSize = headQIndices.size();
            kin_tasks_stand.taskLib[id].errX = -gather(q, headQIndices);
            if (headSize > 1)
                kin_tasks_stand.taskLib[id].errX(1) += base_rpy_cur(1);
            kin_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(headSize);
            kin_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(headSize);
            kin_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(headSize);
            kin_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(headSize, headSize) * 100;
            kin_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(headSize, headSize) * 10;
            kin_tasks_stand.taskLib[id].J = Eigen::MatrixXd::Zero(headSize, model_nv);
            for (int row = 0; row < headSize; ++row)
                kin_tasks_stand.taskLib[id].J(row, headVIndices[row]) = 1;
            kin_tasks_stand.taskLib[id].dJ = Eigen::MatrixXd::Zero(headSize, model_nv);
            kin_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);
        }

        id = kin_tasks_stand.getId("Roll_Pitch_Yaw");
        kin_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(3);
        desRot = eul2Rot(base_rpy_des(0), base_rpy_des(1), base_rpy_des(2));
        kin_tasks_stand.taskLib[id].errX = diffRot(base_rot, desRot);
        kin_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(3);
        kin_tasks_stand.taskLib[id].derrX = -dq.block<3, 1>(3, 0);
        kin_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(3);
        kin_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(3);
        kin_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(3, 3) * 2000;
        kin_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(3, 3) * 100;
        taskMap = Eigen::MatrixXd::Zero(3, 6);
        taskMap(0, 3) = 1;
        taskMap(1, 4) = 1;
        taskMap(2, 5) = 1;
        kin_tasks_stand.taskLib[id].J = taskMap * J_base;
        kin_tasks_stand.taskLib[id].dJ = taskMap * dJ_base;
        kin_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_stand.getId("fixedWaist");
        const int waistSize = waistQIndices.size();
        kin_tasks_stand.taskLib[id].errX = -gather(q, waistQIndices);
        kin_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(waistSize);
        kin_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(waistSize);
        kin_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(waistSize);
        kin_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(waistSize, waistSize) * 200;
        kin_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(waistSize, waistSize) * 20;
        kin_tasks_stand.taskLib[id].J = Eigen::MatrixXd::Zero(waistSize, model_nv);
        for (int row = 0; row < waistSize; ++row)
            kin_tasks_stand.taskLib[id].J(row, waistVIndices[row]) = 1;
        kin_tasks_stand.taskLib[id].dJ = Eigen::MatrixXd::Zero(waistSize, model_nv);
        kin_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);
    }

    if (motionStateCur == DataBus::Walk || motionStateCur == DataBus::Walk2Stand)
    {
        kin_tasks_walk.computeAll(des_delta_q, des_dq, des_ddq, dyn_M, dyn_M_inv, dq);
        delta_q_final_kin = kin_tasks_walk.out_delta_q;
        dq_final_kin = kin_tasks_walk.out_dq;
        ddq_final_kin = kin_tasks_walk.out_ddq;
    }
    else if (motionStateCur == DataBus::Stand)
    {
        kin_tasks_stand.computeAll(des_delta_q, des_dq, des_ddq, dyn_M, dyn_M_inv, dq);
        delta_q_final_kin = kin_tasks_stand.out_delta_q;
        dq_final_kin = kin_tasks_stand.out_dq;
        ddq_final_kin = kin_tasks_stand.out_ddq;
    }
    else
    {
        delta_q_final_kin = Eigen::VectorXd::Zero(model_nv);
        dq_final_kin = Eigen::VectorXd::Zero(model_nv);
        ddq_final_kin = Eigen::VectorXd::Zero(model_nv);
    }

    // final WBC output collection
}

void WBC_priority::copy_Eigen_to_real_t(qpOASES::real_t *target, const Eigen::MatrixXd &source, int nRows, int nCols)
{
    int count = 0;

    for (int i = 0; i < nRows; i++)
    {
        for (int j = 0; j < nCols; j++)
        {
            target[count++] = isinf(source(i, j)) ? qpOASES::INFTY : source(i, j);
        }
    }
}

void WBC_priority::setQini(const Eigen::VectorXd &qIniDesIn, const Eigen::VectorXd &qIniCurIn)
{
    qIniDes = qIniDesIn;
    qIniCur = qIniCurIn;
}
