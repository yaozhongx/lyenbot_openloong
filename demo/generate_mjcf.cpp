#include "robot_model_config.h"
#include "urdf_model_loader.h"

#include "mujoco/mujoco.h"

#include <Eigen/Geometry>
#include <array>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace
{
std::string readFile(const std::filesystem::path &path)
{
    std::ifstream input(path, std::ios::binary);
    if (!input)
        throw std::runtime_error("Cannot read: " + path.string());
    return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

void writeFile(const std::filesystem::path &path, const std::string &text)
{
    std::ofstream output(path, std::ios::binary);
    if (!output)
        throw std::runtime_error("Cannot write: " + path.string());
    output << text;
}

std::string shellQuote(const std::string &value)
{
    std::string result = "'";
    for (const char character : value)
        result += character == '\'' ? "'\\''" : std::string(1, character);
    return result + "'";
}

std::string sha256(const std::string &path)
{
    const std::string command = "sha256sum " + shellQuote(path);
    FILE *pipe = popen(command.c_str(), "r");
    if (!pipe)
        throw std::runtime_error("Cannot execute sha256sum");
    std::array<char, 256> line{};
    const bool readOk = fgets(line.data(), line.size(), pipe) != nullptr;
    const int status = pclose(pipe);
    if (!readOk || status != 0)
        throw std::runtime_error("sha256sum failed for: " + path);
    const std::string result(line.data(), 64);
    if (result.size() != 64)
        throw std::runtime_error("Invalid sha256sum output");
    return result;
}

void insertInsideBody(std::string &xml, const std::string &bodyName, const std::string &content)
{
    const auto body = xml.find("<body name=\"" + bodyName + "\"");
    if (body == std::string::npos)
        throw std::runtime_error("Generated MJCF body not found: " + bodyName);
    const auto bodyEnd = xml.find('>', body);
    xml.insert(bodyEnd + 1, "\n" + content);
}

void wrapWorldBodyInFloatingBase(std::string &xml, const pinocchio::Model &model, const std::string &baseBody)
{
    const auto world = xml.find("<worldbody>");
    const auto worldClose = xml.find("</worldbody>", world);
    if (world == std::string::npos || worldClose == std::string::npos)
        throw std::runtime_error("Generated MJCF worldbody was not found");
    const auto innerStart = world + std::string("<worldbody>").size();
    const auto inner = xml.substr(innerStart, worldClose - innerStart);

    const auto &baseInertia = model.inertias[1];
    const auto matrix = baseInertia.inertia().matrix();
    std::ostringstream body;
    body << "\n    <body name=\"" << baseBody << "\">\n"
         << "      <freejoint name=\"float_base\"/>\n"
         << "      <inertial pos=\"" << baseInertia.lever().transpose() << "\" mass=\"" << baseInertia.mass()
         << "\" fullinertia=\"" << matrix(0, 0) << ' ' << matrix(1, 1) << ' ' << matrix(2, 2) << ' '
         << matrix(0, 1) << ' ' << matrix(0, 2) << ' ' << matrix(1, 2) << "\"/>\n"
         << "      <site name=\"imu\" quat=\"1 0 0 0\"/>" << inner
         << "\n    </body>\n  ";
    xml.replace(innerStart, worldClose - innerStart, body.str());
}

void insertSimulationDefaults(std::string &xml, const RobotModelConfig &config)
{
    const auto compilerEnd = xml.find("/>", xml.find("<compiler"));
    if (compilerEnd == std::string::npos)
        throw std::runtime_error("Generated MJCF compiler element was not found");
    std::ostringstream settings;
    settings << "\n  <option timestep=\"" << config.simulationTimeStep
             << "\" tolerance=\"1e-10\" solver=\"Newton\" jacobian=\"dense\" cone=\"pyramidal\"/>\n"
             << "  <default><joint damping=\"" << config.simulationJointDamping
             << "\" frictionloss=\"" << config.simulationJointFrictionLoss
             << "\" armature=\"" << config.simulationJointArmature << "\"/></default>";
    xml.insert(compilerEnd + 2, settings.str());
}

std::string bodyFrameForFixedFrame(const pinocchio::Model &model, pinocchio::FrameIndex frameId)
{
    const auto parentJoint = model.frames[frameId].parent;
    for (const auto &frame : model.frames)
        if (frame.parent == parentJoint && frame.type == pinocchio::BODY && frame.name != model.frames[frameId].name)
            return frame.name;
    throw std::runtime_error("Cannot find moving body for fixed frame: " + model.frames[frameId].name);
}

std::string siteXml(const pinocchio::Model &model, const std::string &frameName, const std::string &siteName,
                    double friction)
{
    const auto frameId = model.getFrameId(frameName, pinocchio::BODY);
    const auto &placement = model.frames[frameId].placement;
    const Eigen::Quaterniond quaternion(placement.rotation());
    std::ostringstream text;
    const Eigen::Vector3d collisionCenter = placement.translation() + placement.rotation() * Eigen::Vector3d(0, 0, 0.005);
    text << std::setprecision(16)
         << "      <site name=\"" << siteName << "\" type=\"box\" size=\"0.08 0.04 0.005\" pos=\""
         << placement.translation().transpose() << "\" quat=\"" << quaternion.w() << ' ' << quaternion.x() << ' '
         << quaternion.y() << ' ' << quaternion.z() << "\" rgba=\"0 1 0 0\"/>\n"
         << "      <geom name=\"" << siteName << "-collision\" type=\"box\" size=\"0.08 0.04 0.005\" pos=\""
         << collisionCenter.transpose() << "\" quat=\"" << quaternion.w() << ' ' << quaternion.x() << ' '
         << quaternion.y() << ' ' << quaternion.z() << "\" rgba=\"0 0 0 0\" friction=\""
         << friction << " 0.3 0.3\"/>";
    return text.str();
}

std::string motorXml(const pinocchio::Model &model, const std::vector<std::string> &jointNames)
{
    std::ostringstream text;
    text << "  <actuator>\n";
    for (const auto &name : jointNames)
    {
        const auto jointId = model.getJointId(name);
        const auto effort = model.effortLimit[model.idx_vs[jointId]];
        text << "    <motor name=\"motor_" << name << "\" joint=\"" << name
             << "\" gear=\"1\" ctrllimited=\"true\" ctrlrange=\"-" << effort << ' ' << effort << "\"/>\n";
    }
    text << "  </actuator>\n";
    return text.str();
}

std::string keyframeXml(const RobotModelConfig &config)
{
    std::ostringstream text;
    text << "  <keyframe>\n    <key name=\"home\" qpos=\"0 0 " << config.nominalBaseHeight << " 1 0 0 0";
    for (const auto value : config.initialJointPositions)
        text << ' ' << value;
    text << "\"/>\n  </keyframe>\n";
    return text.str();
}
}

int main(int argc, char **argv)
{
    try
    {
        const std::string configPath = argc > 1 ? argv[1] : "../common/robot_configs/lyenbot.json";
        const auto config = loadRobotModelConfig(configPath);
        const std::string sourceHash = sha256(config.urdfPath);
        const auto outputDir = std::filesystem::path(config.scenePath).parent_path();
        std::filesystem::create_directories(outputDir);

        pinocchio::Model pinModel;
        int normalizedAttributes = 0;
        buildFloatingBaseModelFromUrdf(config.urdfPath, pinModel, &normalizedAttributes);

        std::string normalizedUrdf = readNormalizedUrdfXml(config.urdfPath);
        const auto compiler = normalizedUrdf.find("<compiler meshdir=\"\"");
        if (compiler == std::string::npos)
            throw std::runtime_error("URDF mujoco/compiler meshdir attribute was not found");
        normalizedUrdf.replace(compiler, std::string("<compiler meshdir=\"\"").size(),
                               "<compiler meshdir=\"../meshes\"");
        std::size_t meshPos = 0;
        while ((meshPos = normalizedUrdf.find("filename=\"meshes/", meshPos)) != std::string::npos)
        {
            normalizedUrdf.erase(meshPos + 10, std::string("meshes/").size());
            meshPos += 10;
        }
        const auto normalizedPath = outputDir / "normalized_source.urdf";
        writeFile(normalizedPath, normalizedUrdf);

        char error[2048] = {};
        mjModel *baseModel = mj_loadXML(normalizedPath.c_str(), nullptr, error, sizeof(error));
        if (!baseModel)
            throw std::runtime_error("MuJoCo URDF compile failed: " + std::string(error));
        const auto basePath = outputDir / "Lyenbot.base.xml";
        if (!mj_saveLastXML(basePath.c_str(), baseModel, error, sizeof(error)))
        {
            mj_deleteModel(baseModel);
            throw std::runtime_error("MuJoCo XML save failed: " + std::string(error));
        }
        mj_deleteModel(baseModel);

        std::string mjcf = readFile(basePath);
        const auto rootEnd = mjcf.find('>');
        mjcf.insert(rootEnd + 1, "\n  <!-- generated from " + config.urdfPath + " sha256=" + sourceHash
                                 + "; command: generate_mjcf " + configPath + " -->");
        insertSimulationDefaults(mjcf, config);
        wrapWorldBodyInFloatingBase(mjcf, pinModel, config.baseBody);

        const auto leftFootId = pinModel.getFrameId(config.leftFootFrame, pinocchio::BODY);
        const auto rightFootId = pinModel.getFrameId(config.rightFootFrame, pinocchio::BODY);
        insertInsideBody(mjcf, bodyFrameForFixedFrame(pinModel, leftFootId),
                         siteXml(pinModel, config.leftFootFrame, "lf-tc", config.simulationFloorFriction));
        insertInsideBody(mjcf, bodyFrameForFixedFrame(pinModel, rightFootId),
                         siteXml(pinModel, config.rightFootFrame, "rf-tc", config.simulationFloorFriction));

        const auto close = mjcf.rfind("</mujoco>");
        if (close == std::string::npos)
            throw std::runtime_error("Generated MJCF has no closing mujoco tag");
        std::ostringstream additions;
        additions << motorXml(pinModel, config.joints.actuated)
                  << "  <sensor>\n"
                  << "    <framequat name=\"" << config.orientationSensor << "\" objtype=\"site\" objname=\"imu\"/>\n"
                  << "    <velocimeter name=\"" << config.velocitySensor << "\" site=\"imu\"/>\n"
                  << "    <gyro name=\"" << config.gyroSensor << "\" site=\"imu\"/>\n"
                  << "    <accelerometer name=\"" << config.accelerationSensor << "\" site=\"imu\"/>\n"
                  << "    <touch name=\"lf-touch\" site=\"lf-tc\"/>\n"
                  << "    <touch name=\"rf-touch\" site=\"rf-tc\"/>\n"
                  << "  </sensor>\n" << keyframeXml(config);
        mjcf.insert(close, additions.str());

        const auto robotPath = outputDir / "Lyenbot.xml";
        writeFile(robotPath, mjcf);

        std::ostringstream scene;
        scene << "<mujoco model=\"lyenbot_scene\">\n"
              << "  <include file=\"Lyenbot.xml\"/>\n"
              << "  <statistic center=\"0 0 0.1\" extent=\"0.8\"/>\n"
              << "  <visual>\n"
              << "    <headlight diffuse=\"0.6 0.6 0.6\" ambient=\"0.3 0.3 0.3\" specular=\"0 0 0\"/>\n"
              << "    <rgba haze=\"0.15 0.25 0.35 1\"/>\n"
              << "    <global azimuth=\"150\" elevation=\"-20\"/>\n"
              << "  </visual>\n"
              << "  <asset>\n"
              << "    <texture type=\"skybox\" builtin=\"gradient\" rgb1=\"0.3 0.5 0.7\" rgb2=\"0 0 0\" width=\"512\" height=\"3072\"/>\n"
              << "    <texture type=\"2d\" name=\"groundplane\" builtin=\"checker\" mark=\"edge\""
              << " rgb1=\"0.2 0.3 0.4\" rgb2=\"0.1 0.2 0.3\" markrgb=\"0.8 0.8 0.8\""
              << " width=\"300\" height=\"300\"/>\n"
              << "    <material name=\"groundplane\" texture=\"groundplane\" texuniform=\"true\""
              << " texrepeat=\"5 5\" reflectance=\"0.2\"/>\n"
              << "  </asset>\n"
              << "  <worldbody>\n"
              << "    <light pos=\"0 0 1.5\" dir=\"0 0 -1\" directional=\"true\"/>\n"
              << "    <geom name=\"floor\" type=\"plane\" size=\"0 0 1\" pos=\"0 0 0\""
              << " material=\"groundplane\" friction=\"" << config.simulationFloorFriction << " 0.3 0.3\"/>\n"
              << "  </worldbody>\n</mujoco>\n";
        writeFile(config.scenePath, scene.str());
        std::ostringstream manifest;
        manifest << "source_urdf=" << config.urdfPath << '\n'
                 << "source_sha256=" << sourceHash << '\n'
                 << "command=./generate_mjcf " << configPath << '\n'
                 << "normalized_attribute_whitespace=" << normalizedAttributes << '\n';
        writeFile(outputDir / "generation_manifest.txt", manifest.str());

        mjModel *finalModel = mj_loadXML(config.scenePath.c_str(), nullptr, error, sizeof(error));
        if (!finalModel)
            throw std::runtime_error("Generated scene validation failed: " + std::string(error));
        const bool dimensionsOk = finalModel->nq == pinModel.nq && finalModel->nv == pinModel.nv
                                  && finalModel->nu == static_cast<int>(config.joints.actuated.size());
        std::cout << "generated=" << robotPath << " nq=" << finalModel->nq << " nv=" << finalModel->nv
                  << " nu=" << finalModel->nu << " normalized_attributes=" << normalizedAttributes << '\n';
        mj_deleteModel(finalModel);
        if (!dimensionsOk)
            throw std::runtime_error("Generated MuJoCo dimensions do not match Pinocchio/configuration");
        std::cout << "GENERATE_MJCF_OK\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "GENERATE_MJCF_FAILED: " << error.what() << '\n';
        return 1;
    }
}
