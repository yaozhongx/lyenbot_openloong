#pragma once

#include "pinocchio/multibody/model.hpp"

#include <string>

std::string readNormalizedUrdfXml(const std::string &urdfPath, int *trimmedAttributeCount = nullptr);
void buildFloatingBaseModelFromUrdf(const std::string &urdfPath, pinocchio::Model &model,
                                    int *trimmedAttributeCount = nullptr);
void buildFixedBaseModelFromUrdf(const std::string &urdfPath, pinocchio::Model &model,
                                 int *trimmedAttributeCount = nullptr);

