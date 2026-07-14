#include "urdf_model_loader.h"

#include "pinocchio/parsers/urdf.hpp"

#include <cctype>
#include <fstream>
#include <iterator>
#include <stdexcept>

std::string readNormalizedUrdfXml(const std::string &urdfPath, int *trimmedAttributeCount)
{
    std::ifstream input(urdfPath, std::ios::binary);
    if (!input)
        throw std::runtime_error("Cannot open URDF: " + urdfPath);

    std::string xml((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    int trimmed = 0;
    std::size_t cursor = 0;
    while ((cursor = xml.find('=', cursor)) != std::string::npos)
    {
        const auto quote = xml.find_first_of("\"'", cursor + 1);
        if (quote == std::string::npos)
            break;
        bool onlyWhitespace = true;
        for (std::size_t i = cursor + 1; i < quote; ++i)
            onlyWhitespace = onlyWhitespace && std::isspace(static_cast<unsigned char>(xml[i]));
        if (!onlyWhitespace)
        {
            ++cursor;
            continue;
        }

        const auto endQuote = xml.find(xml[quote], quote + 1);
        if (endQuote == std::string::npos)
            break;
        auto first = quote + 1;
        auto last = endQuote;
        while (first < last && std::isspace(static_cast<unsigned char>(xml[first])))
            ++first;
        while (last > first && std::isspace(static_cast<unsigned char>(xml[last - 1])))
            --last;
        if (first != quote + 1 || last != endQuote)
        {
            xml.erase(last, endQuote - last);
            xml.erase(quote + 1, first - quote - 1);
            ++trimmed;
            cursor = quote + 1 + (last - first) + 1;
        }
        else
            cursor = endQuote + 1;
    }
    if (trimmedAttributeCount)
        *trimmedAttributeCount = trimmed;
    return xml;
}

void buildFloatingBaseModelFromUrdf(const std::string &urdfPath, pinocchio::Model &model,
                                    int *trimmedAttributeCount)
{
    pinocchio::urdf::buildModelFromXML(readNormalizedUrdfXml(urdfPath, trimmedAttributeCount),
                                      pinocchio::JointModelFreeFlyer(), model);
}

void buildFixedBaseModelFromUrdf(const std::string &urdfPath, pinocchio::Model &model,
                                 int *trimmedAttributeCount)
{
    pinocchio::urdf::buildModelFromXML(readNormalizedUrdfXml(urdfPath, trimmedAttributeCount), model);
}

