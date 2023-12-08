/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Justin Kottinger */
#pragma once
#include <cmath>
#include <filesystem>
#include <fstream>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/base/State.h>

namespace fs = std::filesystem;
namespace og = ompl::geometric;
namespace oc = ompl::control;
namespace ob = ompl::base;


// generate date/time information to solutions or solution directories
std::string GetCurrentTimeForFileName()
{
    auto time = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y_%m_%d_%T"); // ISO 8601 without timezone information.
    auto s = ss.str();
    std::replace(s.begin(), s.end(), ':', '_');
    return s;
}

// parent function for including date/time information to files
fs::path appendTimeToFileName(const fs::path& fileName)
{
    return fileName.stem().string() + "_" + GetCurrentTimeForFileName() + fileName.extension().string();
}

// write solultion to the system
void write2sys(const og::SimpleSetupPtr problem, const std::vector<Agent*> agents)
{
    fs::path sol_dir = "solutions";
    std::string fileName = "geoPlan.txt";
    auto filePath = fs::current_path() / sol_dir / fs::path(fileName);
    std::ofstream file(filePath);
    const og::PathGeometric p = problem->getSolutionPath();
    p.printAsMatrix(std::cout);
    p.printAsMatrix(file);
}

// write solultion to the system
void write2sys(const oc::SimpleSetupPtr problem, const std::vector<Agent*> agents)
{
    fs::path sol_dir = "solutions";
    std::string fileName = "geoPlan.txt";
    auto filePath = fs::current_path() / sol_dir / fs::path(fileName);
    std::ofstream file(filePath);
    const oc::PathControl p = problem->getSolutionPath();
    p.printAsMatrix(std::cout);
    p.asGeometric().printAsMatrix(file);
}

std::vector<double> split(const std::string& s) {
    std::vector<double> tokens;
    std::istringstream iss(s);
    std::string token;
    while (std::getline(iss, token, ' ')) {
        tokens.push_back(std::stod(token));
    }
    return tokens;
}

std::vector<std::vector<double>> getStates() {
    std::ifstream file("solutions/geoPlan.txt"); // Replace with your file name
    if (!file.is_open()) {
        std::cerr << "Error opening the file." << std::endl;
    }
    std::vector<std::vector<double>> matrix;
    std::string line;

    // Read the file line by line
    while (std::getline(file, line)) {
        std::vector<double> row = split(line);
        matrix.push_back(row);
    }
    file.close();
    auto lastRow = matrix[matrix.size() - 2];
    lastRow[2] = 0.0;
    lastRow[3] = 0.0;
    matrix[matrix.size() - 1] = lastRow; 
    
    return matrix;
}

double wrapToPi(double angle) {
    angle = std::fmod(angle, 2.0 * M_PI);
    if (angle < -M_PI) angle += 2.0 * M_PI;
    else if (angle > M_PI) angle -= 2.0 * M_PI;
    return angle;
}