//
// Created by yyhu on 4/19/19.
//

#include <iostream>

#include "StereoBase.hpp"

StereoBase::StereoBase()
: flagDebug_(false),
  debugWorkingDir_("./")
{

}

StereoBase::~StereoBase()
{

}

void StereoBase::enable_debug(const std::string& debugWorkingDir)
{
    debugWorkingDir_ = debugWorkingDir;
    flagDebug_ = true;

    std::cout << "Debug mode enabled with debugWorkingDir = " << debugWorkingDir_ << std::endl;
}