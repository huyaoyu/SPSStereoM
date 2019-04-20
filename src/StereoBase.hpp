//
// Created by yyhu on 4/19/19.
//

#ifndef SPSSTEREO_STEREOBASE_HPP
#define SPSSTEREO_STEREOBASE_HPP

#include <string>

class StereoBase
{
public:
    StereoBase();
    ~StereoBase();

    void enable_debug(const std::string& debugWorkingDir);

protected:
    bool flagDebug_;
    std::string debugWorkingDir_;
};

#endif //SPSSTEREO_STEREOBASE_HPP
