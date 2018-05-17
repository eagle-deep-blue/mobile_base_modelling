/*
 * Copyright (c) 2017, <Solomon Jingchun YIN> <jingchun.yin@live.com>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY <Solomon Jingchun YIN> <jingchun.yin@live.com> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <Solomon Jingchun YIN> <jingchun.yin@live.com> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#ifndef UNICYCLEBASE_H
#define UNICYCLEBASE_H

#include "mobile_base.h"

class UnicycleBase :  MobileBase
{
public:
UnicycleBase();
~UnicycleBase();
bool updateCurrentPoseUsingWheelEncoderIncrementalDisplacement(Eigen::VectorXf joint_angular_displacement);
bool transformJointIncrementalDisplacement2BaseIncrementalDisplacement(Eigen::Vector3f& base_cartesian_displacement, Eigen::VectorXf joint_angular_displacement);
bool transformJointCmdVel2BaseCmdVel(Eigen::VectorXf joint_cmd_vel, Eigen::Vector3f& base_cmd_vel);
bool transformBaseCmdVel2JointCmdVel(Eigen::Vector3f base_cmd_vel, Eigen::VectorXf& joint_cmd_vel);

private:
  float m_radius_wheel;
  float m_width_chassis;
};

#endif // UNICYCLEBASE_H
