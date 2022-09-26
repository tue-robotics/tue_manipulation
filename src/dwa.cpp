#include "tue/manipulation/dwa.h"

#include <urdf/model.h>

#include <kdl_parser/kdl_parser.hpp>

#include <kdl/tree.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include <geolib/datatypes.h>

namespace tue
{
namespace manipulation
{

// ----------------------------------------------------------------------------------------------------

geo::Pose3D toGeo(const KDL::Frame& f)
{
    return geo::Pose3D(geo::Matrix3(f.M.data), geo::Vector3(f.p.data));
}

// ----------------------------------------------------------------------------------------------------

DWA::DWA() : constraint_(nullptr)
{
}

// ----------------------------------------------------------------------------------------------------

DWA::~DWA()
{
    delete constraint_;
}

// ----------------------------------------------------------------------------------------------------

bool DWA::initFromURDF(const std::string& urdf, const std::string root_name,
                       const std::string& tip_name, std::string& error)
{
    urdf::Model robot_model;
    KDL::Tree tree;

    if (!robot_model.initString(urdf))
    {
        error += "Could not initialize robot model";
        return false;
    }

    if (!kdl_parser::treeFromString(urdf, tree))
    {
        error += "Could not initialize tree object";
        return false;
    }

    if (tree.getSegment(root_name) == tree.getSegments().end())
    {
        error += "Could not find root link '" + root_name + "'.";
        return false;
    }

    if (tree.getSegment(tip_name) == tree.getSegments().end())
    {
        error += "Could not find tip link '" + tip_name + "'.";
        return false;
    }

    if (!tree.getChain(root_name, tip_name, chain_))
    {
        error += "Could not initialize chain object";
        return false;
    }

    // Get the joint limits from the robot model

    q_min_.resize(chain_.getNrOfJoints());
    q_max_.resize(chain_.getNrOfJoints());
    q_seed_.resize(chain_.getNrOfJoints());

    joint_names_.resize(chain_.getNrOfJoints());
    joint_index_to_segment_index_.resize(chain_.getNrOfJoints());

    unsigned int j = 0;
    for(unsigned int i = 0; i < chain_.getNrOfSegments(); ++i)
    {
//        std::cout << chain_.getSegment(i).getName() << std::endl;

        const KDL::Joint& kdl_joint = chain_.getSegment(i).getJoint();
        if (kdl_joint.getType() != KDL::Joint::None)
        {
            urdf::JointConstSharedPtr joint = robot_model.getJoint(kdl_joint.getName());
            if (joint && joint->limits)
            {
                q_min_(j) = joint->limits->lower;
                q_max_(j) = joint->limits->upper;
                q_seed_(j) = (q_min_(j) + q_max_(j)) / 2;
            }
            else
            {
                q_min_(j) = -1e9;
                q_max_(j) = 1e9;
                q_seed_(j) = 0;

            }

            joint_names_[j] = kdl_joint.getName();
            joint_index_to_segment_index_[j] = i;
            joint_name_to_index_[kdl_joint.getName()] = j;
            ++j;
        }
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool DWA::getJointIndex(const std::string& name, unsigned int& i_joint) const
{
    std::map<std::string, unsigned int>::const_iterator it = joint_name_to_index_.find(name);
    if (it == joint_name_to_index_.end())
        return false;

    i_joint = it->second;
    return true;
}

// ----------------------------------------------------------------------------------------------------

void DWA::calculateVelocity(const KDL::JntArray& q_current, double dt, std::vector<double>& q_wanted) const
{
    if (!constraint_)
    {
        for(unsigned int i = 0; i < q_current.rows(); ++i)
            q_wanted[i] = q_current(i);
        return;
    }

    for(unsigned int i_joint = 0; i_joint < q_current.rows(); ++i_joint)
    {

        unsigned int i_seg = joint_index_to_segment_index_[i_joint];

        KDL::Frame f_before = KDL::Frame::Identity();
        KDL::Frame f_after = KDL::Frame::Identity();

        unsigned int j = 0;
        const KDL::Segment* q_seg = 0;
        for(unsigned int i = 0; i < chain_.getNrOfSegments(); ++i)
        {
            double pos;
            const KDL::Segment& seg = chain_.getSegment(i);
            if (seg.getJoint().getType() != KDL::Joint::None)
            {
                pos = q_current(j);
                ++j;
            }
            else
            {
                pos = 0;
            }

            KDL::Frame f = seg.pose(pos);

            if (i < i_seg)
                f_before = f_before * f;
            else if (i > i_seg)
                f_after = f_after * f;
            else
                q_seg = &seg;
        }

        double best_dist = 1e10;
        double best_vel = 0;

        double pos = q_current(i_joint);
        for(double vel = -0.4; vel < 0.41; vel += 0.01)
        {
            double p = pos + vel;

            //        std::cout << q_min_(q) << ", " << q_max_(q) << std::endl;

            if (p > q_min_(i_joint) && p < q_max_(i_joint))
            {
                KDL::Frame f = f_before * q_seg->pose(p) * f_after;

                double dist = constraint_->test(toGeo(f));
                if (dist < best_dist)
                {
                    best_dist = dist;
                    best_vel = vel;
                }
            }
        }

        q_wanted[i_joint] = q_current(i_joint) + (best_vel * dt);
    }
}


} // end namespace tue

} // end namespace manipulation

