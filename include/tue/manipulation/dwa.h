#ifndef TUE_MANIPULATION_DWA_H_
#define TUE_MANIPULATION_DWA_H_

#include <string>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>

#include <boost/shared_ptr.hpp>

namespace tue
{
namespace manipulation
{

class DWA
{

public:

    DWA();

    virtual ~DWA();

    bool initFromURDF(const std::string& urdf, const std::string root_name,
                      const std::string& tip_name, std::string& error);

    double calculateVelocity(const KDL::JntArray& q_current, unsigned int q);

private:

    //!Object to describe the (serial) kinematic chain
    KDL::Chain chain_;

    KDL::JntArray q_min_, q_max_, q_seed_;

    std::vector<std::string> joint_names_;

    std::vector<unsigned int> joint_index_to_segment_index_;

};

} // end namespace tue

} // end namespace manipulation

#endif
