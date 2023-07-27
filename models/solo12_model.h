#ifndef TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_SOLO12_MODEL_H_
#define TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_SOLO12_MODEL_H_

#include <towr/models/kinematic_model.h>
#include <towr/models/single_rigid_body_dynamics.h>
#include <towr/models/endeffector_mappings.h>

namespace towr {

/**
 * @brief The Kinematics of the quadruped robot Solo12.
 */
class Solo12KinematicModel : public KinematicModel {
public:
  Solo12KinematicModel () : KinematicModel(4)
  {
    const double x_nominal_b = 0.193;
    const double y_nominal_b = 0.15;
    const double z_nominal_b = -0.23;

    nominal_stance_.at(LF) <<  x_nominal_b,   y_nominal_b, z_nominal_b;
    nominal_stance_.at(RF) <<  x_nominal_b,  -y_nominal_b, z_nominal_b;
    nominal_stance_.at(LH) << -x_nominal_b,   y_nominal_b, z_nominal_b;
    nominal_stance_.at(RH) << -x_nominal_b,  -y_nominal_b, z_nominal_b;

    max_dev_from_nominal_ << 0.12, 0.05, 0.08;
  }
};

/**
 * @brief The Dynamics of the quadruped robot Solo12.
 */
class Solo12DynamicModel : public SingleRigidBodyDynamics {
public:
  Solo12DynamicModel() : SingleRigidBodyDynamics(2.5, 0.0089, 0.024, 0.0266, 0, 0, 0, 4) {} 
                                    //(0.0089, 0.024, 0.0266, 0, 0, 0) Inertia tensor from adding inertia values from the URDF file
                                    //(2.5, 0.0212708, 0.0447083, 0.0609375 , 0, 0, 0, 4) Inertia tensor from modeling robot as rectangular prism
};

} /* namespace towr */

#endif /* TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_SOLO12_MODEL_H_ */
