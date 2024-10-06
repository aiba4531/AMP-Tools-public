#include "ManipulatorSkeleton.h"


MyManipulator2D::MyManipulator2D()
    : LinkManipulator2D({1.0, 1.0}) // Default to a 2-link with all links of 1.0 length
{}

// Override this method for implementing forward kinematics
Eigen::Vector2d MyManipulator2D::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const {
    // Implement forward kinematics to calculate the joint position given the manipulator state (angles)

    int numLinks = nLinks();
    int numAngles = state.size();
    amp::ManipulatorState new_state;
     
    if (numLinks != numAngles) {
       new_state = Eigen::VectorXd::Zero(numLinks);
    }
    else    {
        new_state = state;
    }
    std::vector<Eigen::Vector2d> joint_positions(numLinks+1);

    // Define a new manipulator state to store the angles incase the state provided is not valid
    amp::ManipulatorState angles = state;

    // Get the link lengths
    std::vector<double> link_lengths = getLinkLengths();
    
    // Push 0 to the front of link_lengths since base is at (0, 0)
    link_lengths.insert(link_lengths.begin(), 0.0);
    
    // Define a vector that represents the zero vector in homegenous coordinates
    Eigen::Vector3d last_vector(0.0, 0.0, 1.0);
    
    // Stacked Transformation Matrix
    const int numRows = 3*(numLinks+1);
    Eigen::MatrixXd transformation_matrix(numRows, 3);

    double link_length, angle;
    Eigen::Matrix3d link_transformation;


    // Implement forward kinematics 
    for (int i = 0; i < numLinks+1; i++) { // for all links plus end effector

        if (i != numLinks){
            link_length = link_lengths[i];
            angle = state(i);        
        } else {
            link_length = link_lengths[i];
            angle = 0;
        }
        
        // Define the transformation matrix for the link   
        link_transformation << cos(angle), -sin(angle), link_length,
                               sin(angle),  cos(angle), 0,
                               0,           0,          1;

        // Assign the transformation matrix to the appropriate block in the larger matrix
        transformation_matrix.block<3, 3>(3 * i, 0) = link_transformation;
    }

    // Multiply the transformation matrices to get the final transformation matrix
    Eigen::Matrix3d final_transformation = Eigen::Matrix3d::Identity();
    
    for (int i = 0; i < numLinks+1; i++) {
        final_transformation *= transformation_matrix.block<3, 3>(3 * i, 0);
        last_vector = final_transformation * Eigen::Vector3d(0.0, 0.0, 1.0);
        joint_positions[i] = Eigen::Vector2d(last_vector[0], last_vector[1]);

    }
    return joint_positions[joint_index];
}

// Override this method for implementing inverse kinematics
amp::ManipulatorState MyManipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
    // Implement inverse kinematics here

    amp::ManipulatorState joint_angles(2);
    joint_angles.setZero();
    std::vector<double> link_lengths = getLinkLengths();
    double a1, a2, x, y;
    
    // If you have different implementations for 2/3/n link manipulators, you can separate them here
    if (nLinks() == 2) {
         // Implement inverse kinematics for a 2-link manipulator
        a1 = link_lengths[0];
        a2 = link_lengths[1];
        x = end_effector_location[0];
        y = end_effector_location[1];

        double costheta2 = (1/(2*a1*a2)) * (x*x + y*y - a1*a1 - a2*a2);
        double sintheta2 = sqrt(1 - costheta2*costheta2);

        double costheta1 = 1/(x*x + y*y) * (x*(a1 + a2*costheta2) + y*a2*sintheta2);
        double sintheta1 = (1/(x*x + y*y) * (y*(a1 + a2*costheta2) - x*a2*sintheta2));

        joint_angles(0) = atan2(sintheta1, costheta1);
        joint_angles(1) = atan2(sintheta2, costheta2);

        return joint_angles;


        return joint_angles;
    } else if (nLinks() == 3) {

        return joint_angles;
    } else {

        return joint_angles;
    }

    return joint_angles;
}