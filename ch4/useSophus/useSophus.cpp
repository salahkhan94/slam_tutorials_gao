#include <iostream>
#include <cmath>
using namespace std; 

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sophus/so3.h"
#include "sophus/se3.h"

int main( int argc, char** argv )
{
    // Rotation matrix rotated 90 degrees along the Z axis
    Own :: Matrix3d ​​R = Own :: AngleAxisd (M_PI / 2, Own :: Vector3d (0,0,1)) toRotationMatrix ();
    
    Sophus::SO3 SO3_R(R); // Sophus::SO(3) can be constructed directly from the rotation matrix
    Sophus::SO3 SO3_v( 0, 0, M_PI/2 ); // can also be constructed from a rotation vector
    Eigen::Quaterniond q(R); // or quaternion
    Sophus::SO3 SO3_q( q );
    // The above expressions are all equivalent
    // When SO(3) is output, it is output as so(3)
    cout<<"SO(3) from matrix: "<<SO3_R<<endl;
    cout<<"SO(3) from vector: "<<SO3_v<<endl;
    cout<<"SO(3) from quaternion :"<<SO3_q<<endl;
    
    // Use the logarithmic map to get its Lie algebra
    Eigen::Vector3d so3 = SO3_R.log();
    cout<<"so3 = "<<so3.transpose()<<endl;
    // hat is vector to antisymmetric matrix
    cout<<"so3 hat=\n"<<Sophus::SO3::hat(so3)<<endl;
    // Relative, vee is the objection vector
    Cout<<"so3 hat vee= "<<Sophus::SO3::vee( Sophus::SO3::hat(so3) ).transpose()<<endl; // transpose is purely for the sake of output
    
    // Update of the incremental disturbance model
    Eigen::Vector3d update_so3(1e-4, 0, 0); //assuming the update is so much
    Sophus::SO3 SO3_updated = Sophus::SO3::exp(update_so3)*SO3_R;
    cout<<"SO3 updated = "<<SO3_updated<<endl;
    
    /******************** Meng Meng's dividing line ************************** *****/
    Cout<<"************ I am the dividing line *************"<<endl;
    // The operation of SE(3) is similar
    Eigen::Vector3d t(1,0,0); // translate 1 along the X axis
    Sophus::SE3 SE3_Rt(R, t); // Construct SE(3) from R, t
    Sophus::SE3 SE3_qt(q,t); // Construct SE(3) from q,t
    cout<<"SE3 from R,t= "<<endl<<SE3_Rt<<endl;
    cout<<"SE3 from q,t= "<<endl<<SE3_qt<<endl;
    // The Lie algebra se(3) is a six-dimensional vector, which is convenient for typedef first.
    typedef Eigen::Matrix<double,6,1> Vector6d;
    Vector6d se3 = SE3_Rt.log();
    cout<<"se3 = "<<se3.transpose()<<endl;
    // Observe the output, you will find that in Sophus, the translation of se(3) is in front and the rotation is in the back.
    // Same, there are two operators of hat and vee
    cout<<"se3 hat = "<<endl<<Sophus::SE3::hat(se3)<<endl;
    cout<<"se3 hat vee = "<<Sophus::SE3::vee( Sophus::SE3::hat(se3) ).transpose()<<endl;
    
    // Finally, demonstrate the update
    Vector6d update_se3; //update volume
    update_se3.setZero();
    update_se3(0,0) = 1e-4d;
    Sophus::SE3 SE3_updated = Sophus::SE3::exp(update_se3)*SE3_Rt;
    cout<<"SE3 updated = "<<endl<<SE3_updated.matrix()<<endl;
    
    return 0;
}