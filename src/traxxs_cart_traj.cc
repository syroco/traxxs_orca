#include <iostream>

#include <traxxs/trajectory/trajectory.hpp>
#include <traxxs/impl/traxxs_softmotion/traxxs_softmotion.hpp>

#include <orca_ros/orca_ros.h>
// #include "traxxs_orca/samples_helpers.hh"
// #include "traxxs_orca/samples_helpers_cart.hh"

template< class T >
using sptr = std::shared_ptr<T>;

using namespace traxxs;

#include <iostream>


int main(int argc, char *argv[])
{

    ros::init(argc, argv, "traxxs_cart_traj");

    std::string robot_name("");
    if(!ros::param::get("~robot_name",robot_name))
    {
        ROS_ERROR_STREAM("" << ros::this_node::getName() << " Could not find robot_name in namespace "
        << ros::this_node::getNamespace()
        << "/" << ros::this_node::getName());
        return 0;
    }

    std::string controller_name("");
    if(!ros::param::get("~controller_name",controller_name))
    {
        ROS_ERROR_STREAM("" << ros::this_node::getName() << " Could not find controller_name in namespace "
        << ros::this_node::getNamespace()
        << "/" << ros::this_node::getName());
        return 0;
    }

    std::string task_name("");
    if(!ros::param::get("~task_name",task_name))
    {
        ROS_ERROR_STREAM("" << ros::this_node::getName() << " Could not find task_name in namespace "
        << ros::this_node::getNamespace()
        << "/" << ros::this_node::getName());
        return 0;
    }

    auto cart_task_proxy = std::make_shared<orca_ros::task::RosCartesianTaskProxy>(robot_name,controller_name,task_name);


    Eigen::Affine3d starting_task_pose(cart_task_proxy->getCurrentPose());


    // starting_task_pose.translation() = Eigen::Vector3d(1.,0.75,0.5); // x,y,z in meters
    // starting_task_pose.linear() = Eigen::Quaterniond::Identity().toRotationMatrix();



    // define the path bounds
    path::PathBounds4d path_bounds;
    path_bounds.dx << 1.0, 1.0, 1.0, 10.0;
    path_bounds.dx*=10.0;
    path_bounds.ddx = 20.0 * path_bounds.dx;
    path_bounds.j = 20.0 * path_bounds.ddx;

    // we will use three Cartesian waypoints to define the path: a start, and end, and an intermediate waypoint
    path::CartesianPathWaypoint pt_start, pt_wpt, pt_wpt2, pt_end;

    // set the position of the waypoints
    double box_dim = -0.4;
    pt_start.x.p  = starting_task_pose.translation() + Eigen::Vector3d(0, 0, 0);//0, 0, 0;
    pt_wpt.x.p    = starting_task_pose.translation() + Eigen::Vector3d(box_dim, 0, 0); //1, 0, 0;
    pt_wpt2.x.p   = starting_task_pose.translation() + Eigen::Vector3d(box_dim, box_dim, 0); //1, 1, 0;
    pt_end.x.p    = starting_task_pose.translation() + Eigen::Vector3d(box_dim, box_dim, box_dim); //1, 1, 1;

    // set the orientation of the waypoints
    // pt_start.x.q  = Eigen::Quaterniond::Identity();
    pt_start.x.q  = Eigen::Quaterniond(starting_task_pose.linear());
    // pt_wpt.x.q    = Eigen::Quaterniond::Identity();
    pt_wpt.x.q    = Eigen::Quaterniond(starting_task_pose.linear());
    // pt_wpt2.x.q   = Eigen::Quaterniond::Identity();
    pt_wpt2.x.q   = Eigen::Quaterniond(starting_task_pose.linear());
    // pt_end.x.q    = Eigen::Quaterniond::Identity();
    pt_end.x.q    = Eigen::Quaterniond(starting_task_pose.linear());

    //
    // we can define conditions on the waypoints: here we define velocities
    //

    // velocities at start
    pt_start.pathConditionsPosition.dx << 0, 0, 0;
    pt_start.pathConditionsOrientation.dx << 0;

    //   // force arriving at the waypoint with null velocity to ensure acceleration continuity
    //   pt_wpt.pathConditionsPosition.dx << 0, 0, 0;
    //   pt_wpt.pathConditionsOrientation.dx << 0;

    // velocities at end
    pt_end.pathConditionsPosition.dx << 0, 0, 0;
    pt_end.pathConditionsOrientation.dx << 0;

    // we define the types of segments we want to use.
    // a Cartesian segment is composed of a segment type for translation, and a segment type for orientation
    // for orientation, a SmoothStep7 type (shape so that dx = 0, ddx = 0, j = 0 always at start/finish) is an ideal candidate
    //  since it avoids to impose constraints on arc velocity/acceleration/jerk from orientation in order to preserve continuity between segments.
    // we might need two segment types: one for joining waypoints (JoiningSegment_t), the other for blends (BlendSegment_t) (i.e. smoothening of corners)
    using JoiningSegment_t  = path::CartesianSegment< path::LinearSegment, path::SmoothStep7 >;
    using BlendSegment_t    = path::CartesianSegment< path::CircularBlend, path::SmoothStep7 >;

    // we use an helper function to create the segments w.r.t. the waypoints and the segment types we defined.
    std::vector< path::CartesianPathWaypoint > waypoints = { pt_start, pt_wpt, pt_wpt2, pt_end};
    auto segments = path::blendedSegmentsFromWaypoints< path::CartesianPathWaypoint, JoiningSegment_t, BlendSegment_t, double>(
    path_bounds, waypoints, 0.1 ); // std::vector< sptr< path::PathSegment > >


    // create a trajectory on these segments using the softmotion implementation
    auto trajectory = std::make_shared< trajectory::Trajectory >();
    if ( !trajectory->set< ArcTrajGenSoftMotion >( segments ) )
    {
        ROS_ERROR("Could not generate the trajectory with SoftMotion.");
        return 0;
    }

    int seg_idx;
    bool is_beyond;
    trajectory::TrajectoryState state;
    arc::ArcConditions conds;
    std::shared_ptr< path::PathSegment > seg;


    double loop_freq = 100;
    double loop_period_s = 1./loop_freq;
    ros::Rate loop_rate(loop_freq);


    double t=0.0;
    while(ros::ok())
    {
        // get the arc conditions at that time
        if ( !trajectory->getArcConditions( t, conds, seg, &seg_idx ) )
        {
            ROS_ERROR_STREAM("!trajectory->getArcConditions( t, conds, seg, &seg_idx ) ");
            break;
        }
        // get the state at that time
        trajectory->getState( t, state, nullptr, &is_beyond );
        if ( is_beyond )
        {
            ROS_INFO_STREAM("Trajectory is finished.");
            break;
        }


        std::cout << "------------------------------" << '\n';
        std::cout << "t: " << t << '\n';
        std::cout << "position: " << state.x.head(3).transpose() << '\n';
        std::cout << "orientation: " << state.x.tail(4).transpose() << '\n';

        Eigen::Affine3d desired_pose;
        desired_pose.translation() = state.x.head(3);
        // desired_pose.linear()= starting_task_pose.linear();
        starting_task_pose.linear() = Eigen::Quaterniond(state.x(6), state.x(3),state.x(4),state.x(5)).toRotationMatrix();
        cart_task_proxy->setDesiredPose(desired_pose.matrix());

        ros::spinOnce();

        loop_rate.sleep();
        t += loop_period_s;
    }

    return 0;


}
