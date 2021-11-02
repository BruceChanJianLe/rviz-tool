#include "rviz-tool/rviz_tool.hpp"
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rviz::mbfGoal, rviz::Tool)

namespace rviz
{
    mbfGoal::mbfGoal()
    {
        shortcut_key_ = 'g';

        topic_property_ = std::make_shared<StringProperty>( "Topic", "/move_base_flex/move_base",
                                                "The topic on which to publish navigation goals.",
                                                getPropertyContainer(), SLOT( updateTopic() ), this );
        planner_property_ = std::make_shared<StringProperty>( "Planner", "navfn/NavfnROS",
                                                "The topic on which to publish navigation goals.",
                                                getPropertyContainer(), SLOT( updateTopic() ), this );
        controller_property_ = std::make_shared<StringProperty>( "Controller", "base_local_planner/TrajectoryPlannerROS",
                                                "The topic on which to publish navigation goals.",
                                                getPropertyContainer(), SLOT( updateTopic() ), this );
    }

    mbfGoal::~mbfGoal()
    {
    }

    void mbfGoal::onInitialize()
    {
        PoseTool::onInitialize();
        setName("Move Base Flex Goal");
        ac_ = std::make_shared<actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>>("/move_base_flex/move_base", true);
    }

    // Reference: https://github.com/ros-visualization/rviz/blob/noetic-devel/src/rviz/default_plugin/tools/goal_tool.cpp
    void mbfGoal::onPoseSet(double x, double y, double theta)
    {
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, theta);
        geometry_msgs::PoseStamped goal;
        goal.pose.orientation = tf2::toMsg(quat);
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.position.z = 0.0;
        goal.header.frame_id = context_->getFixedFrame().toStdString();
        goal.header.stamp = ros::Time::now();

        ROS_INFO_STREAM(
            "Setting goal: Frame: "
            << context_->getFixedFrame().toStdString()
            << ", Position("
            << goal.pose.position.x
            << ", "
            << goal.pose.position.y
            << ", "
            << goal.pose.position.z
            << "), Orientation("
            << goal.pose.orientation.x
            << ", "
            << goal.pose.orientation.y
            << ", "
            << goal.pose.orientation.z
            << ", "
            << goal.pose.orientation.w
            << ") = Angle: "
            << theta
        );

        mbf_msgs::MoveBaseGoal mbf_goal;
        mbf_goal.target_pose = goal;
        mbf_goal.planner = planner_property_->getStdString();
        mbf_goal.controller = controller_property_->getStdString();

        // Send move base flex goal
        ac_->sendGoal(mbf_goal);
    }

    void mbfGoal::updateTopic()
    {
        try
        {
            ac_.reset(new actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>(topic_property_->getStdString(), true));
        }
        catch(const ros::Exception & e)
        {
            ROS_ERROR_STREAM("Move base flex goal: " << e.what());
        }
    }
} // namespace rviz