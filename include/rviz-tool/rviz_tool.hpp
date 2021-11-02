#ifndef RVIZ_MBF_GOAL_H_
#define RVIZ_MBF_GOAL_H_

// Qt
#include <QObject>

// ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

// Rviz
#include <rviz/default_plugin/tools/pose_tool.h>
#include <rviz/display_context.h>
#include <rviz/properties/string_property.h>

// Tf2
#include <tf2/utils.h>

// msgs and services
#include <mbf_msgs/MoveBaseAction.h>
#include <mbf_msgs/MoveBaseGoal.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

// STL
#include <memory>

namespace rviz
{
    class mbfGoal : public PoseTool
    {
    Q_OBJECT
    public:
        mbfGoal();

        virtual ~mbfGoal();
        virtual void onInitialize() override;

    protected:
        virtual void onPoseSet(double x, double y, double theta) override;

    private Q_SLOTS:
        void updateTopic();

    private:
        StringProperty * topic_property_;
        StringProperty * planner_property_;
        StringProperty * controller_property_;

        // Move base flex action client
        std::shared_ptr<actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>> ac_;
    };
} // namespace rviz


#endif