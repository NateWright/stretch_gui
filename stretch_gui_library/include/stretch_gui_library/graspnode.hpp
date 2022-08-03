#ifndef GRASPNODE_HPP
#define GRASPNODE_HPP

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <QDebug>
#include <QThread>
#include <QTimer>
#include <QWidget>

enum Position { VERTICAL,
                HORIZONTAL };

class GraspNode : public QThread {
    Q_OBJECT
   public:
    explicit GraspNode(ros::NodeHandlePtr nh);
    ~GraspNode();
    void run() override;

   private:
    ros::NodeHandlePtr nh_;
    ros::Publisher cmdVel_;
    ros::Subscriber centerPointSub_;

    geometry_msgs::Twist cmdMsg_;
    double turnTime_;

    ros::AsyncSpinner *spinner_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener *tfListener_;

    geometry_msgs::PointStamped::Ptr point_;
    geometry_msgs::PointStamped::Ptr pointBaseLink_;

    geometry_msgs::PoseStamped::Ptr homePose_;

    Position orientation_ = VERTICAL;
    double verticalOffset_;
    double horizontalOffset_;

    bool robotMoving_;

    void centerPointCallback(const geometry_msgs::PointStamped::ConstPtr &input);

   signals:
    //    void navigateToPoint(const geometry_msgs::PointStamped::ConstPtr& input);
    void headSetRotation(double degPan = 0, double degTilt = 0);
    void headSetPan(double degPan = 0);
    void headSetTilt(double degTilt = 0);
    void armSetHeight(double meters = 0.2);
    void armSetReach(double meters = 0);
    void gripperSetRotate(double deg = 180);
    void gripperSetGrip(double deg = 0);
    void homeRobot();
    void navigateHome();
    void navigate(const geometry_msgs::PoseStamped::Ptr pose);
    void turnLeft(int degrees);
    void graspDone(bool);
    void enableMapping();
    void disableMapping();
    void hasObject(bool);
    void canNavigate(bool);
    bool moving();
   public slots:
    void setHorizontal();
    void setVertical();
    void lineUp();
    void lineUpOffset(double offset);
    void replaceObjectOffset(double offset);
    void replaceObject();
    void stowObject();
    void home();
    void releaseObject();
};

#endif  // GRASPNODE_HPP