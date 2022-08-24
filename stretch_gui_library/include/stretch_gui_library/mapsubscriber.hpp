#ifndef MAPSUBSCRIBER_H
#define MAPSUBSCRIBER_H

#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/Empty.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <QDebug>
#include <QGraphicsScene>
#include <QImage>
#include <QObject>
#include <QPainter>
#include <QPen>
#include <QPoint>
#include <QSize>
#include <QThread>
#include <QTimer>
#include <cmath>

using std::vector;

namespace MAPSUBSCRIBER {
const QImage::Format FORMAT = QImage::Format_RGB444;
}

class MapSubscriber : public QThread {
    Q_OBJECT
   public:
    explicit MapSubscriber(ros::NodeHandlePtr nodeHandle);
    ~MapSubscriber();
    void run() override;

   private:
    ros::NodeHandlePtr nh_;
    ros::Subscriber mapSub_;
    ros::Subscriber mapPointCloudSub_;
    ros::Subscriber posSub_;
    ros::Publisher movePub_;
    ros::Publisher mapPub_;

    ros::AsyncSpinner* spinner_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener* tfListener_;

    QSize mapSize_;
    QPoint origin_;
    double resolution_;

    QPoint robotPos_;
    double robotRot_;

    geometry_msgs::PoseStamped robotHome_;
    nav_msgs::OccupancyGrid::ConstPtr mapMsg_;

    void
    mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void posCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void mapPointCloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
   signals:
    void homeSet(bool);
    void robotPose(QPoint, double);

   public slots:
    void moveRobot(QPoint press, QPoint release, QSize screen);
    void moveRobotLoc(const geometry_msgs::PoseStamped::Ptr pose);
    void setHome();
    void setHomeIfNone();
    void navigateHome();
    void enableMapping();
    void disableMapping();
    void rotate(int degrees);
    void rotateLeft(int degrees = 5);
    void rotateRight(int degrees = 5);
    void drive(double meter);
};

QPoint translateScreenToMap(QPoint p, QSize screen, QSize map);

#endif  // MAPSUBSCRIBER_H
