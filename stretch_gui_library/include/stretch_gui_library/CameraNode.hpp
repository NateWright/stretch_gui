#pragma once

#include <geometry_msgs/PointStamped.h>
#include <pcl/common/distances.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <QColor>
#include <QDebug>
#include <QImage>
#include <QObject>
#include <QPainter>
#include <QPoint>
#include <QRgb>
#include <QSharedPointer>
#include <QSize>
#include <QThread>
#include <QTimer>
#include <algorithm>
#include <vector>

#include "ObjectSegmenter.hpp"

namespace Camera {
const QImage::Format FORMAT = QImage::Format_RGB16;
}

class CameraNode : public QThread {
    Q_OBJECT
   public:
    explicit CameraNode(ros::NodeHandlePtr nh);
    ~CameraNode();
    void run() override;

   private:
    ros::NodeHandlePtr nh_;

    ros::Subscriber colorCameraSub_;
    ros::Subscriber segmentedCameraSub_;
    ros::Subscriber centerPointSub_;

    ros::Publisher pointPick_;
    ros::Publisher cameraPub_;

    ros::AsyncSpinner *spinner_;
    tf2_ros::Buffer *tfBuffer_;
    tf2_ros::TransformListener *tfListener_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sceneClickCloud_;

    QSharedPointer<QImage> camera_;

    ObjectSegmenterPtr segmenter_;

    float distToTable_;

    void cameraCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &);
    void segmentedCameraCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &);
    void centerPointCallback(const geometry_msgs::PointStamped::ConstPtr &);

   signals:
    void imgUpdate(const QPixmap &);
    void imgUpdateQImage(QImage);
    void imgUpdateWithPoint(const QPixmap &);
    void imgUpdateWithPointQImage(QImage);
    void checkPointInRange(const geometry_msgs::PointStamped::ConstPtr &);
    void clickSuccess();
    void clickFailure();
    void clickInitiated();
    void validPoint();
    void invalidPoint();
   public slots:
    void sceneClicked(QPoint press, QPoint release, QSize screen);
};
