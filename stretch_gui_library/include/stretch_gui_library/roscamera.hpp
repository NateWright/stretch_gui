#ifndef ROSCAMERA_HPP
#define ROSCAMERA_HPP

#include <geometry_msgs/PointStamped.h>
#include <pcl/common/distances.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

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

namespace ROSCAMERA {
const QImage::Format FORMAT = QImage::Format_RGB16;
}

class RosCamera : public QThread {
    Q_OBJECT
   public:
    explicit RosCamera(ros::NodeHandlePtr nh);
    ~RosCamera();
    void run() override;

   private:
    ros::NodeHandlePtr nh_;

    ros::Subscriber colorCameraSub_;
    ros::Subscriber segmentedCameraSub_;
    ros::Subscriber centerPointSub_;

    ros::Publisher pointPick_;
    ros::Publisher cameraPub_;

    ros::AsyncSpinner *spinner_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;

    QSharedPointer<QImage> camera_;

    ObjectSegmenterPtr segmenter_;

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
   public slots:
    void sceneClicked(QPoint press, QPoint release, QSize screen);
};

#endif  // ROSCAMERA_HPP
