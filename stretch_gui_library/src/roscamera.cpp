#include "roscamera.hpp"

RosCamera::RosCamera(ros::NodeHandlePtr nh) : nh_(nh) {
    segmenter_.reset(new ObjectSegmenter(nh_));
    std::string pointCloudTopic;
    nh->getParam("/stretch_gui/pointCloudTopic", pointCloudTopic);
    colorCameraSub_ = nh_->subscribe(pointCloudTopic, 30, &RosCamera::cameraCallback, this);
    segmentedCameraSub_ = nh_->subscribe("/stretch_pc/cluster", 30, &RosCamera::segmentedCameraCallback, this);
    pointPick_ = nh->advertise<geometry_msgs::PointStamped>("/clicked_point", 30);
    centerPointSub_ = nh_->subscribe("/stretch_pc/centerPoint", 30, &RosCamera::centerPointCallback, this);
    cameraPub_ = nh_->advertise<sensor_msgs::Image>("/stretch_gui/image", 30);
    moveToThread(this);
}
RosCamera::~RosCamera() {
    spinner_->stop();
    delete spinner_;
}

void RosCamera::run() {
    spinner_ = new ros::AsyncSpinner(0);
    spinner_->start();
    exec();
}

void RosCamera::cameraCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc) {
    cloud_ = pc;

    const int width = pc->width, height = pc->height;
    camera_.reset(new QImage(height, width, ROSCAMERA::FORMAT));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotatedCloud(new pcl::PointCloud<pcl::PointXYZRGB>(height, width, pcl::PointXYZRGB(0, 0, 0)));

    pcl::PointXYZRGB point;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            point = pc->at(x, y);
            camera_->setPixel(height - 1 - y, x, QColor(point.r, point.g, point.b).rgb());
            rotatedCloud->at(height - 1 - y, x) = point;
        }
    }
    sensor_msgs::Image img;
    pcl::toROSMsg(*rotatedCloud, img);
    cameraPub_.publish(img);
}

void RosCamera::segmentedCameraCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& pc) {
    const int width = sceneClickCloud_->width, height = sceneClickCloud_->height;
    QSharedPointer<QImage> img = camera_;

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(sceneClickCloud_);
    const int count = 20;
    std::vector<int> pointIdxNKNSearch(count);
    std::vector<float> pointNKNSquaredDistance(count);

    QRgb red = QColor(Qt::red).rgb();
    for (pcl::PointXYZRGB p : *pc) {
        kdtree.nearestKSearch(p, count, pointIdxNKNSearch, pointNKNSquaredDistance);
        for (int pos : pointIdxNKNSearch) {
            img->setPixel(height - 1 - pos / width, pos % width, red);
        }
    }

    emit imgUpdateWithPointQImage(*img.data());
}

void RosCamera::centerPointCallback(const geometry_msgs::PointStamped::ConstPtr& point) {
    emit checkPointInRange(point);
    return;
}

void RosCamera::sceneClicked(QPoint press, QPoint release, QSize screen) {
    sceneClickCloud_ = cloud_;
    if (!sceneClickCloud_ || sceneClickCloud_->size() == 0) {
        emit clickFailure();
        return;
    }
    int locX = press.x() * static_cast<double>(sceneClickCloud_->height) / static_cast<double>(screen.width());
    int locY = press.y() * static_cast<double>(sceneClickCloud_->width) / static_cast<double>(screen.height());

    try {
        if (locY > sceneClickCloud_->width || locX > sceneClickCloud_->height) {
            throw(std::runtime_error("Not in range"));
        }
        emit clickInitiated();
        pcl::PointXYZRGB p = sceneClickCloud_->at(locY, sceneClickCloud_->height - locX);

        if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) {
            qDebug() << "click fail";
            throw(std::runtime_error("Point contains NaN"));
        }

        geometry_msgs::PointStamped::Ptr point(new geometry_msgs::PointStamped());
        point->header.frame_id = sceneClickCloud_->header.frame_id;

        point->point.x = p.x;
        point->point.y = p.y;
        point->point.z = p.z;

        try {
            segmenter_->segmentAndFind(sceneClickCloud_, p);
        } catch (...) {
            ROS_INFO_STREAM("Point cloud was empty after segmentation");
            emit clickFailure();
            return;
        }
        pointPick_.publish(point);
        emit clickSuccess();
    } catch (...) {
        emit clickFailure();
        return;
    }
}
