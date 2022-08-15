#include "Server.hpp"

Server::Server(QObject* parent) : ServerSimpleSource(parent), headPanTilt_({0, -30}) {
    nh_.reset(new ros::NodeHandle("stretch_gui_server"));
    mapNode_ = new MapSubscriber(nh_);
    moveBaseStatusNode_ = new MoveBaseStatus(nh_);
    cameraNode_ = new RosCamera(nh_);
    graspNode_ = new GraspNode(nh_);
    moveItNode_ = new StretchMoveItInterface(nh_);

    setPageNumber_(0);
    setHasObject_(false);
    setCanNavigate_(true);

    initConnections();

    mapNode_->start();
    moveBaseStatusNode_->start();
    cameraNode_->start();
    graspNode_->start();
    moveItNode_->start();
}

void Server::initConnections() {
    // Navigation Page
    connect(this, &Server::enableMapping, mapNode_, &MapSubscriber::enableMapping);
    connect(this, &Server::disableMapping, mapNode_, &MapSubscriber::disableMapping);

    connect(this, &Server::ButtonStopClicked, moveBaseStatusNode_, &MoveBaseStatus::stopRobot);
    connect(this, &Server::ButtonSetHomeClicked, mapNode_, &MapSubscriber::setHome);
    connect(this, &Server::ButtonNavigateHomeClicked, mapNode_, &MapSubscriber::navigateHome);

    connect(this, &Server::DisplayMapMouseClick, mapNode_, &MapSubscriber::moveRobot);

    connect(mapNode_, &MapSubscriber::robotPose, this, &Server::robotPose);
    connect(mapNode_, &MapSubscriber::homeSet, this, [this](bool b) { emit uiButtonNavigateHomeSetEnabled(b); });

    connect(moveBaseStatusNode_, &MoveBaseStatus::robotMoving, this, [this](bool b) { emit uiPleaseWaitSetVisible(b); });

    // Select Object Page
    connect(this, &Server::homeRobot, moveItNode_, &StretchMoveItInterface::homeRobot);
    connect(this, &Server::cameraSetRotation, moveItNode_, &StretchMoveItInterface::headSetRotation);

    connect(this, &Server::CameraMoveButtonUpClicked, moveItNode_, &StretchMoveItInterface::headUp);        // Client to server
    connect(this, &Server::CameraMoveButtonDownClicked, moveItNode_, &StretchMoveItInterface::headDown);    // Client to server
    connect(this, &Server::CameraMoveButtonLeftClicked, moveItNode_, &StretchMoveItInterface::headLeft);    // Client to server
    connect(this, &Server::CameraMoveButtonRightClicked, moveItNode_, &StretchMoveItInterface::headRight);  // Client to server
    connect(this, &Server::CameraMoveButtonHomeClicked, moveItNode_, &StretchMoveItInterface::headHome);    // Client to server

    // Find point in Camera
    connect(this, &Server::DisplayCameraMouseClicked, cameraNode_, &RosCamera::sceneClicked);

    connect(cameraNode_, &RosCamera::clickInitiated, this, &Server::uiPointPleaseWaitShow);  // Sever to client

    // Camera feed
    // Server to client
    // Error: Displays if NaN point was selected
    connect(cameraNode_, &RosCamera::clickFailure, this, &Server::uiErrorNanPointShow);    // Server to client
    connect(cameraNode_, &RosCamera::clickFailure, this, &Server::uiPointPleaseWaitHide);  // Server to client

    connect(cameraNode_, &RosCamera::checkPointInRange, mapNode_, &MapSubscriber::checkPointInRange);  // Server only

    // True
    connect(mapNode_, &MapSubscriber::validPoint, this, &Server::changeToConfirmObject);  // Both
    connect(mapNode_, &MapSubscriber::validPoint, this, &Server::uiChangeToConfirmObject);
    // False
    connect(mapNode_, &MapSubscriber::invalidPoint, this, &Server::uiErrorOutOfRangeShow);  // Server to client
    connect(mapNode_, &MapSubscriber::invalidPoint, this, &Server::uiPointPleaseWaitHide);  // Server to client

    // Confirm Object Page

    connect(this, &Server::SetVertical, graspNode_, &GraspNode::setVertical);
    connect(this, &Server::SetHorizontal, graspNode_, &GraspNode::setHorizontal);

    connect(this, &Server::ConfirmButtonNoClicked, this, &Server::changeToSelectScreen);  // Client to Both
    connect(this, &Server::ConfirmButtonYesClicked, this, &Server::changeToGrasping);     // Client to Both
    connect(this, &Server::ConfirmButtonYesClicked, graspNode_, &GraspNode::lineUp);      // Client to server

    connect(cameraNode_, &RosCamera::imgUpdateWithPointQImage, this, &Server::uiDisplayImageSetCamera);  // Server to ui

    // Grasp Page

    connect(graspNode_, &GraspNode::canNavigate, this, &Server::setCanNavigate_);
    connect(graspNode_, &GraspNode::hasObject, this, &Server::setHasObject_);

    connect(this, &Server::ButtonStopReplaceClicked, graspNode_, &GraspNode::stopReplace);
    connect(this, &Server::ButtonBackClicked, graspNode_, &GraspNode::home);                    // Client to Server
    connect(this, &Server::ButtonBackClicked, this, &Server::changeToSelectScreen);             // Client to Both
    connect(this, &Server::ButtonStowObjectClicked, graspNode_, &GraspNode::stowObject);        // Client to server
    connect(this, &Server::ButtonReleaseClicked, graspNode_, &GraspNode::releaseObject);        // Client to server
    connect(this, &Server::ButtonReplaceObjectClicked, graspNode_, &GraspNode::replaceObject);  // Client to server

    connect(graspNode_, &GraspNode::headSetRotation, moveItNode_, &StretchMoveItInterface::headSetRotation, Qt::BlockingQueuedConnection);    // Server
    connect(graspNode_, &GraspNode::headSetPan, moveItNode_, &StretchMoveItInterface::headSetPan, Qt::BlockingQueuedConnection);              // Server
    connect(graspNode_, &GraspNode::headSetTilt, moveItNode_, &StretchMoveItInterface::headSetTilt, Qt::BlockingQueuedConnection);            // Server
    connect(graspNode_, &GraspNode::armSetHeight, moveItNode_, &StretchMoveItInterface::armSetHeight, Qt::BlockingQueuedConnection);          // Server
    connect(graspNode_, &GraspNode::armSetReach, moveItNode_, &StretchMoveItInterface::armSetReach, Qt::BlockingQueuedConnection);            // Server
    connect(graspNode_, &GraspNode::gripperSetRotate, moveItNode_, &StretchMoveItInterface::gripperSetRotate, Qt::BlockingQueuedConnection);  // Server
    connect(graspNode_, &GraspNode::gripperSetGrip, moveItNode_, &StretchMoveItInterface::gripperSetGrip, Qt::BlockingQueuedConnection);
    connect(graspNode_, &GraspNode::enableMapping, mapNode_, &MapSubscriber::enableMapping, Qt::BlockingQueuedConnection);
    connect(graspNode_, &GraspNode::disableMapping, mapNode_, &MapSubscriber::disableMapping, Qt::BlockingQueuedConnection);
    connect(graspNode_, &GraspNode::homeRobot, moveItNode_, &StretchMoveItInterface::homeRobot, Qt::BlockingQueuedConnection);
    connect(graspNode_, &GraspNode::navigate, mapNode_, &MapSubscriber::moveRobotLoc);
    connect(graspNode_, &GraspNode::navigateHome, mapNode_, &MapSubscriber::navigateHome);
    connect(graspNode_, &GraspNode::graspDone, this, &Server::uiButtonReturnObjectSetEnabled);  // Server to client
    connect(graspNode_, &GraspNode::turnLeft, mapNode_, &MapSubscriber::rotateLeft);
    connect(graspNode_, &GraspNode::moving, moveBaseStatusNode_, &MoveBaseStatus::robotMovingSlot, Qt::BlockingQueuedConnection);
}

Server::~Server() {
    mapNode_->quit();
    moveBaseStatusNode_->quit();
    cameraNode_->quit();
    graspNode_->quit();
    moveItNode_->quit();
    mapNode_->wait();
    moveBaseStatusNode_->wait();
    cameraNode_->wait();
    graspNode_->wait();
    moveItNode_->wait();

    delete mapNode_;
    delete moveBaseStatusNode_;
    delete cameraNode_;
    delete graspNode_;
    delete moveItNode_;
}

void Server::changeToNavigation() {
    emit cameraSetRotation(0, -30);
    emit enableMapping();
    headPanTilt_ = {0, -30};
}
void Server::changeToSelectScreen() {
    emit disableMapping();
    emit cameraSetRotation(headPanTilt_.first, headPanTilt_.second);
    setPageNumber_(0);
}
void Server::changeToConfirmObject() {
    headPanTilt_ = moveItNode_->getHeadPanTilt();
    emit disableMapping();
    setPageNumber_(1);
}
void Server::changeToGrasping() {
    emit disableMapping();
    setPageNumber_(2);
}

void Server::uiGrasp() {
    switch (pageNumber_()) {
        case 0:
            changeToSelectScreen();
            break;
        case 2:
            changeToGrasping();
            break;
    }
}

// Navigate Page

void Server::uiButtonStopClicked() { emit ButtonStopClicked(); }
void Server::uiButtonSetHomeClicked() { emit ButtonSetHomeClicked(); }
void Server::uiButtonNavigateHomeClicked() { emit ButtonNavigateHomeClicked(); }
void Server::uiDisplayMapMouseClick(QPoint press, QPoint release, QSize screen) { emit DisplayMapMouseClick(press, release, screen); }

// Object Selection Page

void Server::uiCameraMoveButtonUpClicked() { emit CameraMoveButtonUpClicked(); }
void Server::uiCameraMoveButtonDownClicked() { emit CameraMoveButtonDownClicked(); }
void Server::uiCameraMoveButtonLeftClicked() { emit CameraMoveButtonLeftClicked(); }
void Server::uiCameraMoveButtonRightClicked() { emit CameraMoveButtonRightClicked(); }
void Server::uiCameraMoveButtonHomeClicked() { emit CameraMoveButtonHomeClicked(); }
void Server::uiDisplayCameraMouseClicked(QPoint press, QPoint release, QSize screen) { emit DisplayCameraMouseClicked(press, release, screen); }

// Confirm Selection Page
void Server::setVertical() { emit SetVertical(); }
void Server::setHorizontal() { emit SetHorizontal(); }

void Server::uiConfirmButtonNoClicked() { emit ConfirmButtonNoClicked(); }
void Server::uiConfirmButtonYesClicked() { emit ConfirmButtonYesClicked(); }

// Grasp Page
void Server::uiButtonStopReplaceClicked() {
    ROS_INFO_STREAM("Server got it");
    emit ButtonStopReplaceClicked();
}
void Server::uiButtonBackClicked() { emit ButtonBackClicked(); }
void Server::uiButtonStowObjectClicked() { emit ButtonStowObjectClicked(); }
void Server::uiButtonReleaseClicked() { emit ButtonReleaseClicked(); }
void Server::uiButtonReplaceObjectClicked() { emit ButtonReplaceObjectClicked(); }
void Server::uiButtonNavigateClicked() { emit ButtonNavigateClicked(); }
