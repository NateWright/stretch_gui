#pragma once

#include <QBuffer>
#include <QByteArray>
#include <QObject>
#include <QPixmap>
#include <QPoint>
#include <QSize>
#include <QTimer>
#include <stretch_gui_library/CameraNode.hpp>
#include <stretch_gui_library/GraspNode.hpp>
#include <stretch_gui_library/MapNode.hpp>
#include <stretch_gui_library/MoveBaseStatusNode.hpp>
#include <stretch_gui_library/StretchInterface.hpp>

#include "rep_Server_source.h"

class Server : public ServerSimpleSource {
    Q_OBJECT
   public:
    explicit Server(QObject *parent = nullptr);
    ~Server();

    void uiGrasp();

    // Navigate Page
    void uiButtonStopClicked();
    void uiButtonSetHomeClicked();
    void uiButtonNavigateHomeClicked();
    void uiDisplayMapMouseClick(QPoint press, QPoint release, QSize screen);

    // Object Selection Page
    void uiCameraMoveButtonUpClicked();
    void uiCameraMoveButtonDownClicked();
    void uiCameraMoveButtonLeftClicked();
    void uiCameraMoveButtonRightClicked();
    void uiCameraMoveButtonHomeClicked();
    void uiDisplayCameraMouseClicked(QPoint press, QPoint release, QSize screen);

    // Confirm Selection Page
    void setVertical();
    void setHorizontal();

    void uiConfirmButtonNoClicked();
    void uiConfirmButtonYesClicked();

    // Grasp Page
    void uiButtonStopReplaceClicked();
    void uiButtonBackClicked();
    void uiButtonStowObjectClicked();
    void uiButtonReleaseClicked();
    void uiButtonReplaceObjectClicked();
    void uiButtonNavigateClicked();

   private:
    ros::NodeHandlePtr nh_;

    MapNode *mapNode_;
    MoveBaseStatus *moveBaseStatusNode_;
    CameraNode *cameraNode_;
    GraspNode *graspNode_;
    StretchInterface *moveItNode_;

    std::pair<int, int> headPanTilt_;

    void initConnections();

   signals:
    void enableMapping();
    void disableMapping();
    void homeRobot();
    void cameraSetRotation(int, int);

    // Navigation Page
    void ButtonStopClicked();
    void ButtonSetHomeClicked();
    void ButtonNavigateHomeClicked();
    void DisplayMapMouseClick(QPoint press, QPoint release, QSize screen);

    // Select Object Page
    void CameraMoveButtonUpClicked();
    void CameraMoveButtonDownClicked();
    void CameraMoveButtonLeftClicked();
    void CameraMoveButtonRightClicked();
    void CameraMoveButtonHomeClicked();
    void DisplayCameraMouseClicked(QPoint press, QPoint release, QSize screen);

    // Object Confirm Page
    void SetVertical();
    void SetHorizontal();

    void ConfirmButtonNoClicked();
    void ConfirmButtonYesClicked();

    // Grasp Page

    void ButtonStopReplaceClicked();
    void ButtonBackClicked();
    void ButtonStowObjectClicked();
    void ButtonReleaseClicked();
    void ButtonReplaceObjectClicked();
    void ButtonNavigateClicked();

   private slots:
    void changeToNavigation();
    void changeToSelectScreen();
    void changeToConfirmObject();
    void changeToGrasping();
};
