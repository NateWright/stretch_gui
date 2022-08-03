#ifndef SCENEVIEWER_H
#define SCENEVIEWER_H

#include <QDebug>
#include <QLabel>
#include <QMouseEvent>
#include <QObject>
#include <QPoint>
#include <QSize>
#include <QWidget>
#include <QImage>
#include <QFuture>
#include <QtConcurrent/QtConcurrent>

class SceneViewer : public QLabel {
    Q_OBJECT
   public:
    SceneViewer(QWidget* parent = 0) : QLabel(parent) {}
    ~SceneViewer() {}

   private:
    QPoint press;
    QPoint release;
   signals:
    void mouseClick(QPoint press, QPoint release, QSize screen);
    void mousePressInitiated(QPoint press, QSize screen);
    void mousePressCurrentLocation(QPoint loc, QSize screen);
   public slots:
    void setMap(const QPixmap& map){
        setPixmap(map.scaled(size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
    void setMapQImage(const QImage img){
      setPixmap(QPixmap::fromImage(img));
    }
    void setCamera(const QPixmap& pix){
        resize(pix.width(), pix.height());
        setPixmap(pix);
    }
    void setCameraQImage(const QImage img){
      setCamera(QPixmap::fromImage(img));
    }

   protected:
    virtual void mousePressEvent(QMouseEvent* event){
        press = event->pos();
        setMouseTracking(true);
        emit mousePressInitiated(press, frameSize());
    }
    virtual void mouseReleaseEvent(QMouseEvent* event){
        release = event->pos();
        setMouseTracking(false);
        emit mouseClick(press, release, frameSize());
    }
    virtual void mouseMoveEvent(QMouseEvent* event){
        emit mousePressCurrentLocation(event->pos(), frameSize());
    }
};

#endif  // SCENEVIEWER_H
