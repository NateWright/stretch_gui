#include <QCoreApplication>
#include <QHostAddress>
#include <QHostInfo>
#include <QList>
#include <QNetworkInterface>
#include <QString>
#include <csignal>

#include "Server.hpp"

QCoreApplication *app;

void closeApplication(int signal) {
    if (app) {
        app->quit();
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "stretch_gui_server");
    ros::NodeHandlePtr nh(new ros::NodeHandle("stretch_gui_server"));
    app = new QCoreApplication(argc, argv);
    Server server(nh);

    // https://www.codegrepper.com/code-examples/whatever/qt+get+local+ip+address
    QString localhostname = QHostInfo::localHostName();
    QString ip = "tcp://";
    // QList<QHostAddress> hostList = QNetworkInterface::allAddresses();
    // for (const QHostAddress &address : hostList) {
    //     if (address.protocol() == QAbstractSocket::IPv4Protocol && address.isLoopback() == false) {
    //         ip += address.toString();
    //         break;
    //     }
    // }
    std::string s;
    nh->getParam("/stretch_gui/ip", s);
    ip += s;
    qDebug() << "Localhost name: " << localhostname;
    ip += ":9999";
    qDebug() << "IP = " << ip;
    QRemoteObjectHost srcNode(QUrl(ip), nullptr);
    srcNode.enableRemoting(&server);
    qDebug() << "start";

    signal(SIGINT, closeApplication);

    return app->exec();
}
