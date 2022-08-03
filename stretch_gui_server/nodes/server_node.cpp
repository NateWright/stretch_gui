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
    app = new QCoreApplication(argc, argv);
    Server server;

    // https://www.codegrepper.com/code-examples/whatever/qt+get+local+ip+address
    QString localhostname = QHostInfo::localHostName();
    QString ip = "tcp://";
    QList<QHostAddress> hostList = QNetworkInterface::allAddresses();
    for (const QHostAddress &address : hostList) {
        if (address.protocol() == QAbstractSocket::IPv4Protocol && address.isLoopback() == false) {
            ip += address.toString();
            break;
        }
    }
    qDebug() << "Localhost name: " << localhostname;
    ip += ":stretch";
    qDebug() << "IP = " << ip;
    QRemoteObjectHost srcNode(QUrl(ip), nullptr);
    srcNode.enableRemoting(&server);
    qDebug() << "start";

    signal(SIGINT, closeApplication);

    return app->exec();
}
