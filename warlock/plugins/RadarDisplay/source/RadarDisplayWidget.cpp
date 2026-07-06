// RadarDisplayWidget.cpp
#include "RadarDisplayWidget.hpp"
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDateTime>
#include <QRandomGenerator>
#include <QDebug>
#include "WkfVtkEnvironment.hpp"
#include "WkPlatform.hpp"
#include "WkfScenario.hpp"
#include "WkfViewer.hpp"
#include "WkfPlatform.hpp"
namespace RadarDisplay
{
    RadarDisplayWidget::RadarDisplayWidget(QWidget* parent, Qt::WindowFlags f)
        : QDockWidget(parent, f)
        , m_udpSocket(nullptr)
        , m_timer(nullptr)
    {
        setObjectName("RadarDisplayUDP");
        setupUdp();

        // ´´½¨¶¨Ê±Æ÷£¬Ã¿Ãë´¥·¢Ò»´Î
        m_timer = new QTimer(this);
        connect(m_timer, &QTimer::timeout, this, &RadarDisplayWidget::sendVirtualData);
        m_timer->start(1000);
    }

    RadarDisplayWidget::~RadarDisplayWidget()
    {
        if (m_timer) {
            m_timer->stop();
        }
        if (m_udpSocket) {
            m_udpSocket->close();
        }
    }

    void RadarDisplayWidget::setupUdp()
    {
        m_udpSocket = new QUdpSocket(this);
        m_sendAddress = QHostAddress::LocalHost;  // 127.0.0.1
        m_sendPort = SEND_PORT;

        if (!m_udpSocket->bind(QHostAddress::AnyIPv4, RECV_PORT, QUdpSocket::ShareAddress)) {
            qDebug() << "RadarDisplay UDP: Failed to bind port" << RECV_PORT;
        }
        else {
            connect(m_udpSocket, &QUdpSocket::readyRead, this, &RadarDisplayWidget::processPendingDatagrams);
            qDebug() << "RadarDisplay UDP: Listening on port" << RECV_PORT;
        }
    }

    void RadarDisplayWidget::processPendingDatagrams()
    {
        while (m_udpSocket->hasPendingDatagrams()) {
            QByteArray datagram;
            datagram.resize(m_udpSocket->pendingDatagramSize());
            QHostAddress senderAddr;
            quint16 senderPort;
            m_udpSocket->readDatagram(datagram.data(), datagram.size(), &senderAddr, &senderPort);

            qDebug() << "RadarDisplay UDP received from" << senderAddr.toString() << senderPort
                << ":" << datagram;

            if (datagram.startsWith("1001")) {
                qDebug() << "RadarDisplay: trigger received, enabling sensor data send.";
                emit sendTriggered();
            }

            // ¿ÉÑ¡£º½âÎöJSON²¢´òÓ¡¹Ø¼ü×Ö¶Î£¨½ö×÷ÑÝÊ¾£©
            QJsonParseError parseError;
            QJsonDocument doc = QJsonDocument::fromJson(datagram, &parseError);
            if (parseError.error == QJsonParseError::NoError && doc.isObject()) {
                QJsonObject obj = doc.object();
                if (obj.contains("radar_name")) {
                    qDebug() << "  -> radar_name:" << obj["radar_name"].toString();
                }
            }
        }
    }

    void RadarDisplayWidget::sendDataOnce()
    {
        QJsonArray dataArray2;
        dataArray2.append("1001");
        dataArray2.append("ship");
        dataArray2.append("ACQ_RADAR");
        dataArray2.append("111.727");
        dataArray2.append("21.5001");
        dataArray2.append("0.3048");
        QJsonDocument doc2(dataArray2);
        //QByteArray datagram2 = doc2.toJson(QJsonDocument::Compact);
        //sendUdpData(datagram2, m_sendAddress, m_sendPort);

        QJsonArray dataArray1;
        dataArray1.append("1002");
        dataArray1.append("9.7e+09");
        dataArray1.append("1e+06");
        dataArray1.append("1e+06");
        dataArray1.append("8e-06");
        dataArray1.append("2");
        dataArray1.append("20");
        dataArray1.append("1");
        dataArray1.append("15.9447");
        QJsonDocument doc1(dataArray1);
        QByteArray datagram1 = doc1.toJson(QJsonDocument::Compact);
        //sendUdpData(datagram1, m_sendAddress, m_sendPort);
    }

    void RadarDisplayWidget::sendVirtualData()
    {
        QJsonArray dataArray3;
        dataArray3.append("1004");
        dataArray3.append("target_3");
        dataArray3.append("111.903");
        dataArray3.append("21.5449");
        dataArray3.append("30.48");
        dataArray3.append("0");
        QJsonDocument doc3(dataArray3);
        QByteArray datagram3 = doc3.toJson(QJsonDocument::Compact);
        //sendUdpData(datagram3, m_sendAddress, m_sendPort);
    }

    void RadarDisplayWidget::sendUdpData(const QByteArray& data, const QHostAddress& address, quint16 port)
    {
        if (!m_udpSocket) return;
        qint64 sent = m_udpSocket->writeDatagram(data, address, port);
        if (sent == -1) {
            qDebug() << "RadarDisplay UDP: Failed to send data to" << address.toString() << port;
        }
        else {
            // ¿ÉÑ¡£º¼õÉÙÈÕÖ¾Êä³öÒÔ±ÜÃâË¢ÆÁ£¬´Ë´¦ÎªÁËµ÷ÊÔ±£Áô£¬ÓÃ»§¿É×¢ÊÍ
            // qDebug() << "RadarDisplay UDP: Sent" << sent << "bytes to" << address.toString() << port;
        }
	
		
		
    }
}