// RadarDisplayWidget.hpp
#ifndef RADARDISPLAYWIDGET_HPP
#define RADARDISPLAYWIDGET_HPP

#include <QDockWidget>
#include <QUdpSocket>
#include <QHostAddress>
#include <QTimer>

#include "WsfSimulation.hpp"
#include "VaEnvironment.hpp"
namespace RadarDisplay
{
    class RadarDisplayWidget : public QDockWidget
    {
        Q_OBJECT

    public:
        explicit RadarDisplayWidget(QWidget* parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags());
        ~RadarDisplayWidget() override;

        //·¢ËÍ×Ô¶¨ÒåUDPÊý¾Ý
        void sendUdpData(const QByteArray& data, const QHostAddress& address, quint16 port);
        void sendDataOnce();   // Ö»·¢ËÍÒ»´Î

		
    signals:
        void sendTriggered();

    private slots:
        void processPendingDatagrams();//½ÓÊÕ

        void sendVirtualData();   // Ã¿Ãë·¢ËÍ

    private:
        void setupUdp();

        QUdpSocket* m_udpSocket;
        QHostAddress m_sendAddress;   // Ä¬ÈÏ·¢ËÍµØÖ·
        quint16 m_sendPort;           // Ä¬ÈÏ·¢ËÍ¶Ë¿Ú
        QTimer* m_timer;              // ¶¨Ê±Æ÷£¬ÓÃÓÚÃ¿Ãë·¢ËÍ

        static constexpr quint16 RECV_PORT = 12345;   // ½ÓÊÕ¶Ë¿Ú
        static constexpr quint16 SEND_PORT = 12346;   // ·¢ËÍ¶Ë¿Ú
    };
}

#endif // RADARDISPLAYWIDGET_HPP