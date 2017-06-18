#ifndef QREMOTETIVA_H
#define QREMOTETIVA_H

#include <QObject>
#include <QSerialPort>      // Comunicacion por el puerto serie
#include <QSerialPortInfo>  // Comunicacion por el puerto serie


#include<stdint.h>      // Cabecera para usar tipos de enteros con tamaño
#include<stdbool.h>     // Cabecera para usar booleanos




class QRemoteTIVA : public QObject
{

    Q_OBJECT
public:
    explicit QRemoteTIVA(QObject *parent = 0);

    //Define una serie de etiqueta para los errores y estados notificados por la señal statusChanged(...)
    enum TivaStatus {TivaConnected,
                     TivaDisconnected,
                     OpenPortError,
                     BaudRateError,
                     DataBitError,
                     ParityError,
                     StopError,
                     FlowControlError,
                     UnexpectedPacketError,
                     FragmentedPacketError
                    };
    Q_ENUM(TivaStatus)

    //Metodo publico
    QString getLastErrorMessage();

signals:
    void statusChanged(int status, QString message); //Esta señal se genera al realizar la conexión/desconexion o cuando se produce un error de comunicacion
    void pingReceivedFromTiva(void); //Esta señal se genera al recibir una respuesta de PING de la TIVA
    void switchReceivedFromTiva(bool sw1, bool sw2); //Esta señal se genera al recibir la pulsación de un SW de la TIVA
    void commandRejectedFromTiva(int16_t code); //Esta señal se genera al recibir una respuesta de Comando Rechazado desde la TIVA
    void ADCReceivedFromTiva(bool pb0, bool pb1, bool pb2, bool pb3,
                             bool ain0, bool ain1, bool ain2, bool ain3); // Esta señal se genera al recibir la activación de una entrada

    void oscReceivedFromTiva(uint8_t *valor0, uint8_t *valor1, uint8_t *valor2, uint8_t *valor3);
    void hourReceivedFromTiva(int hora, int min, int sec);
public slots:
    void startSlave(QString puerto); // Este Slot arranca la comunicacion
    void pingTiva(void); // Este Slot provoca el envio del comando PING
    void LEDsGpioTiva(bool red, bool green, bool blue); // Este Slot provoca el envio del comando LED
    void LEDsColorWheel(uint8_t red, uint8_t green, uint8_t blue); // Este Slot provoca el envio del comando LED vía colorWheel
    void LEDPwmBrightness(double value); //Este Slot provoca el envio del comando Brightness
    void detectaPulsadores(); // Este Slot provoca el envio del comando Switch
    void modeChange(int mode); // Este Slot provoca el envio del comando Mode
    void RTIChange(bool ok); // Este Slot provoca el envio del comando ButtonsInt
    // Segunda parte
    void ADCChange(bool pb0, bool pb1, bool pb2, bool pb3, bool mon,
                   double l0, double l1, double l2, double l3); // Este Slot provoca el envio del comando ADC
    void cambiar_frecuencia_muestreo(bool osc, int frec); // Este Slot provoca el envio del comando OSC
    // Tercera parte
    void enviar_hora(void);
    void receiveHour(void);
private slots:
    void readRequest(); //Este Slot se conecta a la señal readyRead(..) del puerto serie. Se encarga de procesar y decodificar los mensajes que llegan de la TIVA y
                        //generar señales para algunos de ellos.

private:
    QSerialPort serial;
    QString LastError;
    bool connected;
    QByteArray request;

};

#endif // QREMOTETIVA_H
