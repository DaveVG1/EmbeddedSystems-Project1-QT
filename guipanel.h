#ifndef GUIPANEL_H
#define GUIPANEL_H

#include <QWidget>
#include <QtSerialPort/qserialport.h>
#include "qremotetiva.h"
#include <qwt.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_grid.h>

namespace Ui {
class GUIPanel;
}

//QT4:QT_USE_NAMESPACE_SERIALPORT

class GUIPanel : public QWidget
{
    Q_OBJECT
    
public:
    //GUIPanel(QWidget *parent = 0);
    explicit GUIPanel(QWidget *parent = 0);
    ~GUIPanel(); // Da problemas
    
private slots:
    //void on_pingButton_clicked();
    void on_runButton_clicked();
    //void readRequest();
    void on_serialPortComboBox_currentIndexChanged(const QString &arg1);

    void on_rojo_stateChanged(int arg1);
    void on_verde_stateChanged(int arg1);
    void on_azul_stateChanged(int arg1);

    void on_pushButton_clicked();    

    void tivaStatusChanged(int status,QString message);
    void pingResponseReceived(void);
    void switchResponseReceived(bool sw1, bool sw2); // Encendemos LED cuando recibimos la señal de la TIVA
    void encenderLedsADC(bool pb0, bool pb1, bool pb2, bool pb3, bool ain0, bool ain1, bool ain2, bool ain3);
    void CommandRejected(int16_t code);

    void on_colorWheel_colorChanged(const QColor &arg1);

    void on_PB0CheckBox_stateChanged(int arg1);
    void on_PB1Checkbox_stateChanged(int arg1);
    void on_PB2Checkbox_stateChanged(int arg1);
    void on_PB3Checkbox_stateChanged(int arg1);

    void timerDone();

    void on_oscCheckBox_clicked();

    void on_frecSpinBox_valueChanged(int arg1);


    void cambiarGrafica(uint8_t *v0, uint8_t *v1, uint8_t *v2, uint8_t *v3);

    void on_escalaSpinBox_valueChanged(int arg1);

    void on_monitorCheckBox_clicked();

    void on_AIN0Knob_valueChanged(double value);

    void on_AIN1Knob_valueChanged(double value);

    void on_AIN2Knob_valueChanged(double value);

    void on_AIN3Knob_valueChanged(double value);

    void on_hourPushButton_clicked();

    void clockChange(int hora, int min, int sec);

private: // funciones privadas
    void pingDevice();
    //void startSlave();
    void processError(const QString &s);
    void activateRunButton();
    void cambiaLEDs();
    void enciendeLED(bool sw1, bool sw2);
    void cambiaDin();
    void cambiaLevel();
private:
    Ui::GUIPanel *ui;
    QTimer  *timer;
    int counter;
    int transactionCount;
    QwtPlotCurve *m_curve0;
    QwtPlotCurve *m_curve1;
    QwtPlotCurve *m_curve2;
    QwtPlotCurve *m_curve3;
    QwtPlotGrid  *m_Grid;
    double xVal[256];
    double yVal0[256];
    double yVal1[256];
    double yVal2[256];
    double yVal3[256];
    QRemoteTIVA tiva;
};

#endif // GUIPANEL_H
