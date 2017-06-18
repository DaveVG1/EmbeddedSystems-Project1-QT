#include "guipanel.h"
#include "ui_guipanel.h"
#include <QSerialPort>      // Comunicacion por el puerto serie
#include <QSerialPortInfo>  // Comunicacion por el puerto serie
#include <QMessageBox>      // Se deben incluir cabeceras a los componentes que se vayan a crear en la clase
// y que no estén incluidos en el interfaz gráfico. En este caso, la ventana de PopUp <QMessageBox>
// que se muestra al recibir un PING de respuesta
#include <QTimer>
#include <QTime>

#include<stdint.h>      // Cabecera para usar tipos de enteros con tamaño
#include<stdbool.h>     // Cabecera para usar booleanos

extern "C" {
#include "protocol.h"    // Cabecera de funciones de gestión de tramas; se indica que está en C, ya que QTs
// se integra en C++, y el C puede dar problemas si no se indica.
}

GUIPanel::GUIPanel(QWidget *parent) :  // Constructor de la clase
    QWidget(parent),
    ui(new Ui::GUIPanel)               // Indica que guipanel.ui es el interfaz grafico de la clase
  , transactionCount(0)
{
    ui->setupUi(this);                // Conecta la clase con su interfaz gráfico.
    timer = new QTimer(this);
    setWindowTitle(tr("Interfaz de Control")); // Título de la ventana

    // Inicializa la lista de puertos serie
    ui->serialPortComboBox->clear(); // Vacía de componentes la comboBox
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
    {
        // La descripción nos permite que SOLO aparezcan los interfaces tipo USB serial con el identificador de fabricante y producto de la TIVA
        if ((info.vendorIdentifier()==0x1CBE) && (info.productIdentifier()==0x0002))
        {
            ui->serialPortComboBox->addItem(info.portName());
        }
    }

    ui->serialPortComboBox->setFocus();   // Componente del GUI seleccionado de inicio

    ui->modeComboBox->addItem("PWM");
    ui->modeComboBox->addItem("GPIO");
    ui->modeComboBox->setFocus();

    ui->pingButton->setEnabled(false);
    ui->switchButton->setEnabled(false);
    ui->SW1->setChecked(false);
    ui->SW2->setChecked(false);
    ui->AIN0Led->setChecked(false);
    ui->AIN1Led->setChecked(false);
    ui->AIN2Led->setChecked(false);
    ui->AIN3Led->setChecked(false);
    ui->PB0Led->setChecked(false);
    ui->PB1Led->setChecked(false);
    ui->PB2Led->setChecked(false);
    ui->PB3Led->setChecked(false);
    // Se deshabilita el botón de ping del interfaz gráfico, hasta que
    //Conectamos Slots del objeto "Tiva" con Slots de nuestra aplicacion (o con widgets)

    // Primera parte
    connect(&tiva,SIGNAL(statusChanged(int,QString)),this,SLOT(tivaStatusChanged(int,QString)));
    connect(ui->pingButton,SIGNAL(clicked(bool)),&tiva,SLOT(pingTiva()));
    connect(ui->Knob,SIGNAL(valueChanged(double)),&tiva,SLOT(LEDPwmBrightness(double)));
    connect(ui->switchButton,SIGNAL(clicked(bool)),&tiva,SLOT(detectaPulsadores()));
    connect(&tiva,SIGNAL(pingReceivedFromTiva()),this,SLOT(pingResponseReceived()));
    connect(&tiva,SIGNAL(switchReceivedFromTiva(bool, bool)),this,SLOT(switchResponseReceived(bool, bool)));
    connect(&tiva,SIGNAL(commandRejectedFromTiva(int16_t)),this,SLOT(CommandRejected(int16_t)));
    connect(ui->modeComboBox,SIGNAL(activated(int)),&tiva,SLOT(modeChange(int)));
    connect(ui->RTIcheckBox,SIGNAL(clicked(bool)),&tiva,SLOT(RTIChange(bool)));

    // Segunda parte
    // ADC
    connect(&tiva, SIGNAL(ADCReceivedFromTiva(bool, bool, bool, bool, bool, bool, bool, bool)),
            this, SLOT(encenderLedsADC(bool, bool, bool, bool, bool, bool, bool, bool)));
    // Timer
    connect( timer, SIGNAL(timeout()), SLOT(timerDone()) );
    // Osciloscopio
    connect(&tiva, SIGNAL(oscReceivedFromTiva(uint8_t*,uint8_t*,uint8_t*,uint8_t*)),
                          this, SLOT(cambiarGrafica(uint8_t*,uint8_t*,uint8_t*,uint8_t*)));

    // Tercera parte
    connect(ui->receiveHourPushButton, SIGNAL(clicked()), &tiva, SLOT(receiveHour()));
    connect(&tiva, SIGNAL(hourReceivedFromTiva(int, int, int)), this, SLOT(clockChange(int, int, int)));

}

GUIPanel::~GUIPanel() // Destructor de la clase
{
    delete ui;   // Borra el interfaz gráfico asociado a la clase
}

void GUIPanel::on_serialPortComboBox_currentIndexChanged(const QString &arg1)
{
    activateRunButton();
}

// Funcion auxiliar de procesamiento de errores de comunicación
void GUIPanel::processError(const QString &s)
{
    activateRunButton(); // Activa el botón RUN
    // Muestra en la etiqueta de estado la razón del error (notese la forma de pasar argumentos a la cadena de texto)
    ui->statusLabel->setText(tr("Status: Not running, %1.").arg(s));
}

// Funcion de habilitacion del boton de inicio/conexion
void GUIPanel::activateRunButton()
{
    ui->runButton->setEnabled(true);
}

//Este Slot lo hemos conectado con la señal statusChange de la TIVA, que se activa cuando
//El puerto se conecta o se desconecta y cuando se producen determinados errores.
//Esta función lo que hace es procesar dichos eventos
void GUIPanel::tivaStatusChanged(int status,QString message)
{
    switch (status)
    {
        case QRemoteTIVA::TivaConnected:

            //Caso conectado..
            // Deshabilito el boton de conectar
            ui->runButton->setEnabled(false);

            // Se indica que se ha realizado la conexión en la etiqueta 'statusLabel'
            ui->statusLabel->setText(tr("Ejecucion, conectado al puerto %1.")
                             .arg(ui->serialPortComboBox->currentText()));

            //    // Y se habilitan los controles deshabilitados
            ui->pingButton->setEnabled(true);
            ui->switchButton->setEnabled(true);

        break;

        case QRemoteTIVA::TivaDisconnected:
            //Caso desconectado..
            // Rehabilito el boton de conectar
            ui->runButton->setEnabled(false);
        break;
        case QRemoteTIVA::UnexpectedPacketError:
        case QRemoteTIVA::FragmentedPacketError:
            //Errores detectados en la recepcion de paquetes
            ui->statusLabel->setText(message);
        default:
            //Otros errores (por ejemplo, abriendo el puerto)
            processError(tiva.getLastErrorMessage());
    }
}


// SLOT asociada a pulsación del botón RUN
void GUIPanel::on_runButton_clicked()
{
    //Intenta arrancar la comunicacion con la TIVA
    tiva.startSlave( ui->serialPortComboBox->currentText());
}

//Slot asociado al chechbox rojo (por nombre)
void GUIPanel::on_rojo_stateChanged(int arg1)
{
    cambiaLEDs();
}

//Slot asociado al chechbox verde (por nombre)
void GUIPanel::on_verde_stateChanged(int arg1)
{
    cambiaLEDs();
}

//Slot asociado al chechbox azul (por nombre)
void GUIPanel::on_azul_stateChanged(int arg1)
{
    cambiaLEDs();
}

void GUIPanel::cambiaLEDs(void)
{
    //Invoca al metido LEDsGpioTiva para enviar la orden a la TIVA
    tiva.LEDsGpioTiva(ui->rojo->isChecked(),ui->verde->isChecked(),ui->azul->isChecked());
}

//====================================================
// Esta funcion enciende los LEDs de la pantalla según
// dos parámetros que se les pasen.
void GUIPanel::switchResponseReceived(bool sw1, bool sw2){
    ui->SW1->setChecked(!sw1);
    ui->SW2->setChecked(!sw2);
}

//Slots Asociado al boton que limpia los mensajes del interfaz
void GUIPanel::on_pushButton_clicked()
{
    ui->statusLabel->setText(tr(""));
}

//**** Slots asociados a la recepción de mensajes desde la TIVA ********/
//Están conectados a señales generadas por el objeto TIVA de clase QRemoteTiva (se conectan en el constructor de la ventana, más arriba en este fichero))
//Se pueden añadir los que se quieran para completar la aplicación

//Este se ejecuta cuando se recibe una respuesta de PING
void GUIPanel::pingResponseReceived()
{

    // Ventana popUP para el caso de comando PING; no te deja definirla en un "caso"
    QMessageBox ventanaPopUp(QMessageBox::Information,tr("Evento"),tr("Status: RESPUESTA A PING RECIBIDA"),QMessageBox::Ok,this,Qt::Popup);
    ventanaPopUp.setStyleSheet("background-color: lightgrey");
    ventanaPopUp.exec();
}


//Este se ejecuta cuando se recibe un mensaje de comando rechazado
void GUIPanel::CommandRejected(int16_t code)
{
    ui->statusLabel->setText(tr("Status: Comando rechazado,%1").arg(code));
}


void GUIPanel::on_colorWheel_colorChanged(const QColor &arg1)
{
    //Poner aqui el codigo para pedirle al objeto "tiva" (clase QRemoteTIVA) que envíe la orden de cambiar el Color hacia el microcontrolador
    tiva.LEDsColorWheel(arg1.red(), arg1.green(), arg1.blue());

}

// ENTRADAS ANALÓGICAS Y DIGITALES

void GUIPanel::on_PB0CheckBox_stateChanged(int arg1)
{
    cambiaDin();
}

void GUIPanel::on_PB1Checkbox_stateChanged(int arg1)
{
    cambiaDin();
}

void GUIPanel::on_PB2Checkbox_stateChanged(int arg1)
{
    cambiaDin();
}

void GUIPanel::on_PB3Checkbox_stateChanged(int arg1)
{
    cambiaDin();
}

void GUIPanel::on_AIN0Knob_valueChanged(double value)
{
    cambiaLevel();
}

void GUIPanel::on_AIN1Knob_valueChanged(double value)
{
    cambiaLevel();
}

void GUIPanel::on_AIN2Knob_valueChanged(double value)
{
    cambiaLevel();
}

void GUIPanel::on_AIN3Knob_valueChanged(double value)
{
    cambiaLevel();
}

void GUIPanel::cambiaLevel()
{
    tiva.ADCChange(ui->PB0CheckBox->isChecked(), ui->PB1Checkbox->isChecked(), ui->PB2Checkbox->isChecked(),
                   ui->PB3Checkbox->isChecked(), ui->monitorCheckBox->isChecked(),
                   ui->AIN0Knob->value(), ui->AIN1Knob->value(), ui->AIN2Knob->value(), ui->AIN3Knob->value());
}

void GUIPanel::on_monitorCheckBox_clicked()
{
    tiva.ADCChange(ui->PB0CheckBox->isChecked(), ui->PB1Checkbox->isChecked(), ui->PB2Checkbox->isChecked(),
                   ui->PB3Checkbox->isChecked(), ui->monitorCheckBox->isChecked(),
                   ui->AIN0Knob->value(), ui->AIN1Knob->value(), ui->AIN2Knob->value(), ui->AIN3Knob->value());
}


void GUIPanel::cambiaDin()
{
    tiva.ADCChange(ui->PB0CheckBox->isChecked(), ui->PB1Checkbox->isChecked(), ui->PB2Checkbox->isChecked(),
                   ui->PB3Checkbox->isChecked(), ui->monitorCheckBox->isChecked(),
                   ui->AIN0Knob->value(), ui->AIN1Knob->value(), ui->AIN2Knob->value(), ui->AIN3Knob->value());
}

void GUIPanel::encenderLedsADC(bool pb0, bool pb1, bool pb2, bool pb3, bool ain0, bool ain1, bool ain2, bool ain3)
{
    ui->PB0Led->setChecked(pb0);
    ui->PB1Led->setChecked(pb1);
    ui->PB2Led->setChecked(pb2);
    ui->PB3Led->setChecked(pb3);
    ui->AIN0Led->setChecked(ain0);
    ui->AIN1Led->setChecked(ain1);
    ui->AIN2Led->setChecked(ain2);
    ui->AIN3Led->setChecked(ain3);
    timer->start(10000);
}

void GUIPanel::timerDone()
{
    ui->PB0Led->setChecked(false);
    ui->PB1Led->setChecked(false);
    ui->PB2Led->setChecked(false);
    ui->PB3Led->setChecked(false);
    ui->AIN0Led->setChecked(false);
    ui->AIN1Led->setChecked(false);
    ui->AIN2Led->setChecked(false);
    ui->AIN3Led->setChecked(false);
}

// OSCILOSCOPIO

void GUIPanel::on_oscCheckBox_clicked()
{
    if (ui->oscCheckBox->isChecked()){
        //Gráfica----------------

            ui->grafica->setTitle("Osciloscopio"); // Titulo de la grafica
            ui->grafica->setAxisTitle(QwtPlot::xBottom, "Tiempo"); // Etiqueta eje X de coordenadas
            ui->grafica->setAxisTitle(QwtPlot::yLeft, "Valor");    // Etiqueta eje Y de coordenadas
            //ui->grafica->axisAutoScale(true); // Con Autoescala
            ui->grafica->setAxisScale(QwtPlot::yLeft, 0, 256); // Con escala definida
            ui->grafica->setAutoReplot(true);

            // Formateo de la curva
            m_curve0 = new QwtPlotCurve();
            m_curve1 = new QwtPlotCurve();
            m_curve2 = new QwtPlotCurve();
            m_curve3 = new QwtPlotCurve();

            m_curve0->setPen(QPen(Qt::red)); // Color de la curva
            m_curve1->setPen(QPen(Qt::blue)); // Color de la curva
            m_curve2->setPen(QPen(Qt::green)); // Color de la curva
            m_curve3->setPen(QPen(Qt::yellow)); // Color de la curva

            m_Grid = new QwtPlotGrid();     // Rejilla de puntos
            m_Grid->attach(ui->grafica);    // que se asocia al objeto QwtPlot

            m_curve0->attach(ui->grafica);  // Curva se asocia al objeto QwtPlot
            m_curve1->attach(ui->grafica);  // Curva se asocia al objeto QwtPlot
            m_curve2->attach(ui->grafica);  // Curva se asocia al objeto QwtPlot
            m_curve3->attach(ui->grafica);  // Curva se asocia al objeto QwtPlot

            for (int i=0; i<256; i++)
                xVal[i]=i;

            tiva.cambiar_frecuencia_muestreo(ui->oscCheckBox->isChecked(), ui->frecSpinBox->value());

    } else {
        tiva.cambiar_frecuencia_muestreo(ui->oscCheckBox->isChecked(), ui->frecSpinBox->value());
    }
}

void GUIPanel::on_frecSpinBox_valueChanged(int arg1)
{
    tiva.cambiar_frecuencia_muestreo(ui->oscCheckBox->isChecked(), arg1);
}

void GUIPanel::cambiarGrafica(uint8_t *v0, uint8_t *v1, uint8_t *v2, uint8_t *v3)
{
    for (int i=0; i<10; i++) {
        for (int j=ui->escalaSpinBox->value()-1; j>=0; j--) { // Elimina la muestra mas antigua

            yVal0[j]=yVal0[j-1];
            yVal1[j]=yVal1[j-1];
            yVal2[j]=yVal2[j-1];
            yVal3[j]=yVal3[j-1];

        }
    }
    for (int i=0; i<10; i++) {
        yVal0[i]=v0[i]; // Y mete la nueva
        yVal1[i]=v1[i]; // Y mete la nueva
        yVal2[i]=v2[i]; // Y mete la nueva
        yVal3[i]=v3[i]; // Y mete la nueva
    }

    //ui->valor->setText(tr("Val:%1").arg(yVal[99]));
    m_curve0->setRawSamples(xVal,yVal0,ui->escalaSpinBox->value()); // Actualiza los valores de la curva (y se repinta sola)
    m_curve1->setRawSamples(xVal,yVal1,ui->escalaSpinBox->value()); // Actualiza los valores de la curva (y se repinta sola)
    m_curve2->setRawSamples(xVal,yVal2,ui->escalaSpinBox->value()); // Actualiza los valores de la curva (y se repinta sola)
    m_curve3->setRawSamples(xVal,yVal3,ui->escalaSpinBox->value()); // Actualiza los valores de la curva (y se repinta sola)
}

void GUIPanel::on_escalaSpinBox_valueChanged(int arg1)
{
    ui->grafica->setAxisScale(QwtPlot::xBottom,0,ui->escalaSpinBox->value());
}

// Tercera parte

void GUIPanel::on_hourPushButton_clicked()
{
    tiva.enviar_hora();
}

void GUIPanel::clockChange(int hora, int min, int sec)
{
    QTime time(hora, min, sec);
    ui->timeEdit->setTime(time);
}
