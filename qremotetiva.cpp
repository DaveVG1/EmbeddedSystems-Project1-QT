#include "qremotetiva.h"

#include <QSerialPort>      // Comunicacion por el puerto serie
#include <QSerialPortInfo>  // Comunicacion por el puerto serie
#include <QTime>

extern "C" {
#include "protocol.h"    // Cabecera de funciones de gestión de tramas; se indica que está en C, ya que QTs
// se integra en C++, y el C puede dar problemas si no se indica.
}

QRemoteTIVA::QRemoteTIVA(QObject *parent) : QObject(parent)
{

    connected=false;

    // Las funciones CONNECT son la base del funcionamiento de QT; conectan dos componentes
    // o elementos del sistema; uno que GENERA UNA SEÑAL; y otro que EJECUTA UNA FUNCION (SLOT) al recibir dicha señal.
    // En el ejemplo se conecta la señal readyRead(), que envía el componente que controla el puerto USB serie (serial),
    // con la propia clase PanelGUI, para que ejecute su funcion readRequest() en respuesta.
    // De esa forma, en cuanto el puerto serie esté preparado para leer, se lanza una petición de datos por el
    // puerto serie.El envío de readyRead por parte de "serial" es automatico, sin necesidad de instrucciones
    // del programador
    connect(&serial, SIGNAL(readyRead()), this, SLOT(readRequest()));
}


//Este slot se conecta con la señal readyRead() del puerto serie, que se activa cuando hay algo que leer del puerto serie
//Se encarga de procesar y decodificar los datos que llegan de la TIVA y generar señales en respuesta a algunos de ellos
//Estas señales son capturadas por slots de la clase guipanel en este ejemplo.
void QRemoteTIVA::readRequest()
{
    int posicion,tam;   // Solo uso notacin hungara en los elementos que se van a
    // intercambiar con el micro - para control de tamaño -
    uint8_t *pui8Frame; // Puntero a zona de memoria donde reside la trama recibida
    void *ptrtoparam;
    uint8_t ui8Command; // Para almacenar el comando de la trama entrante

    request.append(serial.readAll()); // Añade el contenido del puerto serie USB al array de bytes 'request'
    // así vamos acumulando  en el array la información que va llegando

    // Busca la posición del primer byte de fin de trama (0xFD) en el array
    posicion=request.indexOf((char)STOP_FRAME_CHAR,0);
    //Metodo poco eficiente pero seguro...
    while (posicion>0)
    {
        pui8Frame=(uint8_t*)request.data(); // Puntero de trama al inicio del array de bytes
        tam=posicion-1;    //Caracter de inicio y fin no cuentan en el tamaño
        // Descarta posibles bytes erroneos que no se correspondan con el byte de inicio de trama
        while (((*pui8Frame)!=(uint8_t)START_FRAME_CHAR)&&(tam>0)) // Casting porque Qt va en C++ (en C no hace falta)
        {
            pui8Frame++;  // Descarta el caracter erroneo
            tam--;    // como parte de la trama recibida
        }
        // A. Disponemos de una trama encapsulada entre dos caracteres de inicio y fin, con un tamaño 'tam'
        if (tam > 0)
        {   //Paquete aparentemente correcto, se puede procesar
            pui8Frame++;  //Quito el byte de arranque (START_FRAME_CHAR, 0xFC)
            //Y proceso normalmente el paquete
            // Paso 1: Destuffing y cálculo del CRC. Si todo va bien, obtengo la trama
            // con valores actualizados y sin bytes de checksum
            tam=destuff_and_check_checksum((unsigned char *)pui8Frame,tam);
            // El tamaño se actualiza (he quitado 2bytes de CRC, mas los de "stuffing")
            if (tam>=0)
            {
                //El paquete está bien, luego procedo a tratarlo.
                ui8Command=decode_command_type(pui8Frame); // Obtencion del byte de Comando
                tam=get_command_param_pointer(pui8Frame,tam,&ptrtoparam);



                switch(ui8Command) // Segun el comando tengo que hacer cosas distintas
                {
                /** A PARTIR AQUI ES DONDE SE DEBEN AÑADIR NUEVAS RESPUESTAS ANTE LOS COMANDOS QUE SE ENVIEN DESDE LA TIVA **/
                case COMANDO_PING:  // Algunos comandos no tiene parametros
                    // Crea una ventana popup con el mensaje indicado
                    emit pingReceivedFromTiva();
                    break;

                    //====================================================
                    //
                    //
                    //
                    //====================================================
                case COMANDO_SWITCH:
                {
                    PARAM_COMANDO_SWITCHES parametro;
                    if (check_and_extract_command_param(ptrtoparam, tam, sizeof(parametro), &parametro) > 0){
                        // Enciendo el LED correspondiente al switch pulsado
                        emit switchReceivedFromTiva(parametro.switches.SW1,parametro.switches.SW2);
                    }
                } break;

                case COMANDO_ADC:
                {
                    PARAM_COMANDO_ADC parametro;
                    if (check_and_extract_command_param(ptrtoparam, tam, sizeof(parametro), &parametro) > 0){
                        // Enciendo el LED correspondiente al switch pulsado
                        emit ADCReceivedFromTiva(parametro.din.PB0, parametro.din.PB1,
                                                 parametro.din.PB2, parametro.din.PB3,
                                                 parametro.anin.AIN0, parametro.anin.AIN1,
                                                 parametro.anin.AIN2, parametro.anin.AIN3);
                    }
                }
                break;
                case COMANDO_RECIBIR_OSCILOSCOPIO:
                {
                    PARAM_COMANDO_RECIBIR_OSCILOSCOPIO parametro;

                    if (check_and_extract_command_param(ptrtoparam, tam, sizeof(parametro), &parametro) > 0){
                        emit oscReceivedFromTiva(parametro.valores.valor0, parametro.valores.valor1,
                                                 parametro.valores.valor2, parametro.valores.valor3);
                    }
                 }
                break;
                case COMANDO_TIME:
                {
                    PARAM_COMANDO_TIME parametro;
                    if (check_and_extract_command_param(ptrtoparam, tam, sizeof(parametro), &parametro) > 0){
                        emit hourReceivedFromTiva(parametro.time.hora, parametro.time.min, parametro.time.sec);
                    }
                }
                break;
                case COMANDO_RECHAZADO:
                {
                    // En otros comandos hay que extraer los parametros de la trama y copiarlos
                    // a una estructura para poder procesar su informacion
                    PARAM_COMANDO_RECHAZADO parametro;
                    if (check_and_extract_command_param(ptrtoparam, tam, sizeof(parametro),&parametro)>0)
                    {
                       // Muestra en una etiqueta (statuslabel) del GUI el mensaje
                       emit commandRejectedFromTiva(parametro.command);
                    }
                    else
                    {
                       emit commandRejectedFromTiva(-1);
                    }
                }
                break;
                case COMANDO_ERROR:
                {
                    PARAM_COMANDO_ERROR parametro;
                    if (check_and_extract_command_param(ptrtoparam, tam, sizeof(parametro),&parametro)>0)
                    {
                       // Muestra en una etiqueta (statuslabel) del GUI el mensaje
                       emit commandRejectedFromTiva(parametro.error.tipo);
                    }
                    else
                    {
                       emit commandRejectedFromTiva(-1);
                    }
                }

                default:
                    //Este error lo notifico mediante la señal statusChanged
                    LastError=QString("Status: Recibido paquete inesperado");
                    emit statusChanged(QRemoteTIVA::UnexpectedPacketError,LastError);
                    break;
                }
            }
        }
        else
        {
            // B. La trama no está completa... no lo procesa, y de momento no digo nada
            //Este error lo notifico mediante la señal statusChanged
            LastError=QString("Status:Fallo trozo paquete recibido");
            emit statusChanged(QRemoteTIVA::FragmentedPacketError,LastError);
        }
        request.remove(0,posicion+1); // Se elimina todo el trozo de información erroneo del array de bytes
        posicion=request.indexOf((char)STOP_FRAME_CHAR,0); // Y se busca el byte de fin de la siguiente trama
    }
}

// Este método realiza el establecimiento de la comunicación USB serie con la TIVA a través del interfaz seleccionado
// Se establece una comunicacion a 9600bps 8N1 y sin control de flujo en el objeto
// 'serial' que es el que gestiona la comunicación USB serie en el interfaz QT
// Si la conexion no es correcta, se generan señales de error.
void QRemoteTIVA::startSlave(QString puerto)
{
    if (serial.portName() != puerto) {
        serial.close();
        serial.setPortName(puerto);

        if (!serial.open(QIODevice::ReadWrite)) {
            LastError=QString("No puedo abrir el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error());

            emit statusChanged(QRemoteTIVA::OpenPortError,LastError);
            return ;
        }

        if (!serial.setBaudRate(9600)) {
            LastError=QString("No puedo establecer tasa de 9600bps en el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error());

            emit statusChanged(QRemoteTIVA::BaudRateError,LastError);
            return;
        }

        if (!serial.setDataBits(QSerialPort::Data8)) {
            LastError=QString("No puedo establecer 8bits de datos en el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error());


             emit statusChanged(QRemoteTIVA::DataBitError,LastError);
            return;
        }

        if (!serial.setParity(QSerialPort::NoParity)) {
            LastError=QString("NO puedo establecer parida en el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error());

            emit statusChanged(QRemoteTIVA::ParityError,LastError);
            return ;
        }

        if (!serial.setStopBits(QSerialPort::OneStop)) {
            LastError=QString("No puedo establecer 1bitStop en el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error());
            emit statusChanged(QRemoteTIVA::StopError,LastError);
            return;
        }

        if (!serial.setFlowControl(QSerialPort::NoFlowControl)) {
            LastError=QString("No puedo establecer el control de flujo en el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error());
             emit statusChanged(QRemoteTIVA::FlowControlError,LastError);
            return;
        }
    }

     emit statusChanged(QRemoteTIVA::TivaConnected,QString(""));

    // Variable indicadora de conexión a TRUE, para que se permita enviar comandos en respuesta
    // a eventos del interfaz gráfico
    connected=true;
}

//Método para leer el último mensaje de error
QString QRemoteTIVA::getLastErrorMessage()
{
    return LastError;
}

// **** Slots asociados al envio de comandos hacia la TIVA. La estructura va a ser muy parecida en casi todos los
// casos. Se va a crear una trama de un tamaño maximo (100), y se le van a introducir los elementos de
// num_secuencia, comando, y parametros.

//Este Slot realiza el envío de un mensaje de PING a la TIVA
void QRemoteTIVA::pingTiva()
{
    char paquete[MAX_FRAME_SIZE];
    int size;

    if (connected) // Para que no se intenten enviar datos si la conexion USB no esta activa
    {
        // El comando PING no necesita parametros; de ahí el NULL, y el 0 final.
        // No vamos a usar el mecanismo de numeracion de tramas; pasamos un 0 como n de trama
        size=create_frame((unsigned char *)paquete, COMANDO_PING, NULL, 0, MAX_FRAME_SIZE);
        // Si la trama se creó correctamente, se escribe el paquete por el puerto serie USB
        if (size>0) serial.write(paquete,size);
    }


}

//Este Slot realiza el envío del comando LED a la TIVA
void QRemoteTIVA::LEDsGpioTiva(bool red, bool green, bool blue)
{
    PARAM_COMANDO_LEDS parametro;
    uint8_t pui8Frame[MAX_FRAME_SIZE];
    int size;
    if(connected)
    {
        // Se rellenan los parametros del paquete (en este caso, el estado de los LED)
        parametro.leds.fRed=red;
        parametro.leds.fGreen=green;
        parametro.leds.fBlue=blue;
        // Se crea la trama con n de secuencia 0; comando COMANDO_LEDS; se le pasa la
        // estructura de parametros, indicando su tamaño; el nº final es el tamaño maximo
        // de trama
        size=create_frame((uint8_t *)pui8Frame, COMANDO_LEDS, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
        // Se se pudo crear correctamente, se envia la trama
        if (size>0) serial.write((char *)pui8Frame,size);
    }
}

//Este Slot realiza el envio del comando de LED a la TIVA vía ColorWheel
void QRemoteTIVA::LEDsColorWheel(uint8_t red, uint8_t green, uint8_t blue)
{
    PARAM_COMANDO_RGB parametro;
    uint8_t pui8Frame[MAX_FRAME_SIZE];
    int size;

    if(connected)
    {
        // Se rellenan los parametros del paquete (en este caso, el estado de los LED)
        parametro.rgb.red = red;
        parametro.rgb.green = green;
        parametro.rgb.blue = blue;
        // Se crea la trama con n de secuencia 0; comando COMANDO_LEDS; se le pasa la
        // estructura de parametros, indicando su tamaño; el nº final es el tamaño maximo
        // de trama
        size=create_frame((uint8_t *)pui8Frame, COMANDO_RGB, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
        // Se se pudo crear correctamente, se envia la trama
        if (size>0) serial.write((char *)pui8Frame,size);
    }
}
//Este Slot realiza el envio del comando de brillo a la TIVA
void QRemoteTIVA::LEDPwmBrightness(double value)
{
    PARAM_COMANDO_BRILLO parametro;
    uint8_t pui8Frame[MAX_FRAME_SIZE];
    int size;
    if(connected)
    {
        // Se rellenan los parametros del paquete (en este caso, el brillo)
        parametro.rIntensity=(float)value;
        // Se crea la trama con n de secuencia 0; comando COMANDO_LEDS; se le pasa la
        // estructura de parametros, indicando su tamaño; el nº final es el tamaño maximo
        // de trama
        size=create_frame((uint8_t *)pui8Frame, COMANDO_BRILLO, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
        // Se se pudo crear correctamente, se envia la trama
        if (size>0) serial.write((char *)pui8Frame,size);
    }
}

//----------------------------------------------------------------------------------------------
//=====================================================
// Si el botón Detectar Botones es clickeado, se envía un
// COMANDO_SWITCH vacío al microcontrolador que responde
// con una trama con el estado actual de los botones.
//=====================================================
void QRemoteTIVA::detectaPulsadores(){

    uint8_t pui8Frame[MAX_FRAME_SIZE];
    int size;

    if(connected)
    {
        size=create_frame((uint8_t *)pui8Frame, COMANDO_SWITCH, NULL, 0, MAX_FRAME_SIZE);
        if (size>0) serial.write((char *)pui8Frame,size);
    }
}

void QRemoteTIVA::RTIChange(bool ok)
{
    PARAM_COMANDO_BUTTONSINT parametro;
    uint8_t pui8Frame[MAX_FRAME_SIZE];
    int size;
    if(connected){
        parametro.buttonsint.OK = ok;
        size=create_frame((uint8_t *)pui8Frame, COMANDO_BUTTONSINT, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
        // Se se pudo crear correctamente, se envia la trama
        if (size>0) serial.write((char *)pui8Frame,size);
    }
}

void QRemoteTIVA::modeChange(int mode)
{
    PARAM_COMANDO_MODE parametro;
    uint8_t pui8Frame[MAX_FRAME_SIZE];
    int size;
    if(connected)
    {
        // Se rellenan los parametros del paquete (en este caso, el modo)
        parametro.mode=mode;
        size=create_frame((uint8_t *)pui8Frame, COMANDO_MODE, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
        // Se se pudo crear correctamente, se envia la trama
        if (size>0) serial.write((char *)pui8Frame,size);
    }
}

// Segunda parte
// Entradas digitales y analógicas

void QRemoteTIVA::ADCChange(bool pb0, bool pb1, bool pb2, bool pb3, bool mon, double l0, double l1, double l2, double l3)
{
    PARAM_COMANDO_ADC parametro;
    uint8_t pui8Frame[MAX_FRAME_SIZE];
    int size;
    if(connected)
    {
        parametro.ok = mon;
        parametro.din.PB0 = pb0;
        parametro.din.PB1 = pb1;
        parametro.din.PB2 = pb2;
        parametro.din.PB3 = pb3;
        parametro.level.L0 = uint32_t(l0*4095/3.3);
        parametro.level.L1 = uint32_t(l1*4095/3.3);
        parametro.level.L2 = uint32_t(l2*4095/3.3);
        parametro.level.L3 = uint32_t(l3*4095/3.3);

        size=create_frame((uint8_t *)pui8Frame, COMANDO_ADC, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
        // Se se pudo crear correctamente, se envia la trama
        if (size>0) serial.write((char *)pui8Frame,size);
    }
}

// Osciloscopio

void QRemoteTIVA::cambiar_frecuencia_muestreo(bool osc, int frec) {
    PARAM_COMANDO_OSC parametro;
    uint8_t pui8Frame[MAX_FRAME_SIZE];
    int size;
    if(connected)
    {
        // Se rellenan los parametros del paquete
        parametro.osc.frecuencia = frec; // Obtiene la frecuencia de muestreo que se usará para la tarea del osciloscopio
        parametro.osc.on = osc; // Activa o desactiva el osciloscopio

        size=create_frame((uint8_t *)pui8Frame, COMANDO_OSC, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
        // Si se pudo crear correctamente, se envia la trama
        if (size>0) serial.write((char *)pui8Frame,size);
    }
}

// Tercera parte
// Hora

void QRemoteTIVA::enviar_hora()
{
    PARAM_COMANDO_TIME parametro;
    uint8_t pui8Frame[MAX_FRAME_SIZE];
    int size;
    QTime time = QTime::currentTime();
    if(connected)
    {
        parametro.time.hora = time.hour();
        parametro.time.min = time.minute();
        parametro.time.sec = time.second();
        parametro.recibir = 0;
        size=create_frame((uint8_t *)pui8Frame, COMANDO_TIME, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
        // Si se pudo crear correctamente, se envia la trama
        if (size>0) serial.write((char *)pui8Frame,size);
    }
}

void QRemoteTIVA::receiveHour()
{
    PARAM_COMANDO_TIME parametro;
    uint8_t pui8Frame[MAX_FRAME_SIZE];
    int size;
    if(connected)
    {
        parametro.recibir = 1;
        size=create_frame((uint8_t *)pui8Frame, COMANDO_TIME, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
        // Si se pudo crear correctamente, se envia la trama
        if (size>0) serial.write((char *)pui8Frame,size);
    }
}
