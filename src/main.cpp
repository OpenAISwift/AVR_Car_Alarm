#include <Arduino.h>
#include <RCSwitch.h>
#include <LowPower.h>
#include "configuration.h"
#include "constant.h"
#include "macros.h"
#include "pins.h"

/*DECLARACION DE VARIABLES GLOBALES*/

/*
Variable para el control de las configuraciones de la alarma
/++++++++++++++++++++++++++++++++++++++++++++++/
(BOOLX = 1) == Activado
(BOOLX = 0) == Desactivado
BOOL1 = Visualizacion de la velocidad
/++++++++++++++++++++++++++++++++++++++++++++++/
(BOOLX = 1) == Activado
(BOOLX = 0) == Desactivado
BOOL2 = Visualizacion de estado de las entradas
/++++++++++++++++++++++++++++++++++++++++++++++/
*/
uint8_t settingsAlarma = 255;

/*
Estados boleanos para el control del estado de entrada
(BOOLX = 1) == Cerrado
(BOOLX = 0) == Abierto
BOOL1 = Estado del Capo 
BOOL2 = Estado Baul 
BOOL3 = Estado Puerta 1 
BOOL4 = Estado Puerta 2 
BOOL5 = Estado Puerta 3 
BOOL6 = Estado Puerta 4 
BOOL7 = Estado Ignicion 
BOOL8 = Estado Llave 
*/
uint8_t statusInputs = 0;
/*
Estados boleanos para el control del estado de entrada
/++++++++++++++++++++++++++++++++++++++++++++++/
(BOOLX = 1) == Conectado 
(BOOLX = 0) == Desconectado
BOOL1 = Estado coneccion bluetooth
/++++++++++++++++++++++++++++++++++++++++++++++/
(BOOLX = 1) == Todas las entradas desactivadas
(BOOLX = 0) == Entradas activa
BOOL2 = Estado de todas las entradas
/++++++++++++++++++++++++++++++++++++++++++++++/
*/
uint8_t statusInputs2 = 0;

char charSerialPort[lengthBufferIn]; // Vector almacenamiento de datos recibidos
int intSerialPort = 0;				 // Valor dato recibido

/*
Estados boleanos para el control del puerto serial
(BOOLX = 1) ==  Datos disponibles
(BOOLX = 0) ==  No existen datos disponibles
BOOL1 = Estado de datos disponible
/++++++++++++++++++++++++++++++++++++++++++++++/
(BOOLX = 1) ==  Lectura en proceso
(BOOLX = 0) ==  Lectura de datos en espera
BOOL2 = Lectura de de datos en proceso
/++++++++++++++++++++++++++++++++++++++++++++++/
(BOOLX = 1) ==  Envio de mensaje
(BOOLX = 0) ==  Envio de mensaje en espera
BOOL3 = Control de envios de mensaje
/++++++++++++++++++++++++++++++++++++++++++++++/
*/
uint8_t statusSerialPort = 0;

/*
Variable para el control de envio de datos
/++++++++++++++++++++++++++++++++++++++++++++++/
Valores de control para envio de datos
BOOL1 = Nuevo dato de velocidad
*/
uint8_t statusSendData = 0;

/*
Variable para el control de envio de datos
/++++++++++++++++++++++++++++++++++++++++++++++/
Valores de control para envio de datos
BOOL1 = Nuevo dato capo
BOOL2 = Nuevo dato maletero
BOOL3 = Nuevo dato Puerta 1
BOOL4 = Nuevo dato Puerta 2
BOOL5 = Nuevo dato Puerta 3
BOOL6 = Nuevo dato Puerta 4
BOOL7 = Nuevo dato ignicion
BOOL8 = Nuevo dato llave
*/
uint8_t statusSendData2 = 0;

/*
Variable para el control de la apertura del maletero
/++++++++++++++++++++++++++++++++++++++++++++++/
Valores de control del puerto del maletero
0 = Desactiva el puerto de apertura de maletero
1 = Activa el puerto de apertura del maletero
2 = Temporizacion de tiempo activo del puerto de apertura del maletero 
/++++++++++++++++++++++++++++++++++++++++++++++/
Valores de comprobacion del estado del puerto del maletero
255 = Estado de inactividad
*/
uint8_t statusOpenTrunk = 0;

/*
Variable para el control de la apertura seguros
/++++++++++++++++++++++++++++++++++++++++++++++/
Valores de control del puerto de apertura de seguros
0 = Desactiva el puerto de apertura de seguros
1 = Activa el puerto de apertura de seguros
2 = Temporizacion de tiempo activo del puerto de apertura seguros
/++++++++++++++++++++++++++++++++++++++++++++++/
Valores de comprobacion del estado del puerto de apertura de seguros
254 = Estado de apertura  de seguros activados 
255 = Estado de inactividad
*/
uint8_t statusOpenDoorsLocks = 0;

/*
Variable para el control de cierre de seguros
/++++++++++++++++++++++++++++++++++++++++++++++/
Valores de control del puerto de cierre de seguros
0 = Desactiva el puerto de cierre de seguros
1 = Activa el puerto de cierre de seguros
2 = Temporizacion de tiempo activo del puerto de cierre de seguros
/++++++++++++++++++++++++++++++++++++++++++++++/
Valores de comprobacion del estado del puerto de cierre de seguros
254 = Estado de cierre de seguros activados 
255 = Estado de inactividad
*/
uint8_t statusCloseDoorsLocks = 0;

/*
Variable para el control de luz de emergencia
/++++++++++++++++++++++++++++++++++++++++++++++/
Valores de control del puerto de luz de emergencia
0 = Desactiva el puerto de luz de emergencia
1 = Activa el puerto de luz de emergencia y inicia la temporizacion
2 = Activa el puerto de luz de emergencia y lo mantiene activado
3 = Temporizacion del tiempo activo del puerto de luz de emergencia
/++++++++++++++++++++++++++++++++++++++++++++++/
Valores de comprobacion del estado del puerto de luz de emergencia
254 = Estado de luz de emergencia activados 
255 = Estado de inactividad
*/
uint8_t statusEmergencyLight = 0;

/*
Variable para el control de la luz de cortecia de la cabina
/++++++++++++++++++++++++++++++++++++++++++++++/
Valores de control del puerto de luz de cortecia de la cabina
0 = Desactiva el puerto del efecto fade de la cabina
1 = Activa el puerto del efecto fade de la cabina sin temporizador
2 = Activa el puerto con retraso de inicio del efecto fade de la cabina
3 = Comprueba el inicio del efecto fade
4 = Temporizacion de inicio del efecto fade 
5 = Inicio de efecto fade de la cabina
6 = Disminucion del valor para el efecto fade
/++++++++++++++++++++++++++++++++++++++++++++++/
Valores de comprobacion del estado del puerto de luz de cortecia de la cabina
255 = Estado de inactividad
*/
uint8_t statusFadeCabin = 0;

/*
Variable para el control de la luz de cortecia del maletero
/++++++++++++++++++++++++++++++++++++++++++++++/
Valores de control del puerto de luz de cortecia del maletero
0 = Desactiva el puerto del efecto fade del maletero
1 = Activa el puerto del efecto fade del maletero sin temporizador
2 = Activa el puerto con retraso de inicio del efecto fade del maletero
3 = Temporizacion de inicio del efecto fade 
4 = Inicio de efecto fade del maletero
5 = Disminucion del valor para el efecto fade
/++++++++++++++++++++++++++++++++++++++++++++++/
Valores de comprobacion del estado del puerto de luz de cortecia del maletero
255 = Estado de inactividad
*/
uint8_t statusFadeTrunk = 0;

/*
Variable para el control del blink del puerto 
/++++++++++++++++++++++++++++++++++++++++++++++/
Valores de control del puerto del blink
0 = Descativa el parpadeo del blink
1 = Inicia la secuencia de blink
2 = Inicia la espera para el apagado de la salida
3 = Comprueba el numero de blinks que se han realizado
/++++++++++++++++++++++++++++++++++++++++++++++/
Valores de comprobacion del estado del puerto
255 = Estado inactivo
*/
uint8_t statusBlinkSirena = 0;
/*
255 = Blink siempre encendido
*/
uint8_t blinksSirena = 0;

/*
Variable para el control del blink del puerto 
/++++++++++++++++++++++++++++++++++++++++++++++/
Valores de control del puerto del blink
0 = Descativa el parpadeo del blink
1 = Inicia la secuencia de blink
2 = Inicia la espera para el apagado de la salida
3 = Comprueba el numero de blinks que se han realizado
/++++++++++++++++++++++++++++++++++++++++++++++/
Valores de comprobacion del estado del puerto
255 = Estado inactivo
*/
uint8_t statusBlinkClaxon = 0;
/*
255 = Blink siempre encendido
*/
uint8_t blinkClaxon = 0;

/*
Variable para el control del cierre automatico
/++++++++++++++++++++++++++++++++++++++++++++++/
Valores de control del cierre de seguros automatico
0 = Desactiva la temporizacion el cierre automatico
1 = Inicia la temporizacion de cierre automatico
2 = Temporizador cierre automatico
3 = Cierre automatico por velocidad
/++++++++++++++++++++++++++++++++++++++++++++++/
Estado de comprobacion de cierre automatico
254 = Estado de cierre automatico
255 = Estado de desactivado cierre automatico
*/
uint8_t statusCloseAutomaticLocks = 0;

/*
Variable para el control del estado de alerta alarma
/++++++++++++++++++++++++++++++++++++++++++++++/
Estados de control de alerta alarma
0 = Desactiva alarma
1 = Activa la alarma
2 = Temporizador para la desactivacion de alerta alarma
3 = Desactiva aviso alarma
/++++++++++++++++++++++++++++++++++++++++++++++/
Estados de comprobacion de estado de alerta alarma 
255 = Estado de reposo alarma 
*/
uint8_t statusAlertAlarm = 255;

/*
Variable para el control de estado de alerta por golpes
/++++++++++++++++++++++++++++++++++++++++++++++/
0 = Desactiva la alerta 
1 = Comprueba el numero de avisos que se a emitido
2 = Activa el aviso por golpe leve 
3 = Activa el aviso por golpe fuerte
/++++++++++++++++++++++++++++++++++++++++++++++/
Estados de comprobacion de estado de alarma 
255 = Estado de reposo
*/
uint8_t statusKnockAlert = 255;
uint8_t counterKnockAlert = 0; // Contador para el numero de veces que ha sonado la alerta

/*
Variable para el control de estado de lectura de velocidad
/++++++++++++++++++++++++++++++++++++++++++++++/
0 = Desactiva la lectura de velocidad
1 = Inicia La temporizacion de lectura de velocidad
2 = Reinicia la lectura de velocidad
3 = Temporizador de la lectura de velocidad
4 = Inicia la lectura de velocidad
5 = Cierre de seguros por velocidad
/++++++++++++++++++++++++++++++++++++++++++++++/
255 = Estado de reposo
*/
uint8_t statusReadSpeed = 0;
uint8_t speedVss = 0; // Lectura de velocidad

/*
Variable para el control de estado de la lectura del sensor golpe
/++++++++++++++++++++++++++++++++++++++++++++++/
0 = Desactiva la lectura del sensor de golpe
1 = Activa la lectura del sensor de golpe
2 = Lectura del sensor de golpe
/++++++++++++++++++++++++++++++++++++++++++++++/
255 = Estado de reposo
*/
uint8_t statusKnock = 0;

/*
Variable para el control de estado de las interupciones
/++++++++++++++++++++++++++++++++++++++++++++++/
BOOL1 = Primera lectura
BOOL2 = Triggered
BOOL3 = Habilitar Temporizador VSS
BOOL4 = Primera lectura
BOOL5 = Triggered
/++++++++++++++++++++++++++++++++++++++++++++++/
*/
volatile uint8_t statusISR = 0;

volatile unsigned long OverFlowCount = 0;
volatile unsigned long StartTime = 0;
volatile unsigned long FinishTime = 0;
volatile unsigned long TimerCurrentVss = 0;
volatile unsigned long KnockStartTime = 0;
volatile unsigned long KnockFinishTime = 0;

unsigned long TimerCurrent = 0; //Temporizadores para el tiempo actual

/*Declaracion de funciones prototipo*/
void InitSystem();
void ReadSerialPort();
void SerialPortCommand();
void ReadRF();
void ReadInputs();
void CourtesyLight();
void BlinkOutputs();
void TimedOuputs();
void AutomaticControl();
void OpenDoors();
void CloseDoors();
void OpenTrunk();
void ReadVssSpeed();
void KnockSignal();
void KnockPrepareInterrupts();
void PrepareInterrupts();
void SendData();
void HomeMode();

RCSwitch RFControl = RCSwitch();
/*Servicio de interrupcion (INT4) */
void ISRSpeed()
{
	unsigned int counter = TCNT1;
	statusISR |= BOOL3;
	TimerCurrentVss = TimerCurrent;
	if (statusISR & BOOL2)
	{
		return;
	}
	if (statusISR & BOOL1)
	{
		StartTime = (OverFlowCount << 16) + counter;
		statusISR &= ~BOOL1;
		return;
	}
	FinishTime = (OverFlowCount << 16) + counter;
	statusISR |= BOOL2;
	detachInterrupt(digitalPinToInterrupt(InputVss));
}

/*Servicio de interrupcion (INT5) */
void ISRKnock()
{
	if (statusISR & BOOL5)
	{
		return;
	}
	if (statusISR & BOOL4)
	{
		KnockStartTime = millis();
		statusISR &= ~BOOL4;
		return;
	}
	KnockFinishTime = millis();
	statusISR |= BOOL5;
	detachInterrupt(digitalPinToInterrupt(InputKnock));
}

/*Servicio de interrupcion (INT3)*/
void wakeUp()
{
	// Just a handler for the pin interrupt.
}

/*Servicio de interrupcion del Timer1*/
ISR(TIMER1_OVF_vect)
{
	OverFlowCount++;
}

void setup()
{
	Serial2.begin(SerialBaudRate);
	RFControl.enableReceive(digitalPinToInterrupt(InputRF));

	/*PUERTOS DE SALIDA PWM*/
	pinMode(PwmLightCabin, OUTPUT);
	pinMode(PwmLightTrunk, OUTPUT);

	/*PUERTOS DE SALIDA DIGITALES */
	pinMode(OutSirenBlink, OUTPUT);
	pinMode(OutHornBlink, OUTPUT);
	pinMode(OutOpenTrunk, OUTPUT);
	pinMode(OutOpenDoors, OUTPUT);
	pinMode(OutCloseDoors, OUTPUT);
	pinMode(OutEmergencyLight, OUTPUT);
	pinMode(OutSegurityLight, OUTPUT);
	pinMode(LED_BUILTIN, OUTPUT);

	/*PUERTOS DE ENTRADA DIGITALES*/
	pinMode(InputCapo, INPUT);
	pinMode(InputTrunk, INPUT);
	pinMode(InputDoor1, INPUT);
	pinMode(InputDoor2, INPUT);
	pinMode(InputDoor3, INPUT);
	pinMode(InputDoor4, INPUT);
	pinMode(InputIgn, INPUT);
	pinMode(InputKey, INPUT);
	pinMode(InputStateBt, INPUT);

	/*PUERTOS DE ENTRADA INTERRUPCIONES EXTERNAS*/
	pinMode(InputVss, INPUT);
	pinMode(InputKnock, INPUT);
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////

	/*CONFIGURACIONES INICIALES DE LA ALRMA*/
	InitSystem();
}

void loop()
{
	TimerCurrent = millis();
	ReadSerialPort();
	ReadRF();
	ReadInputs();
	CourtesyLight();
	BlinkOutputs();
	TimedOuputs();
	AutomaticControl();
	SendData();
}

/*Funcion para el control del incio del programa*/
void InitSystem()
{
	/*VARIABLES*/
	OverFlowCount = 0;

	/*CONFIGURACION DE TIMERS*/
	TCCR1A = 0;
	TCCR1B = 0;
	TCNT1 = 0;
	TCCR1B |= ((0 << CS12) | (0 << CS11) | (1 << CS10)); // Timer1 sin preescalador
	TIMSK1 |= (1 << TOIE1);								 // Habilita la interrupcion del Timer1

	/*CONFIGURACION DE INTERRUPCIONES*/
	/*INICIO DE FUNCIONES*/
	PrepareInterrupts();
	KnockPrepareInterrupts();

	/*INICIO DE PUERTOS DE SALIDA*/
	digitalWrite(LED_BUILTIN, LOW);
	digitalWrite(OutSegurityLight, HIGH);
	statusFadeCabin = 0;
	statusFadeTrunk = 0;
}

/*Reinicia las variables para la siguiente lectura VSS */
void PrepareInterrupts()
{
	statusISR |= BOOL1;
	statusISR &= ~BOOL2;
	EIFR |= (1 << INTF4); //Reinicia la Flag de la interrupcion (INT4)
	attachInterrupt(digitalPinToInterrupt(InputVss), ISRSpeed, RISING);
}

/*Reinicia las variables para la siguiente lectura INT5*/
void KnockPrepareInterrupts()
{
	statusISR |= BOOL4;
	statusISR &= ~BOOL5;
	EIFR |= (1 << INTF5); //Reinicia la Flag de la interrupcion (INT5)
	attachInterrupt(digitalPinToInterrupt(InputKnock), ISRKnock, CHANGE);
}

/*Funcion para la lecutura de la velocidad VSS*/
void ReadVssSpeed()
{
	/*Variables locales de la funcion*/
	static uint8_t oldSpeed = 0;
	/*++++++++++++++++++++++++++++++++++++++++++++++*/

	if (!(statusISR & BOOL2))
	{
		if (statusISR & BOOL3)
		{
			if (TimerCurrent - TimerCurrentVss > 1100)
			{
				statusISR &= ~BOOL3;
				oldSpeed = 0;
				PrepareInterrupts();
			}
		}
	}
	else
	{
		unsigned long elapsedTime = FinishTime - StartTime;
		float freq = F_CPU / float(elapsedTime);
		unsigned long rpm = (freq * 60) / VSSPulsePerRevolution;
		oldSpeed = (rpm * VSSWheelPerimeter * 60) / 1000;
		PrepareInterrupts();
	}
	if (oldSpeed != speedVss)
	{
		speedVss = oldSpeed;
		if (settingsAlarma & BOOL1)
		{
			statusSerialPort |= BOOL3;
			statusSendData |= BOOL1;
		}
		if (speedVss >= VSSCloseDoors)
		{
			if (statusCloseAutomaticLocks == 255)
			{
				statusCloseAutomaticLocks = 3;
			}
		}
	}
}

/*Funcion para la lecutura del sensor de golpe*/
void KnockSignal()
{
	if (!(statusISR & BOOL5))
	{
		return;
	}
	else
	{
		unsigned long elapsedTime = KnockFinishTime - KnockStartTime;
		if ((elapsedTime >= KnockSlight) && (elapsedTime < KnockHard))
		{
			statusKnockAlert = 1;
		}
		else
		{
			if (elapsedTime >= KnockHard)
			{
				statusKnockAlert = 3;
			}
		}
		KnockPrepareInterrupts();
	}
}

/*Funcion para la lectura de comandos por el puerto serial*/
void ReadSerialPort()
{
	static char ReceivedChars[lengthBufferIn];
	if (Serial2.available() > 0 && !(statusSerialPort & BOOL1))
	{
		char RecivedChar = Serial2.read();
		if (statusSerialPort & BOOL2)
		{
			static uint8_t AuxSerial = 0;
			if (RecivedChar != EndMark)
			{
				ReceivedChars[AuxSerial] = RecivedChar;
				AuxSerial++;
				if (AuxSerial >= lengthBufferIn)
				{
					AuxSerial = lengthBufferIn - 1;
					return;
				}
				return;
			}
			else
			{
				ReceivedChars[AuxSerial] = '\0';
				AuxSerial = 0;
				statusSerialPort |= BOOL1;
				statusSerialPort &= ~BOOL2;
				return;
			}
		}
		else
		{
			if (RecivedChar == StartMark)
			{
				statusSerialPort |= BOOL2;
				return;
			}
			else
			{
				return;
			}
		}
	}
	if (statusSerialPort & BOOL1)
	{
		static char charTemp[lengthBufferIn];
		strcpy(charTemp, ReceivedChars);
		char *strtokIndx;
		strtokIndx = strtok(charTemp, ",");
		strcpy(charSerialPort, strtokIndx);
		strtokIndx = strtok(NULL, ",");
		intSerialPort = atoi(strtokIndx);
		statusSerialPort &= ~BOOL1;
		SerialPortCommand();
	}
}

/*Funcion para la lectura de comandos por RF*/
void ReadRF()
{
	if (RFControl.available())
	{
		unsigned long RFControlData = RFControl.getReceivedValue();
		switch (RFControlData)
		{
		case 4887889: // Cierre de seguros
			CloseDoors();
			break;

		case 4887890: // Apertura de seguros
			OpenDoors();
			break;

		case 4887891: // Apertura de Maleter0
			OpenTrunk();
			break;

		case 4887892: // Desactivo de aviso Alarma
			if (statusAlertAlarm == 2)
			{
				statusAlertAlarm = 3;
			}
			break;

		case 4887896: // Señal de Aviso
			statusBlinkSirena = 1;
			blinksSirena = 8;
			statusEmergencyLight = 1;
			break;

		case 4887897: // Alarma modo casa
			HomeMode();
			break;

		default:
			Serial2.print(F("Codigo Nuevo: "));
			Serial2.println(RFControlData);
			break;
		}
		RFControl.resetAvailable();
	}
}

/*Funcion para manejar los mensajes del puerto serial*/
void SerialPortCommand()
{
	/************************************************************************/
	/*COMANDOS DISPONIBLES                                                  */
	/************************************************************************/
	/*
	Estructura de la cadena: <x,xn> 
	Caracter de inicio de trama: <
	Comando: x
	Separador: ,
	Entero: n
	Caracter de fin de trama: >
	/++++++++++++++++++++++++++++++++++++++++++++++/
	Control de la apertura, cierre de seguros y apertura de maletero: s,S 
	<s,"valor">____<S,"valor">
	1 = Abrir seguros
	2 = Cerrar seguros
	3 = Apertura de maletero
	/++++++++++++++++++++++++++++++++++++++++++++++/
	Control de pruebas de salidas y entradas de la alarma: t,T
	<t,"valor">____<T,"valor"> 
	1 = Enciende la luz de estado alarma
	2 = Apaga la luz de estado alarma
	/++++++++++++++++++++++++++++++++++++++++++++++/
	Imprime informacion de la alarma: i,I
	<i,"valor">____<I,"valor"> 
	1 = Vercion del codigo de la alarma 
	/++++++++++++++++++++++++++++++++++++++++++++++/
	Modo de estados de alarma: m,M
	<m,"valor">____<M,"valor"> 
	1 = Modo casa bajo consumo desactiva la alarma
	*/
	switch (charSerialPort[0])
	{

	case 's':
	case 'S':
		switch (intSerialPort)
		{
		case 1: // Apertura de seguros
			OpenDoors();
			break;
		case 2: // Cierre de seguros
			CloseDoors();
			break;
		case 3: // Apertura de Maletero
			OpenTrunk();
			break;

		default:
			break;
		}
		break;

	case 't':
	case 'T':
		switch (intSerialPort)
		{
		case 1: // Enciende la luz de estado alarma
			digitalWrite(OutSegurityLight, HIGH);
			break;
		case 2: // Apaga la luz de estado alarma
			digitalWrite(OutSegurityLight, LOW);
			break;

		default:
			break;
		}
		break;

	case 'i':
	case 'I':
		switch (intSerialPort)
		{
		case 1: // Envia informacion de vercion y fecha
			Serial2.print(F(_Version));
			break;

		default:
			break;
		}
		break;

	case 'm':
	case 'M':
		switch (intSerialPort)
		{
		case 1: // Modo casa bajo consumo desactiva la alarma
			HomeMode();
			break;

		default:
			break;
		}
		break;

	default:
		Serial2.println(F("Comando no disponible"));
		break;
	}
	return;
}

/*Funcion para lectura de puertos*/
void ReadInputs()
{
	if (digitalRead(InputCapo))
	{
		if (!(statusInputs & BOOL1))
		{
			statusInputs |= BOOL1;
			if (settingsAlarma & BOOL2)
			{
				statusSerialPort |= BOOL3;
				statusSendData2 |= BOOL1;
			}
		}
	}
	else
	{
		if (statusInputs & BOOL1)
		{
			statusInputs &= ~BOOL1;
			if (settingsAlarma & BOOL2)
			{
				statusSerialPort |= BOOL3;
				statusSendData2 |= BOOL1;
			}
		}
	}

	if (digitalRead(InputTrunk))
	{
		if (!(statusInputs & BOOL2))
		{
			statusInputs |= BOOL2;
			if (settingsAlarma & BOOL2)
			{
				statusSerialPort |= BOOL3;
				statusSendData2 |= BOOL2;
			}
		}
	}
	else
	{
		if (statusInputs & BOOL2)
		{
			statusInputs &= ~BOOL2;
			if (settingsAlarma & BOOL2)
			{
				statusSerialPort |= BOOL3;
				statusSendData2 |= BOOL2;
			}
		}
	}

	if (digitalRead(InputDoor1))
	{
		if (!(statusInputs & BOOL3))
		{
			statusInputs |= BOOL3;
			statusFadeCabin = 3;
			if (settingsAlarma & BOOL2)
			{
				statusSerialPort |= BOOL3;
				statusSendData2 |= BOOL3;
			}
		}
	}
	else
	{
		if (statusInputs & BOOL3)
		{
			statusInputs &= ~BOOL3;
			statusFadeCabin = 1;
			if (settingsAlarma & BOOL2)
			{
				statusSerialPort |= BOOL3;
				statusSendData2 |= BOOL3;
			}
		}
	}

	if (digitalRead(InputDoor2))
	{
		if (!(statusInputs & BOOL4))
		{
			statusInputs |= BOOL4;
			statusFadeCabin = 3;
			if (settingsAlarma & BOOL2)
			{
				statusSerialPort |= BOOL3;
				statusSendData2 |= BOOL4;
			}
		}
	}
	else
	{
		if (statusInputs & BOOL4)
		{
			statusInputs &= ~BOOL4;
			statusFadeCabin = 1;
			if (settingsAlarma & BOOL2)
			{
				statusSerialPort |= BOOL3;
				statusSendData2 |= BOOL4;
			}
		}
	}

	if (digitalRead(InputDoor3))
	{
		if (!(statusInputs & BOOL5))
		{
			statusInputs |= BOOL5;
			statusFadeCabin = 3;
			if (settingsAlarma & BOOL2)
			{
				statusSerialPort |= BOOL3;
				statusSendData2 |= BOOL5;
			}
		}
	}
	else
	{
		if (statusInputs & BOOL5)
		{
			statusInputs &= ~BOOL5;
			statusFadeCabin = 1;
			if (settingsAlarma & BOOL2)
			{
				statusSerialPort |= BOOL3;
				statusSendData2 |= BOOL5;
			}
		}
	}

	if (digitalRead(InputDoor4))
	{
		if (!(statusInputs & BOOL6))
		{
			statusInputs |= BOOL6;
			statusFadeCabin = 3;
			if (settingsAlarma & BOOL2)
			{
				statusSerialPort |= BOOL3;
				statusSendData2 |= BOOL6;
			}
		}
	}
	else
	{
		if (statusInputs & BOOL6)
		{
			statusInputs &= ~BOOL6;
			statusFadeCabin = 1;
			if (settingsAlarma & BOOL2)
			{
				statusSerialPort |= BOOL3;
				statusSendData2 |= BOOL6;
			}
		}
	}

	if (digitalRead(InputIgn))
	{
		if (!(statusInputs & BOOL7))
		{
			OpenDoors();
			statusInputs |= BOOL7;
			statusReadSpeed = 0;
			if (settingsAlarma & BOOL2)
			{
				statusSerialPort |= BOOL3;
				statusSendData2 |= BOOL7;
			}
		}
	}
	else
	{
		if (statusInputs & BOOL7)
		{
			statusInputs &= ~BOOL7;
			statusReadSpeed = 1;
			if (settingsAlarma & BOOL2)
			{
				statusSerialPort |= BOOL3;
				statusSendData2 |= BOOL7;
			}
		}
	}

	if (digitalRead(InputKey))
	{
		if (!(statusInputs & BOOL8))
		{
			statusInputs |= BOOL8;
			statusFadeCabin = 1;
			if (settingsAlarma & BOOL2)
			{
				statusSerialPort |= BOOL3;
				statusSendData2 |= BOOL8;
			}
		}
	}
	else
	{
		if (statusInputs & BOOL8)
		{
			statusInputs &= ~BOOL8;
			if (settingsAlarma & BOOL2)
			{
				statusSerialPort |= BOOL3;
				statusSendData2 |= BOOL8;
			}
		}
	}

	if (!digitalRead(InputStateBt))
	{
		if (statusInputs2 & BOOL1)
		{
			statusInputs2 &= ~BOOL1;
		}
	}
	else
	{
		if (!(statusInputs2 & BOOL1))
		{
			statusInputs2 |= BOOL1;
		}
	}

	if (statusInputs == 255)
	{
		if (!(statusInputs2 & BOOL2))
		{
			statusInputs2 |= BOOL2;
			if (statusCloseAutomaticLocks == 255)
			{
				statusCloseAutomaticLocks = 1;
			}
		}
	}
	else
	{
		if (statusInputs2 & BOOL2)
		{

			statusInputs2 &= ~BOOL2;
			if (statusCloseAutomaticLocks == 254)
			{
				statusAlertAlarm = 1;
			}
			else
			{
				statusCloseAutomaticLocks = 0;
			}
		}
	}
}

/* Funcion para controlar el Efecto Fade en la luz de cortesia */
void CourtesyLight()
{
	static uint8_t valueFadeCabin = 0; // Valor del PWM para la salida de luz de cabina
	static unsigned long timerFadeDelayCabin = 0;
	static unsigned long timerFadeCabin = 0;

	static uint8_t ValueFadeTrunk = 0; // Valor del PWM para la salida de luz del maletero
	static unsigned long TimerFadeDelayTrunk = 0;
	static unsigned long TimerFadeTrunk = 0;

	/*Efecto fade luz cabina*/
	switch (statusFadeCabin)
	{
	case 0: // Desactiva el puerto del efecto fade de la cabina
		statusFadeCabin = 255;
		valueFadeCabin = 255;
		analogWrite(PwmLightCabin, valueFadeCabin);
		break;
	case 1: // Activa el puerto del efecto fade de la cabina sin temporizador
		statusFadeCabin = 255;
		valueFadeCabin = 0;
		analogWrite(PwmLightCabin, valueFadeCabin);
		break;
	case 2: // Activa el puerto con retraso de inicio del efecto fade de la cabina
		statusFadeCabin = 4;
		timerFadeCabin = TimerCurrent;
		valueFadeCabin = 0;
		analogWrite(PwmLightCabin, valueFadeCabin);
		break;
	case 3: // Comprueba el inicio del efecto fade
		statusFadeCabin = 5;
		if (valueFadeCabin != 0)
		{
			valueFadeCabin = 0;
			analogWrite(PwmLightCabin, valueFadeCabin);
			break;
		}
		else
		{
			break;
		}
	case 4: // Temporizacion de inicio del efecto fade
		if (TimerCurrent - timerFadeCabin >= delayStartFadeCabin)
		{
			statusFadeCabin = 5;
			break;
		}
		else
		{
			break;
		}
	case 5: // Inicio de efecto fade de la cabina
		if (TimerCurrent - timerFadeDelayCabin >= intervalFadeCabin)
		{
			statusFadeCabin = 6;
			timerFadeDelayCabin = TimerCurrent;
			break;
		}
		else
		{
			break;
		}
	case 6: // Disminucion del valor para el efecto fade
		if (valueFadeCabin < 255)
		{
			statusFadeCabin = 5;
			valueFadeCabin++;
			analogWrite(PwmLightCabin, valueFadeCabin);
			break;
		}
		else
		{
			statusFadeCabin = 0;
			break;
		}
	default:
		break;
	}

	/*Efecto fade luz maletero*/
	switch (statusFadeTrunk)
	{
	case 0: //Desactivacion del efecto fade
		ValueFadeTrunk = 255;
		statusFadeTrunk = 255;
		analogWrite(PwmLightTrunk, ValueFadeTrunk);
		break;
	case 1: //Activa la salida del efecto fade sin temporizador
		statusFadeTrunk = 255;
		if (ValueFadeTrunk != 0)
		{
			ValueFadeTrunk = 0;
			analogWrite(PwmLightTrunk, ValueFadeTrunk);
			break;
		}
		else
		{
			break;
		}
	case 2: // Inicio retraso de apagado efecto fade
		statusFadeTrunk = 3;
		TimerFadeDelayTrunk = TimerCurrent;
		if (ValueFadeTrunk != 0)
		{
			ValueFadeTrunk = 0;
			analogWrite(PwmLightTrunk, ValueFadeTrunk);
			break;
		}
		else
		{
			break;
		}

	case 3: // Temporizacion de inicio del efecto fade
		if (TimerCurrent - TimerFadeDelayTrunk >= delayStartFadeTrunk)
		{
			statusFadeTrunk = 4;
			break;
		}
		else
		{
			break;
		}
	case 4: // Inicio de efecto fade
		if (TimerCurrent - TimerFadeTrunk >= intervalFadeTrunk)
		{
			statusFadeTrunk = 5;
			TimerFadeTrunk = TimerCurrent;
			break;
		}
		else
		{
			break;
		}
	case 5: // Disminucion del valor para el efecto fade
		if (ValueFadeTrunk < 255)
		{
			ValueFadeTrunk += 1;
			statusFadeTrunk = 4;
			analogWrite(PwmLightTrunk, ValueFadeTrunk);
			break;
		}
		else
		{
			statusFadeTrunk = 0;
			break;
		}
	default:
		break;
	}
}

/* Funcion para controlar blink */
void BlinkOutputs()
{
	static unsigned long timerBlinkSirena = 0; // Temporizdor para el blink sirena
	static uint8_t counterBlinkSirena = 0;	   // Contador para el blink sirena

	static unsigned long timerBlinkClaxon = 0; // Temporizdor para el blink claxon
	static uint8_t counterBlinkClaxon = 0;	   // Contador para el claxon

	/*Control para el blink de la sirena*/
	switch (statusBlinkSirena)
	{
	case 0: // Desactiva el blink de la sirena y reinicia las variables
		statusBlinkSirena = 255;
		counterBlinkSirena = 0;
		digitalWrite(OutSirenBlink, LOW);
		break;
	case 1: // Inicia el blink de la sirena
		if (TimerCurrent - timerBlinkSirena >= blinkSirenOff)
		{
			statusBlinkSirena = 2;
			timerBlinkSirena = TimerCurrent;
			counterBlinkSirena++;
			digitalWrite(OutSirenBlink, HIGH);
			break;
		}
		else
		{
			break;
		}
	case 2: // Inicia la espera para el apagado de la salida
		if (TimerCurrent - timerBlinkSirena >= blinkSirenOn)
		{
			statusBlinkSirena = 3;
			timerBlinkSirena = TimerCurrent;
			digitalWrite(OutSirenBlink, LOW);
			break;
		}
		else
		{
			break;
		}
	case 3: // Comprueba el numero de blinks que se han realizado
		if (blinksSirena == 255)
		{
			statusBlinkSirena = 1;
		}
		else
		{
			if (counterBlinkSirena < blinksSirena)
			{
				statusBlinkSirena = 1;
				break;
			}
			else
			{
				statusBlinkSirena = 0;
				break;
			}
			break;
		}
		break;

	default:
		break;
	}

	/*Control para el blink del claxon*/
	switch (statusBlinkClaxon)
	{
	case 0: // Apaga el blink del claxon y reinicia variables
		statusBlinkClaxon = 255;
		counterBlinkClaxon = 0;
		blinkClaxon = 0;
		digitalWrite(OutHornBlink, LOW);
		break;
	case 1: // Inicia el encendido blink
		if (TimerCurrent - timerBlinkClaxon >= blinkClaxonOff)
		{
			statusBlinkClaxon = 2;
			counterBlinkClaxon++;
			timerBlinkClaxon = TimerCurrent;
			digitalWrite(OutHornBlink, HIGH);
			break;
		}
		else
		{
			break;
		}
	case 2: // Inicia la espera para el apagado
		if (TimerCurrent - timerBlinkClaxon >= blinkClaxonOn)
		{
			statusBlinkClaxon = 3;
			timerBlinkClaxon = TimerCurrent;
			digitalWrite(OutHornBlink, LOW);
			break;
		}
		else
		{
			break;
		}
	case 3: // Comprueba el numero de blink que se han realizado
		if (blinkClaxon == 255)
		{
			statusBlinkClaxon = 1;
			break;
		}
		else
		{
			if (counterBlinkClaxon < blinkClaxon)
			{
				statusBlinkClaxon = 1;
				break;
			}
			else
			{
				statusBlinkClaxon = 0;
				break;
			}
			break;
		}
		break;

	default:
		break;
	}
}

/* Funcion para salidas temporizadas*/
void TimedOuputs()
{
	/*Variables locales de la funcion*/
	static unsigned long TimerOutTrunk = 0;		   // Temporizador salida apertura baul
	static unsigned long TimerOpenDoorsLocks = 0;  // Temporizador salida apertura de seguros
	static unsigned long TimerCloseDoorsLocks = 0; // Temporizador salida cierre de seguros
	static unsigned long TimerEmergencyLight = 0;  // Temporizador salida luz de emergencia

	/*Control del puerto de la apertura del maletero*/
	switch (statusOpenTrunk)
	{
	case 0: // Desactiva el puerto de apertura del maletero
		statusOpenTrunk = 255;
		digitalWrite(OutOpenTrunk, LOW);
		break;
	case 1: // Activa el puerto de apertura del maletero
		statusOpenTrunk = 2;
		TimerOutTrunk = TimerCurrent;
		digitalWrite(OutOpenTrunk, HIGH);
		break;
	case 2: // Temporizacion de tiempo activo del puerto de apertura del maletero
		if (TimerCurrent - TimerOutTrunk >= intervalOutputTrunk)
		{
			statusOpenTrunk = 0;
			break;
		}
		else
		{
			break;
		}
	default:
		break;
	}
	/*Control del puerto de la apertura de seguros*/
	switch (statusOpenDoorsLocks)
	{
	case 0: // Desactiva el puerto de apertura de seguros
		statusOpenDoorsLocks = 254;
		statusCloseDoorsLocks = 255;
		digitalWrite(OutOpenDoors, LOW);
		break;
	case 1: // Activa el puerto de apertura de seguros
		statusOpenDoorsLocks = 2;
		TimerOpenDoorsLocks = TimerCurrent;
		digitalWrite(OutOpenDoors, HIGH);
		break;
	case 2: // Temporizacion de tiempo activado del puerto apertura de seguros
		if (TimerCurrent - TimerOpenDoorsLocks >= intervalOutputOpenDoors)
		{
			statusOpenDoorsLocks = 0;
			break;
		}
		else
		{
			break;
		}
	default:
		break;
	}
	/*Control del puerto de cierre de seguros*/
	switch (statusCloseDoorsLocks)
	{
	case 0: // Desactiva el puerto de cierre de seguros
		statusCloseDoorsLocks = 254;
		statusOpenDoorsLocks = 255;
		digitalWrite(OutCloseDoors, LOW);
		break;

	case 1: // Activa el puerto de cierre de seguros
		statusCloseDoorsLocks = 2;
		TimerCloseDoorsLocks = TimerCurrent;
		digitalWrite(OutCloseDoors, HIGH);
		break;

	case 2: // Temporizacion del tiempo activo del puerto de cierre de seguros
		if (TimerCurrent - TimerCloseDoorsLocks >= intervalOutputCloseDoors)
		{
			statusCloseDoorsLocks = 0;
			break;
		}
		else
		{
			break;
		}

	default:
		break;
	}
	/*Control del puerto de luz de emergencia*/
	switch (statusEmergencyLight)
	{
	case 0: // Desactiva el puerto de luz de emergencia
		statusEmergencyLight = 255;
		digitalWrite(OutEmergencyLight, LOW);
		break;

	case 1: // Activa el puerto de luz de emergencia y inicia la temporizacion
		statusEmergencyLight = 3;
		TimerEmergencyLight = TimerCurrent;
		digitalWrite(OutEmergencyLight, HIGH);
		break;

	case 2: // Aciva el puerto de luz de emergencia y lo mantiene encendido
		statusEmergencyLight = 255;
		digitalWrite(OutEmergencyLight, HIGH);
		break;

	case 3: // Temporizacion del tiempo activo del puerto de luz de emergencia
		if (TimerCurrent - TimerEmergencyLight >= intervalOutputEmergencyLight)
		{
			statusEmergencyLight = 0;
			break;
		}
		else
		{
			break;
		}

	default:
		break;
	}
}

/* Funcion para el control automatico */
void AutomaticControl()
{
	static unsigned long timerAutoCloseDoor = 0; // Temporizador cierre automatico de puertas
	static unsigned long timerOffAlarma = 0;	 // Temporizador desactivacion de alarma por golpe o apertura de puertas
	static unsigned long timerVss = 0;			 // Temporizador lectura de velocidad
	//static unsigned long timerAlert = 0;		 // Temporizador para reinicia del aviso

	/*Control para el cierre automatico de seguros*/
	switch (statusCloseAutomaticLocks)
	{
	case 0: // Desactiva la temporizacion para el cierre Automatico
		statusCloseAutomaticLocks = 255;
		break;
	case 1: // Inicia la temporizacion para el cierre Automatico
		statusCloseAutomaticLocks = 2;
		timerAutoCloseDoor = TimerCurrent;
		break;
	case 2: // Temporizador cierre automatico
		if (TimerCurrent - timerAutoCloseDoor >= intervalAutoCloseDoor)
		{
			statusCloseAutomaticLocks = 254;
			CloseDoors();
			break;
		}
		break;
	case 3: // Cierre automatico por velocidad
		statusCloseAutomaticLocks = 254;
		CloseDoors();
		break;
	default:
		break;
	}

	/*Control para el estado de la alarma*/
	switch (statusAlertAlarm)
	{
	case 0: // Desactiva la alarma
		statusAlertAlarm = 255;
		statusEmergencyLight = 0;
		statusBlinkClaxon = 0;
		break;
	case 1: // Activa la alarma
		statusAlertAlarm = 2;
		statusEmergencyLight = 2;
		statusBlinkClaxon = 1;
		blinkClaxon = 255;
		timerOffAlarma = TimerCurrent;
#if DEBUG
		if (statusInputs2 & BOOL1)
		{
			Serial2.println(F("Alarma Activada"));
		}
#endif
		break;
	case 2: // Temporizador para la desactivacion automatica
		if (TimerCurrent - timerOffAlarma >= intervalAutoOffAlarma)
		{
			statusAlertAlarm = 3;
			break;
		}
		else
		{
			break;
		}
	case 3: // Desactivacion de aviso alarma
		statusEmergencyLight = 0;
		statusBlinkClaxon = 0;
		statusCloseAutomaticLocks = 254;
		statusAlertAlarm = 255;
	default:
		break;
	}

	/*Control para el estado de lectura de velocidad*/
	switch (statusReadSpeed)
	{
	case 0: // Desactiva el cierre de seguros por velocidad
		statusReadSpeed = 255;
		detachInterrupt(digitalPinToInterrupt(InputVss));
		break;
	case 1: // Inicia el temporizador de la lectura de velocidad
		statusReadSpeed = 3;
		timerVss = TimerCurrent;
		break;
	case 2: // Reinica la lectura de velocidad
		statusReadSpeed = 4;
		speedVss = 0;
		PrepareInterrupts();
		break;
	case 3: // Temporizador de la lectura de velocidad
		if (TimerCurrent - timerVss >= delayVSS)
		{
			statusReadSpeed = 4;
			PrepareInterrupts();
			break;
		}
		else
		{
			break;
		}
	case 4: // Inicia la lectura de velocidad
		ReadVssSpeed();
		break;
	default:
		break;
	}

	/*Control para el estado de lectura del sensor de golpe*/
	switch (statusKnock)
	{
	case 0: // Desactiva la lectura del sensor de golpe
		statusKnock = 255;
		detachInterrupt(digitalPinToInterrupt(InputKnock));
		break;

	case 1: // Activa la lectura del sensor de golpe
		statusKnock = 2;
		KnockPrepareInterrupts();
		break;

	case 2: // Lectura del sensor
		KnockSignal();
		break;

	default:
		break;
	}

	/*Control para el estado de las alertas del sensor de golpe */
	switch (statusKnockAlert)
	{
	case 0: // Desactiva la alerta
		counterKnockAlert = 0;
		statusKnockAlert = 255;
		break;
	case 1: // Comprueba el numero de avisos que se a emitido
		if (counterKnockAlert > alertCounter)
		{
			statusBlinkClaxon = 1;
			statusEmergencyLight = 1;
			blinkClaxon = 2;
			statusKnockAlert = 0;
			break;
		}
		else
		{
			statusKnockAlert = 2;
			break;
		}
		break;
	case 2: // Activa el aviso por golpe leve
		statusKnockAlert = 255;
		blinksSirena = 5;
		statusBlinkSirena = 1;
		statusEmergencyLight = 1;
		counterKnockAlert++;
		break;
	case 3: // Activa el aviso por golpe fuerte
		statusAlertAlarm = 1;
		statusKnockAlert = 0;
		break;
	default:
		break;
	}
}

/*Funcion para la apertura de seguros*/
void OpenDoors()
{
	if (statusInputs & BOOL8)
	{
		statusFadeCabin = 2;
		statusKnock = 0;
		if (statusAlertAlarm == 2)
		{
			statusAlertAlarm = 0;
		}
		if (statusBlinkSirena == 255)
		{
			statusBlinkSirena = 1;
			if (statusOpenDoorsLocks == 254)
			{
				blinksSirena = 1;
			}
			else
			{
				blinksSirena = 2;
			}
		}
		if (statusEmergencyLight == 255)
		{
			statusEmergencyLight = 1;
		}
	}
	if (statusOpenDoorsLocks == 255)
	{
		statusOpenDoorsLocks = 1;
		if (!(statusInputs & BOOL7))
		{
			statusCloseAutomaticLocks = 0;
		}
		else
		{
			if (statusInputs2 & BOOL2)
			{
				statusCloseAutomaticLocks = 1;
			}
			else
			{
				statusCloseAutomaticLocks = 0;
			}
		}
	}
}

/*Funcion para el cierre de seguros*/
void CloseDoors()
{
	statusCloseAutomaticLocks = 254;
	if (statusInputs & BOOL8)
	{
		statusKnock = 1;
		if (statusBlinkSirena == 255)
		{
			statusBlinkSirena = 1;
			if (statusCloseDoorsLocks == 254)
			{
				blinksSirena = 1;
			}
			else
			{
				blinksSirena = 3;
			}
		}
		if (statusEmergencyLight == 255)
		{
			statusEmergencyLight = 1;
		}
		if (statusAlertAlarm == 2)
		{
			statusAlertAlarm = 3;
		}
	}
	if (statusCloseDoorsLocks == 255)
	{
		statusCloseDoorsLocks = 1;
		statusFadeCabin = 5;
	}
	return;
}

/* Funcion para abrir el maletero */
void OpenTrunk()
{
	if (statusInputs & BOOL2)
	{
		if (statusOpenTrunk == 255)
		{
			statusOpenTrunk = 1;
		}
	}
	if (statusInputs & BOOL8)
	{
		statusCloseAutomaticLocks = 0;
		statusKnock = 0;
		if (statusEmergencyLight == 255)
		{
			statusEmergencyLight = 1;
		}
		if (statusAlertAlarm == 2)
		{
			statusAlertAlarm = 0;
		}
		if (statusBlinkSirena == 255)
		{
			statusBlinkSirena = 1;
			if (statusInputs & BOOL2)
			{
				blinksSirena = 5;
			}
			else
			{
				blinksSirena = 1;
			}
		}
	}
	return;
}

/*Funcion para activar el modo casa de la alarma*/
void HomeMode()
{
	RFControl.disableReceive();
	attachInterrupt(digitalPinToInterrupt(InputDoor1), wakeUp, FALLING);
	LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
	detachInterrupt(digitalPinToInterrupt(InputDoor1));
	RFControl.enableReceive(digitalPinToInterrupt(InputRF));
}

/*Funcion para el envio de la informacion por puerto serie*/
void SendData()
{
	/*
	LISTA DE COMANDOS DISPONIBLES
	<P#,#> = Identifica el estado de la puerta en la que se encuentra
	<V,#> = Mansaje del estado de la velocidad
	*/
	if (statusSerialPort & BOOL3)
	{
		
		if (statusInputs2 & BOOL1)
		{
			String sendData;
			sendData += StartMark;
			sendData += " ";
			if (statusSendData & BOOL1)
			{
				statusSendData &= ~BOOL1;
				sendData += "V:";
				sendData += speedVss;
				sendData += " ";
			}
			if (statusSendData2 & BOOL1)
			{
				statusSendData2 &= ~BOOL1;
				sendData += "C:";
				sendData += !bool(statusInputs & BOOL1);
				sendData += " ";
			}
			if (statusSendData2 & BOOL2)
			{
				statusSendData2 &= ~BOOL2;
				sendData += "M:";
				sendData += !bool(statusInputs & BOOL2);
				sendData += " ";
			}
			if (statusSendData2 & BOOL3)
			{

				statusSendData2 &= ~BOOL3;
				sendData += "P1:";
				sendData += !bool(statusInputs & BOOL3);
				sendData += " ";
			}
			if (statusSendData2 & BOOL4)
			{
				statusSendData2 &= ~BOOL4;
				sendData += "P2:";
				sendData += !bool(statusInputs & BOOL4);
				sendData += " ";
			}
			if (statusSendData2 & BOOL5)
			{
				statusSendData2 &= ~BOOL5;
				sendData += "P3:";
				sendData += !bool(statusInputs & BOOL5);
				sendData += " ";
			}
			if (statusSendData2 & BOOL6)
			{
				statusSendData2 &= ~BOOL6;
				sendData += "P4:";
				sendData += !bool(statusInputs & BOOL6);
				sendData += " ";
			}
			if (statusSendData2 & BOOL7)
			{
				statusSendData2 &= ~BOOL7;
				sendData += "I:";
				sendData += !bool(statusInputs & BOOL7);
				sendData += " ";
			}
			if (statusSendData2 & BOOL8)
			{
				statusSendData2 &= ~BOOL8;
				sendData += "K:";
				sendData += !bool(statusInputs & BOOL8);
				sendData += " ";
			}
			sendData += EndMark;

			Serial2.println(sendData);
		}
		statusSerialPort &= ~BOOL3;
	}
}