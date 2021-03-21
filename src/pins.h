#pragma once
#ifndef _PINS_H
#define _PINS_H

/*PUERTO ENTRADA RF*/
#define InputRF 19

/*PUERTOS DE ENTRADA*/
#define InputDoor1 18   // Pin de entrada para la puerta 1 (Interrupcion externa necesaria)
#define InputCapo 22    // Pin de entrada para el capo
#define InputTrunk 23   // Pin de entrada para la cajuela
#define InputDoor2 25   // Pin de entrada para el puerta 2
#define InputDoor3 26   // Pin de entrada para el puerta 3
#define InputDoor4 27   // Pin de entrada para el puerta 4
#define InputIgn 28     // Pin de entrada para Ignicion
#define InputKey 29     // Pin de entrada para la llave
#define InputStateBt 30 // Pin de entrada para el estado de conexion bluetooth

/*PUERTOS INTERRUPCIONES EXTERNAS*/
#define InputVss 2   // Pin de entrada para la lectura de velocidad (Interrupcion requerida)
#define InputKnock 3 // Pin de entrada para el sensor de vibracion (Interrupcion requerida)

/*PUERTOS DE SALIDA*/
#define PwmLightCabin 44     // Pin de salida PWM para la luz de cortesia cabian (Salida PWM Invertida)
#define PwmLightTrunk 45     // Pin de salida PWM para la luz de cortesia maletero (Salida PWM Invertida)
#define OutSirenBlink 35     // Pin de salida para la sirena
#define OutHornBlink 36      // Pin de salida para el claxon
#define OutOpenTrunk 37      // Pin de salida para apertura de maletero
#define OutOpenDoors 38      // Pin de salida para apertura apertura seguros
#define OutCloseDoors 39     // Pin de salida para apertura cierre seguros
#define OutEmergencyLight 40 // Pin de salida para activacion de luz Emergencia
#define OutSegurityLight 41  // Pin de salida para activacion de indicacion de luz de seguridad (Salida Invertida)
#define OutCutPump 42        // Pin de salida para el corte de bomba de gasolina
#define OutCutCoil 43        // Pin de salida para el corte de alimentacion de bobinas
#endif
