#pragma once
#ifndef _CONFIGURATION_H
#define _CONFIGURATION_H

//Version del codigo
#define _Version "1.0.0"

/*Depuracion*/
#define DEBUG 1 // Estado de la depuracion

// Configuraciones para el puerto serial
#define SerialBaudRate 115200 // Velocidad de conunicacion puerto serial
#define StartMark '<'         // Caracter de inicio de trama de comunicacion
#define EndMark '>'           // Caracter de finla de trama de comunicacion

// Configuraciones para la lectura de velocidad (VSS)
#define VSSPulsePerRevolution 4.858251013 // Pulsos por revolucion VSS
#define VSSWheelPerimeter 1.932079482     // Distancio por pulso VSS
#define VSSCloseDoors 30                  // Velocidad para el cierre de seguros VSS

#endif