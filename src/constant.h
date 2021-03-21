#pragma once
#ifndef _CONSTANT_H
#define _CONSTANT_H

// Constantes Knock
#define KnockSlight 240 // Valor bajo para el intervalo de lectura del sensor de golpe
#define KnockHard 350   // Valor alto para el intervalo de lectura del sensor de golpe

// Constantes Serial Port
#define lengthBufferIn 20  // Tamaño del vector para datos de entrada
#define lengthBufferOut 20 // Tamaño del vector para la salida de datos

// Constantes efecto fade

#define delayStartFadeCabin 10000 // Tiempo de espera para inicio del efecto fade de la cabina
#define intervalFadeCabin 25      // Intervalo para el efecto fade

#define delayStartFadeTrunk 10000 // Tiempo de espera para inicio del efecto fade del maletero
#define intervalFadeTrunk 25      // Intervalo para el efecto fade

// Constantes blink sirena
#define blinkSirenOn 80  // Tiempo de encendido de la sirena
#define blinkSirenOff 80 // Tiempo de apagado de la sitena

// Constantes blink claxon
#define blinkClaxonOn 200  // Tiempo de encendido del claxon
#define blinkClaxonOff 700 // Tiempo de apagado del claxon

// Temporizador Salidas
#define intervalOutputTrunk 400           // Tiempo de encendido de la salida apertura maletero
#define intervalOutputOpenDoors 100       // Tiempo de encendido de la salida apertura de seguros
#define intervalOutputCloseDoors 100      // Tiempo de encendido de la salida cierre de seguros
#define intervalOutputEmergencyLight 1200 // Tiempo de encendido de la salida de luz de emergencias

/*Constantes Control Automatico */
// Valores temporizadores
#define intervalAutoCloseDoor 30000 // Tiempo de espera para el cierre automatico de los seguros
#define intervalAutoOffAlarma 30000 // Tiempo de espera para desactivar el aviso de alarma
// Valores aviso alarma
#define alertCounter 3 // Numero de veces que debe alertar para avisar con claxon

// Temporizador Lectura VSS
#define delayVSS 3000 // Tiempo de espera para el inicio de lectura de velocidad

#endif