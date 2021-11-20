#ifndef HARDWARE_CHECK
    #define HARDWARE_CHECK

    #define HARDWARE_CHECK_VERSION      "Hardware_check v1.0"

    #ifndef STM32F1
      #error Este codigo esta diseñado para correr en la plataforma STM32F103C8 (Bluepill). Por favor revise su configuración Tools->Board setting.
    #endif
#endif