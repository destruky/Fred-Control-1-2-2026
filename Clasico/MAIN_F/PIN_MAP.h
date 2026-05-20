#ifndef PIN_MAP_h
#define PIN_MAP_h

#define C1            18               // RAMPS: Z-MIN para señal A encoder
#define C2            19               // RAMPS: Z-MAX para señal B encoder 

#define pinFan         8               // RAMPS: D8 (terminales HEATED BED) — fan deshabilitado en pruebas

#define pinMotor       9               // RAMPS: Pin D9 para control PWM

#define pinHotend     10               // RAMPS: D10 para heater (terminales D10 del MOSFET)
#define termPin       15               // RAMPS: Pin T2 para termistor (A15 del Arduino) — T0/T1 dañados

#endif