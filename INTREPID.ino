/**
 * Nome progetto            IN one T-ouch R-ewrite E-mitted P-hase from motorbike I-AT D-evice
 *                          INTREPID - SMART IAT
 * Creato da                Pietro Terracciano
 *                          pterracciano95@gmail.com
 * Data creazione           15 Novembre 2018
 * Prima release            0.181115
 * Data ultima modifica     04 Dicembre 2018
 * Release attuale          0.181204
 */
 

/**
 * ==========
 * INCLUSIONI
 * ==========
 */
#include <math.h>
#include <EEPROM.h>
#include <OneWire.h> 
#include <DallasTemperature.h>
#include <NTCThermistorCalculator.h>
/* ======================== */


/**
 * ========================
 * VARIABILI GLOBALI ERRORI
 * ========================
 * 
 * 1  Impossibile trovare il Sensore di Temperatura
 * 2  Impossibile richiedere informazioni al Sensore di Temperatura
 * 3  Impossibile leggere informazioni dal Sensore di Temperatura 
 * 4  Impossibile trovare il Sensore IAT della moto
 * 5  Problema sconosciuto
 */
const uint8_t _ui8ERROR_ID__IMPOSSIBLE_TO_FIND_TMPSNS               = 1;
const uint8_t _ui8ERROR_ID__IMPOSSIBLE_TO_REQUEST_INFO_TO_TMPSNS    = 2;
const uint8_t _ui8ERROR_ID__IMPOSSIBLE_TO_READ_INFO_FROM_TMPSNS     = 3;
const uint8_t _ui8ERROR_ID__IMPOSSIBLE_TO_FIND_MB_IATSNS            = 4;
const uint8_t _ui8ERROR_ID__UNKNOWN_PROBLEM                         = 5;
/* ======================== */


/*
 * ====================
 * IMPOSTAZIONI ARDUINO
 * ====================
 * 
 * Ho deciso di utilizzare una scheda Arduino compatibile:
 * la Elegoo Nano V3.0 basata sul microcontroller ATmega328P
 * 
 * Elegoo Nano V3.0
 *    Microcontroller USB                         CH340
 *    Microcontroller                             ATmega328P
 *    Numero pin digitali                         14
 *    Numero pin analogici                        8
 *    Digital Pins con funzionalità Interrupt     2, 3
 *    Digital Pins con funzionalità PWM           3, 5, 6, 9, 10, 11
 *    Memoria flash                               32KB
 *    EEPROM                                      1KB
 *    SRAM                                        1KB
 *    
 *    ADC
 *    For a 16 MHz Arduino the ADC clock is set to 16 MHz/128 = 125 KHz. Each conversion in AVR takes 13 ADC clocks so 125 KHz /13 = 9615 Hz.
 *    @reference https://arduino.stackexchange.com/questions/699/how-do-i-know-the-sampling-frequency
 */
const bool      _bIS_DEBUG_MODE_ACTIVE                  = false;

const uint8_t   _ui8DIRECT_VLTIN_SGN_PIN                = 2;            // Pin

const uint8_t   _ui8BUILTIN_LED_PIN                     = LED_BUILTIN;
const uint8_t   _ui8ERROR_LED_PIN                       = A3;
const uint8_t   _ui8PROCESSING_LED_PIN                  = A4;
const uint8_t   _ui8READY_LED_PIN                       = A5;

const uint8_t   _ui8DGTPINs_NUMBER                      = 14;           // Adimensionale
const uint8_t   _ui8ANLPINs_NUMBER                      = 8;            // Adimensionale
/* ***************************************************** */


/* 
 * ===============================================
 * Impostazioni del SENSORE DI TEMPERATURA DS18B20
 * ===============================================
 * 
 * Il sensore di temperatura DS18B20 consente di 
 * leggere un buon range di valori. Inoltre possiamo 
 * decidere la risoluzione in bits di funzionamento 
 * che andrà ad influire sulla sensibilità dello strumento.
 * E' chiaro che migliorando la sensibilità, andremo a 
 * diminuire la velocità di lettura dell'informazione
 * 
 * Sensibilità in bits    Sensibilità in °C     Velocità di lettura 
 *  9bit                  0.5000°C               93.75ms
 * 10bit                  0.2500°C              187.50ms
 * 11bit                  0.1250°C              375.00ms
 * 12bit                  0.0625°C              750.00ms
 * 
 * Il pin A0 che abbiamo scelto si comporta come un
 * BUS in quanto la libreria DallasTemperature supporta
 * più sensori di temperatura sullo stesso canale. Ogni
 * sensore può essere utilizzato semplicemente considerando
 * l'index (posizione) del sensore stesso sul bus
 * 
 * A0
 * ----------- DS18B20_0 ----------- DS18B20_1 -------- ...._N ---
 * 
 * Nel nostro caso avendo un singolo sensore utilizzeremo
 * sempre index 0
 * 
 * Bisogna fare una considerazione importante: ogni DS18B20 ha 
 * un proprio "DeviceAddress" interno univoco, simile ad un "MAC
 * Address", possiamo utilizzarlo al posto dell'index per migliorare
 * notevolmente il tempo di risposta. Ciò è vero perchè l'index 
 * posizionale 0, 1, 2, ... viene utilizzato proprio per trovare
 * il "DeviceAddress", che a sua volta viene utilizzato in tutte le
 * operazioni della libreria DallasTemperature. Quindi utilizzando 
 * direttamente l'indirizzo del dispositivo, possiamo ottenere dei
 * tempi di responso più brevi
*/
const uint8_t   _ui8TMPSNs__BUS                             = A0;       // Pin
const uint8_t   _ui8TMPSNS__BITs_RESOLUTION                 = 11;       // bits
const float     _fTMPSNS__REFRACTORY_TMP_ZONE               = 0.010;    // °C
/* ================================================ */


/** 
 * =======================================
 * Impostazioni del SENSORE IAT della MOTO
 * =======================================
 * 
 * Non conosco la "natura" del sensore IAT della moto.
 * Generalmente i sensori IAT sono dei termistori NTC,
 * pertanto posso "approssimarne" le caratteristich
 * utilizzando le formule di Steinhart-Hart
 * 
 * Equazione di Steinhart-Hart
 * https://it.wikipedia.org/wiki/Equazione_di_Steinhart-Hart
 * 
 * Una volta trovata la migliore approssimazione del
 * termistore IAT posso generare un segnale di output
 * "personalizzato" da inviare alla ECU della moto
 * 
 * C'è però da fare una considerazione importantissima!
 * Un termistore si comporta similmente ad un sensore di
 * temperatura, ciò vuol dire che ad una differenza di
 * potenziale di pochi mV è associata una variazione di 
 * temperatura che può essere consistente. 
 * Il problema è che il microcontroller ATmega328P 
 * fornisce un campionamento di soli 8bit in uscita 
 * per i segnali PWM. Dal momento che la tensione 
 * massima è pari a 5,000V vuol dire che i segnali PWM, 
 * convertiti in segnali analogici attraverso un DAC, 
 * consentono di ottenere una precisione di appena 
 * 0,019607V/LVLEnergetico, a discapito di errori.
 * Alla ECU della moto arriverà un segnale errato già
 * prima di effettuarne una "personalizzazione".
 * Per fortuna studiando l'architettura dell'ATmega328P
 * si scopre che la MCU utilizza 3 timers per "generare" 
 * le onde quadre che sono collegati a dei registri 
 * interni al processore. Andando a "modificare"
 * i valori di tali registri è possibile in qualche
 * modo impostare un campionamento PWM maggiore a discapito
 * della frequenza. Ogni timers possiede esattamente 
 * 2 comparatori ed ogni comparatore è collegato ad 
 * uno ed un solo pin digitale
 * 
 * Timers     Comparators     Digital Pin     Pin Name
 * T0         OC0A            6               PD6            
 *            OC0B            5               PD5
 * T1         OC1A            9               PB1*
 *            OC1B            10              PB2*
 * T2         OC2A            11              PB3
 *            OC2B            3               PD3
 *            
 * *Pins che possono essere utilizzati
 * 
 * Inoltre :
 *    1.  è opportuno utilizzare sempre pin derivanti
 *        dallo stesso comparatore per evitare problemi di 
 *        "frequenze" non sincrone
 *    2.  è opportuno evitare di "modificare" le impostazioni
 *        sul Timer0 perchè è collegato direttamente alle
 *        funzioni delay(), micros(), ...
 *    3. il Timer1 è l'unico a possedere "registri" a 16bit
 */
const uint8_t   _ui8MB__IATSNS__SGN_PIN                     = A1;       // Pin
const uint8_t   _ui8MB__IATSNS__ALTERED_SGN_PIN             = 9;        // Pin PWM @10bit impostato in setup()

const uint8_t   _ui8MB__CRBLVL__TACTILE_BUTTON_PIN          = 3;        // Pin

const uint8_t   _ui8MB__CRBLVL__L0_LED_PIN                  = 4;        // Pin
const uint8_t   _ui8MB__CRBLVL__L1_LED_PIN                  = 5;        // Pin
const uint8_t   _ui8MB__CRBLVL__L2_LED_PIN                  = 6;        // Pin
const uint8_t   _ui8MB__CRBLVL__L3_LED_PIN                  = 7;        // Pin
const uint8_t   _ui8MB__CRBLVL__L4_LED_PIN                  = 8;        // Pin
const uint8_t   _ui8MB__CRBLVL__L5_LED_PIN                  = 11;       // Pin
const uint8_t   _ui8MB__CRBLVL__L6_LED_PIN                  = 12;       // Pin

const float     _fMB__IATSNS__REFRACTORY_RSS_ZONE           = 1;        // Ohm
/* ======================================= */

/* VARIABILI GLOBALI *********************************** */
uint8_t     _ui8ErrorID                                           = NAN;
uint8_t     _ui8ErrorLEDBlinks                                    = NAN;
uint8_t     _ui16MLLSCNElapsedInErrorRoutine                      = NAN;

uint16_t    _ui16MLLSCNElapsedInLoopRoutine                       = NAN;        // Overflow dopo 65'536ms ~ 65s

bool        _bInVoltageDropDetectedRoutine                        = NAN;
uint16_t    _ui16MLLSCNElapsedInVoltageDropDetectedRoutine        = NAN;

uint8_t     _ui8TotalPinsNumber                                   = NAN;
uint8_t     _ui8MB__CRBLVL__PreSelectedLEDPin                     = NAN;
uint8_t     _ui8MB__CRBLVL__SelectedLEDPin                        = NAN;

bool        _bMB__CRBLVL__IsTactileButtonPressedInRefractoryMode  = NAN;
bool        _bMB__CRBLVL__InTactileButtonPressedRoutine           = NAN;
uint16_t    _ui16MLLSCNElapsedInTactileButtonPressedRoutine       = NAN;

bool        _bMB__CRBLVL__IsDataAltered                           = NAN;

bool        _bMB__IATSNS__IsDataAltered                           = NAN;
uint16_t    _ui16MB__IATSNS__LastDataRead                         = NAN;
/* ***************************************************** */

/** Puntatore al bus dei sensori di temperatura */
OneWire* _OneWire;
/** Puntatore ad un tipo DallasTemperature */
DallasTemperature* _DallasTemperatureController;
/** 
 * Imposto una variabile per il salvataggio 
 * del DeviceAddress del sensore di temperatura 
 * DS18B20 collegato sul bus _ui8TMPSNs__BUS
 * identificato da _OneWire
 */
DeviceAddress _abTMPSNS__DeviceAddress;
/** Puntatore ad un tipo NTCThermistorCalculator */
NTCThermistorCalculator* _NTCThermistorCalculator;

void setup() {
  //if(_bIS_DEBUG_MODE_ACTIVE)
    Serial.begin(9600);
    
  /**
   * ==================================
   * INIZIALIZZAZIONE VARIABILI GLOBALI
   * ==================================
   */
  _ui8ErrorID = 0;
  _ui8ErrorLEDBlinks = 0;
  _ui16MLLSCNElapsedInErrorRoutine = 0;

  _ui16MLLSCNElapsedInLoopRoutine = 0;
  
  _ui8TotalPinsNumber = _ui8DGTPINs_NUMBER + _ui8ANLPINs_NUMBER;

  _bInVoltageDropDetectedRoutine = false;
  _ui16MLLSCNElapsedInVoltageDropDetectedRoutine = 0;

  _ui8MB__CRBLVL__SelectedLEDPin      = 0;
  _ui8MB__CRBLVL__PreSelectedLEDPin   = 0;

  _bMB__CRBLVL__InTactileButtonPressedRoutine           = false;
  _bMB__CRBLVL__IsTactileButtonPressedInRefractoryMode  = false;
  _bMB__CRBLVL__IsDataAltered                           = false;
  
  _ui16MLLSCNElapsedInTactileButtonPressedRoutine = 0;

  _bMB__IATSNS__IsDataAltered = false;
  _ui16MB__IATSNS__LastDataRead = 0;
  /** ================================== */


  /**
   * =================================================
   * INIZIALIZZAZIONE LIBRERIA NTCThermistorCalculator
   * =================================================
   */
  _NTCThermistorCalculator = new NTCThermistorCalculator();
  _NTCThermistorCalculator->setTemperatureUnit(NTCThermistorCalculator::CELSIUS__TEMPERATURE_UNIT);
  /** ================================== */

  _NTCThermistorCalculator->throwTemperature(0);
  //_NTCThermistorCalculator->throwTemperature(25);
  _NTCThermistorCalculator->throwTemperature(50);

  _NTCThermistorCalculator->throwResistance(4000);
  //_NTCThermistorCalculator->throwResistance(10000);
  _NTCThermistorCalculator->throwResistance(25000);

  Serial.println("Alpha Coefficient");
  Serial.println(_NTCThermistorCalculator->calcSHAlphaCoefficient(), 16);

  Serial.println("Beta Coefficient");
  Serial.println(_NTCThermistorCalculator->calcSHBetaCoefficient(), 16);

  Serial.println("Gamma Coefficient");
  Serial.println(_NTCThermistorCalculator->calcSHGammaCoefficient(), 16);

  Serial.println("Beta Parameter");
  Serial.println(_NTCThermistorCalculator->calcBetaParameter(), 16);
  return;
  /**
   * =======================================
   * INIZIALIZZAZIONE SENSORE DI TEMPERATURA
   * =======================================
   * 1  Init del bus _ui8TMPSNs__BUS utilizzato 
   *    dai sensori di temperatura
   * 2  Init del DallasTemperatureController che
   *    gestisce il bus _ui8TMPSNs__BUS descritto
   *    tramite il modello _OneWire
   * 3  Ricerca del DeviceAddress del sensore di
   *    temperatura in posizione 0 (Prima posizione sul bus)
   * 4  Imposta risoluzione in bits del sensore
   *    di temperatura in posizione 0
   * 5  Imposta i metodi del DallasTemperatureController
   *    come asincroni
   */
  _OneWire = new OneWire(_ui8TMPSNs__BUS);
  _DallasTemperatureController = new DallasTemperature();
  _DallasTemperatureController->setOneWire(_OneWire);
  _DallasTemperatureController->begin();
  if(_DallasTemperatureController->getAddress(_abTMPSNS__DeviceAddress, 0)) {
    _DallasTemperatureController->setResolution(_abTMPSNS__DeviceAddress, _ui8TMPSNS__BITs_RESOLUTION, true);
    _DallasTemperatureController->setWaitForConversion(false);
  }
  else
    setErrorByID(_ui8ERROR_ID__IMPOSSIBLE_TO_FIND_TMPSNS);
  /** ======================================= */


  /**
   * Considerando i seguenti valori bitmask
   * with OR otteniamo:
   *    Fast PWM non invertito
   *    Valore TOP contenuto in ICR1
   *    Valore Duty Cicle PB1 contenuto in OCR1A
   *    Valore Duty Cicle PB2 contenuto in OCR1B
   *    Frequenza Fast PWM ~ 16 / 8 / 2^ICR1Value * 1 000 000
   *    
   * Inoltre ho deciso di utilizzare 1 << VALORE invece di
   * _BV(VALORE) perchè il primo è più veloce e più preciso
   */  
  TCCR1A = 1 << COM1A1 | 1 << COM1B1 |  1 << WGM11;
  TCCR1B = 1 << WGM13 | 1 << WGM12 | 1 << CS11;
  
  /**
   * Dal momento che il valore TOP viene letto dal
   * registro ICR1, possiamo decidere facilmente la 
   * risoluzione del Fast PWM
   *
   *    Esadecimale    Risoluzione in bits     Risoluzione in Livelli energetici    Frequenza PWM
   *    0x01ff          9bit                   0 @ 511                              ~3906.2500Hz
   *    0x03ff         10bit                   0 @ 1023                             ~1953.1250Hz
   *    0x07ff         11bit                   0 @ 2047                             ~ 976.5625Hz
   *    
   * L'ATmega328P riesce a convertire un segnale 
   * analogico in un segnale digitale con una risoluzione
   * massima di 10bit. Siccome questo sketch deve
   * modificare un segnale di input e generarne un nuovo
   * output, è inutile utilizzare una risoluzione di output
   * maggiore di 10bit
   * 
   * Si conclude che l'impostazione migliore per il PWM
   * è una risoluzione massima di 10bit
   */
  ICR1 = 0x03ff;

  /** Imposto il Duty Cicle di PB1 e di PB2 a 0 */
  OCR1A = 0;
  OCR1B = 0;

  MB__CRBLVL__preSelectByLEDPin( _ui8MB__CRBLVL__L0_LED_PIN );
  MB__CRBLVL__confirmSelection();
  
  /**
   * =====================
   * INIZIALIZZAZIONE PINS
   * =====================
   */
  pinMode(_ui8BUILTIN_LED_PIN,    OUTPUT);
  pinMode(_ui8ERROR_LED_PIN,      OUTPUT);
  pinMode(_ui8PROCESSING_LED_PIN, OUTPUT);
  pinMode(_ui8READY_LED_PIN,      OUTPUT);

  pinMode(_ui8DIRECT_VLTIN_SGN_PIN,              INPUT);

  pinMode(_ui8TMPSNs__BUS,        INPUT);
  
  pinMode(_ui8MB__IATSNS__SGN_PIN,              INPUT);
  pinMode(_ui8MB__IATSNS__ALTERED_SGN_PIN,      OUTPUT);
  
  pinMode(_ui8MB__CRBLVL__TACTILE_BUTTON_PIN,   INPUT);
  pinMode(_ui8MB__CRBLVL__L0_LED_PIN,           OUTPUT);
  pinMode(_ui8MB__CRBLVL__L1_LED_PIN,           OUTPUT);
  pinMode(_ui8MB__CRBLVL__L2_LED_PIN,           OUTPUT);
  pinMode(_ui8MB__CRBLVL__L3_LED_PIN,           OUTPUT);
  pinMode(_ui8MB__CRBLVL__L4_LED_PIN,           OUTPUT);
  pinMode(_ui8MB__CRBLVL__L5_LED_PIN,           OUTPUT);
  pinMode(_ui8MB__CRBLVL__L6_LED_PIN,           OUTPUT); 
  /** ===================== */

  /**
   * ===========================
   * INIZIALIZZAZIONE INTERRUPTS
   * ===========================
   */
  attachInterrupt(digitalPinToInterrupt(_ui8DIRECT_VLTIN_SGN_PIN), onVoltageDropDetected, LOW);  
  attachInterrupt(digitalPinToInterrupt(_ui8MB__CRBLVL__TACTILE_BUTTON_PIN), MB__CRBLVL__onTactileButtonPressed, FALLING);
  /** ===================== */
}

void saveDataToEEPROM() {
  String sMillis = (String)millis();
  char sEEPROMValue[32];
  sMillis.toCharArray(sEEPROMValue, 32);
  EEPROM.put(0,  sEEPROMValue);
  EEPROM.put(64,  sEEPROMValue);
  EEPROM.put(128,  sEEPROMValue);
  EEPROM.put(256,  sEEPROMValue);
  writeDigitalHighSignal( _ui8BUILTIN_LED_PIN );
}

void loadDataFromEEPROM() {
  char sEEPROMValue[32];
  EEPROM.get(0, sEEPROMValue);
}

/** 
 * ==================
 * FUNZIONI GENERICHE
 * ==================
 */
bool isOnError() { return _ui8ErrorID > 0; }

void setErrorByID(uint8_t ui8ErrorID) {
  if(
    ui8ErrorID == _ui8ERROR_ID__IMPOSSIBLE_TO_FIND_TMPSNS
    ||
    ui8ErrorID == _ui8ERROR_ID__IMPOSSIBLE_TO_REQUEST_INFO_TO_TMPSNS
    ||
    ui8ErrorID == _ui8ERROR_ID__IMPOSSIBLE_TO_READ_INFO_FROM_TMPSNS
    ||
    ui8ErrorID == _ui8ERROR_ID__IMPOSSIBLE_TO_FIND_MB_IATSNS
  )
    _ui8ErrorID = ui8ErrorID;
  else
    ui8ErrorID = _ui8ERROR_ID__UNKNOWN_PROBLEM;

  _ui16MLLSCNElapsedInErrorRoutine = 0;
}

void clearError() { _ui8ErrorID = 0; }
/** ================== */


/** 
 * ================
 * FUNZIONI ARDUINO
 * ================
 */
bool isValidPin(uint8_t ui8DP) { return !isnan(ui8DP) && ui8DP > 0 && ui8DP <= _ui8TotalPinsNumber; }

bool writeDigitalHighSignal(uint8_t ui8DP) {
  if ( !isValidPin(ui8DP) )
    return false;

  digitalWrite(ui8DP, HIGH);
  return true;
}

bool writeDigitalLowSignal(uint8_t ui8DP) {
  if ( !isValidPin(ui8DP) )
    return false;

  digitalWrite(ui8DP, LOW);
  return true;
}

bool writeAnalogSignalUsingICR1(uint8_t ui8DP, uint16_t ui16DutyCicleInENGLVLUnits)
{
  if ( isnan(ui8DP) || ( ui8DP != 9 && ui8DP != 10) )
    return false;

  //if( ICR1 < 0 )
   // ICR1 = 0x0000;

  OCR1A = ui16DutyCicleInENGLVLUnits % 1024; //ICR1 da convertire binario to dec
  OCR1B = ui16DutyCicleInENGLVLUnits % 1024;

  return true;
}

void onVoltageDropDetected() {
  if(_bInVoltageDropDetectedRoutine)
    return;

  _bInVoltageDropDetectedRoutine = true;
  _ui16MLLSCNElapsedInVoltageDropDetectedRoutine = 0;

  if(!_bIS_DEBUG_MODE_ACTIVE)
    saveDataToEEPROM();
  else
    Serial.println("onVoltageDropDetected()->saveDataToEEPROM()");
}
/** ================ */


/** 
 * =============
 * FUNZIONI MOTO
 * =============
 */
void MB__IATSNS__readAndStoreData() {
  uint16_t ui16DataRead = analogRead( _ui8MB__IATSNS__SGN_PIN );
  _bMB__IATSNS__IsDataAltered = _ui16MB__IATSNS__LastDataRead != ui16DataRead;
  _ui16MB__IATSNS__LastDataRead = ui16DataRead;
}

 
void MB__CRBLVL__onTactileButtonPressed() {
  if( _bMB__CRBLVL__IsTactileButtonPressedInRefractoryMode )
    return;
    
  _bMB__CRBLVL__IsTactileButtonPressedInRefractoryMode = true;
  _bMB__CRBLVL__InTactileButtonPressedRoutine = true;
  _ui16MLLSCNElapsedInTactileButtonPressedRoutine = 0;
  
  MB__CRBLVL__preSelectNext();
}

void MB__CRBLVL__preSelectNext() {
  if ( _ui8MB__CRBLVL__PreSelectedLEDPin == _ui8MB__CRBLVL__L0_LED_PIN )
    MB__CRBLVL__preSelectByLEDPin( _ui8MB__CRBLVL__L1_LED_PIN );
  else if ( _ui8MB__CRBLVL__PreSelectedLEDPin == _ui8MB__CRBLVL__L1_LED_PIN )
    MB__CRBLVL__preSelectByLEDPin( _ui8MB__CRBLVL__L2_LED_PIN );
  else if ( _ui8MB__CRBLVL__PreSelectedLEDPin == _ui8MB__CRBLVL__L2_LED_PIN )
    MB__CRBLVL__preSelectByLEDPin( _ui8MB__CRBLVL__L3_LED_PIN );
  else if ( _ui8MB__CRBLVL__PreSelectedLEDPin == _ui8MB__CRBLVL__L3_LED_PIN )
    MB__CRBLVL__preSelectByLEDPin( _ui8MB__CRBLVL__L4_LED_PIN );
  else if ( _ui8MB__CRBLVL__PreSelectedLEDPin == _ui8MB__CRBLVL__L4_LED_PIN )
    MB__CRBLVL__preSelectByLEDPin( _ui8MB__CRBLVL__L5_LED_PIN );
  else if ( _ui8MB__CRBLVL__PreSelectedLEDPin == _ui8MB__CRBLVL__L5_LED_PIN )
    MB__CRBLVL__preSelectByLEDPin( _ui8MB__CRBLVL__L6_LED_PIN );
  else if ( _ui8MB__CRBLVL__PreSelectedLEDPin == _ui8MB__CRBLVL__L6_LED_PIN )
    MB__CRBLVL__preSelectByLEDPin( _ui8MB__CRBLVL__L0_LED_PIN );
}

bool MB__CRBLVL__preSelectByLEDPin(uint8_t ui8DP) {
  if ( !MB__CRBLVL__isValidLEDPin( ui8DP ) )
    return false;
  else if ( MB__CRBLVL__isValidLEDPin( _ui8MB__CRBLVL__PreSelectedLEDPin ) )
    writeDigitalLowSignal( _ui8MB__CRBLVL__PreSelectedLEDPin );
    
  _ui8MB__CRBLVL__PreSelectedLEDPin = ui8DP;
  
  writeDigitalHighSignal( _ui8MB__CRBLVL__PreSelectedLEDPin );
  return true;
}

void MB__CRBLVL__confirmSelection() {
  _bMB__CRBLVL__IsDataAltered = true;
  _ui8MB__CRBLVL__SelectedLEDPin = _ui8MB__CRBLVL__PreSelectedLEDPin;
}

bool MB__CRBLVL__isValidLEDPin(uint8_t ui8DP) {
  return
    !isnan(ui8DP)
    &&
    (
      ui8DP == _ui8MB__CRBLVL__L0_LED_PIN
      ||
      ui8DP == _ui8MB__CRBLVL__L1_LED_PIN
      ||
      ui8DP == _ui8MB__CRBLVL__L2_LED_PIN
      ||
      ui8DP == _ui8MB__CRBLVL__L3_LED_PIN
      ||
      ui8DP == _ui8MB__CRBLVL__L4_LED_PIN
      ||
      ui8DP == _ui8MB__CRBLVL__L5_LED_PIN
      ||
      ui8DP == _ui8MB__CRBLVL__L6_LED_PIN
    );
}
/** *********************************************** */


/**
 * INTREPID deve
 *    1.  Convertire i segnali analogici provenienti 
 *        rispettivamente dal Sensore di Temperatura e dal
 *        Sensore IAT della moto in segnali digitali
 *    2.  Analizzare i segnali digitali ottenuti e ricreare
 *        in maniera più fedele possibile l'equazione
 *        del termistore che descrive il sensore IAT della
 *        moto
 *    3.  "Traslare" l'equazione del termistore...     
 * 
 * Il modulo ADC presente sull'ATmega328P effettua letture 
 * analogiche con conversione in digitale @~9600Hz. Siccome non
 * conosciamo le frequenze dei segnali analogici in ingresso,
 * sarebbe opportuno "saturare" l'intera banda di lettura ottenendo
 * il maggior numero di informazioni possibili ma dal momento che
 * il sensore di temperatura "adotta" una risoluzione di 11bit con
 * un tempo di lettura di ~375.00ms, è inutile tenere INTREPID
 * costantemente attivo
 * 
 * Se consideriamo la seguente tabella
 * 
 * DELAY us     DELAY ms        FREQUENZA MASSIMA 
 *  200us       0.200ms         5000.000Hz
 *  250us       0.250ms         4000.000Hz
 *  300us       0.300ms         3333.333Hz
 *  350us       0.350ms         2857.143Hz
 *  400us       0.400ms         2500.000Hz
 *  450us       0.450ms         2222.222Hz
 *  500us       0.500ms         2000.000Hz
 * 1000us       1.000ms         1000.000Hz
 * 1500us       1.500ms          666.667Hz
 * 2000us       2.000ms          500.000Hz
 * 
 * possiamo scegliere il valore 1ms = 1000us per il delay
 */
void loop() {
  return;
  /** 
   * ======================
   * ERROR LED ROUTINE
   * ======================
   * 
   * Questa routine accende e spegne, ad intervalli dinamici a 
   * seconda dell'ID errore, il LED rosso. Permettendoci di 
   * conoscere facilmente il tipo di errore che è stato riscontrato
   */
  if(isOnError()) {
    if(_ui8ErrorLEDBlinks >= _ui8ErrorID) {
      if(_ui16MLLSCNElapsedInErrorRoutine % 1000 == 0) {
        _ui8ErrorLEDBlinks = 0;
        _ui16MLLSCNElapsedInErrorRoutine = 0;
      }
    }
    else if(_ui16MLLSCNElapsedInErrorRoutine % (1000 / _ui8ErrorID) == 0 || _ui16MLLSCNElapsedInErrorRoutine == 0) {
      writeDigitalHighSignal( _ui8PROCESSING_LED_PIN );
    }
    else if(_ui16MLLSCNElapsedInErrorRoutine % 100 == 0) {
      writeDigitalLowSignal( _ui8PROCESSING_LED_PIN );
      _ui8ErrorLEDBlinks++;
    }

    _ui16MLLSCNElapsedInErrorRoutine++;
  }
  /** ===================== */


  /** 
   * =============================
   * VOLTAGE DROP DETECTED ROUTINE
   * =============================
   */
  if(_bInVoltageDropDetectedRoutine) {
    if(_ui16MLLSCNElapsedInVoltageDropDetectedRoutine % 1000 == 0)
      _bInVoltageDropDetectedRoutine = false;
      
    _ui16MLLSCNElapsedInVoltageDropDetectedRoutine++;
  }
  /** ===================== */

  /** 
   * ========================================
   * MB_CRBLVL TACTILE BUTTON PRESSED ROUTINE
   * ========================================
   */
  if(_bMB__CRBLVL__InTactileButtonPressedRoutine) {
    if ( _ui16MLLSCNElapsedInTactileButtonPressedRoutine >= 1000 )
      MB__CRBLVL__confirmSelection();
    else if(_ui16MLLSCNElapsedInTactileButtonPressedRoutine % 200 == 0 || _ui16MLLSCNElapsedInTactileButtonPressedRoutine == 0) {
      if(_ui16MLLSCNElapsedInTactileButtonPressedRoutine % 400 == 0) 
        _bMB__CRBLVL__IsTactileButtonPressedInRefractoryMode = false;
        
      writeDigitalHighSignal( _ui8PROCESSING_LED_PIN );
    }
    else if(_ui16MLLSCNElapsedInTactileButtonPressedRoutine % 100 == 0)
      writeDigitalLowSignal( _ui8PROCESSING_LED_PIN );
      
    _ui16MLLSCNElapsedInTactileButtonPressedRoutine++;
  }
  /** ===================== */
  
  
  /**
   * ======================
   * PROCESSING LED ROUTINE
   * ======================
   * 
   * Questa routine accende e spegne, ad intervalli regolari, il  
   * LED giallo. Consente di conoscere lo status di operatività
   * della scheda INTREPID: siamo in grado di capire se la scheda
   * Arduino sta chiamando correttamente il metodo loop()
   */
  if( _ui16MLLSCNElapsedInLoopRoutine % 1000 == 0 || _ui16MLLSCNElapsedInLoopRoutine == 0 )
    writeDigitalHighSignal( _ui8PROCESSING_LED_PIN );
  else if( _ui16MLLSCNElapsedInLoopRoutine % 100 == 0 )
    writeDigitalLowSignal( _ui8PROCESSING_LED_PIN );
  /** ===================== */


  /**  
   * =============================
   * ALTERED MB IAT SIGNAL ROUTINE
   * =============================
   */
   MB__IATSNS__readAndStoreData();

   /** CRB LEVEL ALTERED || TMPSNS ALTERED || IATSNS ALTERED ===> WRITE OUT SIGNAL */
  if(_bMB__CRBLVL__IsDataAltered || _bMB__IATSNS__IsDataAltered) {
    writeAnalogSignalUsingICR1( _ui8MB__IATSNS__ALTERED_SGN_PIN, 0000);
  }
  
   
  delay(1);
  
  _ui16MLLSCNElapsedInLoopRoutine++;
}
