#ifndef NTCThermistorCalculator_h
#define NTCThermistorCalculator_h

#if ARDUINO >= 100
#include <Arduino.h>
#else
#include "WProgram.h"
#endif

class NTCThermistorCalculator {
    private:
        double  _dMinTmp;
        double  _dAvgTmp;
        double  _dMaxTmp;
        double  _dMinRes;
        double  _dAvgRes;
        double  _dMaxRes;
        double  _dLastAvgTmp;
        double  _dLastAvgRes;
        double  _dLastSHAlphaCoef;
        double  _dLastSHBetaCoef;
        double  _dLastSHGammaCoef;
        double  _dLastSHGammaCoefPiece;
        double  _dLastBetaParameter;
        double  _dLastlnMaxRes;
        double  _dLastlnAvgRes;
        double  _dLastInvMinTmp;
        double  _dTmpRefractoryZone;
        double  _dResRefractoryZone;
        bool    _bAreThrownInfosForBetaParameterChanged;
        bool    _bAreThrownInfosForSHAlphaCoeffChanged;
        bool    _bAreThrownInfosForSHBetaCoeffChanged;
        bool    _bAreThrownInfosForSHGammaCoeffChanged;
        bool    _bIsReady;
        uint8_t _ui8TemperatureUnit;

    public:
        static const uint8_t CELSIUS__TEMPERATURE_UNIT      = 1;
        static const uint8_t FAHRENHEIT__TEMPERATURE_UNIT   = 2;
        static const uint8_t KELVIN__TEMPERATURE_UNIT       = 4;
        
    public:
        NTCThermistorCalculator();
        void setTemperatureUnit( uint8_t );
        void setTemperatureRefractoryZone( double );
        void throwTemperature( double );
        void setResistanceRefractoryZone( double );
        void throwResistance( double );
        bool isReady();
        bool areThrownInfosConsistent();
        double calcSHAlphaCoefficient();
        double calcSHBetaCoefficient();
        double calcSHGammaCoefficient();
        double calcBetaParameter();
};

#endif