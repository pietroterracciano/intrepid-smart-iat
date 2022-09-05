#include "NTCThermistorCalculator.h"

NTCThermistorCalculator::NTCThermistorCalculator() {
    _ui8TemperatureUnit = KELVIN__TEMPERATURE_UNIT;

    _dMinTmp = NAN;
    _dAvgTmp = NAN;
    _dMaxTmp = NAN;
    _dTmpRefractoryZone = NAN;
    
    _dMinRes = NAN;
    _dAvgRes = NAN;
    _dMaxRes = NAN;
    _dResRefractoryZone = NAN;

    _dLastAvgTmp = NAN;
    _dLastAvgRes = NAN;

    _dLastSHAlphaCoef = NAN;
    _dLastSHBetaCoef = NAN;
    _dLastSHGammaCoef = NAN;
    _dLastSHGammaCoefPiece = NAN;

    _dLastlnMaxRes = NAN;
    _dLastlnAvgRes = NAN;
    _dLastInvMinTmp = NAN;

    _dLastBetaParameter = NAN;

    _bIsReady =
        _bAreThrownInfosForSHAlphaCoeffChanged =
        _bAreThrownInfosForSHBetaCoeffChanged =
        _bAreThrownInfosForBetaParameterChanged =
        _bAreThrownInfosForSHGammaCoeffChanged = false;
}

void NTCThermistorCalculator::setTemperatureRefractoryZone( double dTmpRefractoryZone ) {
    _dTmpRefractoryZone = dTmpRefractoryZone;
}

void NTCThermistorCalculator::setTemperatureUnit( uint8_t ui8TemperatureUnit ) {
    if(
        ui8TemperatureUnit != CELSIUS__TEMPERATURE_UNIT
        && ui8TemperatureUnit != FAHRENHEIT__TEMPERATURE_UNIT
        && ui8TemperatureUnit != KELVIN__TEMPERATURE_UNIT
    )
        ui8TemperatureUnit = KELVIN__TEMPERATURE_UNIT;
    
    _ui8TemperatureUnit = ui8TemperatureUnit;
}

void NTCThermistorCalculator::throwTemperature( double dTmp )
{
    if( isnan(dTmp) )
        return;

    if( _ui8TemperatureUnit == CELSIUS__TEMPERATURE_UNIT)
        dTmp += 273.15;
    else if( _ui8TemperatureUnit == FAHRENHEIT__TEMPERATURE_UNIT)
        dTmp = (dTmp - 32.0) * 0.5555555555 + 273.15;

    if( isnan(_dMinTmp) || _dMinTmp >= dTmp )
        _dMinTmp = dTmp;
    if( isnan(_dMaxTmp) || _dMaxTmp <= dTmp )
        _dMaxTmp = dTmp;

    _dAvgTmp = ( _dMinTmp + _dMaxTmp ) / 2.0;

    if( _dLastAvgTmp != _dAvgTmp ) {
        _bAreThrownInfosForSHAlphaCoeffChanged =
            _bAreThrownInfosForBetaParameterChanged =
            _bAreThrownInfosForSHBetaCoeffChanged =
            _bAreThrownInfosForSHGammaCoeffChanged = true;

        _dLastAvgTmp = _dAvgTmp;
    }
}

void NTCThermistorCalculator::setResistanceRefractoryZone( double dResRefractoryZone ) {
    _dResRefractoryZone = dResRefractoryZone;
}

void NTCThermistorCalculator::throwResistance( double dRes )
{
    if( isnan(dRes) )
        return;

    if( isnan(_dMinRes) || _dMinRes >= dRes )
        _dMinRes = dRes;
    if( isnan(_dMaxRes) || _dMaxRes <= dRes )
        _dMaxRes = dRes;

    //_dAvgRes = ( _dMaxRes + _dMinRes ) / 2.0;

    if( isnan(_dAvgTmp) || _dAvgTmp == 0 || isnan(_dMinTmp) || _dMinTmp == 0 )
        _dAvgRes = ( _dMaxRes + _dMinRes ) / 2.0;
    else {
        calcBetaParameter();
        _dAvgRes = _dMaxRes * exp( ( 1 / _dAvgTmp - 1 / _dMinTmp ) * _dLastBetaParameter );
        //  + ( _dMaxRes + _dMinRes ) / _dLastBetaParameter

        Serial.println("_dAvgRes");
        Serial.println(_dAvgRes, 16);
        Serial.println();
    }

    if( _dLastAvgRes != _dAvgRes ) {
        _bAreThrownInfosForSHAlphaCoeffChanged =
            _bAreThrownInfosForBetaParameterChanged =
            _bAreThrownInfosForSHBetaCoeffChanged =
            _bAreThrownInfosForSHGammaCoeffChanged = true;

        _dLastAvgRes = _dAvgRes;
    }
}

double NTCThermistorCalculator::calcSHAlphaCoefficient() {
    if( !_bAreThrownInfosForSHAlphaCoeffChanged )
        return _dLastSHAlphaCoef;

    _bAreThrownInfosForSHAlphaCoeffChanged = false;

    if( isnan(_dLastSHBetaCoef) )
        calcSHBetaCoefficient();

    if( isnan(_dLastSHBetaCoef) )
        return NAN;
        
    if( isnan(_dLastSHGammaCoef) )
        calcSHGammaCoefficient();

    if( isnan(_dLastSHGammaCoef) || isnan(_dLastInvMinTmp) || isnan(_dLastlnMaxRes) )
        return NAN;

    _dLastSHAlphaCoef = _dLastInvMinTmp - _dLastlnMaxRes * ( _dLastSHBetaCoef + _dLastlnMaxRes * _dLastlnMaxRes * _dLastSHGammaCoef );
    
    return _dLastSHAlphaCoef; 
}

double NTCThermistorCalculator::calcSHBetaCoefficient() {
    if( !_bAreThrownInfosForSHBetaCoeffChanged )
        return _dLastSHBetaCoef;

     _bAreThrownInfosForSHBetaCoeffChanged = false;

    if( isnan(_dLastSHGammaCoef) )
        calcSHGammaCoefficient();

    if( 
        isnan(_dLastSHGammaCoef) 
        || isnan(_dLastSHGammaCoefPiece)
        || isnan(_dLastlnMaxRes)
        || isnan(_dLastlnAvgRes)
    )
        return NAN;

    _dLastSHBetaCoef =
        _dLastSHGammaCoefPiece -
        _dLastSHGammaCoef *
        ( 
            _dLastlnMaxRes * _dLastlnMaxRes +
            _dLastlnMaxRes * _dLastlnAvgRes +
            _dLastlnAvgRes * _dLastlnAvgRes
        );

    return _dLastSHBetaCoef;
}

double NTCThermistorCalculator::calcSHGammaCoefficient() {
    if( !_bAreThrownInfosForSHGammaCoeffChanged )
        return _dLastSHGammaCoef;

    _bAreThrownInfosForSHGammaCoeffChanged = false;

    if( 
        isnan(_dMaxRes) 
        || isnan(_dAvgRes) 
        || isnan(_dMinRes) 
        || _dMaxRes <= 0 
        || _dAvgRes <= 0 
        || _dMinRes <= 0 
    )
        return NAN;

    double dlnMaxRes = _dLastlnMaxRes = log(_dMaxRes);
    double dlnAvgRes  = _dLastlnAvgRes = log(_dAvgRes);
    double dlnMinRes = log(_dMinRes);
    
    double dSublnsRess13 = dlnMinRes - dlnMaxRes;
    double dSublnsRess12 = dlnAvgRes - dlnMaxRes;
    double dSublnsRess23 = dlnMinRes - dlnAvgRes;
    double dSumlnsRess123 = dlnMinRes + dlnAvgRes + dlnMaxRes;
    
    if( 
        dSublnsRess13 == 0
        || dSublnsRess12 == 0
        || dSublnsRess23 == 0
        || dSumlnsRess123 == 0
        || _dMaxTmp == 0
        || _dAvgTmp == 0
        || _dMinTmp == 0
    )
        return NAN;
        
    double dInvTmp3 = 1 / _dMaxTmp;
    double dInvTmp2 = 1 / _dAvgTmp;
    double dInvTmp1 = _dLastInvMinTmp = 1 / _dMinTmp;

    Serial.println("T1");
    Serial.println(_dMinTmp);
    Serial.println("T2");
    Serial.println(_dAvgTmp);
    Serial.println("T3");
    Serial.println(_dMaxTmp);
    Serial.println("R1");
    Serial.println(_dMaxRes);
    Serial.println("R2");
    Serial.println(_dAvgRes);
    Serial.println("R3");
    Serial.println(_dMinRes);
  
    _dLastSHGammaCoefPiece = ( dInvTmp2 - dInvTmp1 ) / dSublnsRess12;
    _dLastSHGammaCoef = ( ( dInvTmp3 - dInvTmp1 ) / dSublnsRess13 - _dLastSHGammaCoefPiece ) / dSublnsRess23 / dSumlnsRess123;
    
    return _dLastSHGammaCoef;
}

double NTCThermistorCalculator::calcBetaParameter() {
    if( !_bAreThrownInfosForBetaParameterChanged )
        return _dLastBetaParameter;

    _bAreThrownInfosForBetaParameterChanged = false;

    if( 
        isnan(_dMinRes) 
        || isnan(_dMaxRes) 
        || isnan(_dMaxTmp) 
        || isnan(_dMinTmp)
        || _dMaxRes == 0
        || _dMaxTmp == 0
        || _dMinTmp == 0
    )
        return NAN;
    
    double dDiv13 = _dMinRes / _dMaxRes;

    if( dDiv13 == 0)
        return NAN;

    double dInvSub31 = 1 / _dMaxTmp - 1 / _dMinTmp;

    if( dInvSub31 == 0)
        return NAN;

    _dLastBetaParameter = log( dDiv13 ) / dInvSub31;

    return _dLastBetaParameter;
}

bool NTCThermistorCalculator::isReady() {

}