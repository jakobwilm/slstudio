#include "Codec.h"

#include "CodecPhaseShift2x3.h"
#include "CodecPhaseShift3.h"
#include "CodecPhaseShift3FastWrap.h"
#include "CodecPhaseShift3Unwrap.h"
#include "CodecPhaseShift4.h"
#include "CodecPhaseShift2p1.h"
#include "CodecFastRatio.h"
#include "CodecGrayCode.h"
#include "CodecPhaseShiftModulated.h"
#include "CodecPhaseShiftMicro.h"
#include "CodecPhaseShiftNStep.h"

Encoder* Encoder::NewEncoder(CodecType codec, unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir){
    switch (codec){ 
    case codecTypePhaseShift2x3:
        return new EncoderPhaseShift2x3(_screenCols, _screenRows, _dir);
    case codecTypePhaseShift3:
        return new EncoderPhaseShift3(_screenCols, _screenRows, _dir);
    case codecTypePhaseShift3FastWrap:
        return new EncoderPhaseShift3FastWrap(_screenCols, _screenRows, _dir);
    case codecTypePhaseShift3Unwrap:
        return new EncoderPhaseShift3FastWrap(_screenCols, _screenRows, _dir);
    case codecTypePhaseShift4:
        return new EncoderPhaseShift4(_screenCols, _screenRows, _dir);
    case codecTypeGrayCode:
        return new EncoderGrayCode(_screenCols, _screenRows, _dir);
    case codecTypePhaseShift2p1:
        return new EncoderPhaseShift2p1(_screenCols, _screenRows, _dir);
    case codecTypeFastRatio:
        return new EncoderFastRatio(_screenCols, _screenRows, _dir);
    case codecTypePhaseShiftModulated:
        return new EncoderPhaseShiftModulated(_screenCols, _screenRows, _dir);
    case codecTypePhaseShiftMicro:
        return new EncoderPhaseShiftMicro(_screenCols, _screenRows, _dir);
    case codecTypePhaseShiftNStep:
        return new EncoderPhaseShiftNStep(_screenCols, _screenRows, _dir);
    default:
        return 0;
    }
}

Decoder* Decoder::NewDecoder(CodecType codec, unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir)
{
    switch (codec){ 
    case codecTypePhaseShift2x3:
        return new DecoderPhaseShift2x3(_screenCols, _screenRows, _dir);
    case codecTypePhaseShift3:
        return new DecoderPhaseShift3(_screenCols, _screenRows, _dir);
    case codecTypePhaseShift3FastWrap:
        return new DecoderPhaseShift3FastWrap(_screenCols, _screenRows, _dir);
    case codecTypePhaseShift3Unwrap:
        return new DecoderPhaseShift3FastWrap(_screenCols, _screenRows, _dir);
    case codecTypePhaseShift4:
        return new DecoderPhaseShift4(_screenCols, _screenRows, _dir);
    case codecTypeGrayCode:
        return new DecoderGrayCode(_screenCols, _screenRows, _dir);
    case codecTypePhaseShift2p1:
        return new DecoderPhaseShift2p1(_screenCols, _screenRows, _dir);
    case codecTypeFastRatio:
        return new DecoderFastRatio(_screenCols, _screenRows, _dir);
    case codecTypePhaseShiftModulated:
        return new DecoderPhaseShiftModulated(_screenCols, _screenRows, _dir);
    case codecTypePhaseShiftMicro:
        return new DecoderPhaseShiftMicro(_screenCols, _screenRows, _dir);
    case codecTypePhaseShiftNStep:
        return new DecoderPhaseShiftNStep(_screenCols, _screenRows, _dir);
    default:
        return 0;
    }
}
