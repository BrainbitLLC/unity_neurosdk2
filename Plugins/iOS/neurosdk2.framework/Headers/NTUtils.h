//
//  NTUtils.h
//  neurosdk
//
//  Created by Aseatari on 06.09.2022.
//

#ifndef NTUtils_h
#define NTUtils_h

#include "NTTypes.h"

extern "C" {
    #include "cmn_type.h"
}

@interface NTUtils : NSObject

extern UInt16 const ErrMsgLen;
extern UInt16 const SensorNameLen;
extern UInt16 const SensorAddrLen;
extern UInt16 const SensorSnLen;
extern UInt16 const SensorChannelNameLen;
extern UInt16 const NeuroEEGMaxChCount;
extern UInt16 const FileNameMaxLen;


+ (void) raise_exception_if: (OpStatus) opStatus;

@end

#endif /* NTUtils_h */
