//
//  NTNeuroEEGSignalProcessing.h
//  neurosdk
//
//  Created by Aseatari on 20.10.2023.
//

#ifndef NTNeuroEEGSignalProcessing_h
#define NTNeuroEEGSignalProcessing_h

#include "NTTypes.h"

@interface NTNeuroEEGSignalProcessing : NSObject

- (nonnull instancetype)init NS_UNAVAILABLE;
- (nonnull instancetype)initWithAmpParam:(NTNeuroEEGAmplifierParam*_Nonnull)ampParam;

+ (UInt32) calcCRC32:(NSData*_Nonnull) data;
- (UInt8) ParseRawSignalNeuroEEG:(NSData*_Nonnull)data signalOut:(NSMutableArray<NTSignalChannelsData*>*_Nonnull)signalOut resistOut:(NSMutableArray<NTResistChannelsData*>*_Nonnull)resistOut;

@end

#endif /* NTNeuroEEGSignalProcessing_h */
