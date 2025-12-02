#ifndef NTNeuroEEG_h
#define NTNeuroEEG_h

#import <Foundation/Foundation.h>
#import "NTTypes.h"
#include "NTSensor.h"

@interface NTNeuroEEG : NTSensor
- ( instancetype _Nonnull )init NS_UNAVAILABLE;

+ (NSNumber*_Nonnull) getMaxChCount;

@property (NS_NONATOMIC_IOSONLY) NSNumber* _Nonnull SurveyId;
@property (NS_NONATOMIC_IOSONLY, readonly) NTSensorAmpMode AmpMode;
@property (NS_NONATOMIC_IOSONLY, readonly) NSArray<NTEEGChannelInfo*>*_Nullable SupportedChannels;
@property (NS_NONATOMIC_IOSONLY, readonly) NTNeuroEEGFSStatus*_Nonnull FSStatus;
@property (NS_NONATOMIC_IOSONLY, readonly) NTSensorDiskInfo*_Nonnull FSDiskInfo;
@property (NS_NONATOMIC_IOSONLY) NTNeuroEEGAmplifierParam*_Nonnull AmplifierParam;
@property (NS_NONATOMIC_IOSONLY, readonly) NTSensorSamplingFrequency SamplingFrequencyResist;

- (void) setAmpModeCallback:(void (^_Nullable)(NTSensorAmpMode))callback;
- (void) setSignalCallback:(void (^_Nullable)(NSArray<NTSignalChannelsData*>*_Nonnull))callback;
- (void) setResistCallback:(void (^_Nullable)(NSArray<NTResistChannelsData*>*_Nonnull))callback;
- (void) setSignalResistCallback:(void (^_Nullable)(NSArray<NTSignalChannelsData*>*_Nonnull, NSArray<NTResistChannelsData*>*_Nonnull))callback;
- (void) setSignalRawCallback:(void (^_Nullable)(NSData*_Nonnull))callback;//uint8_t* data
- (void) setFileStreamReadCallback:(void (^_Nullable)(NSArray<NTSensorFileData*>*_Nonnull))callback;


- (NTSensorFileInfo*_Nonnull) readFileInfoNeuroEEG:(NSString*_Nonnull)fileName;
- (NSArray<NTSensorFileInfo*>*_Nonnull) readFileInfoAllNeuroEEG:(NSNumber*_Nonnull)maxFiles;
- (void) writeFileNeuroEEG:(NSString*_Nonnull)fileName data:(NSData*_Nonnull)data;
- (void) writeFileNeuroEEG:(NSString*_Nonnull)fileName data:(NSData*_Nonnull)data offset:(NSNumber*_Nonnull)offsetStart;
- (NSData*_Nonnull) readFileNeuroEEG:(NSString*_Nonnull)fileName;//byte[] ReadFileNeuroEEG(string fileName, uint szData = 0xFFFFFFFF, uint offsetStart = 0)
- (NSData*_Nonnull) readFileNeuroEEG:(NSString*_Nonnull)fileName szData:(NSNumber*_Nonnull)szData;
- (NSData*_Nonnull) readFileNeuroEEG:(NSString*_Nonnull)fileName szData:(NSNumber*_Nonnull)szData offset:(NSNumber*_Nonnull)offsetStart;
- (void) deleteFileNeuroEEG:(NSString*_Nonnull)fileName;
- (void) deleteAllFilesNeuroEEG:(NSString*_Nonnull) fileExt;
- (UInt32) readFileCRC32NeuroEEG:(NSString*_Nonnull)fileName totalSize:(NSNumber*_Nonnull)totalSize offsetStart:(NSNumber*_Nonnull)offsetStart;
- (void) fileStreamAutosaveNeuroEEG:(NSString*_Nonnull)fileName;
- (void) fileStreamReadNeuroEEG:(NSString*_Nonnull)fileName;// - (void) fileStreamReadNeuroEEG:(NSString*)fileName uint totalSize = 0xFFFFFFFF, uint offsetStart = 0)
- (void) fileStreamReadNeuroEEG:(NSString*_Nonnull)fileName totalSize:(NSNumber*_Nonnull)totalSize;
- (void) fileStreamReadNeuroEEG:(NSString*_Nonnull)fileName totalSize:(NSNumber*_Nonnull)totalSize offset:(NSNumber*_Nonnull)offsetStart;

@end

#endif /* NTNeuroEEG_h */
