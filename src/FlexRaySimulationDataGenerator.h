#ifndef FLEXRAY_SIMULATION_DATA_GENERATOR
#define FLEXRAY_SIMULATION_DATA_GENERATOR

#include <SimulationChannelDescriptor.h>
#include <vector>

class FlexRayAnalyzerSettings;

class FlexRaySimulationDataGenerator
{
public:
	FlexRaySimulationDataGenerator();
	~FlexRaySimulationDataGenerator() = default;

	void Initialize( U32 simulation_sample_rate, FlexRayAnalyzerSettings* settings );
	U32 GenerateSimulationData( U64 newest_sample_requested, U32 sample_rate, SimulationChannelDescriptor** simulation_channel );

protected:
	void OutputFrame( U16 frame_id, U8 cycle, const std::vector<U8>& payload, bool dynamic_frame, bool sync_frame, bool startup_frame );
	void OutputBit( U8 wire_bit, U32 bit_count = 1 );
	void OutputExtendedByte( U8 value );

	FlexRayAnalyzerSettings* mSettings;
	U32 mSimulationSampleRateHz;
	U32 mSamplesPerBit;
	U64 mFrameCounter;
	SimulationChannelDescriptor mSimulationData;
};

#endif //FLEXRAY_SIMULATION_DATA_GENERATOR
