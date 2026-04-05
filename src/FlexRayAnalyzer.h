#ifndef FLEXRAY_ANALYZER_H
#define FLEXRAY_ANALYZER_H

#include <Analyzer.h>
#include <memory>

#include "FlexRayAnalyzerResults.h"
#include "FlexRayAnalyzerSettings.h"
#include "FlexRaySimulationDataGenerator.h"

class ANALYZER_EXPORT FlexRayAnalyzer : public Analyzer2
{
public:
	FlexRayAnalyzer();
	virtual ~FlexRayAnalyzer();

	virtual void SetupResults();
	virtual void WorkerThread();

	virtual U32 GenerateSimulationData( U64 newest_sample_requested, U32 sample_rate, SimulationChannelDescriptor** simulation_channels );
	virtual U32 GetMinimumSampleRateHz();

	virtual const char* GetAnalyzerName() const;
	virtual bool NeedsRerun();

protected:
	FlexRayAnalyzerSettings mSettings;
	std::unique_ptr<FlexRayAnalyzerResults> mResults;
	AnalyzerChannelData* mInput;

	FlexRaySimulationDataGenerator mSimulationDataGenerator;
	bool mSimulationInitialized;
	U32 mSampleRateHz;
};

extern "C" ANALYZER_EXPORT const char* __cdecl GetAnalyzerName();
extern "C" ANALYZER_EXPORT Analyzer* __cdecl CreateAnalyzer();
extern "C" ANALYZER_EXPORT void __cdecl DestroyAnalyzer( Analyzer* analyzer );

#endif //FLEXRAY_ANALYZER_H
