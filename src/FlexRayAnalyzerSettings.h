#ifndef FLEXRAY_ANALYZER_SETTINGS
#define FLEXRAY_ANALYZER_SETTINGS

#include <AnalyzerSettings.h>
#include <AnalyzerTypes.h>

class FlexRayAnalyzerSettings : public AnalyzerSettings
{
public:
	FlexRayAnalyzerSettings();
	virtual ~FlexRayAnalyzerSettings() = default;

	virtual bool SetSettingsFromInterfaces();
	void UpdateInterfacesFromSettings();
	virtual void LoadSettings( const char* settings );
	virtual const char* SaveSettings();

	
	Channel mInputChannel;
	U32 mBitRate;
	U32 mChannelType;
	U32 mSamplePointPercent;
	bool mInvertInput;

protected:
	AnalyzerSettingInterfaceChannel	mInputChannelInterface;
	AnalyzerSettingInterfaceNumberList mBitRateInterface;
	AnalyzerSettingInterfaceNumberList mChannelTypeInterface;
	AnalyzerSettingInterfaceInteger mSamplePointPercentInterface;
	AnalyzerSettingInterfaceBool mInvertInputInterface;
};

#endif //FLEXRAY_ANALYZER_SETTINGS
