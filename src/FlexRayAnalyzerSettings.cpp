#include "FlexRayAnalyzerSettings.h"
#include <AnalyzerHelpers.h>


FlexRayAnalyzerSettings::FlexRayAnalyzerSettings()
:	mInputChannel( UNDEFINED_CHANNEL ),
	mBitRate( 10000000 ),
	mChannelType( 0 ),
	mSamplePointPercent( 62 ),
	mInvertInput( false ),
	mInputChannelInterface(),
	mBitRateInterface(),
	mChannelTypeInterface(),
	mSamplePointPercentInterface(),
	mInvertInputInterface()
{
	mInputChannelInterface.SetTitleAndTooltip( "Input Channel", "Single-ended FlexRay bus signal." );
	mInputChannelInterface.SetChannel( mInputChannel );

	mBitRateInterface.SetTitleAndTooltip( "Bit Rate (Bits/s)", "Nominal FlexRay bus bit rate." );
	mBitRateInterface.AddNumber( 10000000, "10 Mbit/s", "Standard FlexRay bit rate." );
	mBitRateInterface.AddNumber( 5000000, "5 Mbit/s", "Standard FlexRay bit rate." );
	mBitRateInterface.AddNumber( 2500000, "2.5 Mbit/s", "Standard FlexRay bit rate." );
	mBitRateInterface.SetNumber( mBitRate );

	mChannelTypeInterface.SetTitleAndTooltip( "CRC Channel", "Select the channel-specific frame CRC initialization vector." );
	mChannelTypeInterface.AddNumber( 0, "Channel A", "Use FlexRay channel A frame CRC settings." );
	mChannelTypeInterface.AddNumber( 1, "Channel B", "Use FlexRay channel B frame CRC settings." );
	mChannelTypeInterface.SetNumber( mChannelType );

	mSamplePointPercentInterface.SetTitleAndTooltip( "Sample Point (%)", "Percent of the bit time used for sampling after clock recovery. FlexRay bit strobing is close to 62.5%." );
	mSamplePointPercentInterface.SetMin( 1 );
	mSamplePointPercentInterface.SetMax( 99 );
	mSamplePointPercentInterface.SetInteger( mSamplePointPercent );

	mInvertInputInterface.SetTitleAndTooltip( "Invert Input", "Invert the captured digital signal before decoding." );
	mInvertInputInterface.SetValue( mInvertInput );

	AddInterface( &mInputChannelInterface );
	AddInterface( &mBitRateInterface );
	AddInterface( &mChannelTypeInterface );
	AddInterface( &mSamplePointPercentInterface );
	AddInterface( &mInvertInputInterface );

	AddExportOption( 0, "Export as csv file" );
	AddExportExtension( 0, "csv", "csv" );

	ClearChannels();
	AddChannel( mInputChannel, "FlexRay", false );
}

FlexRayAnalyzerSettings::~FlexRayAnalyzerSettings()
{
}

bool FlexRayAnalyzerSettings::SetSettingsFromInterfaces()
{
	mInputChannel = mInputChannelInterface.GetChannel();
	mBitRate = U32( mBitRateInterface.GetNumber() );
	mChannelType = U32( mChannelTypeInterface.GetNumber() );
	mSamplePointPercent = U32( mSamplePointPercentInterface.GetInteger() );
	mInvertInput = mInvertInputInterface.GetValue();

	ClearChannels();
	AddChannel( mInputChannel, "FlexRay", mInputChannel != UNDEFINED_CHANNEL );

	return true;
}

void FlexRayAnalyzerSettings::UpdateInterfacesFromSettings()
{
	mInputChannelInterface.SetChannel( mInputChannel );
	mBitRateInterface.SetNumber( mBitRate );
	mChannelTypeInterface.SetNumber( mChannelType );
	mSamplePointPercentInterface.SetInteger( mSamplePointPercent );
	mInvertInputInterface.SetValue( mInvertInput );
}

void FlexRayAnalyzerSettings::LoadSettings( const char* settings )
{
	SimpleArchive text_archive;
	text_archive.SetString( settings );

	text_archive >> mInputChannel;
	text_archive >> mBitRate;
	text_archive >> mChannelType;
	text_archive >> mSamplePointPercent;
	text_archive >> mInvertInput;

	ClearChannels();
	AddChannel( mInputChannel, "FlexRay", mInputChannel != UNDEFINED_CHANNEL );

	UpdateInterfacesFromSettings();
}

const char* FlexRayAnalyzerSettings::SaveSettings()
{
	SimpleArchive text_archive;

	text_archive << mInputChannel;
	text_archive << mBitRate;
	text_archive << mChannelType;
	text_archive << mSamplePointPercent;
	text_archive << mInvertInput;

	return SetReturnString( text_archive.GetString() );
}
