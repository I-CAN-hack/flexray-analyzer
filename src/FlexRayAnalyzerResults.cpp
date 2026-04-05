#include "FlexRayAnalyzerResults.h"

#include <AnalyzerHelpers.h>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "FlexRayAnalyzer.h"
#include "FlexRayAnalyzerSettings.h"

namespace
{
std::string GetDisplayString( U64 value, DisplayBase display_base, U32 bit_count )
{
	char buffer[128];
	AnalyzerHelpers::GetNumberString( value, display_base, bit_count, buffer, sizeof( buffer ) );
	return buffer;
}

std::string GetPayloadString( const std::vector<U8>& payload )
{
	if( payload.empty() == true )
		return "-";

	std::ostringstream stream;
	stream << std::hex << std::uppercase << std::setfill( '0' );

	for( size_t i = 0; i < payload.size(); ++i )
	{
		if( i != 0 )
			stream << ' ';

		stream << std::setw( 2 ) << static_cast<U32>( payload[ i ] );
	}

	return stream.str();
}

std::string GetFrameSummary( const FlexRayFrameRecord& record, DisplayBase display_base )
{
	if( record.mIsError == true )
		return record.mErrorText;

	if( record.mSymbolName.empty() == false )
	{
		if( record.mSymbolName == "WUP" && record.mWakeupSymbolCount != 0 )
		{
			std::ostringstream stream;
			stream << "WUP x" << record.mWakeupSymbolCount;
			return stream.str();
		}

		return record.mSymbolName;
	}

	std::ostringstream stream;
	stream << ( record.mIsDynamic ? "Dyn" : "Static" ) << " ID "
		   << GetDisplayString( record.mFrameId, display_base, 11 )
		   << " Cyc " << GetDisplayString( record.mCycle, display_base, 6 )
		   << " Len " << static_cast<U32>( record.mPayload.size() ) << "B";

	if( record.mHeaderCrcOk == false || record.mFrameCrcOk == false )
		stream << " CRC ERR";
	else
		stream << " CRC OK";

	if( record.mCidOk == false )
		stream << " CID WARN";

	return stream.str();
}
}

FlexRayAnalyzerResults::FlexRayAnalyzerResults( FlexRayAnalyzer* analyzer, FlexRayAnalyzerSettings* settings )
:	AnalyzerResults(),
	mSettings( settings ),
	mAnalyzer( analyzer )
{
}

FlexRayAnalyzerResults::~FlexRayAnalyzerResults()
{
}

void FlexRayAnalyzerResults::AddFlexRaySegment( const Frame& frame, FlexRaySegmentRecord record )
{
	mSegmentRecords.push_back( std::move( record ) );
	AddFrame( frame );
}

U64 FlexRayAnalyzerResults::CommitFlexRayPacket( FlexRayFrameRecord record )
{
	mPacketRecords.push_back( std::move( record ) );
	return CommitPacketAndStartNewPacket();
}

const FlexRaySegmentRecord& FlexRayAnalyzerResults::GetSegmentRecord( U64 frame_index ) const
{
	return mSegmentRecords.at( static_cast<size_t>( frame_index ) );
}

const FlexRayFrameRecord& FlexRayAnalyzerResults::GetPacketRecord( U64 packet_index ) const
{
	return mPacketRecords.at( static_cast<size_t>( packet_index ) );
}

void FlexRayAnalyzerResults::GenerateBubbleText( U64 frame_index, Channel& channel, DisplayBase display_base )
{
	(void)channel;
	(void)display_base;
	ClearResultStrings();

	const FlexRaySegmentRecord& record = GetSegmentRecord( frame_index );
	AddResultString( record.mShortText.c_str() );

	if( record.mLongText.empty() == false && record.mLongText != record.mShortText )
		AddResultString( record.mLongText.c_str() );
}

void FlexRayAnalyzerResults::GenerateExportFile( const char* file, DisplayBase display_base, U32 export_type_user_id )
{
	(void)export_type_user_id;
	std::ofstream file_stream( file, std::ios::out );

	U64 trigger_sample = mAnalyzer->GetTriggerSample();
	U32 sample_rate = mAnalyzer->GetSampleRate();

	file_stream << "Time [s],Type,Segment,Frame ID,Cycle,Payload Bytes,PPI,NF,Sync,Startup,Header CRC,Header CRC OK,Frame CRC,Frame CRC OK,Payload,Info"
				<< std::endl;

	U64 num_packets = static_cast<U64>( mPacketRecords.size() );
	for( U64 i = 0; i < num_packets; ++i )
	{
		const FlexRayFrameRecord& record = GetPacketRecord( i );

		char time_str[128];
		AnalyzerHelpers::GetTimeString( record.mStartSample, trigger_sample, sample_rate, time_str, 128 );

		if( record.mIsError == true )
		{
			file_stream << time_str << ",error,-,-,-,-,-,-,-,-,-,-,-,-,-,\"" << record.mErrorText << "\"" << std::endl;
		}
		else if( record.mSymbolName.empty() == false )
		{
			const std::string summary = GetFrameSummary( record, display_base );
			file_stream << time_str << ",symbol," << record.mSymbolName << ",-,-,-,-,-,-,-,-,-,-,-,-,\"" << summary << "\"" << std::endl;
		}
		else
		{
			file_stream << time_str
						<< ",frame"
						<< "," << ( record.mIsDynamic ? "dynamic" : "static" )
						<< "," << GetDisplayString( record.mFrameId, display_base, 11 )
						<< "," << GetDisplayString( record.mCycle, display_base, 6 )
						<< "," << record.mPayload.size()
						<< "," << ( record.mPayloadPreamble ? 1 : 0 )
						<< "," << ( record.mNullFrame ? 1 : 0 )
						<< "," << ( record.mSyncFrame ? 1 : 0 )
						<< "," << ( record.mStartupFrame ? 1 : 0 )
						<< "," << GetDisplayString( record.mHeaderCrc, display_base, 11 )
						<< "," << ( record.mHeaderCrcOk ? "true" : "false" )
						<< "," << GetDisplayString( record.mFrameCrc, display_base, 24 )
						<< "," << ( record.mFrameCrcOk ? "true" : "false" )
						<< ",\"" << GetPayloadString( record.mPayload ) << "\""
						<< ",\"" << GetFrameSummary( record, display_base ) << "\""
						<< std::endl;
		}

		if( UpdateExportProgressAndCheckForCancel( i, num_packets ) == true )
		{
			file_stream.close();
			return;
		}
	}

	file_stream.close();
}

void FlexRayAnalyzerResults::GenerateFrameTabularText( U64 frame_index, DisplayBase display_base )
{
#ifdef SUPPORTS_PROTOCOL_SEARCH
	(void)display_base;
	ClearTabularText();
	const FlexRaySegmentRecord& record = GetSegmentRecord( frame_index );
	AddTabularText( record.mLongText.empty() ? record.mShortText.c_str() : record.mLongText.c_str() );
#endif
}

void FlexRayAnalyzerResults::GeneratePacketTabularText( U64 packet_id, DisplayBase display_base )
{
#ifdef SUPPORTS_PROTOCOL_SEARCH
	ClearTabularText();
	AddTabularText( GetFrameSummary( GetPacketRecord( packet_id ), display_base ).c_str() );
#else
	(void)packet_id;
	(void)display_base;
#endif
}

void FlexRayAnalyzerResults::GenerateTransactionTabularText( U64 transaction_id, DisplayBase display_base )
{
	(void)transaction_id;
	(void)display_base;
}
