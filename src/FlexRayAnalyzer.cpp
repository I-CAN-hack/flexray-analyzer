#include "FlexRayAnalyzer.h"

#include <AnalyzerChannelData.h>
#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <sstream>
#include <vector>

#include "FlexRayCommon.h"

namespace
{
const U32 kHeaderByteCount = 5;
const U32 kFrameCrcByteCount = 3;
const U32 kChannelIdleDelimiterBits = 11;
const U32 kTssTxLowMinBits = 3;
const U32 kTssTxLowMaxBits = 15;
const U32 kTssRxLowMinBits = 2;
const U32 kTssRxLowMaxBits = kTssTxLowMaxBits;
const U32 kCasRxLowMinBits = 29;
const U32 kCasRxLowMaxBits = 100;
const U32 kHeaderCrcWidth = 11;
const U32 kFrameCrcWidth = 24;

struct ClockRecoveryState
{
	double mBitWidth = 0.0;
	double mSampleOffset = 0.0;
	U64 mAnchorSample = 0;
	U32 mAnchorBitIndex = 0;
};

struct WakeupTiming
{
	U32 mRxLowMinBits = 0;
	U32 mRxLowMaxBits = 0;
	U32 mRxIdleMinBits = 0;
	U32 mRxWindowBits = 0;
};

enum class FlexRayMarkerType : U8
{
	DataBit,
	SyntaxBit
};

struct FlexRayMarker
{
	U64 mSample = 0;
	FlexRayMarkerType mType = FlexRayMarkerType::DataBit;
};

U8 GetWireBit( BitState state, bool invert_input )
{
	U8 bit = state == BIT_HIGH ? 1 : 0;

	if( invert_input )
		bit ^= 1;

	return bit;
}

U32 RoundBitsFromSamples( U64 sample_count, double bit_width )
{
	return static_cast<U32>( std::lround( static_cast<double>( sample_count ) / bit_width ) );
}

U32 BitsToNumber( const std::vector<U8>& bits, size_t offset, size_t count )
{
	U32 value = 0;

	for( size_t index = 0; index < count; ++index )
		value = ( value << 1 ) | bits.at( offset + index );

	return value;
}

WakeupTiming GetWakeupTiming( U32 bit_rate )
{
	WakeupTiming timing;

	switch( bit_rate )
	{
	case 2500000:
		timing.mRxLowMinBits = 11;
		timing.mRxLowMaxBits = 14;
		timing.mRxIdleMinBits = 14;
		timing.mRxWindowBits = 76;
		break;
	case 5000000:
		timing.mRxLowMinBits = 23;
		timing.mRxLowMaxBits = 29;
		timing.mRxIdleMinBits = 29;
		timing.mRxWindowBits = 151;
		break;
	case 10000000:
	default:
		timing.mRxLowMinBits = 46;
		timing.mRxLowMaxBits = 59;
		timing.mRxIdleMinBits = 59;
		timing.mRxWindowBits = 301;
		break;
	}

	return timing;
}
}

FlexRayAnalyzer::FlexRayAnalyzer()
:	Analyzer2(),
	mSettings(),
	mInput( nullptr ),
	mSimulationInitialized( false ),
	mSampleRateHz( 0 )
{
	SetAnalyzerSettings( &mSettings );
	UseFrameV2();
}

FlexRayAnalyzer::~FlexRayAnalyzer()
{
	KillThread();
}

void FlexRayAnalyzer::SetupResults()
{
	mResults.reset( new FlexRayAnalyzerResults( this, &mSettings ) );
	SetAnalyzerResults( mResults.get() );
	mResults->AddChannelBubblesWillAppearOn( mSettings.mInputChannel );
}

void FlexRayAnalyzer::WorkerThread()
{
	mSampleRateHz = GetSampleRate();
	mInput = GetAnalyzerChannelData( mSettings.mInputChannel );

	const double bit_width = static_cast<double>( mSampleRateHz ) / static_cast<double>( mSettings.mBitRate );
	const double sample_offset = bit_width * static_cast<double>( mSettings.mSamplePointPercent ) / 100.0;
	const double minimum_idle_high_samples = bit_width * static_cast<double>( kChannelIdleDelimiterBits );
	const WakeupTiming wakeup_timing = GetWakeupTiming( mSettings.mBitRate );
	const double maximum_wakeup_window_samples = bit_width * static_cast<double>( wakeup_timing.mRxWindowBits );
	const U64 start_padding_samples = static_cast<U64>( std::max( 0L, std::lround( sample_offset ) ) );
	const U64 end_padding_samples = static_cast<U64>( std::max( 1L, std::lround( bit_width - sample_offset ) ) );

	auto segment_start = [&]( U64 sample_number ) -> U64 {
		return sample_number > start_padding_samples ? sample_number - start_padding_samples : 0;
	};

	auto segment_end = [&]( U64 sample_number ) -> U64 {
		return sample_number + end_padding_samples;
	};

	auto format_hex = []( U32 value, U32 width, bool prefix = true ) {
		std::ostringstream stream;
		if( prefix )
			stream << "0x";
		stream << std::hex << std::uppercase << std::setfill( '0' ) << std::setw( static_cast<int>( width ) ) << value;
		return stream.str();
	};

	auto add_segment = [&]( U64 start_sample, U64 end_sample, U8 frame_type, U8 frame_flags, const std::string& short_text,
							const std::string& long_text, const char* frame_v2_type = nullptr, bool* packet_has_segments_ptr = nullptr,
							const std::function<void( FrameV2& )>& populate_frame_v2 = std::function<void( FrameV2& )>() ) {
		Frame frame;
		frame.mStartingSampleInclusive = start_sample;
		frame.mEndingSampleInclusive = std::max( start_sample, end_sample );
		frame.mData1 = 0;
		frame.mData2 = 0;
		frame.mType = frame_type;
		frame.mFlags = frame_flags;

		FlexRaySegmentRecord segment_record;
		segment_record.mShortText = short_text;
		segment_record.mLongText = long_text.empty() ? short_text : long_text;
		mResults->AddFlexRaySegment( frame, std::move( segment_record ) );

		if( frame_v2_type != nullptr )
		{
			FrameV2 frame_v2;
			if( populate_frame_v2 )
				populate_frame_v2( frame_v2 );
			mResults->AddFrameV2( frame_v2, frame_v2_type, frame.mStartingSampleInclusive, frame.mEndingSampleInclusive );
		}

		if( packet_has_segments_ptr != nullptr )
			*packet_has_segments_ptr = true;
	};

	auto add_frame_v2_only = [&]( U64 start_sample, U64 end_sample, const char* frame_v2_type,
								  const std::function<void( FrameV2& )>& populate_frame_v2 ) {
		FrameV2 frame_v2;
		if( populate_frame_v2 )
			populate_frame_v2( frame_v2 );
		mResults->AddFrameV2( frame_v2, frame_v2_type, start_sample, std::max( start_sample, end_sample ) );
	};

	auto commit_record = [&]( U64 start_sample, U64 end_sample, U8 frame_flags, FlexRayFrameRecord record ) {
		record.mStartSample = start_sample;
		record.mEndSample = end_sample;
		mResults->AddMarker( start_sample, AnalyzerResults::Start, mSettings.mInputChannel );
		mResults->AddMarker( end_sample, ( frame_flags & DISPLAY_AS_ERROR_FLAG ) != 0 ? AnalyzerResults::ErrorSquare : AnalyzerResults::Stop,
							 mSettings.mInputChannel );
		mResults->CommitFlexRayPacket( std::move( record ) );
		mResults->CommitResults();
		ReportProgress( end_sample );
		CheckIfThreadShouldExit();
	};

	bool have_pending_tss = false;
	U64 pending_tss_start_sample = 0;
	bool have_pending_wakeup_pattern = false;
	U64 pending_wakeup_start_sample = 0;
	U64 pending_wakeup_end_sample = 0;
	U64 pending_wakeup_last_low_start_sample = 0;
	U32 pending_wakeup_symbol_count = 0;

	auto flush_pending_wakeup_pattern = [&]() {
		if( !have_pending_wakeup_pattern )
			return;

		FlexRayFrameRecord record;
		record.mSymbolName = pending_wakeup_symbol_count >= 2 ? "WUP" : "WUS";
		record.mWakeupSymbolCount = pending_wakeup_symbol_count;
		commit_record( pending_wakeup_start_sample, pending_wakeup_end_sample, 0, std::move( record ) );

		have_pending_wakeup_pattern = false;
		pending_wakeup_start_sample = 0;
		pending_wakeup_end_sample = 0;
		pending_wakeup_last_low_start_sample = 0;
		pending_wakeup_symbol_count = 0;
	};

	for( ;; )
	{
		U64 preceding_high_start_sample = 0;
		U64 tss_start_sample = 0;
		bool bypass_idle_check = false;

		if( have_pending_tss )
		{
			have_pending_tss = false;
			bypass_idle_check = true;
			tss_start_sample = pending_tss_start_sample;

			if( GetWireBit( mInput->GetBitState(), mSettings.mInvertInput ) != 0 )
				continue;
		}
		else
		{
			while( GetWireBit( mInput->GetBitState(), mSettings.mInvertInput ) == 0 )
				mInput->AdvanceToNextEdge();

			preceding_high_start_sample = mInput->GetSampleNumber();
			mInput->AdvanceToNextEdge();
			tss_start_sample = mInput->GetSampleNumber();
		}

		if( GetWireBit( mInput->GetBitState(), mSettings.mInvertInput ) != 0 )
			continue;

		if( have_pending_wakeup_pattern &&
			static_cast<double>( tss_start_sample - pending_wakeup_last_low_start_sample ) + ( bit_width * 0.25 ) > maximum_wakeup_window_samples )
			flush_pending_wakeup_pattern();

		if( !bypass_idle_check &&
			static_cast<double>( tss_start_sample - preceding_high_start_sample ) + ( bit_width * 0.25 ) < minimum_idle_high_samples )
			continue;

		mInput->AdvanceToNextEdge();
		const U64 tss_end_sample = mInput->GetSampleNumber();

		if( GetWireBit( mInput->GetBitState(), mSettings.mInvertInput ) != 1 )
			continue;

		const U32 observed_tss_bits = RoundBitsFromSamples( tss_end_sample - tss_start_sample, bit_width );
		const U64 first_edge_after_tss = mInput->GetSampleOfNextEdge();
		const double high_after_tss_samples = static_cast<double>( first_edge_after_tss - tss_end_sample );
		const U32 observed_post_low_high_bits = RoundBitsFromSamples( first_edge_after_tss - tss_end_sample, bit_width );

		if( observed_tss_bits >= wakeup_timing.mRxLowMinBits && observed_tss_bits <= wakeup_timing.mRxLowMaxBits &&
			observed_post_low_high_bits >= wakeup_timing.mRxIdleMinBits )
		{
			const U64 wakeup_end_sample = first_edge_after_tss > 0 ? first_edge_after_tss - 1 : first_edge_after_tss;
			std::ostringstream wakeup_text;
			wakeup_text << "Wakeup symbol (" << observed_tss_bits << " low, " << observed_post_low_high_bits << " idle bits)";
			add_segment( tss_start_sample, wakeup_end_sample, FlexRayWakeupSymbolField, 0, "WUS", wakeup_text.str(), "wakeup_symbol", nullptr,
						 [&]( FrameV2& frame_v2 ) {
							 frame_v2.AddByte( "low_bits", static_cast<U8>( observed_tss_bits ) );
							 frame_v2.AddByte( "idle_bits", static_cast<U8>( observed_post_low_high_bits ) );
						 } );

			if( have_pending_wakeup_pattern &&
				static_cast<double>( tss_start_sample - pending_wakeup_last_low_start_sample ) + ( bit_width * 0.25 ) <= maximum_wakeup_window_samples )
			{
				++pending_wakeup_symbol_count;
				pending_wakeup_end_sample = wakeup_end_sample;
				pending_wakeup_last_low_start_sample = tss_start_sample;
			}
			else
			{
				flush_pending_wakeup_pattern();
				have_pending_wakeup_pattern = true;
				pending_wakeup_start_sample = tss_start_sample;
				pending_wakeup_end_sample = wakeup_end_sample;
				pending_wakeup_last_low_start_sample = tss_start_sample;
				pending_wakeup_symbol_count = 1;
			}

			continue;
		}

		if( have_pending_wakeup_pattern )
			flush_pending_wakeup_pattern();

		if( high_after_tss_samples < ( bit_width * 1.25 ) || high_after_tss_samples > ( bit_width * 2.75 ) )
			continue;

		const bool is_valid_tss = observed_tss_bits >= kTssRxLowMinBits && observed_tss_bits <= kTssRxLowMaxBits;
		const bool is_valid_cas = observed_tss_bits >= kCasRxLowMinBits && observed_tss_bits <= kCasRxLowMaxBits;

		if( !is_valid_tss && !is_valid_cas )
			continue;

		ClockRecoveryState recovery;
		recovery.mBitWidth = bit_width;
		recovery.mSampleOffset = sample_offset;
		recovery.mAnchorSample = tss_end_sample;
		recovery.mAnchorBitIndex = 0;

		U32 bit_index = 0;
		U64 last_sample = tss_end_sample;
		bool packet_has_segments = false;
		std::vector<FlexRayMarker> packet_markers;

		auto commit_packet = [&]( U64 end_sample, U8 frame_flags, FlexRayFrameRecord record ) {
			if( !record.mIsError && record.mSymbolName.empty() )
			{
				FrameV2 frame_v2_packet;
				frame_v2_packet.AddString( "frame_type", record.mIsDynamic ? "dynamic" : "static" );
				frame_v2_packet.AddString( "identifier", format_hex( record.mFrameId, 3 ).c_str() );
				frame_v2_packet.AddByte( "cycle", record.mCycle );
				frame_v2_packet.AddByte( "tss_bits", static_cast<U8>( record.mTssBits ) );
				frame_v2_packet.AddBoolean( "tss_tx_spec_ok", !record.mTssBelowTxSpec );
				frame_v2_packet.AddByte( "payload_length_words", record.mPayloadLengthWords );
				frame_v2_packet.AddByte( "payload_length_bytes", static_cast<U8>( record.mPayload.size() ) );
				frame_v2_packet.AddBoolean( "reserved_bit", record.mReservedBit );
				frame_v2_packet.AddBoolean( "payload_preamble", record.mPayloadPreamble );
				frame_v2_packet.AddBoolean( "null_frame", record.mNullFrame );
				frame_v2_packet.AddBoolean( "sync_frame", record.mSyncFrame );
				frame_v2_packet.AddBoolean( "startup_frame", record.mStartupFrame );
				frame_v2_packet.AddString( "header_crc", format_hex( record.mHeaderCrc, 3 ).c_str() );
				frame_v2_packet.AddBoolean( "header_crc_ok", record.mHeaderCrcOk );
				frame_v2_packet.AddString( "crc", format_hex( record.mFrameCrc, 6 ).c_str() );
				frame_v2_packet.AddBoolean( "crc_ok", record.mFrameCrcOk );
				frame_v2_packet.AddByte( "cid_bits", static_cast<U8>( record.mCidBits ) );
				frame_v2_packet.AddBoolean( "cid_ok", record.mCidOk );

				if( record.mIsDynamic )
					frame_v2_packet.AddByte( "dts_bits", static_cast<U8>( record.mDtsBits ) );

				frame_v2_packet.AddString( "payload", FormatPayload( record.mPayload ).c_str() );
				mResults->AddFrameV2( frame_v2_packet, "frame", tss_start_sample, end_sample );
			}

			for( const FlexRayMarker& marker : packet_markers )
			{
				mResults->AddMarker( marker.mSample,
									 marker.mType == FlexRayMarkerType::DataBit ? AnalyzerResults::Dot : AnalyzerResults::ErrorX,
									 mSettings.mInputChannel );
			}

			if( record.mIsError && !packet_has_segments )
				add_segment( tss_start_sample, end_sample, FlexRayErrorField, DISPLAY_AS_ERROR_FLAG, "Err", record.mErrorText, "error_field",
							 &packet_has_segments, [&]( FrameV2& frame_v2 ) { frame_v2.AddString( "error", record.mErrorText.c_str() ); } );

			commit_record( tss_start_sample, end_sample, frame_flags, std::move( record ) );
		};

		auto read_bit = [&]( U32 current_bit_index, U8& bit_value, U64& sample_number ) -> bool {
			for( ;; )
			{
				double sample_target = static_cast<double>( recovery.mAnchorSample );
				sample_target += static_cast<double>( current_bit_index - recovery.mAnchorBitIndex ) * recovery.mBitWidth;
				sample_target += recovery.mSampleOffset;

				if( sample_target < static_cast<double>( mInput->GetSampleNumber() ) )
					sample_target = static_cast<double>( mInput->GetSampleNumber() );

				const U64 target_sample = static_cast<U64>( std::floor( sample_target + 0.5 ) );

				if( mInput->WouldAdvancingToAbsPositionCauseTransition( target_sample ) )
				{
					mInput->AdvanceToNextEdge();

					if( GetWireBit( mInput->GetBitState(), mSettings.mInvertInput ) == 0 )
					{
						recovery.mAnchorSample = mInput->GetSampleNumber();
						recovery.mAnchorBitIndex = current_bit_index;
					}

					continue;
				}

				mInput->AdvanceToAbsPosition( target_sample );
				sample_number = mInput->GetSampleNumber();
				bit_value = GetWireBit( mInput->GetBitState(), mSettings.mInvertInput );
				return true;
			}
		};

		auto add_bit_marker = [&]( U64 sample_number, FlexRayMarkerType marker_type ) {
			FlexRayMarker marker;
			marker.mSample = sample_number;
			marker.mType = marker_type;
			packet_markers.push_back( marker );
		};

		auto read_expected_bit = [&]( U8 expected_bit, const char* label, std::string& error_text, U64* observed_sample = nullptr,
									 FlexRayMarkerType marker_type = FlexRayMarkerType::SyntaxBit ) -> bool {
			U8 actual_bit = 0;
			U64 sample_number = 0;
			read_bit( bit_index, actual_bit, sample_number );
			last_sample = sample_number;
			add_bit_marker( sample_number, marker_type );

			if( observed_sample != nullptr )
				*observed_sample = sample_number;

			if( actual_bit != expected_bit )
			{
				std::ostringstream stream;
				stream << "Expected " << label << " = " << static_cast<U32>( expected_bit )
					   << ", observed " << static_cast<U32>( actual_bit )
					   << " at wire bit " << bit_index;
				error_text = stream.str();
				return false;
			}

			++bit_index;
			return true;
		};

		auto read_extended_byte = [&]( U8& value, std::string& error_text, U64* byte_start_sample = nullptr, U64* byte_end_sample = nullptr,
									  std::vector<U64>* data_bit_samples = nullptr ) -> bool {
			U64 sample_number = 0;

			if( !read_expected_bit( 1, "BSS high", error_text, &sample_number ) )
				return false;

			if( byte_start_sample != nullptr )
				*byte_start_sample = sample_number;

			if( !read_expected_bit( 0, "BSS low", error_text ) )
				return false;

			value = 0;

			for( U32 shift = 0; shift < 8; ++shift )
			{
				U8 sampled_bit = 0;
				U64 bit_sample = 0;
				read_bit( bit_index, sampled_bit, bit_sample );
				last_sample = bit_sample;
				add_bit_marker( bit_sample, FlexRayMarkerType::DataBit );
				value = static_cast<U8>( ( value << 1 ) | sampled_bit );
				++bit_index;

				if( data_bit_samples != nullptr )
					data_bit_samples->push_back( bit_sample );

				if( byte_end_sample != nullptr )
					*byte_end_sample = bit_sample;
			}

			return true;
		};

		FlexRayFrameRecord record;
		record.mTssBits = observed_tss_bits;
		record.mTssBelowTxSpec = observed_tss_bits < kTssTxLowMinBits;
		U64 packet_end_sample = tss_end_sample;

		U8 fss_bit = 0;
		U64 fss_sample = 0;
		read_bit( bit_index, fss_bit, fss_sample );
		last_sample = fss_sample;
		add_bit_marker( fss_sample, FlexRayMarkerType::SyntaxBit );

		if( fss_bit != 1 )
			continue;

		++bit_index;

		std::vector<U8> header_bytes;
		header_bytes.reserve( kHeaderByteCount );
		std::vector<U64> header_bit_samples;
		header_bit_samples.reserve( kHeaderByteCount * 8 );
		std::vector<U64> header_byte_start_samples;
		header_byte_start_samples.reserve( kHeaderByteCount );
		std::string syntax_error;

		U8 first_bss_high = 0;
		U64 header_start_sample = 0;
		read_bit( bit_index, first_bss_high, header_start_sample );
		last_sample = header_start_sample;
		add_bit_marker( header_start_sample, FlexRayMarkerType::SyntaxBit );
		if( first_bss_high != 1 )
			continue;
		header_byte_start_samples.push_back( header_start_sample );
		++bit_index;

		U8 first_bss_low = 0;
		read_bit( bit_index, first_bss_low, last_sample );
		add_bit_marker( last_sample, FlexRayMarkerType::SyntaxBit );
		if( first_bss_low != 0 )
		{
			if( observed_tss_bits >= kCasRxLowMinBits && observed_tss_bits <= kCasRxLowMaxBits )
			{
				add_segment( tss_start_sample, last_sample + end_padding_samples, FlexRayCasField, 0, "CAS", "Collision avoidance symbol", "cas_field",
							 &packet_has_segments,
							 [&]( FrameV2& frame_v2 ) { frame_v2.AddByte( "low_bits", static_cast<U8>( observed_tss_bits ) ); } );
				record.mSymbolName = "CAS";
				packet_end_sample = last_sample + end_padding_samples;
				commit_packet( packet_end_sample, 0, std::move( record ) );
			}
			continue;
		}
		++bit_index;

		const U64 fss_segment_start_sample = segment_start( fss_sample );

		{
			std::ostringstream tss_text;
			tss_text << "Transmission start sequence (" << observed_tss_bits << " bits";
			if( record.mTssBelowTxSpec )
				tss_text << ", accepted on RX, below transmitter spec 3-15 bits";
			tss_text << ")";
			add_segment( tss_start_sample, fss_segment_start_sample, FlexRayTssField,
						 record.mTssBelowTxSpec ? DISPLAY_AS_WARNING_FLAG : 0, "TSS", tss_text.str(), "tss_field", &packet_has_segments,
						 [&]( FrameV2& frame_v2 ) {
							 frame_v2.AddByte( "low_bits", static_cast<U8>( observed_tss_bits ) );
							 frame_v2.AddBoolean( "tx_spec_ok", !record.mTssBelowTxSpec );
						 } );
		}
		add_segment( fss_segment_start_sample, segment_end( fss_sample ), FlexRayFssField, 0, "FSS", "Frame start sequence", "fss_field",
					 &packet_has_segments );

		U8 first_header_byte = 0;
		for( U32 bit = 0; bit < 8; ++bit )
		{
			U8 sampled_bit = 0;
			U64 sample_number = 0;
			read_bit( bit_index, sampled_bit, sample_number );
			last_sample = sample_number;
			add_bit_marker( sample_number, FlexRayMarkerType::DataBit );
			header_bit_samples.push_back( sample_number );
			first_header_byte = static_cast<U8>( ( first_header_byte << 1 ) | sampled_bit );
			++bit_index;
		}
		header_bytes.push_back( first_header_byte );
		U64 header_end_sample = last_sample;

		for( U32 byte_index = 1; byte_index < kHeaderByteCount; ++byte_index )
		{
			U8 header_byte = 0;
			U64 byte_start_sample = 0;
			U64 byte_end_sample = 0;

			if( !read_extended_byte( header_byte, syntax_error, &byte_start_sample, &byte_end_sample, &header_bit_samples ) )
				break;

			header_bytes.push_back( header_byte );
			header_byte_start_samples.push_back( byte_start_sample );
			header_end_sample = byte_end_sample;
		}

		if( !syntax_error.empty() )
			continue;

		std::vector<U8> header_bits;
		header_bits.reserve( kHeaderByteCount * 8 );

		for( U8 value : header_bytes )
			AppendByteBits( header_bits, value );

		record.mReservedBit = header_bits[ 0 ] != 0;
		record.mPayloadPreamble = header_bits[ 1 ] != 0;
		record.mNullFrame = header_bits[ 2 ] == 0;
		record.mSyncFrame = header_bits[ 3 ] != 0;
		record.mStartupFrame = header_bits[ 4 ] != 0;
		record.mFrameId = static_cast<U16>( BitsToNumber( header_bits, 5, 11 ) );
		record.mPayloadLengthWords = static_cast<U8>( BitsToNumber( header_bits, 16, 7 ) );
		record.mHeaderCrc = static_cast<U16>( BitsToNumber( header_bits, 23, 11 ) );
		record.mCycle = static_cast<U8>( BitsToNumber( header_bits, 34, 6 ) );

		std::vector<U8> header_crc_input;
		header_crc_input.reserve( 20 );
		header_crc_input.push_back( header_bits[ 3 ] );
		header_crc_input.push_back( header_bits[ 4 ] );
		header_crc_input.insert( header_crc_input.end(), header_bits.begin() + 5, header_bits.begin() + 23 );
		record.mExpectedHeaderCrc = static_cast<U16>( CalculateCrc( header_crc_input, kHeaderCrcPolynomial, kHeaderCrcWidth, kHeaderCrcInit ) );
		record.mHeaderCrcOk = record.mHeaderCrc == record.mExpectedHeaderCrc;

		{
			std::ostringstream header_text;
			header_text << "ID " << format_hex( record.mFrameId, 3 )
						<< " Cyc " << static_cast<U32>( record.mCycle )
						<< " Len " << static_cast<U32>( record.mPayloadLengthWords ) * 2 << "B";
			add_frame_v2_only( segment_start( header_start_sample ), segment_end( header_end_sample ), "header_field", [&]( FrameV2& frame_v2 ) {
				frame_v2.AddString( "identifier", format_hex( record.mFrameId, 3 ).c_str() );
				frame_v2.AddByte( "cycle", record.mCycle );
				frame_v2.AddByte( "payload_length_words", record.mPayloadLengthWords );
				frame_v2.AddByte( "payload_length_bytes", static_cast<U8>( record.mPayloadLengthWords * 2 ) );
				frame_v2.AddBoolean( "reserved_bit", record.mReservedBit );
				frame_v2.AddBoolean( "payload_preamble", record.mPayloadPreamble );
				frame_v2.AddBoolean( "null_frame", record.mNullFrame );
				frame_v2.AddBoolean( "sync_frame", record.mSyncFrame );
				frame_v2.AddBoolean( "startup_frame", record.mStartupFrame );
				frame_v2.AddString( "header_crc", format_hex( record.mHeaderCrc, 3 ).c_str() );
				frame_v2.AddBoolean( "header_crc_ok", record.mHeaderCrcOk );
			} );

			if( header_bit_samples.size() == ( kHeaderByteCount * 8 ) && header_byte_start_samples.size() == kHeaderByteCount )
			{
				auto header_subfield_start_sample = [&]( U32 bit_offset ) {
					if( ( bit_offset % 8 ) == 0 )
						return segment_start( header_byte_start_samples.at( bit_offset / 8 ) );

					return segment_start( header_bit_samples.at( bit_offset ) );
				};

				auto add_header_subfield = [&]( U32 bit_offset, U32 bit_count, const std::string& short_text, const std::string& long_text ) {
					add_segment( header_subfield_start_sample( bit_offset ),
								 segment_end( header_bit_samples.at( bit_offset + bit_count - 1 ) ), FlexRayHeaderField, 0,
								 short_text, long_text, nullptr, &packet_has_segments );
				};

				add_header_subfield( 0, 5, "Flg", "Header flags" );
				add_header_subfield( 5, 11, "ID " + format_hex( record.mFrameId, 3 ), "Frame identifier " + format_hex( record.mFrameId, 3 ) );

				{
					std::ostringstream length_text;
					length_text << "Len " << static_cast<U32>( record.mPayloadLengthWords ) * 2 << "B";
					add_header_subfield( 16, 7, length_text.str(), "Payload length " + length_text.str().substr( 4 ) );
				}

				add_header_subfield( 23, 11, "HCRC " + format_hex( record.mHeaderCrc, 3 ),
									 "Header CRC " + format_hex( record.mHeaderCrc, 3 ) );

				{
					std::ostringstream cycle_text;
					cycle_text << "Cyc " << static_cast<U32>( record.mCycle );
					add_header_subfield( 34, 6, cycle_text.str(), "Cycle " + std::to_string( static_cast<U32>( record.mCycle ) ) );
				}
			}
			else
			{
				add_segment( segment_start( header_start_sample ), segment_end( header_end_sample ), FlexRayHeaderField, 0, "Hdr",
							 header_text.str(), nullptr, &packet_has_segments );
			}
		}

		std::vector<U8> payload_bits;
		const U32 payload_byte_count = static_cast<U32>( record.mPayloadLengthWords ) * 2;
		record.mPayload.reserve( payload_byte_count );
		payload_bits.reserve( payload_byte_count * 8 );

		for( U32 payload_index = 0; payload_index < payload_byte_count; ++payload_index )
		{
			U8 payload_byte = 0;
			U64 payload_start_sample = 0;
			U64 payload_end_sample = 0;

			if( !read_extended_byte( payload_byte, syntax_error, &payload_start_sample, &payload_end_sample ) )
				break;

			record.mPayload.push_back( payload_byte );
			AppendByteBits( payload_bits, payload_byte );

			std::ostringstream payload_text;
			payload_text << "Payload[" << payload_index << "] " << format_hex( payload_byte, 2 );
			add_segment( segment_start( payload_start_sample ), segment_end( payload_end_sample ), FlexRayPayloadByteField, 0,
						 format_hex( payload_byte, 2, false ), payload_text.str(), "payload_byte", &packet_has_segments,
						 [&]( FrameV2& frame_v2 ) {
							 frame_v2.AddByte( "index", static_cast<U8>( payload_index ) );
							 frame_v2.AddByte( "data", payload_byte );
						 } );
		}

		if( !syntax_error.empty() )
		{
			record.mIsError = true;
			record.mErrorText = syntax_error;
			commit_packet( last_sample + end_padding_samples, DISPLAY_AS_ERROR_FLAG, std::move( record ) );
			continue;
		}

		std::vector<U8> frame_crc_bytes;
		frame_crc_bytes.reserve( kFrameCrcByteCount );
		U64 frame_crc_start_sample = 0;
		U64 frame_crc_end_sample = 0;

		for( U32 byte_index = 0; byte_index < kFrameCrcByteCount; ++byte_index )
		{
			U8 frame_crc_byte = 0;
			U64 byte_start_sample = 0;
			U64 byte_end_sample = 0;

			if( !read_extended_byte( frame_crc_byte, syntax_error, &byte_start_sample, &byte_end_sample ) )
				break;

			frame_crc_bytes.push_back( frame_crc_byte );

			if( byte_index == 0 )
				frame_crc_start_sample = byte_start_sample;
			frame_crc_end_sample = byte_end_sample;
		}

		if( !syntax_error.empty() )
		{
			record.mIsError = true;
			record.mErrorText = syntax_error;
			commit_packet( last_sample + end_padding_samples, DISPLAY_AS_ERROR_FLAG, std::move( record ) );
			continue;
		}

		record.mFrameCrc = 0;
		for( U8 value : frame_crc_bytes )
			record.mFrameCrc = ( record.mFrameCrc << 8 ) | value;

		std::vector<U8> frame_crc_input = header_bits;
		frame_crc_input.insert( frame_crc_input.end(), payload_bits.begin(), payload_bits.end() );
		record.mExpectedFrameCrc = CalculateCrc( frame_crc_input, kFrameCrcPolynomial, kFrameCrcWidth,
												 mSettings.mChannelType == 0 ? kFrameCrcInitA : kFrameCrcInitB );
		record.mFrameCrcOk = record.mFrameCrc == record.mExpectedFrameCrc;

		{
			std::ostringstream crc_text;
			crc_text << "Frame CRC " << format_hex( record.mFrameCrc, 6 ) << ( record.mFrameCrcOk ? " OK" : " BAD" );
			add_segment( segment_start( frame_crc_start_sample ), segment_end( frame_crc_end_sample ), FlexRayFrameCrcField,
						 record.mFrameCrcOk ? 0 : DISPLAY_AS_ERROR_FLAG, "FCRC", crc_text.str(), "frame_crc_field", &packet_has_segments,
						 [&]( FrameV2& frame_v2 ) {
							 frame_v2.AddString( "crc", format_hex( record.mFrameCrc, 6 ).c_str() );
							 frame_v2.AddBoolean( "crc_ok", record.mFrameCrcOk );
						 } );
		}

		U64 fes_low_sample = 0;
		U64 fes_high_sample = 0;
		if( !read_expected_bit( 0, "FES low", syntax_error, &fes_low_sample ) ||
			!read_expected_bit( 1, "FES high", syntax_error, &fes_high_sample ) )
		{
			record.mIsError = true;
			record.mErrorText = syntax_error.empty() ? "Invalid frame end sequence." : syntax_error;
			commit_packet( last_sample + end_padding_samples, DISPLAY_AS_ERROR_FLAG, std::move( record ) );
			continue;
		}

		add_segment( segment_start( fes_low_sample ), segment_end( fes_high_sample ), FlexRayFesField, 0, "FES",
					 "Frame end sequence", "fes_field", &packet_has_segments );
		packet_end_sample = segment_end( fes_high_sample );

		auto read_cid = [&]( U64 initial_start_sample, U32 initial_bits ) {
			record.mCidBits = initial_bits;
			U64 cid_start_sample = initial_start_sample;
			U64 cid_end_sample = initial_start_sample;

			for( U32 cid_bit = initial_bits; cid_bit < kChannelIdleDelimiterBits; ++cid_bit )
			{
				U8 cid_value = 0;
				U64 sample_number = 0;
				read_bit( bit_index, cid_value, sample_number );
				last_sample = sample_number;
				add_bit_marker( sample_number, FlexRayMarkerType::SyntaxBit );

				if( cid_value != 1 )
				{
					record.mCidOk = false;
					have_pending_tss = true;
					pending_tss_start_sample = recovery.mAnchorSample;
					break;
				}

				if( record.mCidBits == 0 )
					cid_start_sample = sample_number;
				cid_end_sample = sample_number;
				++record.mCidBits;
				++bit_index;
			}

			if( record.mCidBits > 0 )
			{
				std::ostringstream cid_text;
				cid_text << "Channel idle delimiter (" << record.mCidBits << " bits";
				if( !record.mCidOk )
					cid_text << ", truncated";
				cid_text << ")";
				add_segment( segment_start( cid_start_sample ), segment_end( cid_end_sample ), FlexRayCidField,
							 record.mCidOk ? 0 : DISPLAY_AS_WARNING_FLAG, "CID", cid_text.str(), "cid_field", &packet_has_segments,
							 [&]( FrameV2& frame_v2 ) {
								 frame_v2.AddByte( "cid_bits", static_cast<U8>( record.mCidBits ) );
								 frame_v2.AddBoolean( "cid_ok", record.mCidOk );
							 } );
				packet_end_sample = segment_end( cid_end_sample );
			}
		};

		U8 post_fes_bit = 0;
		U64 post_fes_sample = 0;
		read_bit( bit_index, post_fes_bit, post_fes_sample );
		last_sample = post_fes_sample;
		add_bit_marker( post_fes_sample, FlexRayMarkerType::SyntaxBit );

		if( post_fes_bit == 0 )
		{
			record.mIsDynamic = true;
			++bit_index;
			U32 dts_low_bits = 1;
			U64 dts_start_sample = post_fes_sample;
			U64 dts_end_sample = post_fes_sample;

			for( ;; )
			{
				U8 bit = 0;
				U64 sample_number = 0;
				read_bit( bit_index, bit, sample_number );
				last_sample = sample_number;
				dts_end_sample = sample_number;
				add_bit_marker( sample_number, FlexRayMarkerType::SyntaxBit );

				if( bit == 1 )
				{
					record.mDtsBits = dts_low_bits + 1;
					++bit_index;
					break;
				}

				++bit_index;
				++dts_low_bits;
			}

			{
				std::ostringstream dts_text;
				dts_text << "Dynamic trailing sequence (" << record.mDtsBits << " bits)";
				add_segment( segment_start( dts_start_sample ), segment_end( dts_end_sample ), FlexRayDtsField, 0, "DTS", dts_text.str(),
							 "dts_field", &packet_has_segments,
							 [&]( FrameV2& frame_v2 ) { frame_v2.AddByte( "dts_bits", static_cast<U8>( record.mDtsBits ) ); } );
				packet_end_sample = segment_end( dts_end_sample );
			}

			read_cid( 0, 0 );
		}
		else
		{
			record.mIsDynamic = false;
			++bit_index;
			read_cid( post_fes_sample, 1 );
		}

		U8 frame_flags = 0;

		if( record.mReservedBit )
			frame_flags |= DISPLAY_AS_WARNING_FLAG;

		if( record.mTssBelowTxSpec )
			frame_flags |= DISPLAY_AS_WARNING_FLAG;

		if( !record.mCidOk )
			frame_flags |= DISPLAY_AS_WARNING_FLAG;

		if( record.mFrameId == 0 || ( record.mStartupFrame && !record.mSyncFrame ) )
			frame_flags |= DISPLAY_AS_ERROR_FLAG;

		if( !record.mHeaderCrcOk || !record.mFrameCrcOk )
			frame_flags |= DISPLAY_AS_ERROR_FLAG;

		commit_packet( packet_end_sample, frame_flags, std::move( record ) );
	}
}

bool FlexRayAnalyzer::NeedsRerun()
{
	return false;
}

U32 FlexRayAnalyzer::GenerateSimulationData( U64 minimum_sample_index, U32 device_sample_rate, SimulationChannelDescriptor** simulation_channels )
{
	if( !mSimulationInitialized )
	{
		mSimulationDataGenerator.Initialize( GetSimulationSampleRate(), &mSettings );
		mSimulationInitialized = true;
	}

	return mSimulationDataGenerator.GenerateSimulationData( minimum_sample_index, device_sample_rate, simulation_channels );
}

U32 FlexRayAnalyzer::GetMinimumSampleRateHz()
{
	return mSettings.mBitRate * 4;
}

const char* FlexRayAnalyzer::GetAnalyzerName() const
{
	return "FlexRay";
}

const char* GetAnalyzerName()
{
	return "FlexRay";
}

Analyzer* CreateAnalyzer()
{
	return new FlexRayAnalyzer();
}

void DestroyAnalyzer( Analyzer* analyzer )
{
	delete analyzer;
}
