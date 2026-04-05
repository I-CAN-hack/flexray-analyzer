#include "FlexRaySimulationDataGenerator.h"

#include "FlexRayAnalyzerSettings.h"
#include <AnalyzerHelpers.h>

namespace
{
const U32 kHeaderCrcPolynomial = 0x385;
const U32 kHeaderCrcInit = 0x01A;
const U32 kFrameCrcPolynomial = 0x5D6DCB;
const U32 kFrameCrcInitA = 0xFEDCBA;
const U32 kFrameCrcInitB = 0xABCDEF;
const U32 kSimulationTssBits = 5;

void AppendBits( std::vector<U8>& bits, U32 value, U32 bit_count )
{
	for( int shift = static_cast<int>( bit_count ) - 1; shift >= 0; --shift )
		bits.push_back( ( value >> shift ) & 0x1 );
}

void AppendByteBits( std::vector<U8>& bits, U8 value )
{
	AppendBits( bits, value, 8 );
}

std::vector<U8> BitsToBytes( const std::vector<U8>& bits )
{
	std::vector<U8> bytes;
	bytes.reserve( bits.size() / 8 );

	for( size_t bit_index = 0; bit_index < bits.size(); bit_index += 8 )
	{
		U8 value = 0;
		for( size_t shift = 0; shift < 8; ++shift )
			value = static_cast<U8>( ( value << 1 ) | bits.at( bit_index + shift ) );

		bytes.push_back( value );
	}

	return bytes;
}
}

FlexRaySimulationDataGenerator::FlexRaySimulationDataGenerator()
:	mSettings( nullptr ),
	mSimulationSampleRateHz( 0 ),
	mSamplesPerBit( 0 ),
	mFrameCounter( 0 )
{
}

FlexRaySimulationDataGenerator::~FlexRaySimulationDataGenerator()
{
}

void FlexRaySimulationDataGenerator::Initialize( U32 simulation_sample_rate, FlexRayAnalyzerSettings* settings )
{
	mSimulationSampleRateHz = simulation_sample_rate;
	mSettings = settings;
	mSamplesPerBit = std::max<U32>( 1, mSimulationSampleRateHz / mSettings->mBitRate );

	mSimulationData.SetChannel( mSettings->mInputChannel );
	mSimulationData.SetSampleRate( simulation_sample_rate );
	mSimulationData.SetInitialBitState( mSettings->mInvertInput ? BIT_LOW : BIT_HIGH );
}

U32 FlexRaySimulationDataGenerator::GenerateSimulationData( U64 largest_sample_requested, U32 sample_rate,
															SimulationChannelDescriptor** simulation_channel )
{
	const U64 adjusted_largest_sample_requested =
		AnalyzerHelpers::AdjustSimulationTargetSample( largest_sample_requested, sample_rate, mSimulationSampleRateHz );

	while( mSimulationData.GetCurrentSampleNumber() < adjusted_largest_sample_requested )
	{
		const U8 cycle = static_cast<U8>( mFrameCounter % 64 );

		if( ( mFrameCounter % 2 ) == 0 )
			OutputFrame( 1, cycle, { 0x10, 0x01, 0xAA, 0x55 }, false, true, cycle == 0 );
		else
			OutputFrame( 1200, cycle, { 0xBE, 0xEF }, true, false, false );

		++mFrameCounter;
	}

	*simulation_channel = &mSimulationData;
	return 1;
}

void FlexRaySimulationDataGenerator::OutputFrame( U16 frame_id, U8 cycle, const std::vector<U8>& payload, bool dynamic_frame,
													 bool sync_frame, bool startup_frame )
{
	const U8 payload_length_words = static_cast<U8>( payload.size() / 2 );
	std::vector<U8> header_bits;
	std::vector<U8> header_crc_input;
	std::vector<U8> payload_bits;

	header_bits.reserve( 40 );
	header_crc_input.reserve( 20 );
	payload_bits.reserve( payload.size() * 8 );

	header_bits.push_back( 0 ); // Reserved bit
	header_bits.push_back( 0 ); // Payload preamble indicator
	header_bits.push_back( 1 ); // Null frame indicator, inverted: 1 means data frame.
	header_bits.push_back( sync_frame ? 1 : 0 );
	header_bits.push_back( startup_frame ? 1 : 0 );
	AppendBits( header_bits, frame_id, 11 );
	AppendBits( header_bits, payload_length_words, 7 );

	header_crc_input.push_back( sync_frame ? 1 : 0 );
	header_crc_input.push_back( startup_frame ? 1 : 0 );
	AppendBits( header_crc_input, frame_id, 11 );
	AppendBits( header_crc_input, payload_length_words, 7 );
	AppendBits( header_bits, CalculateCrc( header_crc_input, kHeaderCrcPolynomial, 11, kHeaderCrcInit ), 11 );
	AppendBits( header_bits, cycle, 6 );

	for( U8 value : payload )
		AppendByteBits( payload_bits, value );

	std::vector<U8> frame_crc_input = header_bits;
	frame_crc_input.insert( frame_crc_input.end(), payload_bits.begin(), payload_bits.end() );
	const U32 frame_crc = CalculateCrc( frame_crc_input, kFrameCrcPolynomial, 24,
										mSettings->mChannelType == 0 ? kFrameCrcInitA : kFrameCrcInitB );

	const std::vector<U8> header_bytes = BitsToBytes( header_bits );
	const std::vector<U8> frame_crc_bytes = { static_cast<U8>( ( frame_crc >> 16 ) & 0xFF ), static_cast<U8>( ( frame_crc >> 8 ) & 0xFF ),
											  static_cast<U8>( frame_crc & 0xFF ) };

	OutputBit( 1, 8 );
	OutputBit( 0, kSimulationTssBits );
	OutputBit( 1, 1 ); // FSS

	for( U8 value : header_bytes )
		OutputExtendedByte( value );

	for( U8 value : payload )
		OutputExtendedByte( value );

	for( U8 value : frame_crc_bytes )
		OutputExtendedByte( value );

	OutputBit( 0, 1 ); // FES low
	OutputBit( 1, 1 ); // FES high

	if( dynamic_frame == true )
	{
		OutputBit( 0, 3 ); // DTS low
		OutputBit( 1, 1 ); // DTS high
	}

	OutputBit( 1, 11 ); // CID
}

void FlexRaySimulationDataGenerator::OutputBit( U8 wire_bit, U32 bit_count )
{
	const U8 physical_bit = mSettings->mInvertInput ? ( wire_bit ^ 0x1 ) : wire_bit;
	mSimulationData.TransitionIfNeeded( physical_bit != 0 ? BIT_HIGH : BIT_LOW );
	mSimulationData.Advance( mSamplesPerBit * bit_count );
}

void FlexRaySimulationDataGenerator::OutputExtendedByte( U8 value )
{
	OutputBit( 1, 1 );
	OutputBit( 0, 1 );

	for( int shift = 7; shift >= 0; --shift )
		OutputBit( static_cast<U8>( ( value >> shift ) & 0x1 ), 1 );
}

U32 FlexRaySimulationDataGenerator::CalculateCrc( const std::vector<U8>& bits, U32 polynomial, U32 width, U32 init ) const
{
	const U32 msb_mask = 1U << ( width - 1 );
	const U32 value_mask = ( 1U << width ) - 1U;
	U32 register_value = init & value_mask;

	for( U8 bit : bits )
	{
		const bool apply_polynomial = ( ( register_value & msb_mask ) != 0 ) ^ ( bit != 0 );
		register_value = ( register_value << 1 ) & value_mask;

		if( apply_polynomial == true )
			register_value ^= polynomial;
	}

	return register_value & value_mask;
}
