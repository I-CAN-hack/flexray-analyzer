#ifndef FLEXRAY_COMMON_H
#define FLEXRAY_COMMON_H

#include <AnalyzerTypes.h>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

const U32 kHeaderCrcPolynomial = 0x385;
const U32 kHeaderCrcInit = 0x01A;
const U32 kFrameCrcPolynomial = 0x5D6DCB;
const U32 kFrameCrcInitA = 0xFEDCBA;
const U32 kFrameCrcInitB = 0xABCDEF;

inline U32 CalculateCrc( const std::vector<U8>& bits, U32 polynomial, U32 width, U32 init )
{
	const U32 msb_mask = 1U << ( width - 1 );
	const U32 value_mask = ( 1U << width ) - 1U;
	U32 register_value = init & value_mask;

	for( U8 bit : bits )
	{
		const bool apply_polynomial = ( ( register_value & msb_mask ) != 0 ) ^ ( bit != 0 );
		register_value = ( register_value << 1 ) & value_mask;

		if( apply_polynomial )
			register_value ^= polynomial;
	}

	return register_value & value_mask;
}

inline void AppendBits( std::vector<U8>& bits, U32 value, U32 bit_count )
{
	for( int shift = static_cast<int>( bit_count ) - 1; shift >= 0; --shift )
		bits.push_back( ( value >> shift ) & 0x1 );
}

inline void AppendByteBits( std::vector<U8>& bits, U8 value )
{
	AppendBits( bits, value, 8 );
}

inline std::string FormatPayload( const std::vector<U8>& payload )
{
	if( payload.empty() )
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

#endif // FLEXRAY_COMMON_H
