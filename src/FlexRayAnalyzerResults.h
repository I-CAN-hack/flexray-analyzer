#ifndef FLEXRAY_ANALYZER_RESULTS
#define FLEXRAY_ANALYZER_RESULTS

#include <AnalyzerResults.h>
#include <string>
#include <vector>

class FlexRayAnalyzer;
class FlexRayAnalyzerSettings;

enum FlexRayFrameType : U8
{
	FlexRayTssField = 0,
	FlexRayFssField,
	FlexRayHeaderField,
	FlexRayPayloadByteField,
	FlexRayFrameCrcField,
	FlexRayFesField,
	FlexRayDtsField,
	FlexRayCidField,
	FlexRayCasField,
	FlexRayWakeupSymbolField,
	FlexRayErrorField
};

struct FlexRayFrameRecord
{
	bool mIsError = false;
	bool mIsDynamic = false;
	bool mTssBelowTxSpec = false;
	bool mReservedBit = false;
	bool mPayloadPreamble = false;
	bool mNullFrame = false;
	bool mSyncFrame = false;
	bool mStartupFrame = false;
	bool mHeaderCrcOk = false;
	bool mFrameCrcOk = false;
	bool mCidOk = true;
	U64 mStartSample = 0;
	U64 mEndSample = 0;
	U16 mFrameId = 0;
	U8 mPayloadLengthWords = 0;
	U8 mCycle = 0;
	U16 mHeaderCrc = 0;
	U16 mExpectedHeaderCrc = 0;
	U32 mFrameCrc = 0;
	U32 mExpectedFrameCrc = 0;
	U32 mTssBits = 0;
	U32 mDtsBits = 0;
	U32 mCidBits = 0;
	U32 mWakeupSymbolCount = 0;
	std::vector<U8> mPayload;
	std::string mSymbolName;
	std::string mErrorText;
};

struct FlexRaySegmentRecord
{
	std::string mShortText;
	std::string mLongText;
};

class FlexRayAnalyzerResults : public AnalyzerResults
{
public:
	FlexRayAnalyzerResults( FlexRayAnalyzer* analyzer, FlexRayAnalyzerSettings* settings );
	virtual ~FlexRayAnalyzerResults() = default;

	virtual void GenerateBubbleText( U64 frame_index, Channel& channel, DisplayBase display_base );
	virtual void GenerateExportFile( const char* file, DisplayBase display_base, U32 export_type_user_id );

	virtual void GenerateFrameTabularText( U64 frame_index, DisplayBase display_base );
	virtual void GeneratePacketTabularText( U64 packet_id, DisplayBase display_base );
	virtual void GenerateTransactionTabularText( U64 transaction_id, DisplayBase display_base );

	void AddFlexRaySegment( const Frame& frame, FlexRaySegmentRecord record );
	U64 CommitFlexRayPacket( FlexRayFrameRecord record );

protected:
	const FlexRaySegmentRecord& GetSegmentRecord( U64 frame_index ) const;
	const FlexRayFrameRecord& GetPacketRecord( U64 packet_index ) const;

protected:
	FlexRayAnalyzerSettings* mSettings;
	FlexRayAnalyzer* mAnalyzer;
	std::vector<FlexRaySegmentRecord> mSegmentRecords;
	std::vector<FlexRayFrameRecord> mPacketRecords;
};

#endif //FLEXRAY_ANALYZER_RESULTS
