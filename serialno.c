#include "serialno.h"
#include "utils.h"

/** Convert a binary 0-63 to base-64 UTF16 character + Padding character at 64 '='
* @note RFC 4648 base64 (standard)
*/
static uint16_t bin64ToBase64Utf16( const uint8_t bin64 )
{
#if 0
	static const uint8_t cBase64Table[66] =
		"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/=";
	return cBase64Table[bin64];
#else //< 20bytes smaller!
	const uint8_t twentySix = 26U;
	if( bin64 < (twentySix) )  return 'A' + bin64;
	else if( bin64 < (twentySix * 2) )  return ('a' - twentySix) + bin64;
	else if( bin64 < ((twentySix * 2) + 10) )  return ('0' - (twentySix*2)) + bin64;
	else if( bin64 == 62) return '+';
	else if( bin64 == 63 ) return '/';
	return '='; ///< @note no overrun checks i.e. >= 64 returns '='
#endif
}

static uint16_t encodeBase64Utf16( uint16_t* const encodedBuffer, const uint16_t encodedBufferSize, const uint8_t* const binaryBuffer, const uint16_t binaryBufferLength )
{
	const uint16_t outputLength = (4 * binaryBufferLength + 2) / 3; //< Unpadded size 6-bits per character
	if( outputLength > encodedBufferSize )
		return 0;

	const uint8_t* const inEnd = binaryBuffer + binaryBufferLength;
	const uint8_t* inCursor = binaryBuffer;
	uint16_t* outCursor = encodedBuffer;
	while( inEnd - inCursor >= 3 )
	{
		*outCursor++ = bin64ToBase64Utf16(inCursor[0] >> 2);
		*outCursor++ = bin64ToBase64Utf16(((inCursor[0] & 0x03) << 4) | (inCursor[1] >> 4));
		*outCursor++ = bin64ToBase64Utf16(((inCursor[1] & 0x0F) << 2) | (inCursor[2] >> 6));
		*outCursor++ = bin64ToBase64Utf16(inCursor[2] & 0x3F);
		inCursor += 3;
	}

#if 1 ///< We don't have aligned data atm:(
	if( inEnd - inCursor )
	{
		*outCursor++ = bin64ToBase64Utf16(inCursor[0] >> 2);
		if( inEnd - inCursor == 1 )
		{
			*outCursor++ = bin64ToBase64Utf16((inCursor[0] & 0x03) << 4);
		}
		else
		{
			*outCursor++ = bin64ToBase64Utf16(((inCursor[0] & 0x03) << 4) | (inCursor[1] >> 4));
			*outCursor++ = bin64ToBase64Utf16((inCursor[1] & 0x0F) << 2);
		}
	}
#endif
	return outCursor - encodedBuffer;
}

bool readSerialNumberBase64Utf16( uint16_t* const buffer, const uint16_t bufferLength )
{
#if __SAMD51__

	uint16_t* cursor = buffer;
	uint16_t* const cursorEnd = buffer + bufferLength;

	// Add prefix
	if( cursor + 3 < cursorEnd )
	{
		*cursor++ = 'D';
		*cursor++ = 'A';
		*cursor++ = 'B';
	}

#if 0 //< HW version in serial or not?
	const uint8_t hardwareVersion[3] = {
		9, 3, 5
	};
	cursor += encodeBase64Utf16( cursor, cursorEnd - cursor, (const uint8_t*)hardwareVersion, (uint16_t)sizeof( hardwareVersion ) );
#endif

	// Add delimiter
	if( cursor + 1 < cursorEnd )
	{
		*cursor++ = '-';
	}

    /** @see http://ww1.microchip.com/downloads/en/DeviceDoc/60001507E.pdf
    * Chapter: 9.6 Serial Number
    * Page: 60
    */
    const uint32_t serialNumber[4] = {
          *(uint32_t*)0x008061FC, *(uint32_t*)0x00806010
        , *(uint32_t*)0x00806014, *(uint32_t*)0x00806018
    };
	cursor += encodeBase64Utf16( cursor, cursorEnd-cursor, (const uint8_t*)serialNumber, (uint16_t)sizeof(serialNumber) );

#else
#error "Not implemented"
#endif
	return true;
}