#include "serialno.h"
#include "utils.h"

/** Convert a binary 0-63 to base-64 UTF16 character + Padding character at 64 '='
* @todo Optimise for size!
*/
static uint16_t bin64ToBase64Utf16( uint8_t bin64 )
{
	static const uint8_t cBase64Table[66] =
		"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/=";
	return cBase64Table[bin64];
}

static uint16_t encodeBase64Size( const uint16_t binaryBufferLength, bool padding )
{
	const uint16_t unpadded = (4 * binaryBufferLength) / 3;//< 3-byte blocks to 4-byte
	return padding ? unpadded : (unpadded + 3) & ~3;
}

static uint16_t encodeBase64Utf16( uint16_t* const encodedBuffer, const uint16_t encodedBufferSize, const uint8_t* const binaryBuffer, const uint16_t binaryBufferLength, const bool padding )
{
	const uint16_t outputLength = encodeBase64Size( binaryBufferLength, padding );
	if( outputLength < binaryBufferLength )
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

	if( inEnd - inCursor )
	{
		*outCursor++ = bin64ToBase64Utf16(inCursor[0] >> 2);
		if( inEnd - inCursor == 1 )
		{
			*outCursor++ = bin64ToBase64Utf16((inCursor[0] & 0x03) << 4);
			if( padding ) *outCursor++ = bin64ToBase64Utf16(64);
		}
		else
		{
			*outCursor++ = bin64ToBase64Utf16(((inCursor[0] & 0x03) << 4) | (inCursor[1] >> 4));
			*outCursor++ = bin64ToBase64Utf16((inCursor[1] & 0x0F) << 2);
		}
		if( padding ) *outCursor++ = bin64ToBase64Utf16( 64 );
	}

	return outCursor - encodedBuffer;
}

bool readSerialNumberBase64Utf16( uint16_t* const buffer, const uint16_t bufferLength )
{
#if __SAMD51__
    /** @see http://ww1.microchip.com/downloads/en/DeviceDoc/60001507E.pdf
    * Chapter: 9.6 Serial Number
    * Page: 60
    */
    const uint32_t serialNumber[4] = {
          *(uint32_t*)0x008061FC, *(uint32_t*)0x00806010
        , *(uint32_t*)0x00806014, *(uint32_t*)0x00806018
    };

	encodeBase64Utf16( buffer, bufferLength, (const uint8_t*)serialNumber, MIN( bufferLength, (uint16_t)sizeof(serialNumber) ), false );
#else
#error "Not implemented"
#endif
	return true;
}