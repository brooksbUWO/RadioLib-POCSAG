#include <Pager.h>
#include <bitset>


PagerClient::PagerClient(PhysicalLayer* phy) 
{
	_phy = phy;
}

int16_t PagerClient::begin(float base, uint16_t speed) 
{
	// calculate duration of 1 bit in us
	_bitDuration = (uint32_t)1000000/speed;

	// calculate 24-bit frequency
	_base = (base * 1000000.0) / _phy->getFreqStep();

	// calculate module carrier frequency resolution
	uint16_t step = round(_phy->getFreqStep());

	// calculate raw frequency shift
	_shift = FREQ_SHIFT_HZ/step;

	// set module frequency deviation to 0
	int16_t state = _phy->setFrequencyDeviation(0);

	return(state);
}

int16_t PagerClient::transmit(String& message, uint32_t address) 
{
	return(PagerClient::transmit(message.c_str(), address));
}

int16_t PagerClient::transmit(const char* message, uint32_t address) 
{
	return(PagerClient::transmit((char *)message, address));
}

int16_t PagerClient::transmit(char *message, uint32_t address) 
{
	size_t messageLength = textMessageLength(address, strlen(message));

	uint32_t* transmission = (uint32_t*) malloc(sizeof(uint32_t) * messageLength);

	encodeTransmission(address, message, transmission);

	// transmit the message
	PagerClient::write(transmission, messageLength);

	//delete[] message;

	// turn transmitter off
	_phy->standby();

	//  return(ERR_NONE);
	return(-1);
}

void PagerClient::testXmit(uint32_t* data, size_t len)
{
	PagerClient::write(data, len);
}

void PagerClient::write(uint32_t* data, size_t len) 
{
	// write code words from buffer
	for(size_t i = 0; i < len; i++) 
	{
		PagerClient::write(data[i]);
		// Output message in binary
		//Serial.printf("%s\n", std::bitset<sizeof(data[i]) * 8>(data[i]).to_string().insert(0, "0b").c_str());

		// Output message in HEX
		Serial.printf("%X\r\n", data[i]);
	}
}

void PagerClient::write(uint32_t codeWord) 
{
	// write single code word
	for(uint8_t i = 0; i <= 31; i++) 
	{
		uint32_t mask = (uint32_t)0x01 << i;
		if(codeWord & mask) 
		{
			// send 1
			uint32_t start = micros();
			_phy->transmitDirect(_base + _shift);
			while(micros() - start < _bitDuration);
		} 
		else 
		{
			// send 0
			uint32_t start = micros();
			_phy->transmitDirect(_base - _shift);
			while(micros() - start < _bitDuration);
		}
	}
}

/**
 * Calculate the CRC error checking code for the given word.
 * Messages use a 10 bit CRC computed from the 21 data bits.
 * This is calculated through a binary polynomial long division, returning
 * the remainder.
 * See https://en.wikipedia.org/wiki/Cyclic_redundancy_check#Computation
 * for more information.
 */
uint32_t PagerClient::crc(uint32_t inputMsg)
{
    //Align MSB of denominatorerator with MSB of message
    uint32_t denominator = CRC_GENERATOR << 20;

    //Message is right-padded with zeroes to the message length + crc length
    uint32_t msg = inputMsg << CRC_BITS;

    //We iterate until denominator has been right-shifted back to it's original value.
    for (int column = 0; column <= 20; column++)
    {
        //Bit for the column we're aligned to
        int msgBit = (msg >> (30 - column)) & 1;

        //If the current bit is zero, we don't modify the message this iteration
        if (msgBit != 0)
        {
            //While we would normally subtract in long division, we XOR here.
            msg ^= denominator;
        }

        //Shift the denominator over to align with the next column
        denominator >>= 1;
    }

    //At this point 'msg' contains the CRC value we've calculated
    return msg & 0x3FF;
}


/**
 * Calculates the even parity bit for a message.
 * If the number of bits in the message is even, return 0, else return 1.
 */
uint32_t PagerClient::parity(uint32_t x)
{
    //Our parity bit
    uint32_t p = 0;

    //We xor p with each bit of the input value. This works because
    //xoring two one-bits will cancel out and leave a zero bit.  Thus
    //xoring any even number of one bits will result in zero, and xoring
    //any odd number of one bits will result in one.
    for (int i = 0; i < 32; i++)
    {
        p ^= (x & 1);
        x >>= 1;
    }
    return p;
}

/**
 * Encodes a 21-bit message by calculating and adding a CRC code and parity bit.
 */
uint32_t PagerClient::encodeCodeword(uint32_t msg)
{
    uint32_t fullCRC = (msg << CRC_BITS) | crc(msg);
    uint32_t p = parity(fullCRC);
    return (fullCRC << 1) | p;
}

/**
 * ASCII encode a null-terminated string as a series of codewords, written
 * to (*out). Returns the number of codewords written. Caller should ensure
 * that enough memory is allocated in (*out) to contain the message
 *
 * initial_offset indicates which word in the current batch the function is
 * beginning at, so that it can insert SYNC words at appropriate locations.
 */
uint32_t PagerClient::encodeASCII(uint32_t initial_offset, char* str, uint32_t* out)
{
    //Number of words written to *out
    uint32_t numWordsWritten = 0;

    //Data for the current word we're writing
    uint32_t currentWord = 0;

    //Nnumber of bits we've written so far to the current word
    uint32_t currentNumBits = 0;

    //Position of current word in the current batch
    uint32_t wordPosition = initial_offset;

    while (*str != 0)
    {
        unsigned char c = *str;
        str++;
        //Encode the character bits backwards
        for (int i = 0; i < TEXT_BITS_PER_CHAR; i++)
        {
            currentWord <<= 1;
            currentWord |= (c >> i) & 1;
            currentNumBits++;
            if (currentNumBits == TEXT_BITS_PER_WORD)
            {
                //Add the MESSAGE flag to our current word and encode it.
                *out = encodeCodeword(currentWord | FLAG_MESSAGE);
                out++;
                currentWord = 0;
                currentNumBits = 0;
                numWordsWritten++;

                wordPosition++;
                if (wordPosition == BATCH_SIZE)
                {
                    //We've filled a full batch, time to insert a SYNC word
                    //and start a new one.
                    *out = SYNC;
                    out++;
                    numWordsWritten++;
                    wordPosition = 0;
                }
            }
        }
    }

    //Write remainder of message
    if (currentNumBits > 0)
    {
        //Pad out the word to 20 bits with zeroes
        currentWord <<= 20 - currentNumBits;
        *out = encodeCodeword(currentWord | FLAG_MESSAGE);
        out++;
        numWordsWritten++;

        wordPosition++;
        if (wordPosition == BATCH_SIZE)
        {
            //We've filled a full batch, time to insert a SYNC word
            //and start a new one.
            *out = SYNC;
            out++;
            numWordsWritten++;
            wordPosition = 0;
        }
    }

    return numWordsWritten;
}

/**
 * An address of 21 bits, but only 18 of those bits are encoded in the address
 * word itself. The remaining 3 bits are derived from which frame in the batch
 * is the address word. This calculates the number of words (not frames!)
 * which must precede the address word so that it is in the right spot. These
 * words will be filled with the idle value.
 */
int PagerClient::addressOffset(int address)
{
    return (address & 0x3) * FRAME_SIZE;
}

/**
 * Encode a full text POCSAG transmission addressed to (address).
 * (*message) is a null terminated C string.
 * (*out) is the destination to which the transmission will be written.
 */
void PagerClient::encodeTransmission(int address, char* message, uint32_t* out)
{
    //Encode preamble
    //Alternating 1,0,1,0 bits for 576 bits, used for receiver to synchronize
    //with transmitter
    for (int i = 0; i < PREAMBLE_LENGTH / 32; i++)
    {
        //*out = 0xAAAAAAAA;
        *out = 0x55555555;		
        out++;
    }

    uint32_t* start = out;

    //Sync
    *out = SYNC;
    out++;

    //Write out padding before adderss word
    int prefixLength = addressOffset(address);
    for (int i = 0; i < prefixLength; i++)
    {
        *out = IDLE;
        out++;
    }

    //Write address word.
    //The last two bits of word's data contain the message type
    //The 3 least significant bits are dropped, as those are encoded by the
    //word's location.
    *out = encodeCodeword( ((address >> 3) << 2) | FLAG_TEXT_DATA);
    out++;

    //Encode the message itself
    out += encodeASCII(addressOffset(address) + 1, message, out);

    //Finally, write an IDLE word indicating the end of the message
    *out = IDLE;
    out++;

    //Pad out the last batch with IDLE to write multiple of BATCH_SIZE + 1
    //words (+ 1 is there because of the SYNC words)
    size_t written = out - start;
    size_t padding = (BATCH_SIZE + 1) - written % (BATCH_SIZE + 1);
    for (int i = 0; i < padding; i++)
    {
        *out = IDLE;
        out++;
    }
}

/**
 * Calculates the length in words of a text POCSAG message, given the address
 * and the number of characters to be transmitted.
 */
size_t PagerClient::textMessageLength(int address, int numChars)
{
    size_t numWords = 0;

    //Padding before address word.
    numWords += addressOffset(address);

    //Address word itself
    numWords++;

    //numChars * 7 bits per character / 20 bits per word, rounding up
    numWords += (numChars * TEXT_BITS_PER_CHAR + (TEXT_BITS_PER_WORD - 1))
                    / TEXT_BITS_PER_WORD;

    //Idle word representing end of message
    numWords++;

    //Pad out last batch with idles
    numWords += BATCH_SIZE - (numWords % BATCH_SIZE);

    //Batches consist of 16 words each and are preceded by a sync word.
    //So we add one word for every 16 message words
    numWords += numWords / BATCH_SIZE;

    //Preamble of 576 alternating 1,0,1,0 bits before the message
    //Even though this comes first, we add it to the length last so it
    //doesn't affect the other word-based calculations
    numWords += PREAMBLE_LENGTH / 32;

    return numWords;
}
