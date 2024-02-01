/**
 * @file BQ34Z100.cpp
 * @author Kyle Marino, Jamie Smith, Tyler Sehon
 * @brief Software Driver for BQ34Z100-G1 Battery Gauge 
 *
 * Datasheet: http://www.ti.com/lit/ds/symlink/bq34z100-g1.pdf *
 */

#include "BQ34Z100.h"

#include <cinttypes>

BQ34Z100::BQ34Z100(I2C& i2c, int hz):
_i2c(i2c)
{
    // Set the I2C bus frequency
    _i2c.frequency(hz);
}

void BQ34Z100::sendControlCommand(Control control)
{
	uint16_t controlBytes = static_cast<uint16_t>(control);
	write(Command::Control, controlBytes & 0xFF, (controlBytes >> 8) & 0xFF);
}

uint16_t BQ34Z100::readControlCommand(Control control)
{
	sendControlCommand(control);
    return read(Command::Control, 2);
}

void BQ34Z100::write(Command command, const uint8_t cmd)
{
    uint8_t bytes[2] = {static_cast<uint8_t>(command), cmd};
    int writeResult = _i2c.write(
        GAUGE_ADDRESS | 0x0,
        reinterpret_cast<char*>(bytes), 
        2
    );

    if(writeResult != 0)
    {
        printf("A write error has occurred when transmitting address.\r\n");
    }
}

void BQ34Z100::write(Command command, const uint8_t cmd1, const uint8_t cmd2)
{
    uint8_t bytes[3] = {static_cast<uint8_t>(command), cmd1, cmd2};
	int writeResult = _i2c.write(
        GAUGE_ADDRESS | 0x0,
        reinterpret_cast<char*>(bytes),
        3
    );

    if(writeResult != 0)
    {
        printf("A write error has occurred when transmitting address.\r\n");
    }
}

uint32_t BQ34Z100::read(Command command, const uint8_t length)
{
	uint32_t val = 0; // Contains next byte of data that will be read

    for (int i = 0; i < length; i++)
    {
        uint8_t cmdByte = static_cast<uint8_t>(command) + i;
        int writeResult = _i2c.write(
            GAUGE_ADDRESS | 0x0,
            reinterpret_cast<char*>(&cmdByte), 
            1
        );

        if(writeResult != 0)
        {
            printf("A write error has occurred when transmitting address.\r\n");
        }

        char readByte;
        int readResult = _i2c.read(
            GAUGE_ADDRESS | 0x1,
            &readByte,
            1
        );

        if(readResult != 0)
        {
            printf("A read error has occurred when reading data.\r\n");
        }

    	// Swap bytes around (convert to Little Endian)
    	val |= (static_cast<uint32_t>(readByte) << (8 * i));
	}

	return val;
}

void BQ34Z100::enableCal()
{
    sendControlCommand(Control::CAL_ENABLE);
}

void BQ34Z100::enterCal()
{
    sendControlCommand(Control::ENTER_CAL);
}

void BQ34Z100::exitCal()
{
    sendControlCommand(Control::EXIT_CAL);
}

void BQ34Z100::ITEnable()
{
    sendControlCommand(Control::IT_ENABLE);
}

uint16_t BQ34Z100::getStatus()
{
	return readControlCommand(Control::CONTROL_STATUS);
}

uint16_t BQ34Z100::getChemID()
{
	return readControlCommand(Control::CHEM_ID);
}

uint16_t BQ34Z100::getStateOfHealth()
{
	return read(Command::StateOfHealth, 2);
}

uint8_t BQ34Z100::getSOC()
{
	return read(Command::StateOfCharge, 1);
}

uint16_t BQ34Z100::getError()
{
	return read(Command::MaxError, 1);
}

uint16_t BQ34Z100::getRemaining()
{
	return read(Command::RemainingCapacity, 2);
}

uint16_t BQ34Z100::getVoltage()
{
	return read(Command::Voltage, 2);
}

int16_t BQ34Z100::getCurrent()
{
	int16_t result = read(Command::Current, 2);
	if (result < 0) result = -result;
	return result;
}

double BQ34Z100::getTemperature()
{
	// The device returns an internal temperature in units of 0.1K
	// Convert to celsius by subtracting dividing by 10 then subtracting 273.15K
	// e.g. if the device gives us 200, the device read an internal temperature of 200(0.1K) = 20K
	// to convert to C, we subtract 273.15
	return (read(Command::Temperature, 2)/10.0) - 273.15;
}

int BQ34Z100::getSerial()
{
	return read(Command::SerialNumber, 2);
}

void BQ34Z100::reset()
{
    sendControlCommand(Control::RESET);
	ThisThread::sleep_for(175ms); // experimentally determined boot time
}

void BQ34Z100::unseal()
{
    sendControlCommand(Control::UNSEAL_KEY1);
    sendControlCommand(Control::UNSEAL_KEY2);
}

void BQ34Z100::seal()
{
    sendControlCommand(Control::SEALED);
}

/** TODO: see if we can get away with taking the ThisThread::sleep_for away 
 * as sleeping threads slows down the main Hamster loop */
void BQ34Z100::changePage(char subclass, uint16_t offset)
{
	ThisThread::sleep_for(10ms);
	// Enable block data flash control (single byte write)
	write(Command::BlockDataControl, 0x00);
	ThisThread::sleep_for(10ms);

	// Use DataFlashClass() command to access the subclass
	write(Command::DataFlashClass, subclass);
    currFlashPage = subclass;
	ThisThread::sleep_for(10ms);

	// Select the block offset location
	// Blocks are 32 in size, so the offset is which block the data sits in
	// Ex: 16 is block 0x00, 52 is block 0x01 (called "index" variable)
	currFlashBlockIndex =(uint8_t)(offset/32);
	write(Command::DataFlashBlock, currFlashBlockIndex);
	ThisThread::sleep_for(20ms);
}

void BQ34Z100::updateChecksum()
{

    uint8_t newChecksum = calcChecksum();

	// Send new checksum thru I2C
	write(Command::BlockDataCheckSum, newChecksum);
    printf("Writing new checksum for page %" PRIu8 " block %" PRIu8 ": 0x%" PRIx8 "\r\n", currFlashPage, currFlashBlockIndex, newChecksum);

    ThisThread::sleep_for(50ms); //Wait for BQ34Z100 to process, may be totally overkill
}


//Warning: change page first before reading data
//Copies a page in flash into the "flashbytes" array
void BQ34Z100::readFlash()
{
    char command = static_cast<char>(Command::BlockData); //Command to read Data Flash;
	//Request read flash
	int writeResult = _i2c.write(GAUGE_ADDRESS | 0x0, &command, 1);

    if(writeResult != 0)
    {
        printf("A write error has occurred when transmitting address.\r\n");
    }

	//Read all bytes from page (which is 32 bytes long)
    int readResult = _i2c.read(
        GAUGE_ADDRESS | 0x01,
        reinterpret_cast<char*>(flashbytes),
        32
    );

    if(readResult != 0)
    {
        printf("A read error has occurred when reading data.\r\n");
    }

    uint8_t expectedChecksum = read(Command::BlockDataCheckSum, 1);

    if(expectedChecksum != calcChecksum())
    {
        printf("ERROR: Checksum of flash memory block does not match.  I2C read was likely corrupted.");
    }

    printf("Page %" PRIu8 " block %" PRIu8 " contents:", currFlashPage, currFlashBlockIndex);
    for(size_t byteIndex = 0; byteIndex < 32; ++byteIndex)
    {
        printf(" %" PRIx8, flashbytes[byteIndex]);
    }
    printf("\r\n");
	printf("Checksum: 0x%" PRIx8 "\r\n", expectedChecksum);

	ThisThread::sleep_for(10ms); //Is this necessary?
}

void BQ34Z100::writeFlash(uint8_t index, uint32_t value, int len)
{
	//Has to be a number between 0 to 31
	//Use changePage to set the offset correctly beforehand
	if (index > 31) index = index % 32;

	//Write to I2C bus and change flashbytes at the same time
	//Necessary for checksum calculation at the end
	if (len == 1)
	{
		flashbytes[index] = value;
		write(static_cast<Command>(static_cast<uint8_t>(Command::BlockData) + index), (unsigned char)value);
		printf("Flash[%d] <- 0x%" PRIx8 "\n", index, flashbytes[index]);

	} else if (len > 1)
	{
		//Process every byte except the last
		for (int i = 0; i<len-1; i++)
		{
			flashbytes[index+i] = value >> 8*((len-1)-i);
			printf("Flash[%d] <- 0x%" PRIx8 "\n", index + i, flashbytes[index+i]);
			write(static_cast<Command>(static_cast<uint8_t>(Command::BlockData) + index + i), flashbytes[index+i]);
		}

		//Last byte (lower byte)
		flashbytes[index+len-1] = value & 0xFF;
		printf("Flash[%d] <- 0x%" PRIx8 "\n", index + (len - 1), flashbytes[index+len-1]);
		write(static_cast<Command>(static_cast<uint8_t>(Command::BlockData) + index + len - 1), value & 0xFF);
	}
}

uint8_t* BQ34Z100::getFlashBytes()
{
	return flashbytes;
}

void BQ34Z100::changePage48()
{
	changePage(48, 0); //Block 48, offset 0
	readFlash();
	writeFlash(11, DESIGNCAP, 2);
	writeFlash(13, DESIGNENERGY, 2);
	updateChecksum();
	ThisThread::sleep_for(300ms);
}

void BQ34Z100::changePage64()
{
	changePage(64, 0); //Block 64, offset 0
	readFlash();

	//Edit pack configuration register (has an upper and lower byte)
	//Enables external voltage divider use
	uint8_t packConfig_high = flashbytes[0];
	uint8_t packConfig_low = flashbytes[1];
	if (VOLTSEL)
	{
		packConfig_high |= 0x08; //00001000
	}

	//Also allow calibration by switching the CAL_EN bit to 1
	packConfig_high |= 0x40; //01000000

	//Set temps bit.
	packConfig_low &= ~(1);
	packConfig_low |= USE_EXTERNAL_THERMISTOR;

	writeFlash(0, packConfig_high, 1);
	writeFlash(1, packConfig_low, 1);

	// Disable fast convergence as recommended in the calibration procedure
	uint8_t packConfigB = flashbytes[2];
	packConfigB &= ~1;
	writeFlash(2, packConfigB, 1);

	//Update LED config and number of cells in battery
	writeFlash(4, LEDCONFIG, 1);
	writeFlash(7, 0x04, 1);
	updateChecksum();
	ThisThread::sleep_for(300ms);
}

void BQ34Z100::changePage80()
{
	changePage(80, 0);
	readFlash();

	writeFlash(0, LOADSELECT, 1);
	writeFlash(1, LOADMODE, 1);
	writeFlash(10, 10, 2);
	updateChecksum();
	ThisThread::sleep_for(300ms);

	changePage(80, 53);
	readFlash();

	writeFlash(53, ZEROCHARGEVOLT, 2);
	updateChecksum();
	ThisThread::sleep_for(300ms);
}

void BQ34Z100::changePage82()
{
	changePage(82, 0);
	readFlash();
	//Update QMax cell 0
	writeFlash(0, QMAX0, 2);
	updateChecksum();
	ThisThread::sleep_for(300ms);
}

uint16_t BQ34Z100::calibrateVoltage(uint16_t currentVoltage)
{
	changePage(104, 0);
	readFlash();
	//Gets the voltage divider value stored in flash
	uint16_t flashVoltage = (uint16_t)(flashbytes[14] << 8);
	flashVoltage |= (uint16_t)(flashbytes[15]);

	float readVoltage = (float)getVoltage();
	float newSetting = ((float)(currentVoltage)/readVoltage)*(float)(flashVoltage);
	uint16_t writeSetting; //This 2 byte integer will be written to chip
	if (newSetting>65535.0f) writeSetting=65535;
	else if (newSetting < 0.0f) writeSetting=0;
	else writeSetting=(uint16_t)(newSetting);
	writeFlash(14, writeSetting, 2);
	updateChecksum();
	ThisThread::sleep_for(10ms);

	// also change the "Flash Update OK Cell Volt" as the datasheet says to:
    changePage(68, 0);
    readFlash();

    int16_t oldUpdateOK = 0;
    oldUpdateOK |= (static_cast<uint16_t>(flashbytes[0]) << 8);
    oldUpdateOK |= flashbytes[1];

    int16_t newUpdateOK = static_cast<int16_t>(round(FLASH_UPDATE_OK_VOLT * CELLCOUNT * (5000.0f/writeSetting)));
    printf("Changing Flash Update OK Voltage from %" PRIi16 " to %" PRIi16 "\r\n", oldUpdateOK, newUpdateOK);

    writeFlash(0, newUpdateOK, 2);
    updateChecksum();

	//Test output
	printf("Register (voltage divider): %d\r\n", flashVoltage);
	printf("New Ratio: %f\r\n", ((float)(currentVoltage)/readVoltage));
	printf("READ VOLTAGE (mv): %f\r\n", readVoltage);
	return (uint16_t)newSetting;
}

void BQ34Z100::resetVoltageDivider()
{
    changePage(104, 0);
    readFlash();
    writeFlash(14, RESETVOLTAGE, 2);
    updateChecksum();
    ThisThread::sleep_for(300ms);
}

void BQ34Z100::calibrateShunt(int16_t calCurrent)
{
	if (calCurrent < 0) calCurrent = -calCurrent;

	int16_t currentReading = getCurrent();
	if (currentReading < 0) currentReading = -currentReading;

	changePage(104, 0); // see table 11 in datasheet
	readFlash();
	ThisThread::sleep_for(30ms);

	// read and convert CC Gain
	uint32_t currentGainDF = ((uint32_t)flashbytes[0]) << 24 | ((uint32_t)flashbytes[1]) << 16 | ((uint32_t)flashbytes[2]) << 8 | (uint32_t)flashbytes[3];
	float currentGainResistance = (4.768f/xemicsToFloat(currentGainDF));

	uint32_t currentDeltaDF = ((uint32_t)flashbytes[4]) << 24 | ((uint32_t)flashbytes[5]) << 16 | ((uint32_t)flashbytes[6]) << 8 | (uint32_t)flashbytes[7];
	float currentDeltaResistance = (5677445/xemicsToFloat(currentGainDF));

	float newGain = (((float)currentReading)/((float)calCurrent)) * currentGainResistance;

	uint32_t newGainDF = floatToXemics(4.768 / newGain);
    float DeltaDF = floatToXemics(5677445/newGain);

    printf("currentGainDF = 0x%" PRIx32 ", currentGainResistance = %f, currentDeltaDF = %" PRIx32 ", currentDeltaResistance = %f, newGain = %f, newGainDF = 0x%" PRIx32 "\n",
           currentGainDF, currentGainResistance, currentDeltaDF, currentDeltaResistance, newGain, newGainDF);

    writeFlash(0, newGainDF, 4);
	writeFlash(4, DeltaDF, 4);

	updateChecksum();
	ThisThread::sleep_for(300ms);
}

void BQ34Z100::setSenseResistor() {
    changePage(104, 0);
	readFlash(); // This is necessary because we need to have the full "page" in the buffer.
	// Any time we do changePage, we need to run readFlash.
    ThisThread::sleep_for(30ms);

    //Constants use to convert mOhm to the BQ34Z100 data flash units
    uint32_t GainDF = floatToXemics(4.768/SENSE_RES);
    uint32_t DeltaDF = floatToXemics(5677445/SENSE_RES);
    writeFlash(0, GainDF, 4);
	writeFlash(4, DeltaDF, 4);

	printf("newGain = %f, GainDF = 0x%" PRIx32 "\n",
	       SENSE_RES, GainDF);

    updateChecksum();
	ThisThread::sleep_for(300ms);
}

uint16_t BQ34Z100::readDeviceType()
{
	return readControlCommand(Control::DEVICE_TYPE);
}

uint16_t BQ34Z100::readFWVersion() {
    return readControlCommand(Control::FW_VERSION);
}

uint16_t BQ34Z100::readHWVersion() {
    return readControlCommand(Control::HW_VERSION);
}

uint32_t BQ34Z100::floatToXemics(float value) {
	int iByte1, iByte2, iByte3, iByte4, iExp;
	bool bNegative = false;
	float fMantissa;
	// Don't blow up with logs of zero
	if (value == 0) value = 0.00001F;
	if (value < 0)
	{
		bNegative = true;
		value = -value;
	}
	// find the correct exponent
	iExp = (int)((log(value) / log(2)) + 1);// remember - log of any base is ln(x)/ln(base)

	// MS byte is the exponent + 0x80
	iByte1 = iExp + 128;

	// Divide input by this exponent to get mantissa
	fMantissa = value / (pow(2, iExp));

	// Scale it up
	fMantissa = fMantissa / (pow(2, -24));

	// Split the mantissa into 3 bytes
	iByte2 = (int)(fMantissa / (pow(2, 16)));

	iByte3 = (int)((fMantissa - (iByte2 * (pow(2, 16)))) / (pow(2, 8)));

	iByte4 = (int)(fMantissa - (iByte2 * (pow(2, 16))) - (iByte3 * (pow(2, 8))));

	// subtract the sign bit if number is positive
	if (bNegative == false)
	{
		iByte2 = iByte2 & 0x7F;
	}
	return (uint32_t)((uint32_t)iByte1 << 24 | (uint32_t)iByte2 << 16 | (uint32_t)iByte3 << 8 | (uint32_t)iByte4);
}

float BQ34Z100::xemicsToFloat(uint32_t xemics)
{
	bool bIsPositive = false;
	float fExponent, fResult;
	uint8_t vMSByte = (uint8_t)(xemics >> 24);
	uint8_t vMidHiByte = (uint8_t)(xemics >> 16);
	uint8_t vMidLoByte = (uint8_t)(xemics >> 8);
	uint8_t vLSByte = (uint8_t)xemics;
	// Get the sign, its in the 0x00 80 00 00 bit
	if ((vMidHiByte & 128) == 0)
	{ bIsPositive = true; }

	// Get the exponent, it's 2^(MSbyte - 0x80)
	fExponent = pow(2, (vMSByte - 128));
	// Or in 0x80 to the MidHiByte
	vMidHiByte = (uint8_t)(vMidHiByte | 128);
	// get value out of midhi byte
	fResult = (vMidHiByte) * 65536;
	// add in midlow byte
	fResult = fResult + (vMidLoByte * 256);
	// add in LS byte
	fResult = fResult + vLSByte;
	// multiply by 2^-24 to get the actual fraction
	fResult = fResult * pow(2, -24);
	// multiply fraction by the ‘exponent’ part
	fResult = fResult * fExponent;
	// Make negative if necessary
	if (bIsPositive)
		return fResult;
	else
		return -fResult;
}

uint8_t BQ34Z100::getUpdateStatus() {
	changePage(82, 0);
	readFlash();
	return flashbytes[4];
}

std::pair<uint16_t, uint16_t> BQ34Z100::getFlags() {
	uint16_t flags = read(Command::Flags, 2);
	uint16_t flagsB = read(Command::FlagsB, 2);
	return std::make_pair(flags, flagsB);
}

uint8_t BQ34Z100::calcChecksum() {
    uint8_t chkSum = 0;
    //Perform 8 bit summation of the entire BlockData() on byte-per-byte
    //basis. Checksum = (255-sum)
    //For every byte in flashbytes, add to sum
    for (int i = 0; i<32; i++)
    {
        chkSum += flashbytes[i];
    }

    chkSum = 255 - chkSum;

    return chkSum;
}
