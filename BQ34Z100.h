/**
 * @file BQ34Z100.h
 * @author Kyle Marino, Jamie Smith, Tyler Sehon
 * @brief Software Driver for BQ34Z100-G1 Battery Gauge 
 * 
 * Datasheet: http://www.ti.com/lit/ds/symlink/bq34z100-g1.pdf
 * 
 * Existing arduino code to reference from unknown author: https://github.com/Ralim/BQ34Z100
 * Partial library from TI that may be useful: http://www.ti.com/lit/an/slua801/slua801.pdf
 * Battery information: https://www.batteryspace.com/polymerli-ioncell37v1400mah703562-10c518wh10adrainrate-ullisted.aspx
 */

#ifndef BQ34Z100_H
#define BQ34Z100_H

#include <cstdint>

/**
 * @name Definitions 
 * @{
 */

#define GAUGE_ADDRESS 0xAA

/**
 * @name Battery configuration settings
 * @note Run flashSettings() then reset the sensor to save the settings. 
 * @{
 */

#define DESIGNCAP 1400 /** mAh, per cell (Page 48, offset 11) */
#define DESIGNENERGY 5180 /** mWh, per cell (Page 48, offset 14) */
#define CELLCOUNT 0x04 /** number of series cells (Page 65, offset 7) */
#define LEDCONFIG 0x4b /** 5-LED Expander with I2C host comm (Page 64, offset 5) */
#define VOLTSEL true /** Switches to an external battery voltage divider */
#define ZEROCHARGEVOLT 3375 /** mV, charge cut-off voltage/Cell terminate voltages */
#define FLASH_UPDATE_OK_VOLT 2800 /** mV, below this voltage per cell flash writes will not go through */
#define QMAX0 1400 /** mAh, datasheet says to use c-rate current */

/**
 * @}
 */

/**
 * @brief Voltage Divider Gain
 * 
 * The voltage divider works by this formula: Gain = (TOP_LEG_R / BOTTOM_LEG_R) * 1000
 *
 * Top leg resistance (TOP LEG R): 294Kohm
 * Bottom leg resistance (BOTTOM LEG R): 16.5Kohm
 * 
 * This only works if you enable the external voltage divider (VOLTSEL) option for the sensor
 * Note: requires calibration after setting in flash
 */
#define VOLTAGEGAIN 17818

#define LOADSELECT 0x01 /** "Load Select defines the type of power or current model to be used to compute load-compensated capacity in the Impedance Track algorithm" */
#define LOADMODE 0x00 /** "Load Mode is used to select either the constant current or constant power model for the Impedance Track algorithm" */

/** 
 * @brief Reset Voltage
 * 
 * Voltage (in mV) to reset to after unsuccessful voltage calibration
 * Recommended value is 1.5x the current battery voltage 
 */ 
#define RESETVOLTAGE 22200 

/**
 * @brief Sense Resistor value 
 * 
 * mOhms, value of guage sense resistor
 */
#define SENSE_RES 15.0f 

/** 
 * @brief External Thermistor Enable
 * 
 * If 1, use an external thermistor connected to the IT pin.  
 * If 0, use the internal temp sensor. 
 */
#define USE_EXTERNAL_THERMISTOR 0 

/**
 * @}
 */

/**
 * @brief The BQ34Z100 Driver
 *  
 * Reads the current state of the main battery, can determine charge
 * remaining inside as well as read instantaneous voltage, current, or temperature. 
 */
class BQ34Z100
{
public:
        /** 
         * @brief Create an GQ34Z100-G1 object connected to the I2C object
         *
         * @param i2c Reference to the i2c bus used
         * @param hz The I2C bus frequency (400kHz acc. to datasheet).
         */
        BQ34Z100(I2C& i2c, int hz = 400000);

        /** 
         * @brief Returns the status of key features 
         * 
         * Instructs the fuel gauge to return status information to Control addresses 0x00/0x01.
         */
        uint16_t getStatus();

        /** 
         * @brief Toggle CALIBRATION mode enable
         * 
         * @note Use before enterCal or exitCal() 
         * 
         * Instructs the fuel gauge to enable entry and exit to CALIBRATION mode.
         */
        void enableCal();

        /** 
         * @brief Enters CALIBRATION mode 
         */
        void enterCal();

        /**
         * @brief Exits CALIBRATION mode 
         */
        void exitCal();

        /** 
         * @brief Enables the Impedance Track algorithm 
         * 
         * This algorithm uses voltage measurements, characteristics, and properties to create
         * state of charge (SOC) predictions.
         */
        void ITEnable();

        /**
         * @brief Gets the state of charge (SOC) of the battery.
         * 
         * @return Predicted remaining battery capacity as a percentage of total capacity.
         */
        uint8_t getSOC();

        /**
         * @brief Returns the expected margin of error in the SOC calculation.
         * 
         * @return Returns a percentage from 1% to 100%
         */
        uint16_t getError();

        /**
         * @brief Gets the remaining battery capacity. 
         * 
         * @return The battery capacity remaining. Unit is 1 mAh per bit. 
         */
        uint16_t getRemaining();

        /**
         * @brief Returns the measured battery voltage. 
         * 
         * @return Battery voltage in mV in the range 0 to 65535. 
         */
        uint16_t getVoltage();

        /**
         * @brief Gets the current flow through the sense resistor. 
         * 
         * Returns the average current flowing through the sense resistor
         * @return Returns the average current. 
         */
        int16_t getCurrent();

        /**
         * @brief Gets the internal sensor temperature in Celcius.
         * 
         * The device returns an internal temperature in units of 0.1K
         * This function grabs the temperature in K and converts to Celcius
         * 
         * @return Temperature in units of Celcius.
         */
        double getTemperature();

        /**
         * @brief Returns the current ChemID
         * 
         * Returns the ChemID configured inside the chip. 
         * 
         * @return ChemID as a hex number. e.g. programming ID 2109 would cause
         * this function to return 8457 (0x2109).
         */
        uint16_t getChemID();

        /**
         * @brief Returns a percentage ratio of prediced full charge capacity (FCC) over design capacity. 
         * 
         * Just as an old battery will not be able to charge fully to its design capacity,
         * our battery may behave similarly. 
         * 
         * This sensor will tell us how "healthy" our battery is. 
         * 
         * @return Percentage ratio from 0 to 100 of predicted FCC over design capacity. 
         */
        uint16_t getStateOfHealth();

        /**
         * @brief Returns the pack serial number programmed in the data flash
         * 
         * @return Serial number from the data flash.  
         */
        int getSerial();

        /**
         * @brief Perform a full reset.
         * 
         * Instructs the fuel gauge to perform a full reset. 
         * This command is only available when the fuel gauge is UNSEALED.
         */
        void reset();

        /**
         * @brief Unseals the device for DATA FLASH access. 
         * 
         * Unseals the device using the Control() command if the device is sealed.  
         * This is the first step in the sequence of steps required to read DATA FLASH. 
         */
        void unseal();

        /**
         * @brief Seals the device, prohibiting DATA FLASH access. 
         * 
         * Instructs the fuel gauge to transition from UNSEALED state to SEALED state.
         * This blocks access to certain flash registers. 
         */
        void seal();

        /**
         * @name Data Flash functions
         * @{
         */

        /**
         * @brief Changes to a different register in BlockData()
         * 
         * @param subclass 
         * @param offset 
         */
        void changePage(char subclass, uint16_t offset);

        /**
         * @brief Updates the checksum
         * 
         * Reads the checksum and updates it using the flashbytes array.
         * Required by the sensor to save changes to the flash.
         */
        void updateChecksum();

        /**
         * @brief Reads the blockData in the current page.
         * 
         * Saves the read data to the flashbyte array. 
         */
        void readFlash();

        /**
         * @brief Writes to DATA FLASH. 
         * 
         * Writes new data, first to flashbytes array, then updates on the sensor as well
         * 
         * @note Requires update of the checksum after changes are made
         * 
         * @param index subclass ID (index)
         * @param value offset 
         * @param len The length of the write.  
         */
        void writeFlash(uint8_t index, uint32_t value, int len);

        /**
         * @brief Returns the flashbytes array.
         * 
         * The flashbytes array stores the last updated page in flash.
         * 
         * @return Pointer to the flashbytes array.
         */
        uint8_t* getFlashBytes();

        /**
         * @brief Update the Data page in the Data Flash. 
         * 
         * This reconfigures certain data parameters of the Data subclass within the Data Flash. 
         * 
         * The parameters configured in this method are:
         * - the design capacity
         * - the design energy
         */
        void changePage48();

        /**
         * @brief Update the Registers page in the Data Flash. 
         * 
         * This reconfigures certain data parameters of the Registers subclass within the Data Flash. 
         * 
         * The parameters configured in this method are:         
         * - voltage divider enable
         * - calibration enable
         * - internal temperature sensor enable
         * - number of cells in the battery
         * - LED config mode
         */
        void changePage64();

        /**
         * @brief Update the IT Cfg page in the Data Flash. 
         * 
         * This reconfigures certain data parameters of the IT Cfg subclass within the Data Flash. 
         * 
         * The parameters configured in this method are:
         * - LOADSELECT
         * - LOADMODE
         * - ZEROCHARGEVOLT (cell terminate voltage)
         */
        void changePage80();

        /**
         * @brief Update the State page in the Data Flash. 
         * 
         * This reconfigures certain data parameters of the State subclass within the Data Flash. 
         * 
         * The data parameters configured in this method are:
         * - QMax Cell 0
         */
        void changePage82();

        /**
         * @brief Calibrates the Voltage Divider register.
         * 
         * @note Voltage divider changes seem to require a chip reset to take effect, 
         * so we must reset the chip between each calibration. 
         * 
         * @param currentVoltage The actual battery voltage in mV 
         * @return The voltage divider ratio written 
         */
        uint16_t calibrateVoltage(uint16_t currentVoltage);

        /**
         * @brief Reset the Voltage Divider object.
         *  
         * If too much calibration is done, you may want to reset the Voltage Divider. 
         * This function sets the voltage divider register to RESETVOLTAGE. 
         */
        void resetVoltageDivider();

        /**
         * @brief Calibrate current shunt. 
         *
         * Calibrate CCGain and CCOffset registers, which control current shunt
         * and therefore affect the current and voltage readouts. 
         * 
         * @param calCurrent Calibration current input. 
         */
        void calibrateShunt(int16_t calCurrent);

        /**
         * @brief Set the Sense Resistor
         * 
         * Calibrate shunt works to set the sense resistor as well, but
         * requires an external current measurement.
         */
        void setSenseResistor();

        /**
         * @brief Read the device type
         * 
         * Device type should be 0x100 for the BQ34Z100.
         * 
         * @return Device type
         */
        uint16_t readDeviceType();

        /**
         * @brief Read the firmware version.
         * 
         * @return Contents of the FW version register.  
         */
        uint16_t readFWVersion();

        /**
         * @brief Read the hardware version.
         * 
         * @return Contents of the HW version register. 
         */
        uint16_t readHWVersion();

        /**
         * @brief Convert float value to a Xemics floating point. 
         *
         * Documented in this app note: http://www.ti.com/lit/pdf/slva148 
         * 
         * @param value Float value
         * @return Xemics floating point value
         */
		static uint32_t floatToXemics(float value);

        /**
         * @brief Convert Xemics floating point to a float value.
         * 
         * @param xemics Xemics floating point value
         * @return Float value
         */
		static float xemicsToFloat(uint32_t xemics);

        /**
         * @brief Read the Update Status register from flash.
         * 
         * See datasheet section 7.3.6.10
         * 
         * @return Status of the Impedance Track algorithm and learning process.  
         */
		uint8_t getUpdateStatus();

		/**
		 * @brief Get the Gas Gauge Status Flags. 
         * 
         * Uses the Flags and FlagsB commands to get the contents of the Gas Gauge Status Register.
         * 
		 * @return Contents of the Gas Gauge Status register, depicting current operation status. 
		 */
		std::pair<uint16_t, uint16_t> getFlags();

        /**
         * @}
         */
private:
    /** 
     * @brief Top-level commands 
     * 
     * The first byte that you can send on an I2C transaction 
     */
    enum class Command : uint8_t
    {
        Control = 0x0,
        StateOfCharge = 0x2,
        MaxError = 0x3,
        RemainingCapacity = 0x4,
        FullChargeCapacity = 0x6,
        Voltage = 0x8,
        AverageCurrent = 0xA,
        Temperature = 0xC,
        Flags = 0xE,
        Current = 0x10,
        FlagsB = 0x12,
        SerialNumber = 0x28,
        InternalTemperature = 0x2A,
        CycleCount = 0x2C,
        StateOfHealth = 0x2E,
        DataFlashClass = 0x3E,
        DataFlashBlock = 0x3F,
        BlockData = 0x40,
        BlockDataCheckSum = 0x60,
        BlockDataControl = 0x61
    };

    /**
     * @brief Subcommands for the "Control" Command
     */
    enum class Control : uint16_t
    {
        CONTROL_STATUS = 0x0,
        DEVICE_TYPE = 0x1,
        FW_VERSION = 0x02,
        HW_VERSION = 0x03,
        RESET_DATA = 0x5,
        PREV_MACWRITE = 0x7,
        CHEM_ID = 0x8,
        BOARD_OFFSET = 0x9,
        CC_OFFSET = 0xA,
        CC_OFFSET_SAVE = 0xB,
        DF_VERSION = 0xC,
        SET_FULLSLEEP = 0x10,
        STATIC_CHEM_CHECKSUM = 0x17,
        SEALED = 0x20,
        IT_ENABLE = 0x21,
        CAL_ENABLE = 0x2D,
        RESET = 0x41,
        EXIT_CAL = 0x80,
        ENTER_CAL = 0x81,
        OFFSET_CAL = 0x82,
        UNSEAL_KEY1 = 0x0414,
        UNSEAL_KEY2 = 0x3672
    };

    /** @brief Reference to I2C interface */
    I2C& _i2c;

    /** @brief Current flash page that we have read */
    uint8_t currFlashPage = 0; 
    
    /** @brief 32 bit block index into current flash page */
    uint8_t currFlashBlockIndex = 0; 

    /** @brief Stores page in flash memory on Hamster */
    uint8_t flashbytes[32]; 

    /**
     * @name I2C Commands 
     * @{
     */

    /** 
     * @brief Sends a control command
     * 
     * Issuing a Control() command requires a subsequent two-byte subcommand. 
     * These additional bytes specify the particular control function desired.
     * 
     * This function writes two bytes with the LSB first. 
     * e.g. sending Control(0x0414) translates into WRITE 0x00 0x14 0x04, Command1 = 0x04, and Command2 = 0x14. 
     * 
     * @param control The control command to be sent.
     */
    void sendControlCommand(Control control);

    /** 
     * @brief Sends a control command and reads the result.
     * 
     * Sends a Control() command then reads two bytes from the Control register (0x00)
     * 
     * @param control The control command to be sent.
     * 
     * @return the result of the control command
     */
    uint16_t readControlCommand(Control control);

    /**
     * @brief Writes one byte over I2C to the device.
     * 
     * Used to send commands.
     * 
     * @param command command to send
     * @param cmd byte of command
     */
    void write(Command command, const uint8_t cmd);  

    /**
     * @brief Writes two bytes over I2C to the device.
     * 
     * Used to send two-byte subcommands.
     * e.g. sending the start subcommand WRITE 01 00 to Register 00 requires write(0x00, 0x01, 0x00) 
     * 
     * @param command command to send 
     * @param cmd1 first byte of command 
     * @param cmd2 second byte of command
     */
    void write(Command command, const uint8_t cmd1, const uint8_t cmd2); 

    /**
     * @brief Reads over I2C from the device.
     * 
     * @param command 
     * @param length number of bytes to read 
     * @return uint32_t 
     */
    uint32_t read(Command command, const uint8_t length);

    /**
     * @}
     */

    /** @brief Calculate the checksum of the current flashbytes array */
    uint8_t calcChecksum();
};
#endif
