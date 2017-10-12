//
// mux_ini_loader_SD.ino
//
// This code writes eeprom images to the mux eeprom
//
// page 0:	information about the assembly
//		0x0000 - 0x000F:	assembly name; 16 characters null filled; known assemblies are:
//			"TMP275"
//			"MUX7"			'MUX7' to distinguish from a mux that does not have on-board sensors at port 7
//		0x0010 - 0x0011:	assembly revision; two byte little endian; MM.mm in the form 0xMMmm
//		0x0010: mm
//		0x0011: MM
//		0x0012 - 0x0015:	assembly manufacture date; four byte time_t little endian
//		0x0016 - 0x0019:	assembly service date; four byte time_t little endian; not part of ini file; how set? when set?
//		0x001A		   :	number of ports installed (MUX only)
//		0x001B - 0x001F:	undefined; 5 bytes (MUX only)
//		0x001A - 0x001F:	undefined; 6 bytes (TMP275 only)
//
// page 1:	information about a mounted sensor
//		0x0020 - 0x002F:	sensor name; 16 characters null filled; known sensors are:
//			"TMP275"
//			"HDC1080"
//			"MS8607PT"		- the pressure and temperature sensors (i2c address 0x76)
//			"MS8607H"		- the relative humidity sensor (i2c address 0x40)
//		0x0030:				[not supported on MUX7 because mux-mounted sensors at port 7 are always at fixed addresses - reserved for future use]
//							sensor i2c address; one byte:
//								bits 6..0: the i2c address
//								bit 7:
//									when 0, the value in bits 6..0 is a base address modified by the assembly's address jumpers
//									when 1, the value in bits 6..0 is a fixed, non-modifiable address
//		0x0031 - 0x003F:	undefined; 15 bytes
//		
// page 2:	information about a mounted sensor (same format as page 1; repeat as often as necessary within reason)
//
//

//---------------------------< I N C L U D E S >--------------------------------------------------------------


#include	<SALT_power_FRU.h>
#include	<SALT_ext_sensors.h>
#include	<SALT_FETs.h>
#include	<SALT_settings.h>
#include	<SALT_utilities.h>

#include	<Systronix_M24C32.h>		// eeprom
#include	<Systronix_PCA9548A.h>
#include	<Systronix_TMP102.h>		// interface library for TMP102 temperature sensing

#include	<SdFat.h>

//---------------------------< D E C L A R A T I O N S >------------------------------------------------------

SALT_ext_sensors ext_sensors;
SALT_FETs FETs;
SALT_logging logs;
SALT_power_FRU fru;
SALT_settings settings;
SALT_utilities utils;

Systronix_M24C32 eep;
Systronix_PCA9548A mux;

SdFat SD;
SdFile file;

elapsedMillis waiting;


//---------------------------< D E F I N E S >----------------------------------------------------------------

#define ESS_INTERVAL 1	// interval of how many seconds to dispay elapsed time on usb serial monitor

#define PCB_MAJOR_REV	2
#define PCB_MINOR_REV	00

#define		STOP	0
#define		START	1
#define		TIMING	0xFFFFFFFF

#define		ASSEMBLY	0xA0
#define		SENSOR1		0xA1
#define		SENSOR2		0xA2
#define		SENSOR3		0xA3

#define		PAGE_SIZE	32

#define		ASSY_PAGE_ADDR		0
#define		SENSOR1_PAGE_ADDR	ASSY_PAGE_ADDR+PAGE_SIZE
#define		SENSOR2_PAGE_ADDR	SENSOR1_PAGE_ADDR+PAGE_SIZE
#define		SENSOR3_PAGE_ADDR	SENSOR2_PAGE_ADDR+PAGE_SIZE


//---------------------------< P R O T O T Y P E S >----------------------------------------------------------


//-----------------------------< P A G E   S C O P E   V A R I A B L E S >------------------------------------

uint16_t	total_errs = 0;
uint16_t	warn_cnt = 0;

char		file_list [11][64];					// a 1 indexed array of file names; file_list[0] not used
char		rx_buf [8192];
char		out_buf [8192];
char		ln_buf [256];

union
	{
	struct mux_settings
		{
		char		assembly_type[16];
		uint16_t	assembly_revision;
		time_t		manufacture_date;
		time_t		service_date;		// not part of ini file; how set? when set?
		uint8_t		ports;
		uint8_t		unused[5];			// so that the struct totals 32 bytes
		} as_struct;
	uint8_t			as_array[32];
	} assy_page;

union
	{
	struct sensor_settings
		{
		char		sensor_type[16];
		uint8_t		sensor_addr;
		uint8_t		unused[15];			// so that the struct totals 32 bytes
		} as_struct;
	uint8_t			as_array[32];
	} sensor1_page, sensor2_page, sensor3_page;


//---------------------------< S T O P W A T C H >------------------------------------------------------------
//
// Start and stop an elapsed timer.
//
// returns start_time (current millis() value) in response to START;
// return elapsed_time in response to STOP
//

time_t stopwatch (boolean start_stop)
	{
	static uint32_t		state = STOP;
	static time_t		start_time = 0;
	static time_t		end_time = 0;
	static time_t		elapsed_time = TIMING;		// invalidate until started and stopped
	
	switch (state)
		{
		case STOP:
			if (start_stop)							// start command
				{
				start_time = millis();				// set start time
				elapsed_time = TIMING;				// invalidate until stopped
				state = TIMING;						// bump the state
				break;								// done
				}
			return TIMING;							// command to stop while stopped
		
		case TIMING:
			if (!start_stop)						// stop command
				{
				end_time = millis();				// set the end time
				elapsed_time = end_time - start_time;	// calculate
				state = STOP;						// bump the state
				return elapsed_time;				// done
				}
			start_time = millis();					// start command while timing is a restart
		}
	return start_time;						// return new start time
	}


//---------------------------< F I L E _ G E T _ C H A R S >--------------------------------------------------
//
// Fetches characters from a selected file in uSD and writes them into rx_buf.  CRLF ('\r\n') is converted to LF
// ('\n').  Returns the number of characters received.  If the function receives enough characters to fill the
// allotted space in fram then something is not right; null terminates the buffer and returns the number of
// characters received.
//

uint16_t file_get_chars (char* rx_ptr)
	{
	uint16_t	char_cnt = 0;
	char		c;							// temp holding variable
	
	while (file.available())				// while stuff to get
		{
		c = file.read();					// read and save the byte
		if (EOL_MARKER == c)
			{
			char_cnt++;						// tally
			*rx_ptr++ = c;					// save and bump the pointer
			*rx_ptr = EOF_MARKER;			// add end-of-file whether we need to or not
			*(rx_ptr+1) = '\0';				// null terminate whether we need to or not
			
			if (0 == (char_cnt % 2))
				Serial.print (".");			// show that we've received a line
			continue;
			}
		else if ('\r' == c)					// carriage return
			{
			char_cnt ++;					// tally
			continue;						// but don't save; we only want the newline as a line marker
			}
		else
			{
			char_cnt++;						// tally
			*rx_ptr++ = c;					// save and bump the pointer
			*rx_ptr = EOF_MARKER;			// add end-of-file whether we need to or not
			*(rx_ptr+1) = '\0';				// null terminate whether we need to or not
			}

		if (8191 <= char_cnt)				// rcvd too many chars? not a SALT ini file?
			{
			*rx_ptr = '\0';					// null terminate rx_buf
			return char_cnt;
			}
		}
	return char_cnt;
	}


//---------------------------< N O R M A L I Z E _ K V _ P A I R >--------------------------------------------
//
// normalize key value pairs by: removing excess whitespace; converting whitespace in key name to underscore;
// key name to lowercase.
//

uint8_t normalize_kv_pair (char* key_ptr)
	{
	char*	value_ptr;						// pointer to the key's value
	char*	i_ptr;							// an intermediate pointer used to reassemble the k/v pair

	value_ptr = strchr (key_ptr, '=');		// find the assignment operator; assign its address to value_ptr
	if (NULL == value_ptr)
		{
		key_ptr = settings.trim (key_ptr);
		if (settings.is_section_header (key_ptr))
			{
			if (strstr (key_ptr, "[assembly]"))
				return ASSEMBLY;
			else if (strstr (key_ptr, "[sensor 1]"))
				return SENSOR1;
			else if (strstr (key_ptr, "[sensor 2]"))
				return SENSOR2;
			else if (strstr (key_ptr, "[sensor 3]"))
				return SENSOR3;
			}
		return INI_ERROR;					// missing assignment operator or just junk text
		}
		
	*value_ptr++ = '\0';					// null terminate key at the assignment operator and point to the value portion of the original string
	key_ptr = settings.trim (key_ptr);		// trimming key_ptr returns a pointer to the first non-whitespace character of the 'key'
	settings.strip_white_space (key_ptr);	// remove all whitespace from key: 'lights on' becomes 'lights_on'
	settings.str_to_lower (key_ptr);		// make lowercase for future comparisons
	value_ptr = settings.trim (value_ptr);	// adjust value_ptr to point at the first non-whitespace character
	
	i_ptr = key_ptr + strlen (key_ptr);		// point to the key's null terminator
	*i_ptr++ = '=';							// insert assignment operator and bump to point at location for value
	if (i_ptr != value_ptr)
		strcpy (i_ptr, value_ptr);			// move value adjacent to assignment operator

	return SUCCESS;
	}


//---------------------------<  A R R A Y _ G E T _ L I N E >-------------------------------------------------
//
// copies settings line by line into a separate buffer for error checking, formatting before the setting is
// written to fram.
//

char* array_get_line (char* dest, char* array)
	{
	while ((EOF_MARKER != *array) && ('\r' != *array) && (EOL_MARKER != *array))
		*dest++ = *array++;						// copy array to destination until we find \n or \0

	if (('\r' == *array) || (EOL_MARKER == *array))	// if we stopped copying because \r or \n
		{
		*dest++ = *array++;						// add it to the destination
		*dest = '\0';							// null terminate for safety
		return array;							// else return a pointer to the start of the next line
		}
	else										// must be end-of-file marker
		{
		*dest = '\0';							// null terminate for safety
		return NULL;							// return a null pointer
		}
	}


//---------------------------<  A R R A Y _ A D D _ L I N E >-------------------------------------------------
//
// copies a line of text into a buffer and adds an end-of-file marker; returns a pointer to the eof marker.
// source is a null-terminated string without eof terminator.
//

char* array_add_line (char* array_ptr, char* source_ptr)
	{
	while (*source_ptr)
		*array_ptr++ = *source_ptr++;			// copy source into array

	*array_ptr++ = EOL_MARKER;						// add end-of-line marker
	*array_ptr = EOF_MARKER;						// add end-of-file marker
	return array_ptr;							// return a pointer to the end-of-line character
	}


//---------------------------< I S O _ D A T E _ G E T >------------------------------------------------------
//
// attempt to convert iso8601 date string to time_t value
//

extern time_t makeTime(tmElements_t &tm);

uint8_t iso_date_get (char *value_ptr, time_t* manuf_date_ptr)
	{
	tmElements_t tm;
	uint32_t	year=0;
	uint32_t	month=1;
	uint32_t	day=1;
	char*		end_ptr;

	year = strtol (value_ptr, &end_ptr, 10);
	if ('-' == *end_ptr)
		end_ptr++;
	else
		return FAIL;

	month = strtol (end_ptr, &end_ptr, 10);
	if ('-' == *end_ptr)
		end_ptr++;
	else
		return FAIL;

	day = strtol (end_ptr, &end_ptr, 10);
	if ('\0' != *end_ptr)
		return FAIL;

	if (!utils.is_valid_date ((uint16_t)year, (uint8_t)month, (uint8_t)day))
		return FAIL;

	tm.Year = (uint8_t)(year - 1970);
	tm.Month = (uint8_t)month;
	tm.Day = (uint8_t)day;
	tm.Hour = 0;
	tm.Minute = 0;
	tm.Second = 0;

	*manuf_date_ptr = makeTime (tm);
	return SUCCESS;
	}


char* valid_type_str [] = {(char*)"MUX7", (char*)"TMP275", (char*)"HDC1080", (char*)"MS8607PT", (char*)"MS8607H"};	// for type keyword
char* valid_yes_no_str [] = {(char*)"YES", (char*)"NO"};	// for absolute keyword

//---------------------------< C H E C K _ I N I _ A S S E M B L Y >------------------------------------------
//
//

void check_ini_assembly (char* key_ptr)
	{
	uint16_t	temp16;						// temp variable for holding uint16_t sized variables
	char		temp_array[32];

	char*	value_ptr;						// pointers to the key and value items

	value_ptr = strchr (key_ptr, '=');		// find the assignment operator; assign its address to value_ptr
	if (NULL == value_ptr)
		{
		settings.err_msg ((char *)"not key/value pair");		// should never get here
		total_errs++;						// make sure that we don't write to fram
		return;
		}

	*value_ptr++ = '\0';					// null terminate the key and bump the pointer to point at the value

	settings.err_cnt = 0;					// reset the counter
	strcpy (temp_array, value_ptr);			// a copy to use for evaluation

	if (16 < strlen (value_ptr))
		settings.err_msg ((char *)"value string too long");

	else if (!strcmp (key_ptr, "type"))
		{
		settings.str_to_upper (value_ptr);
		if (!strcmp (value_ptr, "MUX7"))
			{
			memset (assy_page.as_struct.assembly_type, '\0', 16);		// zero-fill first
			strcpy (assy_page.as_struct.assembly_type, value_ptr);
			}
		else
			settings.err_msg ((char *)"invalid assembly type; expected MUX7");

//		for (uint8_t i=0; i<3; i++)					// save this in case we combine MUX ini programming with SENSOR programming
//			{
//			if (!strcmp (value_ptr, valid_type_str[i]))
//				{
//				memset (assy_page.as_struct.assembly_type, '\0', 16);		// zero-fill first
//				strcpy (assy_page.as_struct.assembly_type, value_ptr);
//				break;
//				}
//			if (2 <= i)
//				settings.err_msg ((char *)"unknown type");
//			}
		}

	else if (!strcmp (key_ptr, "revision"))
		{
		if (*value_ptr)
			{
			temp16 = settings.revision_check (value_ptr);

			if (FAIL == temp16)
				settings.err_msg ((char *)"invalid revision");
			else
				assy_page.as_struct.assembly_revision = temp16;
			}
		}
	else if (!strcmp (key_ptr, "manuf_date"))
		{
		if (!(*value_ptr) || iso_date_get (value_ptr, &assy_page.as_struct.manufacture_date))
			settings.err_msg ((char *)"invalid manuf date");
		}
	else if (!strcmp (key_ptr, "ports"))
		{
		if (!(*value_ptr) || (1 < strlen (value_ptr)) || ('0' == *value_ptr) ||  ('9' == *value_ptr))
			settings.err_msg ((char *)"invalid port value");
		else
			assy_page.as_struct.ports = *value_ptr -'0';	// convert ascii to number
		}
	else
		settings.err_msg ((char *)"unrecognized setting");

	*(value_ptr-1) = '=';					// restore the '='
	total_errs += settings.err_cnt;					// accumulate for reporting later
	}


//---------------------------< C H E C K _ I N I _ S E N S O R  >---------------------------------------------
//
//
//

void check_ini_sensor (char* key_ptr, char index)
	{
	char*	value_ptr;						// pointers to the key and value items
	char	match_key[16];

	value_ptr = strchr (key_ptr, '=');		// find the assignment operator; assign its address to value_ptr
	if (NULL == value_ptr)
		{
		settings.err_msg ((char *)"not key/value pair");		// should never get here
		total_errs++;						// make sure that we don't write to fram
		return;
		}

	*value_ptr++ = '\0';					// null terminate the key and bump the pointer to point at the value

	settings.err_cnt = 0;					// reset the counter


	if ('\0' == *value_ptr)
		settings.err_msg ((char *)"empty setting");
		
	else if (16 < strlen (value_ptr))
		settings.err_msg ((char *)"value string too long");

	else if (strstr (key_ptr, "type"))
		{
		sprintf (match_key, "%s%c", "type", index);
		if (strcmp (match_key, key_ptr))
			settings.err_msg ((char *)"invalid type key index");
		else
			{
			settings.str_to_upper (value_ptr);
			
			for (uint8_t i=0; i<5; i++)
				{
				if (!strcmp (value_ptr, valid_type_str[i]))
					{
					if ('1' == index)		// TODO: there must be a better way to do this
						{
						memset (sensor1_page.as_struct.sensor_type, '\0', 16);		// zero-fill first
						strcpy (sensor1_page.as_struct.sensor_type, value_ptr);		// then copy type to it
						}
					else if ('2' == index)
						{
						memset (sensor2_page.as_struct.sensor_type, '\0', 16);		// zero-fill first
						strcpy (sensor2_page.as_struct.sensor_type, value_ptr);		// then copy type to it
						}
					else	// if not 1 or 2, must be 2
						{
						memset (sensor2_page.as_struct.sensor_type, '\0', 16);		// zero-fill first
						strcpy (sensor2_page.as_struct.sensor_type, value_ptr);		// then copy type to it
						}
					break;
					}
				if (4 <= i)
					settings.err_msg ((char *)"unknown type");
				}
			}
		}

	else if (strstr (key_ptr, "address"))
		{
		sprintf (match_key, "%s%c", (char*)"address", index);
		if (strcmp (match_key, key_ptr))
			settings.err_msg ((char *)"invalid address key index");
		else
			{
			if (isdigit (value_ptr[0]) && isxdigit (value_ptr[1]) && ('\0' == value_ptr[2]))	// value_ptr[0] must not be hex digit A-F
				{
				if ('1' == index)		// TODO: there must be a better way to do this
					{
					sensor1_page.as_struct.sensor_addr &= 0x80;									// clear but preserve fixed bit if set
					sensor1_page.as_struct.sensor_addr |= ((uint8_t)strtol (value_ptr, NULL, 16)) & 0x7F;
					}
				else if ('2' == index)
					{
					sensor2_page.as_struct.sensor_addr &= 0x80;									// clear but preserve fixed bit if set
					sensor2_page.as_struct.sensor_addr |= ((uint8_t)strtol (value_ptr, NULL, 16)) & 0x7F;
					}
				else					// if not 1 or 2, must be 3
					{					// for MUX7, sensor3 used only when MS8607 installed
					sensor3_page.as_struct.sensor_addr &= 0x80;									// clear but preserve fixed bit if set
					sensor3_page.as_struct.sensor_addr |= ((uint8_t)strtol (value_ptr, NULL, 16)) & 0x7F;
					}
				}
			else
				settings.err_msg ((char *)"invalid sensor address");
			}
		}

	else if (strstr (key_ptr, "fixed"))
		{
		sprintf (match_key, "%s%c", (char*)"fixed", index);
		if (strcmp (match_key, key_ptr))
			settings.err_msg ((char *)"invalid fixed key index");
		else
			{
			if (!strcmp (assy_page.as_struct.assembly_type, (char*)"MUX7"))		// MUX7 on-board sensors are always at fixed addresses
				{
				Serial.printf ("\twarning: %s ignored\n", key_ptr);				// MUX7 on-board sensors always at fixed addresses
				warn_cnt++;
				}
			else
				{
				settings.str_to_upper (value_ptr);
				if (!strcmp (value_ptr, "NO"))
					{
					if ('1' == index)		// TODO: there must be a better way to do this
						sensor1_page.as_struct.sensor_addr |= 0x80;		// not an absolute address so set MSB
					else if ('2' == index)
						sensor2_page.as_struct.sensor_addr |= 0x80;		// not an absolute address so set MSB
					else					// if not 1 or 2, must be 3; for MUX7, sensor3 used only when MS8607 installed
						sensor3_page.as_struct.sensor_addr |= 0x80;		// not an absolute address so set MSB
					}
				else if (strcmp (value_ptr, "YES"))					// does not equal "YES"
					settings.err_msg ((char *)"invalid fixed setting");
				}
			}
		}
	else
		settings.err_msg ((char *)"unrecognized setting");

	*(value_ptr-1) = '=';					// restore the '='
	total_errs += settings.err_cnt;					// accumulate for reporting later
	}


//---------------------------< S E T U P >--------------------------------------------------------------------

void setup(void)
	{
	}


//---------------------------< L O O P >----------------------------------------------------------------------

void loop (void)
	{
	uint8_t		ret_val = 0;							// misc parameter used to hold whatever is returned

	uint8_t		file_count;								// indexer into file_list; a 1-indexed array; file_list[0] not used
	uint16_t	rcvd_count;
	time_t		elapsed_time;
	char*		rx_ptr = rx_buf;						// point to start of rx_buf
	char*		out_ptr = out_buf;						// point to start of out_buf
	uint8_t		c = 0;
	uint8_t		heading = 0;
	const char path [] = {"mux_ini_files"};

	settings.sys_settings.tz = MST;						// for logging if we ever get that far

//---------- arduino libraries setup
	waiting = 0;
	Serial.begin(115200);								// use max baud rate
	while((!Serial) && (waiting<5000));					// wait until serial port is open or timeout

	Serial1.begin(9600);								// UI habitat A LCD and keypad
	while((!Serial1) && (waiting<5000));				// wait until serial port is open or timeout

	waiting = 0;
	Serial2.begin(9600);								// UI habitat B LCD and keypad
	while((!Serial2) && (waiting<5000));				// wait until serial port is open or timeout

	Serial.printf ("NAP utility: mux ini loader SD\nBuild %s %s\r\n", __TIME__, __DATE__);

	Serial2.setRX (26);									// set alternate pins: SALT uses primary RX2 pin ...
	Serial2.setTX (31);									// ... for ETH_RST(L) and primary TX2 pin for ETH_CS(L)

	Serial1.printf ("r\r");								// 'r' initialize display so we can have a sign-on message
	Serial2.printf ("r\r");
	delay(50);

	//                0123456789ABCDEF0123456789ABCDEF
	Serial1.printf ("dmux ini         loader SD\r");	// in case this code ever ends up in a habitat
	Serial2.printf ("dmux ini         loader SD\r");

	pinMode(PERIPH_RST, OUTPUT);						// NOTE: SALT 2.0 resetting the FETs 9557 (U5) allows the gates of the FETs to float; U5 should be immediately initialized following reset
	pinMode (ETHER_RST, OUTPUT);
	pinMode (uSD_DETECT, INPUT_PULLUP);					// so we know if a uSD is in the socket

	utils.fw_reset (RST_E | RST_P);						// assert resets

	utils.spi_port_pins_test ();						// test mobility of spi port pins; leave CS pins as INPUT_PULLUP; results in utils.spi_port_pins_result

	utils.fw_reset (0);									// release resets

	if (utils.spi_port_pins_result)						// if there is a SPI pin stuck high or low can we still use the debug display?
		{
		if ((TEST_MISO | TEST_MOSI | TEST_SCK) & utils.spi_port_pins_result)
			{											// no SPI - one or more of MISO, MOSI, SCK stuck high or low
			Serial.printf ("one or more of MISO, MOSI, SCK stuck high or low\n");
			while (1);
			}
		else if ((uint8_t)(utils.spi_port_pins_result >> 8) & ((uint8_t)(~utils.spi_port_pins_result) & 0xF8))
			{											// one or more spi chip selects stuck low
			Serial.printf ("one or more of spi CS stuck low\n");
			while (1);
			}
		else
			{											// one or more spi chip selects stuck high
			if (TEST_DISP_CS & utils.spi_port_pins_result)	// is it display CS?
				{
				Serial.printf ("display CS stuck high\n");
				while (1);
				}
			}
		}

//============================================================================================================
	
	pinMode (uSD_DETECT, INPUT_PULLUP);			// so we know if a uSD is in the socket

	if (digitalRead (uSD_DETECT))
		{
		Serial.printf ("insert uSD\r\n");
		while (digitalRead (uSD_DETECT));			// hang here while uSD not installed
		delay (50);									// installed, now wait a bit for things to settle
		}
	
	Serial.printf ("uSD installed\r\n");
	
	if (!SD.begin(uSD_CS_PIN, SPI_HALF_SPEED))
		{
		Serial.printf ("\r\nerror initializing uSD; cs: %d; half-speed mode\r\n", uSD_CS_PIN);
		SD.initErrorHalt("loader stopped; reset to restart\r\n");
		while (1);
		}

	if (!SD.chdir (path, false))						// attempt to change to path folder;
		{
		Serial.printf ("%s directory not found; loader stopped; reset to restart\r\n", path);
		while (1);
		}

	Serial.printf ("monitor must send newline\nchoose ini file to load:\r\n");

	file_count = 0;	
	SD.vwd()->rewind();									// rewind to start of virtual working directory
	while (file.openNext(SD.vwd(), O_READ))				// open next file for reading
		{
		if (!file.isFile())								// files only
			{
			file.close ();								// close this directory
			continue;									// from the top
			}
			
		file_count++;									// bump to next file list location
		file.getName (file_list[file_count], 32);		// get and save the file name
		ret_val = strlen (file_list[file_count]) - 4;	// make an index to the last four characters of the file name
		if (strcasecmp(&file_list[file_count][ret_val], ".ini"))	// if last four characters are not ".ini"
			{
			file_count--;								// not a .ini file so recover this spot in the list
			file.close();								// close the file
			continue;									// from the top
			}
		
//		Serial.printf ("\n%d ", file.fileSize());		// is this of any value?  Get it but don't list a file with length 0?
		Serial.printf (" [%2d] %-32s ", file_count, file_list[file_count]);		// print menu item, file name, file date
		file.printModifyDateTime(&Serial);				// and time/date  TODO: must we use this?  is there no better way to get file date and time?
		Serial.printf ("\r\n");							// terminate the menu
		file.close();									// 

		if (99 <= file_count)							// only list 99 files
			break;
		}

	if (0 == file_count)
		{
		Serial.printf ("\r\nno .ini files found on uSD card.\r\nloader stopped; reset to restart");
		while (1);										// hang it up
		}
		
	while (1)
		{
		while (Serial.available())
			ret_val = Serial.read();					// if anything in the Serial input, get rid of it

		Serial.printf ("SALT/loader> ");				// print the SALT prompt and accept serial input

		ret_val = 0;
		while (1)
			{
			if (Serial.available())						// wait for input
				{
				c = Serial.read();						// get whatever is there

				if (!isdigit(c))
					break;

				c -= '0';								// make binary
				if (!c && !ret_val)
					continue;							// ignore leading zeros

				ret_val *= 10;							// make room for new digit
				ret_val += c;							// add the new digit
				}
			}

		if ('\n' == c)									// newline ends entry
			{
			if (ret_val && (ret_val <= file_count))		// entry must be within range of 1 to file_count
				{
				Serial.printf ("%d\r\n", ret_val);		// valid entry, terminate the prompt
				if (file.open (file_list[ret_val]))
					{
					Serial.printf ("\r\nreading %s\r\n", file_list[ret_val]);
					ret_val = 0xFF;
					}
				else
					Serial.printf ("\r\nfile %s did not open\r\n", file_list[ret_val]);
				break;
				}
			else
				Serial.printf ("invalid choice: %d\n", ret_val);
			}
		else
			Serial.printf ("illegal character: %c\n", c);

		ret_val = 0;									// non-digit character that is not a new line; restart
		}

	Serial.printf ("reading\n");

	stopwatch (START);
	rcvd_count = file_get_chars (rx_buf);
	file.close();

	elapsed_time = stopwatch (STOP);					// capture the time
	Serial.printf ("read %d characters in %ldms\n", rcvd_count, (uint32_t)elapsed_time);


	settings.line_num = 0;								// reset to count lines taken from rx_buf
	Serial.printf ("\r\nchecking\r\n");
	stopwatch (START);									// reset

	rx_ptr = rx_buf;									// initialize
	while (rx_ptr)
		{
		rx_ptr = array_get_line (ln_buf, rx_ptr);		// returns null pointer when no more characters in buffer
		if (!rx_ptr)
			break;
	
		if (EOF_MARKER == *ln_buf)						// when we find the end-of-file-marker
			{
			*out_ptr = *ln_buf;							// add it to out_buf
			break;										// done reading rx_buf
			}

		settings.line_num ++;							// tally

		if (('\r' == *ln_buf) || (EOL_MARKER == *ln_buf))	// do these here so we have source line numbers for error messages
			continue;									// cr or lf; we don't save newlines in fram

		strcpy (ln_buf, settings.trim (ln_buf));		// trim leading white space

		if ('#' == *ln_buf)								// first non-whitespace character is '#'?
			continue;									// comment; we don't save comments in fram

		if (strstr (ln_buf, "#"))						// find '#' anywhere in the line
			{
			settings.err_msg ((char *)"misplaced comment");		// misplaced; comment must be on separate lines
			total_errs++;								// prevent writing to fram
			continue;									// back to get next line
			}

		ret_val = normalize_kv_pair (ln_buf);			// if kv pair: trim, spaces to underscores; if heading: returns heading define; else returns error

		if (ret_val)									// if an error or a heading (otherwise returns SUCCESS)
			{
			if (INI_ERROR == ret_val)					// not a heading, missing assignment operator
				settings.err_msg ((char *)"not key/value pair");
			else										// found a new heading
				{
				heading = ret_val;						// so remember which heading we found
				out_ptr = array_add_line (out_ptr, ln_buf);		// copy to out_buf; reset pointer to eof marker
				}
			continue;									// get the next line
			}

		if (ASSEMBLY == heading)						// validate the various settings according to their headings
			{
			Serial.printf ("%d: %s\n", settings.line_num, ln_buf);
			check_ini_assembly (ln_buf);
			}
		else if (SENSOR1 == heading)
			{
			Serial.printf ("%d: %s\n", settings.line_num, ln_buf);
			check_ini_sensor (ln_buf, '1');
			}
		else if (SENSOR2 == heading)
			{
			Serial.printf ("%d: %s\n", settings.line_num, ln_buf);
			check_ini_sensor (ln_buf, '2');
			}
		else if (SENSOR3 == heading)
			{
			Serial.printf ("%d: %s\n", settings.line_num, ln_buf);
			check_ini_sensor (ln_buf, '3');
			}
		}
	
	if (!(*assy_page.as_struct.assembly_type))
		{
		Serial.printf ("error: missing [assembly] definition\n");
		total_errs++;
		}

	if ((*sensor2_page.as_struct.sensor_type) && (!(*sensor1_page.as_struct.sensor_type)))
		{
		Serial.printf ("error: missing [sensor 1] definition\n");
		total_errs++;
		}

	if ((*sensor3_page.as_struct.sensor_type) && (!(*sensor2_page.as_struct.sensor_type)))
		{
		Serial.printf ("error: missing [sensor 2] definition\n");
		total_errs++;
		}

	if (total_errs)										// if there have been any errors
		{
		Serial.printf ("\n###############################################\n");
		Serial.printf ("##\n");
		Serial.printf ("##\t%d error(s); %d warning(s);\n", total_errs, warn_cnt);
		Serial.printf ("##\tconfiguration not written.\r\n");
		Serial.printf ("##\tloader stopped; reset to restart\n");
		Serial.printf ("##\n");
		Serial.printf ("###############################################\n");
		while (1);
		}
	else
		{
		Serial.printf ("0 error(s); %d warning(s).\r\n", warn_cnt);
		
		mux.setup (0x70, Wire1, (char*)"Wire1");					// initialize this instance
		mux.begin (I2C_PINS_29_30, I2C_RATE_100);
		mux.init ();

		if (SUCCESS != mux.control_write (PCA9548A_PORT_7_ENABLE))						// enable access to port[7]
			{
			Serial.printf ("mux.control_write (mux.port[7]) fail\n");
			Serial.printf ("check address jumpers\n");
			Serial.printf ("\r\nloader stopped; reset to restart\r\n");		// give up and enter an endless
			while (1);
			}

		else if (SUCCESS == eep.setup (MUX_EEP_ADDR, Wire1, (char*)"Wire1"))		// initialize eeprom instance
			{
			eep.begin (I2C_PINS_29_30, I2C_RATE_100);
			if (SUCCESS == eep.init ())
				Serial.printf ("\tmux[0] eeprom initialized\n");
			else
				{
				Serial.printf ("\tmux[0] eeprom not detected\n");
				Serial.printf ("\r\nloader stopped; reset to restart\r\n");		// give up and enter an endless
				while (1);
				}
			}

		eep.set_addr16 (ASSY_PAGE_ADDR);						// point to page 0, address 0
		eep.control.rd_wr_len = PAGE_SIZE;						// set page size
		eep.control.wr_buf_ptr = (uint8_t*)assy_page.as_array;	// point to source buffer
		eep.page_write ();										// write the page

//		eep.set_addr16 (MUX_PAGE_ADDR);							// point to page 0, address 0
//		eep.byte_read();										// read a byte before we can use current_address_read
//		Serial.printf ("0: 0x%.2X\n", eep.control.rd_byte);		// display
//		for (uint8_t i=1; i<PAGE_SIZE; i++)						// now read the other 31 bytes of the page
//			{
//			eep.current_address_read();							// read
//			Serial.printf ("%.02X: 0x%.2X\n", i, eep.control.rd_byte);	// display
//			}

		if (*sensor1_page.as_struct.sensor_type)
			{
			eep.set_addr16 (SENSOR1_PAGE_ADDR);						// point to page 1, address 0
			eep.control.rd_wr_len = PAGE_SIZE;						// set page size
			eep.control.wr_buf_ptr = (uint8_t*)sensor1_page.as_array;	// point to source buffer
			eep.page_write ();										// write the page
			}
		else
			{
			memset (sensor1_page.as_array, 0xFF, PAGE_SIZE);		// make sure this page is erased
			eep.set_addr16 (SENSOR1_PAGE_ADDR);						// point to page 1, address 0
			eep.control.rd_wr_len = PAGE_SIZE;						// set page size
			eep.control.wr_buf_ptr = (uint8_t*)sensor1_page.as_array;	// point to source buffer
			eep.page_write ();										// write the page
			}


//		eep.set_addr16 (SENSOR1_PAGE_ADDR);									// point to page 1, address 0
//		eep.byte_read();										// read a byte before we can use current_address_read
//		Serial.printf ("%.02X: 0x%.2X\n", SENSOR1_PAGE_ADDR, eep.control.rd_byte);		// display
//		for (uint8_t i=1; i<=31; i++)							// now read the other 31 bytes of the page
//			{
//			eep.current_address_read();							// read
//			Serial.printf ("%.02X: 0x%.2X\n", i+PAGE_SIZE, eep.control.rd_byte);	// display
//			}

		if (*sensor2_page.as_struct.sensor_type)
			{
			eep.set_addr16 (SENSOR2_PAGE_ADDR);						// point to page 1, address 0
			eep.control.rd_wr_len = PAGE_SIZE;						// set page size
			eep.control.wr_buf_ptr = (uint8_t*)sensor2_page.as_array;	// point to source buffer
			eep.page_write ();										// write the page
			}
		else
			{
			memset (sensor2_page.as_array, 0xFF, PAGE_SIZE);		// make sure this page is erased
			eep.set_addr16 (SENSOR2_PAGE_ADDR);						// point to page 1, address 0
			eep.control.rd_wr_len = PAGE_SIZE;						// set page size
			eep.control.wr_buf_ptr = (uint8_t*)sensor2_page.as_array;	// point to source buffer
			eep.page_write ();										// write the page
			}

//		eep.set_addr16 (SENSOR2_PAGE_ADDR);									// point to page 1, address 0
//		eep.byte_read();										// read a byte before we can use current_address_read
//		Serial.printf ("%.02X: 0x%.2X\n", SENSOR2_PAGE_ADDR, eep.control.rd_byte);		// display
//		for (uint8_t i=1; i<=31; i++)							// now read the other 31 bytes of the page
//			{
//			eep.current_address_read();							// read
//			Serial.printf ("%.02X: 0x%.2X\n", i+SENSOR2_PAGE_ADDR, eep.control.rd_byte);	// display
//			}

		if (*sensor3_page.as_struct.sensor_type)
			{
			eep.set_addr16 (SENSOR3_PAGE_ADDR);						// point to page 1, address 0
			eep.control.rd_wr_len = PAGE_SIZE;						// set page size
			eep.control.wr_buf_ptr = (uint8_t*)sensor3_page.as_array;	// point to source buffer
			eep.page_write ();										// write the page
			}
		else
			{
			memset (sensor3_page.as_array, 0xFF, PAGE_SIZE);		// make sure this page is erased
			eep.set_addr16 (SENSOR3_PAGE_ADDR);						// point to page 1, address 0
			eep.control.rd_wr_len = PAGE_SIZE;						// set page size
			eep.control.wr_buf_ptr = (uint8_t*)sensor3_page.as_array;	// point to source buffer
			eep.page_write ();										// write the page
			}

		Serial.printf ("mux[0] eeprom write complete\n");
		}

	Serial.printf ("\r\nloader stopped; reset to restart\r\n");		// give up and enter an endless

	while (1);
	}
