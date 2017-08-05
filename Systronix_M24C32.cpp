#include <Arduino.h>
#include <Systronix_M24C32.h>


//---------------------------< D E F A U L T   C O N S R U C T O R >------------------------------------------
//
// default constructor assumes lowest base address
//

Systronix_M24C32::Systronix_M24C32 (void)
	{
	_base = EEP_BASE_MIN;
	error.total_error_count = 0;				// clear the error counter
	}


//---------------------------< D E S T R U C T O R >----------------------------------------------------------
//
// destructor
//

Systronix_M24C32::~Systronix_M24C32 (void)
{
	// Anything to do here? Leave I2C as master? Set flag?
}


//---------------------------< S E T U P >--------------------------------------------------------------------
//
// TODO: merge with begin()? This function doesn't actually do anything, it just sets some private values. It's
// redundant and some params must be effectively specified again in begin (Wire net and pins are not independent).	what parameters are specified again? [wsk]
//

uint8_t Systronix_M24C32::setup (uint8_t base, i2c_t3 wire, char* name)
	{
	if ((EEP_BASE_MIN > base) || (EEP_BASE_MAX < base))
		{
		tally_transaction (SILLY_PROGRAMMER);
		return FAIL;
		}

	_base = base;
	_wire = wire;
	_wire_name = wire_name = name;		// protected and public
	return SUCCESS;
	}


//---------------------------< B E G I N >--------------------------------------------------------------------
//
// I2C_PINS_18_19 or I2C_PINS_29_30
//

void Systronix_M24C32::begin (i2c_pins pins, i2c_rate rate)
	{
	_wire.begin (I2C_MASTER, 0x00, pins, I2C_PULLUP_EXT, rate);	// join I2C as master
//	Serial.printf ("275 lib begin %s\r\n", _wire_name);
	_wire.setDefaultTimeout (200000); 							// 200ms
	}


//---------------------------< D E F A U L T   B E G I N >----------------------------------------------------
//
//
//

void Systronix_M24C32::begin (void)
	{
	_wire.begin();				// initialize I2C as master
	}


//---------------------------< B A S E _ G E T >--------------------------------------------------------------
//
//	return the I2C base address for this instance
//

uint8_t Systronix_M24C32::base_get(void)
	{
	return _base;
	}


//---------------------------< I N I T >----------------------------------------------------------------------
//
// determines if there is a MB85RC256V at _base address by attempting to get an address ack from the device
//

uint8_t Systronix_M24C32::init (void)
	{
	error.exists = true;					// necessary to presume that the device exists

	if (SUCCESS != ping_eeprom())
		{
		error.exists = false;				// only place in this file where this can be set false
		return FAIL;
		}

	return SUCCESS;
	}


//---------------------------< T A L L Y _ E R R O R S >------------------------------------------------------
//
// Here we tally errors.  This does not answer the 'what to do in the event of these errors' question; it just
// counts them.
//

void Systronix_M24C32::tally_transaction (uint8_t value)
	{
	if (value && (error.total_error_count < UINT64_MAX))
		error.total_error_count++; 			// every time here incr total error count

	error.error_val = value;

	switch (value)
		{
		case SUCCESS:
			if (error.successful_count < UINT64_MAX)
				error.successful_count++;
			break;
		case 1:								// i2c_t3 and Wire: data too long from endTransmission() (rx/tx buffers are 259 bytes - slave addr + 2 cmd bytes + 256 data)
			error.data_len_error_count++;
			break;
#if defined I2C_T3_H
		case I2C_TIMEOUT:
			error.timeout_count++;			// 4 from i2c_t3; timeout from call to status() (read)
#else
		case 4:
			error.other_error_count++;		// i2c_t3 and Wire: from endTransmission() "other error"
#endif
			break;
		case 2:								// i2c_t3 and Wire: from endTransmission()
		case I2C_ADDR_NAK:					// 5 from i2c_t3
			error.rcv_addr_nack_count++;
			break;
		case 3:								// i2c_t3 and Wire: from endTransmission()
		case I2C_DATA_NAK:					// 6 from i2c_t3
			error.rcv_data_nack_count++;
			break;
		case I2C_ARB_LOST:					// 7 from i2c_t3; arbitration lost from call to status() (read)
			error.arbitration_lost_count++;
			break;
		case I2C_BUF_OVF:
			error.buffer_overflow_count++;
			break;
		case I2C_SLAVE_TX:
		case I2C_SLAVE_RX:
			error.other_error_count++;		// 9 & 10 from i2c_t3; these are not errors, I think
			break;
		case WR_INCOMPLETE:					// 11; Wire.write failed to write all of the data to tx_buffer
			error.incomplete_write_count++;
			break;
		case SILLY_PROGRAMMER:				// 12
			error.silly_programmer_error++;
			break;
		default:
			error.unknown_error_count++;
			break;
		}
	}


//---------------------------< S E T _ A D D R 1 6 >----------------------------------------------------------
//
// byte order is important.  In Teensy memory, a uint16_t is stored least-significant byte in the lower of two
// addresses.  The eep_addr union allows access to a single eep address as a struct of two bytes (high and
// low), as an array of two bytes [0] and [1], or as a uint16_t.
//
// Given the address 0x123, this function stores it in the eep_addr union as:
//		eep.control.addr_as_u16:			0x2301 (use get_eep_addr16() to retrieve a properly ordered address)
//		eep.control.addr_as_struct.low:		0x23
//		eep.control.addr_as_struct.high:	0x01
//		eep.control.addr_as_array[0]:		0x01
//		eep.control.addr_as_array[1]:		0x23
//

uint8_t Systronix_M24C32::set_addr16 (uint16_t addr)
	{
	if (addr & (ADDRESS_MAX+1))
		{
		tally_transaction (SILLY_PROGRAMMER);
		return DENIED;										// memory address out of bounds
		}

	control.addr.as_u16 = __builtin_bswap16 (addr);			// byte swap and set the address
	return SUCCESS;
	}


//---------------------------< G E T _ F R A M _ A D D R 1 6 >------------------------------------------------
//
// byte order is important.  In Teensy memory, a uint16_t is stored least-significant byte in the lower of two
// addresses.  The eep_addr union allows access to a single eep address as a struct of two bytes (high and
// low), as an array of two bytes [0] and [1], or as a uint16_t.
//
// Returns eep address as a uint16_t in proper byte order
//
// See set_addr16() for additional explanation.
//

uint16_t Systronix_M24C32::get_addr16 (void)
	{
	return __builtin_bswap16 (control.addr.as_u16);
	}


//---------------------------< I N C _ A D D R 1 6 >----------------------------------------------------------
//
// byte order is important.  In Teensy memory, a uint16_t is stored least-significant byte in the lower of two
// addresses.  The eep_addr union allows access to a single eep address as a struct of two bytes (high and
// low), as an array of two bytes [0] and [1], or as a uint16_t.
//
// This function simplifies keeping track of the current eep address pointer when using current_address_read()
// which uses the eep's internal address pointer.  Increments the address by one and makes sure that the address
// properly wraps from 0x0FFF to 0x0000 instead of going to 0x1000.
//
// See set_addr16() for additional explanation.
//

void Systronix_M24C32::inc_addr16 (void)
	{
	uint16_t addr = __builtin_bswap16 (control.addr.as_u16);
	control.addr.as_u16 = __builtin_bswap16 ((++addr & ADDRESS_MAX));
	}


//---------------------------< A D V _ A D D R 1 6 >----------------------------------------------------------
//
// This function advances the current eep address pointer by rd_wr_len when using page_read() or page_write()
// to track the eep's internal address pointer.  Advances the address and makes sure that the address
// properly wraps from 0x0FFF to 0x0000 instead of going to 0x1000.
//

void Systronix_M24C32::adv_addr16 (void)
	{
	uint16_t addr = __builtin_bswap16 (control.addr.as_u16);
	control.addr.as_u16 = __builtin_bswap16 ((addr + control.rd_wr_len) & ADDRESS_MAX);
	}


//---------------------------< P I N G _ E E P R O M >--------------------------------------------------------
//
// send slave address to eeprom to determine if the device exists or is busy.
//

uint8_t Systronix_M24C32::ping_eeprom (void)
	{
	uint8_t ret_val;
	
	if (!error.exists)										// exit immediately if device does not exist
		return ABSENT;

	_wire.beginTransmission(_base);							// init tx buff for xmit to slave at _base address
	ret_val = _wire.endTransmission();						// xmit slave address
	if (SUCCESS != ret_val)
		return FAIL;										// device did not ack the address

	return SUCCESS;											// device acked the address
	}


//---------------------------< P I N G _ E E P R O M _ T I M E D >--------------------------------------------
//
// repeatedly send slave address to eeprom to determine when the device becomes unbusy. Timeout and return FAIL
// if device still busy after twait mS; default is 5mS; M24C32-X parts (1.6V-5.5V tW is 10mS max)
//

uint8_t Systronix_M24C32::ping_eeprom_timed (uint32_t t_wait)
	{
	uint8_t		ret_val;
//	uint32_t	start_time = millis ();
	uint32_t	end_time = millis () + t_wait;
	
	while (millis () <= end_time)
		{													// spin
		_wire.beginTransmission(_base);						// init tx buff for xmit to slave at _base address
		ret_val = _wire.endTransmission();					// xmit slave address
		if (SUCCESS == ret_val)
			return SUCCESS;
		}

	return FAIL;											// device did not ack the address within the allotted time
	}


//---------------------------< B Y T E _ W R I T E >----------------------------------------------------------
// TODO: make this work
// i2c_t3 error returns
//		beginTransmission: none, void function sets txBufferLength to 1
//		write: two address bytes; txBufferLength = 3; return 0 if txBuffer overflow (tx buffer is 259 bytes)
//		write: a byte; txBufferLength = 4; return zero if overflow
//		endTransmission: does the write; returns:
//			0=success
//			1=data too long (as a result of Wire.write() causing an overflow)
//			2=recv addr NACK
//			3=recv data NACK
//			4=other error
//
// To use this function:
//		1. use set_addr16 (addr) to set the address in the control.addr union
//		2. write the byte to be transmitted into control.wr_byte
//		3. call this function
//

uint8_t Systronix_M24C32::byte_write (void)
	{
	uint8_t ret_val;
	
	if (!error.exists)										// exit immediately if device does not exist
		return ABSENT;

	if (SUCCESS != ping_eeprom_timed ())					// device should become available within the next 5mS
		{													// it didn't
		tally_transaction (I2C_TIMEOUT);					// increment the appropriate counter
		return FAIL;										// calling function decides what to do with the error
		}

	_wire.beginTransmission(_base);							// init tx buff for xmit to slave at _base address
	control.bytes_written = _wire.write (control.addr.as_array, 2);	// put the memory address in the tx buffer
	control.bytes_written += _wire.write (control.wr_byte);			// add data byte to the tx buffer
	if (3 != control.bytes_written)
		{
		tally_transaction (WR_INCOMPLETE);					// only here 0 is error value since we expected to write more than 0 bytes
		return FAIL;
		}

	ret_val = _wire.endTransmission();						// xmit memory address and data byte
	if (SUCCESS != ret_val)
		{
		tally_transaction (ret_val);						// increment the appropriate counter
		return FAIL;										// calling function decides what to do with the error
		}

	tally_transaction (SUCCESS);
	return SUCCESS;
	}


//---------------------------< I N T 1 6 _ W R I T E >--------------------------------------------------------
// TODO: make this work
// writes the two bytes of an int16_t or uint16_t to eep beginning at address in control.addr; ls byte is first.
//
// To use this function:
//		1. use set_addr16 (addr) to set the address in the control.addr union
//		2. write the 16-bit value to be transmitted into control.wr_int16 (there is no control.wr_uint16)
//		3. call this function
//

uint8_t Systronix_M24C32::int16_write (void)
	{
	if (!error.exists)										// exit immediately if device does not exist
		return ABSENT;
	
	control.wr_buf_ptr = (uint8_t*)(&control.wr_int16);		// point to the int16 member of the control struct
	control.rd_wr_len = sizeof(uint16_t);					// set the write length
	return page_write ();									// do the write and done
	}


//---------------------------< I N T 3 2 _ W R I T E >--------------------------------------------------------
// TODO: make this work
// writes the four bytes of an int32_t or uint32_t to eep beginning at address in control.addr; ls byte is first.
//
// To use this function:
//		1. use set_addr16 (addr) to set the address in the control.addr union
//		2. write the 32-bit value to be transmitted into control.wr_int32 (there is no control.wr_uint32)
//		3. call this function
//

uint8_t Systronix_M24C32::int32_write (void)
	{
	if (!error.exists)										// exit immediately if device does not exist
		return ABSENT;
	
	control.wr_buf_ptr = (uint8_t*)(&control.wr_int32);		// point to the int32 member of the control struct
	control.rd_wr_len = sizeof(uint32_t);					// set the write length
	return page_write ();									// do the write and done
	}


//---------------------------< P A G E _ W R I T E >----------------------------------------------------------
// TODO make this work
// writes an array of control.rd_wr_len number of bytes to eep beginning at address in control.addr.  32 bytes
// per page write max
//
// To use this function:
//		1. use set_addr16 (addr) to set the address in the control.addr union
//		2. set control.wr_buf_ptr to point at the array of bytes to be transmitted
//		3. set control.rd_wr_len to the number of bytes to be transmitted
//		4. call this function
//

uint8_t Systronix_M24C32::page_write (void)
	{
	uint8_t ret_val;

	if (!error.exists)										// exit immediately if device does not exist
		return ABSENT;
	
	if (SUCCESS != ping_eeprom_timed ())					// device should become available within the next 5mS
		{													// it didn't
		tally_transaction (I2C_TIMEOUT);					// increment the appropriate counter
		return FAIL;										// calling function decides what to do with the error
		}

	_wire.beginTransmission(_base);							// init tx buff for xmit to slave at _base address
	control.bytes_written = _wire.write (control.addr.as_array, 2);					// put the memory address in the tx buffer
	control.bytes_written += _wire.write (control.wr_buf_ptr, control.rd_wr_len);	// copy source to wire tx buffer data
	if (control.bytes_written < (2 + control.rd_wr_len))	// did we try to write too many bytes to the i2c_t3 tx buf?
		{
		tally_transaction (WR_INCOMPLETE);					// increment the appropriate counter
		return FAIL;										// calling function decides what to do with the error
		}
		
	ret_val = _wire.endTransmission();						// xmit memory address followed by data
	if (SUCCESS != ret_val)
		{
		tally_transaction (ret_val);						// increment the appropriate counter
		return FAIL;										// calling function decides what to do with the error
		}
	adv_addr16 ();											// advance our copy of the address

	tally_transaction (SUCCESS);
	return SUCCESS;
	}


//---------------------------< C U R R E N T _ A D D R E S S _ R E A D >--------------------------------------
//
// Read a byte from the eep's current address pointer; the eep's address pointer is bumped to the next
// location after the read.  We presume that the eep's address pointer was previously set with byte_read().
// This function attempts to track the eep's internal pointer by incrementing control.addr.
//
// During the internal write cycle, SDA is disabled; the device will ack a slave address but does not respond to any requests.
//
// To use this function:
//		1. perform some operation that correctly sets the eep's internal address pointer
//		2. call this function
//		3. retrieve the byte read from control.rd_byte
//

uint8_t Systronix_M24C32::current_address_read (void)
	{
	uint8_t ret_val;

	if (!error.exists)										// exit immediately if device does not exist
		return ABSENT;
	
	control.bytes_received = _wire.requestFrom(_base, 1, I2C_STOP);
	if (1 != control.bytes_received)						// if we got more than or less than 1 byte
		{
		ret_val = _wire.status();							// to get error value
		tally_transaction (ret_val);						// increment the appropriate counter
		return FAIL;										// calling function decides what to do with the error
		}

	control.rd_byte = _wire.readByte();						// get the byte
	inc_addr16 ();											// bump our copy of the address

	tally_transaction (SUCCESS);
	return SUCCESS;
	}


//---------------------------< B Y T E _ R E A D >------------------------------------------------------------
//
// Read a byte from a specified address.
//
// To use this function:
//		1. use set_addr16 (addr) to set the address in the control.addr union
//		2. call this function
//		3. retrieve the byte read from control.rd_byte
//

uint8_t Systronix_M24C32::byte_read (void)
	{
	uint8_t ret_val;

	if (!error.exists)										// exit immediately if device does not exist
		return ABSENT;

	if (SUCCESS != ping_eeprom_timed ())					// device should become available within the next 5mS
		{													// it didn't
		tally_transaction (I2C_TIMEOUT);					// increment the appropriate counter
		return FAIL;										// calling function decides what to do with the error
		}

	_wire.beginTransmission(_base);							// init tx buff for xmit to slave at _base address
	control.bytes_written = _wire.write (control.addr.as_array, 2);	// put the memory address in the tx buffer
	if (2 != control.bytes_written)							// did we get correct number of bytes into the i2c_t3 tx buf?
		{
		tally_transaction (WR_INCOMPLETE);					// increment the appropriate counter
		return FAIL;										// calling function decides what to do with the error
		}

	ret_val = _wire.endTransmission();						// xmit memory address; will fail if device is busy
	
	if (SUCCESS != ret_val)
		{
		tally_transaction (ret_val);						// increment the appropriate counter
		return FAIL;										// calling function decides what to do with the error
		}
	
	return current_address_read ();							// use current_address_read() to fetch the byte
	}


//---------------------------< I N T 1 6 _ R E A D >----------------------------------------------------------
//
// reads the two bytes of an int16_t or uint16_t from eep beginning at address in control.addr; ls byte is first.
//
// To use this function:
//		1. use set_addr16 (addr) to set the address in the control.addr union
//		2. call this function
//		3. retrieve the 16-bit value from control.rd_int16
//

uint8_t Systronix_M24C32::int16_read (void)
	{
	if (!error.exists)										// exit immediately if device does not exist
		return ABSENT;
	
	control.rd_buf_ptr = (uint8_t*)(&control.rd_int16);
	control.rd_wr_len = sizeof(uint16_t);					// set the read length
	return page_read ();									// do the read and done
	}


//---------------------------< I N T 3 2 _ R E A D >----------------------------------------------------------
//
// reads the four bytes of an int32_t or uint32_t from eep beginning at address in control.addr; ls byte is first.
//
// To use this function:
//		1. use set_addr16 (addr) to set the address in the control.addr union
//		2. call this function
//		3. retrieve the 32-bit value from control.rd_int32
//

uint8_t Systronix_M24C32::int32_read (void)
	{
	if (!error.exists)										// exit immediately if device does not exist
		return ABSENT;
	
	control.rd_buf_ptr = (uint8_t*)(&control.rd_int32);
	control.rd_wr_len = sizeof(uint32_t);					// set the read length
	return page_read ();									// do the read and done
	}


//---------------------------< P A G E _ R E A D >------------------------------------------------------------
//
// Reads control.rd_wr_len bytes from eep beginning at control.addr.  The number of bytes that can be read in
// a single operation is limited by the I2C_RX_BUFFER_LENGTH #define in i2c_t3.h.  Setting control.rd_wr_len to
// 256 is a convenient max (max size of the i2c_t3 buffer).
//
// To use this function:
//		1. use set_addr16 (addr) to set the address in the control.addr union
//		2. set control.rd_wr_len to the number of bytes to be read
//		3. set control.rd_buf_ptr to point to the place where the data are to be stored
//		4. call this function
//

uint8_t Systronix_M24C32::page_read (void)
	{
	uint8_t		ret_val;
	size_t 		i;
	uint8_t*	ptr = control.rd_buf_ptr;					// a copy so we don't disturb the original

	if (!error.exists)										// exit immediately if device does not exist
		return ABSENT;

	if (SUCCESS != ping_eeprom_timed ())					// device should become available within the next 5mS
		{													// it didn't
		tally_transaction (I2C_TIMEOUT);					// increment the appropriate counter
		return FAIL;										// calling function decides what to do with the error
		}

	_wire.beginTransmission(_base);							// init tx buff for xmit to slave at _base address
	control.bytes_written = _wire.write (control.addr.as_array, 2);	// put the memory address in the tx buffer
	if (2 != control.bytes_written)							// did we get correct number of bytes into the i2c_t3 tx buf?
		{
		tally_transaction (WR_INCOMPLETE);					// increment the appropriate counter
		return FAIL;										// calling function decides what to do with the error
		}

	ret_val = _wire.endTransmission (I2C_NOSTOP);			// xmit memory address

	if (SUCCESS != ret_val)
		{
		tally_transaction (ret_val);						// increment the appropriate counter
		return FAIL;										// calling function decides what to do with the error
		}

	control.bytes_received = _wire.requestFrom(_base, control.rd_wr_len, I2C_STOP);	// read the bytes
	if (control.bytes_received != control.rd_wr_len)
		{
		ret_val = _wire.status();							// to get error value
		tally_transaction (ret_val);						// increment the appropriate counter
		return FAIL;										// calling function decides what to do with the error
		}

	for (i=0;i<control.rd_wr_len; i++)						// copy wire rx buffer data to destination
		*ptr++ = _wire.readByte();

	adv_addr16 ();											// advance our copy of the address

	tally_transaction (SUCCESS);
	return SUCCESS;
	}
