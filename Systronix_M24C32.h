#ifndef M24C32_H_
#define	M24C32_H_


//---------------------------< I N C L U D E S >--------------------------------------------------------------

#include <Arduino.h>
#include <Systronix_i2c_common.h>


//---------------------------< D E F I N E S >----------------------------------------------------------------
#define RSVD_SLAVE_ID	(0xF8)

#define		DENIED				0xFE

#define		EEP_BASE_MIN 		0x50					// 7-bit address not including R/W bit
#define		EEP_BASE_MAX 		0x57					// 7-bit address not including R/W bit

#define		EEP_ID_BASE_MIN		0x58					// M24C32-D only; not currently supported in this code
#define		EEP_ID_BASE_MAX		0x5F					// M24C32-D only

#define		ADDRESS_MAX			0x0FFF


//---------------------------< C L A S S >--------------------------------------------------------------------
//
//
//

class Systronix_M24C32
	{
	protected:
		uint8_t		_base;								// base address, eight possible values

		void		adv_addr16 (void);					// advance control.addr.u16 by control.rd_wr_len
		void		inc_addr16 (void);					// increment control.addr.u16 by 1
		void		tally_transaction (uint8_t);		// maintains the i2c_t3 error counters

		char* 		_wire_name = (char*)"empty";
		i2c_t3		_wire = Wire;						// why is this assigned value = Wire? [bab]

	public:
		union eep_addr
			{
			struct
				{										// an address stored here with set_eep_addr16(0x1234) gives this result:
				uint8_t		high;						// 0x12
				uint8_t		low;						// 0x34
				} as_struct;
			uint16_t		as_u16;						// 0x3412 (ARM little endian)
			uint8_t			as_array[2];				// [0]: 0x12; [1]: 0x34
			};

		struct
			{
			union eep_addr		addr;
			uint8_t				wr_byte;				// a place to put write bytes, unit16s, and uint32s
			uint16_t			wr_int16;
			uint32_t			wr_int32;
			uint8_t				rd_byte;				// a place to put read bytes, unit16s, and uint32s
			uint16_t			rd_int16;
			uint32_t			rd_int32;
			uint8_t*			wr_buf_ptr;				// pointers to read / write buffers; buffers must be appropriately sized
			uint8_t*			rd_buf_ptr;
			size_t				rd_wr_len;				// number of bytes to read/write with page_read()/page_write()
			size_t				bytes_written;			// number bytes written by Wire.write(); 0 = fail
			size_t				bytes_received;			// number of bytes read by Wire.requestFrom()
			} control;

		error_t		error;								// error struct typdefed in Systronix_i2c_common.h

		char*		wire_name;							// name of Wire, Wire1, etc in use

		Systronix_M24C32 (void);						// default constructor
		~Systronix_M24C32 (void);						// deconstructor

		uint8_t		base_get(void);						// return this instances _base address

		uint8_t		setup (uint8_t base, i2c_t3 wire = Wire, char* name = (char*)"Wire");

		void 		begin (i2c_pins pins, i2c_rate rate);
		void		begin (void);						// default begin
		uint8_t		init (void);						// determines if the device at _base is correct and communicating

		uint8_t		set_addr16 (uint16_t addr);
		uint16_t	get_addr16 (void);

		uint8_t		byte_write (void);					// write 1 byte to address
		uint8_t		int16_write (void);					// write 2-byte int16 to address
		uint8_t		int32_write (void);					// write 4-byte int32 to address
		uint8_t		page_write (void);					// write n number of bytes beginning at address
		uint8_t		current_address_read (void);		// get the byte at the current position of the device's address pointer
		uint8_t		byte_read (void);					// read 1 byte from address
		uint8_t		int16_read (void);					// read 2-byte int16 from address
		uint8_t		int32_read (void);					// read 4-byte int32 from address
		uint8_t		default_byte_read (void);			// read 1 byte from eep's current address pointer
		uint8_t		page_read (void);					// read n number of bytes beginning at address

		uint8_t		ping_eeprom (void);
		uint8_t		ping_eeprom_timed (uint32_t t_wait = 5);	// call with t_wait for M32C32-X devices set to 10

	private:
	};
	
extern Systronix_M24C32 eep;

#endif	// M24C32_H_