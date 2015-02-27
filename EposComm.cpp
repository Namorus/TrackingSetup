/*
 * EposComm.cpp
 *
 *  Created on: May 22, 2013
 *  Last change on: February 27, 2015
 *      Author: Thomas Mantel
 */

#include <bitset>

#include "trackingsetup/EposComm.h"

using namespace std;

namespace tracking {

EposComm::EposComm(const char* _serialNo, int _maxRetries, int _verbose) :
		vendorid(0x0403), productid(0xa8b0), serialNo(_serialNo), maxRetries(
				_maxRetries), verbose(_verbose), device_opened(false) {
	if (ftdi_init(&ftdic) < 0) {
		std::cout << "ftdi_init failed";
	}
	// Set the timeouts to 0.5 sec (the libftdi defaults to 5 sec)
	ftdic.usb_read_timeout = 500;
	ftdic.usb_write_timeout = 500;

	pEposmutex = new pthread_mutex_t();
}

EposComm::EposComm() :
		vendorid(0x0403), productid(0xa8b0), serialNo(NULL), maxRetries(5), verbose(
				0), device_opened(false) {
	pEposmutex = new pthread_mutex_t();
	EposComm(NULL, 0);
}

EposComm::~EposComm() {
	if (device_opened == true)
		close();
	ftdi_deinit(&ftdic);
}

/*!
 * obsolete function, just for testing purpose
 */
/*
 void Epos2::getDevInfo() {
 if(device_opened) {
 unsigned int chipid;
 ftdi_read_chipid(&ftdic,&chipid);
 std::cout << "Chip id: " << chipid << std::endl;
 }
 }*/

int EposComm::open() {
	// return value
	int ret = 0;

	// open usb ftdi device
	ret = ftdi_usb_open_desc(&ftdic, vendorid, productid, NULL, serialNo);

	if (ret == 0) {
		if (verbose > 1) {
			std::cout << "device opened" << std::endl;
		}
	} else {
		std::cout << "problem opening device: " << ret << "("
				<< ftdi_get_error_string(&ftdic) << ")" << std::endl;
		return 1;
	}

	ret = ftdi_usb_reset(&ftdic);

	// setting device up for communication
	ret = ftdi_set_baudrate(&ftdic, 1000000);
	if (ret < 0) {
		if (verbose > 0) {
			std::cout << "could not set baud rate ("
					<< ftdi_get_error_string(&ftdic) << ")" << std::endl;
		}
	}

	//TODO : add some warnings (low priority)
	ret += ftdi_set_line_property(&ftdic, BITS_8, STOP_BIT_1, NONE);

	ret += ftdi_setflowctrl(&ftdic, SIO_DISABLE_FLOW_CTRL);

	ret += ftdi_set_latency_timer(&ftdic, 1);

	if ((ret != 0) && (verbose > 0)) {
		std::cout << "an error occurred setting up device for communication: "
				<< ftdi_get_error_string(&ftdic) << std::endl;
	}
	device_opened = true;
	return ret;
}

int EposComm::close() {

	if (int ret = ftdi_usb_close(&ftdic) == 0) {
		// FT_Open OK, use ftHandle to access device
		if (verbose > 1) {
			std::cout << "device closed" << std::endl;
		}
		device_opened = false;
	} else {
		std::cout << "could not close device: " << ret << "("
				<< ftdi_get_error_string(&ftdic) << ")" << std::endl;
	}
	return 0;
}

void EposComm::sendFrame(uint16_t* frame) {
	// need LSB of header WORD, contains (len-1). Complete Frame is
	// (len-1) + 3 WORDS long
	unsigned short len = (frame[0] >> 8) + 2;

	// add checksum to frame
	frame[len - 1] = computeChecksum(frame, len);

	// calculate number of 'DLE' characters
	unsigned int stuffed = 0;
	for (int i = 1; i < len - 1; i++) {
		if ((frame[i] & 0x00FF) == 0x90) {
			stuffed++;
		}
		if (((frame[i] & 0xFF00) >> 8) == 0x90) {
			stuffed++;
		}
	}

	// message datagram to write
	uint8_t datagram[2 + 2 + len * 2 + stuffed + 2]; // DLE,STX,OpCode,Len,len*2+stuffed,CRC
	unsigned int idx = 0; // current datagram byte index

	datagram[idx++] = 0x90;	// DLE: Data Link Escape
	datagram[idx++] = 0x02; // STX: Start of Text

	for (int i = 0; i < len; ++i) {
		// character to write
		uint8_t c;

		// LSB
		c = frame[i] & 0x00FF;
		if (c == 0x90) {
			datagram[idx++] = 0x90;
		}
		datagram[idx++] = c;

		// MSB
		c = (frame[i] >> 8);
		if (c == 0x90) {
			datagram[idx++] = 0x90;
		}
		datagram[idx++] = c;
	}

	if (verbose > 1) {
		printf(">> ");
		for (unsigned int i = 0; i < idx; ++i) {
			printf("0x%02X ", datagram[i]);
		}
		printf("\n");
	}

	int w = ftdi_write_data(&ftdic, datagram, idx);

	if (w != (int) idx) {
		fprintf(stderr,
				"ftdi device write failed (%d/%d chars written): (%s)\n", w,
				idx, ftdi_get_error_string(&ftdic));
	}

}

int EposComm::receiveFrame(uint16_t* ans, unsigned int ans_len) {
	// reply buffer
	uint8_t buf[512];

	// FTDI return code
	int wait_cnt = 100; // for Store() command the 352 was fine
	int ret = 0;

	while ((ret == 0) && (wait_cnt > 0)) {
		ret = ftdi_read_data(&ftdic, buf, sizeof(buf));
		wait_cnt--;
	}

	if (ret < 0) {
		printf("ftdi device read failed (%d): (%s)\n", ret,
				ftdi_get_error_string(&ftdic));
		return (-1);
	} else if (ret == 0) {
		printf("no data returned\n");
		return (-1);
	}

	if (verbose > 1) {
		printf("<< ");
		for (int i = 0; i < ret; i++) {
			printf("0x%02X ", buf[i]);
		}
		printf("\n");
	}

	// check DLE
	if (buf[0] != 0x90 && verbose > 0) {
		printf("Datagram error (DLE expected)");
		return (-1);
	}

	// check STX
	if (buf[1] != 0x02) {
		printf("Datagram error (STX expected)");
		return (-1);
	}

	// read OpCode
	if (buf[2] != 0x00) {
		printf("Datagram error (0x00 Answer OpCode expected)");
		return (-1);
	}

	// frame length
	uint16_t framelen = buf[3];

	if (ans_len < framelen) {
		printf("output buffer to short for a message");
		return (-1);
	}

	ans[0] = (framelen << 8);

	int idx = 4; // data buffer index

	for (int i = 1; i <= framelen + 1; ++i) {
		// LSB
		if (buf[idx] == 0x90)
			idx++;
		ans[i] = buf[idx++];

		// MSB
		if (buf[idx] == 0x90)
			idx++;
		ans[i] |= (buf[idx++] << 8);
	}

	if (verbose > 1) {
		printf("<< ");
		for (int i = 0; i <= framelen + 1; i++) {
			printf("%04x ", ans[i]);
		}
		printf("\n");
		fflush(stdout);
	}

	// compute checksum
	uint16_t crc = ans[framelen + 1];
	if (verbose > 1) {
		printf("got this CRC: 0x%04x\n", crc);
	}
	ans[framelen + 1] = 0x0000;
	ans[framelen + 1] = computeChecksum(ans, framelen + 2);

	if (crc == ans[framelen + 1]) {
		if (verbose > 1) {
			printf("CRC test OK!\n");
		}
	} else {
		// fprintf(stderr, "CRC: %04x != %04x\n", crc, ans[framelen + 1]);
		printf("CRC test FAILED");
		// setting framelen = -1 -> catch it in calling function
		framelen = -1;
	}

	/* check for error code */

	/* just to get the bit's at the right place...*/
	//ans[1] = 0x1234; ans[2] = 0xABCD;
//	E_error = ans[1] | (ans[2] << 16);
	//printf(" xxxxxxx ->%#010x<-\n", E_error);
	/*
	 printf("******** sub: ptr= %p  &ptr = %p\n", ptr, &ptr);
	 printf("******** sub: ans   = %p  &ans = %p\n", ans, &ans);
	 */
	return framelen;

}

//     COMPUTE CHECKSUM
// ----------------------------------------------------------------------------

uint16_t EposComm::computeChecksum(uint16_t *pDataArray,
		uint16_t numberOfWords) {
	uint16_t shifter, c;
	uint16_t carry;
	uint16_t CRC = 0;

	//Calculate pDataArray Word by Word
	while (numberOfWords--) {
		shifter = 0x8000;                 //Initialize BitX to Bit15
		c = *pDataArray++;                //Copy next DataWord to c
		do {
			carry = CRC & 0x8000;    //Check if Bit15 of CRC is set
			CRC <<= 1;               //CRC = CRC * 2
			if (c & shifter)
				CRC++;   //CRC = CRC + 1, if BitX is set in c
			if (carry)
				CRC ^= 0x1021; //CRC = CRC XOR G(x), if carry is true
			shifter >>= 1;     //Set BitX to next lower Bit, shifter = shifter/2
		} while (shifter);
	}

	return CRC;
}

unsigned int EposComm::ReadObject(uint16_t *ans, unsigned int ans_len,
		uint16_t index, uint8_t subindex, uint8_t nodeId) {
	uint16_t frame[4];

	frame[0] = (2 << 8) | (ReadObject_Op);
	frame[1] = index;
	/*
	 * high BYTE: 0x00(Node-ID == 0)
	 * low BYTE: subindex
	 */
	frame[2] = ((nodeId << 8) | subindex);
	frame[3] = 0x0000;

	pthread_mutex_lock(pEposmutex);
	unsigned int ret = 0;
	int retries = maxRetries;

	while (ret <= 0 && retries > 0) {
		sendFrame(frame);

		// read response
		ret = receiveFrame(ans, ans_len);
		retries--;
	}
	pthread_mutex_unlock(pEposmutex);

	// check error code
//	checkCanOpenError(E_error);

	return ret;
}

bool EposComm::WriteObject(uint16_t index, uint8_t subindex, uint32_t data,
		uint8_t nodeId) {
	uint16_t frame[6];

	// fixed: (Len << 8) | OpCode
	frame[0] = (4 << 8) | (EposComm::WriteObject_Op);
	frame[1] = index;
	frame[2] = ((nodeId << 8) | subindex); /* high BYTE: Node-ID, low BYTE: subindex */
	// data to transmit
	frame[3] = (data & 0x0000FFFF);
	frame[4] = (data >> 16);
	frame[5] = 0x00; // ZERO word, will be filled with checksum
	pthread_mutex_lock(pEposmutex);
	int framelen = 0;
	int retries = maxRetries;
	while (framelen <= 0 && retries > 0) {
		sendFrame(frame);

		// read response
		uint16_t answer[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
		framelen = receiveFrame(answer, 8);
		retries--;
	}
	pthread_mutex_unlock(pEposmutex);

	return framelen > 0;
}

} /* Namespace tracking */
