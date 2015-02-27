/*
 * Epos2lib.h
 *
 *  Created on: May 22, 2013
 *      Author: thomas
 */
#include <ftdi.h>
#include <pthread.h>

#include "TSincludes.h"

#ifndef EPOSCOMM_H_
#define EPOSCOMM_H_

namespace tracking {
	class EposComm : public TAClass {

	private:
		//! USB FTDI context
		struct ftdi_context ftdic;

		//! USB device identifiers
		const int vendorid, productid;
		const char* serialNo;

		int maxRetries;

		// verbosity level
		int verbose;

		bool device_opened;

		void sendFrame(uint16_t* frame);
		int receiveFrame(uint16_t* ans, unsigned int ans_len);
		uint16_t computeChecksum(uint16_t *pDataArray, uint16_t numberOfWords);

		pthread_mutex_t* pEposmutex;

	public:
		EposComm();
		EposComm(const char* serialNo, int maxRetries = 5, int setVerbose = 0);
		~EposComm();
		void getDevInfo();
		int open();
		int close();
		int32_t readObject(int16_t index, int8_t subindex);
		int32_t writeObject(int16_t index, int8_t subindex, int32_t data);

		/*! \brief Read Object from EPOS memory, firmware definition 6.3.1.1
		 *
		 * @param ans answer buffer
		 * @param ans_len of answer buffer
		 * @param nodeId CAN node ID
		 * @param index object entry index in a dictionary
		 * @param subindex object entry subindex of in a dictionary
		 * @return answer array from the controller
		 */
		unsigned int ReadObject(uint16_t *ans, unsigned int ans_len, uint16_t index, uint8_t subindex, uint8_t nodeId = 0x0);

		/*! \brief write object value to EPOS
		 *
		 * @param index object entry index in a dictionary
		 * @param subindex object entry subindex of in a dictionary
		 * @param data 32bit object data
		 * @param nodeId CAN node ID
		 */
		bool WriteObject(uint16_t index, uint8_t subindex, uint32_t data, uint8_t nodeId = 0x0);

		typedef enum _CanOpen_OpCode
		{
			Response_Op = 0x00,

			ReadObject_Op = 0x10,
			InitiateSegmentedRead_Op = 0x12,
			SegmentedRead_Op = 0x14,

			WriteObject_Op = 0x11,
			InitiateSegmentedWrite_Op = 0x13,
			SegmentedWrite_Op = 0x15,

			SendNMTService_Op = 0x0E,
			SendCANFrame_Op = 0x20,
			RequestCANFrame_Op = 0x21,
			SendLSSFrame_Op = 0x30,
			ReadLSSFrame_Op = 0x31
		}CanOpen_OpCode_t;

	};
} /* namespace tracking */
#endif /* EPOSCOMM_H_ */
