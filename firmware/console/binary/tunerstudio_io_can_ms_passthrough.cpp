/**
 * @file tunerstudio_io_can_ms_passthrough.cpp
 *
 * @date Aug 23, 2023
 * @author Andrey Gusakov, (c) 2023
 *
 * Implementation of MegaSquirt 29bit CAN Protocol Passthrough slave
 *
 * See http://www.msextra.com/doc/pdf/Megasquirt_29bit_CAN_Protocol-2015-01-20.pdf
 *
 */

#include <string.h>

#include "tunerstudio_io.h"
#include "hal.h"
#include "wideband_config.h"

#include "tunerstudio.h"
#include "livedata.h"
#include "byteswap.h"

#include <rusefi/crc.h>
#include <rusefi/fragments.h>
/* getTsSignature() */
#include "port.h"

/* Message types, standart */
#define MS_MSG_CMD			(0)
#define MS_MSG_REQ			(1)
#define MS_MSG_RSP			(2)
#define MS_MSG_XSUB			(3)
#define MS_MSG_BURN			(4)
#define MS_OUTMSG_REQ		(5)
#define MS_OUTMSG_RSP		(6)
#define MS_MSG_XTND			(7)	/* An extended message, message number is in first data byte. */
/* extended types */
#define MS_EXT_MSG_FWD		(8)
#define MS_EXT_MSG_CRC		(9)
#define MS_EXT_MSG_NOP		(11)
#define MS_EXT_MSG_REQX		(12)
#define MS_EXT_MSG_BURNACK	(14)
#define MS_EXT_MSG_PROT		(0x80)
#define MS_EXT_MSG_WCR		(0x81)
#define MS_EXT_MSG_SPND		(0x82)

struct ms_msg_my {
	uint8_t myvarblk;
	uint8_t varbyt;
	uint16_t myvaroffset;
};

struct ms_msg {
	/* From 29 bits of CAN ID */
	uint8_t msg_type;
	uint8_t from_id;
	uint8_t to_id;
	uint8_t table;
	uint16_t offset;	/* 11 bits only, maximum offset is 2047 */
	/* From payload */
	union {
		struct ms_msg_my my;
	};
};

static int CAN_send(CANTxFrame *frame)
{
    frame->IDE = CAN_IDE_EXT;

    canTransmitTimeout(&CAND1, CAN_ANY_MAILBOX, frame, TIME_INFINITE);

	return 0;
}

static int MS_decode_header(CANRxFrame *f, struct ms_msg *msg)
{
	uint32_t id = f->EID;

	msg->msg_type = ((id >> 15) & 0x07);
	/* extended command? */
	if (msg->msg_type == MS_MSG_XTND) {
		if (f->DLC >= 1) {
			msg->msg_type = f->data8[0];
		} else {
			//printf("incorrect CAN message size for MSG_XTND\n");
			return -1;
		}
	}

	msg->table  = ((id >> 3) & 0x0f);
	msg->table |= ((id >> 2) & 0x01) << 4; //bit 4;

	msg->to_id = ((id >> 7) & 0x0f);

	msg->from_id = ((id >> 11) & 0x0f);

	msg->offset = ((id >> 18) & 0x7ff);

	return 0;
}

static uint32_t MS_pack_header(struct ms_msg *msg)
{
	return
		(((uint32_t)msg->offset & 0x7ff) << 18) |
		(((uint32_t)msg->msg_type & 0x07) << 15 ) |
		(((uint32_t)msg->from_id & 0x0f) << 11) |
		(((uint32_t)msg->to_id & 0x0f) << 7) |
		(((uint32_t)msg->table & 0x0f) << 3) |
		(((uint32_t)msg->table & 0x10) >> 2);
}

static int MS_decode_my(CANRxFrame *f, struct ms_msg *msg)
{
	struct ms_msg_my *my = &msg->my;

	/* Not all messages have arguments */
	if ((msg->msg_type == MS_MSG_REQ) ||
		(msg->msg_type == MS_EXT_MSG_CRC) ||
		(msg->msg_type == MS_EXT_MSG_REQX) ||
		(msg->msg_type == MS_EXT_MSG_PROT)) {
		int offset = (msg->msg_type <= MS_MSG_XTND) ? 0 : 1;
		int extra = (msg->msg_type == MS_EXT_MSG_REQX) ? 1 : 0;

		if (f->DLC < offset + 3 + extra)
			return -1;

		my->myvarblk = f->data8[offset + 0] & 0x1f;
		my->myvaroffset  = (f->data8[offset + 2] >> 5) & 0x07;
		my->myvaroffset |= (f->data8[offset + 1] << 3);
		my->varbyt = f->data8[offset + 2] & 0x0f;
		if (msg->msg_type == MS_EXT_MSG_REQX) {
			msg->table = f->data8[offset + 3];
		}
	} else {
		memset(my, 0, sizeof(*my));
	}

	return 0;
}

static int MS_prepare_reply(struct ms_msg *tx, struct ms_msg *rx)
{
	struct ms_msg_my *req = &rx->my;

	/* prepare tx packet fields based on rx packed */
	tx->from_id = rx->to_id;
	tx->to_id = rx->from_id;
	tx->table = req->myvarblk;
	tx->offset = req->myvaroffset;

	return 0;
}

static int MS_send_rsp(struct ms_msg *msg, const char *data, int size)
{
	CANTxFrame tx;

	/* Pack extended CANID */
	tx.EID = MS_pack_header(msg);
	tx.IDE = CAN_IDE_EXT;
	tx.RTR = CAN_RTR_DATA;

	if ((size) && (data)) {
		memcpy(tx.data8, data, size);
	}
	tx.DLC = size;

	return CAN_send(&tx);
}

static int MS_send_table_rsp(struct ms_msg *msg, const char *page, size_t page_size)
{
	size_t size;
	struct ms_msg tx;
	struct ms_msg_my *req = &msg->my;

	tx.msg_type = MS_MSG_RSP;
	MS_prepare_reply(&tx, msg);

	/* check out of bounds */
	if (msg->offset >= page_size) {
		size = 0;
	} else if (msg->offset + req->varbyt > page_size) {
		size = page_size - msg->offset;
	} else {
		size = req->varbyt;
	}

	return MS_send_rsp(&tx, &page[msg->offset], size);
}

static int MS_send_output_channels_rsp(struct ms_msg *msg)
{
	size_t size;
	struct ms_msg tx;
	CANTxFrame frame;
	struct ms_msg_my *req = &msg->my;

	tx.msg_type = MS_MSG_RSP;
	MS_prepare_reply(&tx, msg);

	/* Pack extended CANID */
	frame.EID = MS_pack_header(&tx);
	frame.IDE = CAN_IDE_EXT;
	frame.RTR = CAN_RTR_DATA;

	size = copyRange(frame.data8, getFragments(), msg->offset, req->varbyt);

	/* actual data size copied, may be zero */
	frame.DLC = size;

	return CAN_send(&frame);
}

static int MS_decode_msg_req(struct ms_msg *msg)
{
	/* handle */
	if (msg->table == 0) {
		return MS_send_table_rsp(msg, (char *)getWorkingPageAddr(), getTunerStudioPageSize());
	} else if (msg->table == 1) {
		/* Table 1 is used for scatter-gather list, which is not implemented over CAN */
		return -1;
	} else if (msg->table == 7) {
		/* This is livedata table, see ochGetCommand in ini file using page 0x07 */
		return MS_send_output_channels_rsp(msg);
	} else if (msg->table == 14) {
		/* see versionInfo in ini file
		 * table 14 is used for Version and Copyright string (same as on MS) */
		return MS_send_table_rsp(msg, getTsVersion(), getTsVersionSize());
	} else if (msg->table == 15) {
		/* see queryCommand in ini file
		 * table 15 is used for Signature (same as on MS) */
		return MS_send_table_rsp(msg, getTsSignature(), getTsSignatureSize());
	} else {
		return -1;
	}

	return -1;
}

static int MS_decode_ext_msg_crc(struct ms_msg *msg)
{
	struct ms_msg_my *req = &msg->my;

	if (req->varbyt != 4) {
		return -1;
	}

	if ((msg->table == 0) || (msg->table == 1)) {
		struct ms_msg tx;
		/* Doc says: The message identifier covers all of the addressing. ??? */
		//if (!validateOffsetCount(msg->offset, 0))
		//{
		//	return -1;
		//}

		uint32_t crc = SWAP_UINT32(crc32(getWorkingPageAddr(), getTunerStudioPageSize()));

		tx.msg_type = MS_MSG_RSP;
		MS_prepare_reply(&tx, msg);
		return MS_send_rsp(&tx, (char *)&crc, req->varbyt);
	}

	return -1;
}

/* TODO: run time generate? */
static const char prot_reply[5] = {
	2,	/* serial version */
	0x00, 0x10,	/* table blocking factor: high, low bytes = 16 */
	0x00, 0x10	/* write blocking factor: high, low bytes = 16 */
};

static int MS_decode_ext_msg_prot(struct ms_msg *msg)
{
	struct ms_msg tx;
	struct ms_msg_my *req = &msg->my;

	if (req->varbyt > sizeof(prot_reply)) {
		return -1;
	}

	/* The offset field is not used. */
	/* The table field is not used. */

	tx.msg_type = MS_MSG_RSP;
	MS_prepare_reply(&tx, msg);

	return MS_send_rsp(&tx, prot_reply, req->varbyt);
}

int MS_on_can_message(CANRxFrame *rx)
{
	int ret;
	struct ms_msg msg;

	ret = MS_decode_header(rx, &msg);
	if (ret) {
		return ret;
	}

	/* is message for this ECU? */
	/* TODO */
	if (msg.to_id != 12) {
		return 0;
	}

	ret = MS_decode_my(rx, &msg);
	if (ret) {
		return ret;
	}

	/* Handle commands */
	switch(msg.msg_type) {
		case MS_MSG_REQ:
		case MS_EXT_MSG_REQX:
			ret = MS_decode_msg_req(&msg);
			break;
		/* extended messages */
		case MS_EXT_MSG_CRC:
			ret = MS_decode_ext_msg_crc(&msg);
			break;
		case MS_EXT_MSG_PROT:
			ret = MS_decode_ext_msg_prot(&msg);
			break;
		default:
			ret = -1;
			/* NOP */
			break;
	}

	//if (ret)
	//	__asm volatile("BKPT #0\n");

	return ret;
}
