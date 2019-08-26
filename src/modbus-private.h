/*
 * Copyright © 2010-2012 Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * SPDX-License-Identifier: LGPL-2.1+
 */

#ifndef MODBUS_PRIVATE_H
#define MODBUS_PRIVATE_H

#ifndef _MSC_VER
# include <stdint.h>
# include <sys/time.h>
#else
# include "stdint.h"
# include <time.h>
typedef int ssize_t;
#endif
#include <sys/types.h>
#include <config.h>
#include <time.h>
#include <stdarg.h>

#include <assert.h>

#include "modbus.h"

MODBUS_BEGIN_DECLS

/* It's not really the minimal length (the real one is report slave ID
 * in RTU (4 bytes)) but it's a convenient size to use in RTU or TCP
 * communications to read many values or write a single one.
 * Maximum between :
 * - HEADER_LENGTH_TCP (7) + function (1) + address (2) + number (2)
 * - HEADER_LENGTH_RTU (1) + function (1) + address (2) + number (2) + CRC (2)
 */
#define _MIN_REQ_LENGTH 12

#define _REPORT_SLAVE_ID 180

#define _MODBUS_EXCEPTION_RSP_LENGTH 5

/* Timeouts in microsecond (0.5 s) */
#define _RESPONSE_TIMEOUT    500000
#define _BYTE_TIMEOUT        500000

typedef enum {
    _MODBUS_BACKEND_TYPE_RTU=0,
    _MODBUS_BACKEND_TYPE_TCP
} modbus_backend_type_t;

/*
 *  ---------- Request     Indication ----------
 *  | Client | ---------------------->| Server |
 *  ---------- Confirmation  Response ----------
 */
typedef enum {
    /* Request message on the server side */
    MSG_INDICATION,
    /* Request message on the client side */
    MSG_CONFIRMATION,

    /* MODBUS_EXTENSION: check interruptions on the client side*/
    MSG_INTERRUPTION
} msg_type_t;

/* This structure reduces the number of params in functions and so
 * optimizes the speed of execution (~ 37%). */
typedef struct _sft {
    int slave;
    int function;
    int t_id;
} sft_t;

typedef struct _modbus_backend {
    unsigned int backend_type;
    unsigned int header_length;
    unsigned int checksum_length;
    unsigned int max_adu_length;
    int (*set_slave) (modbus_t *ctx, int slave);
    int (*build_request_basis) (modbus_t *ctx, int function, int addr,
                                int nb, uint8_t *req);
    int (*build_response_basis) (sft_t *sft, uint8_t *rsp);
    int (*prepare_response_tid) (const uint8_t *req, int *req_length);
    int (*send_msg_pre) (uint8_t *req, int req_length);
    ssize_t (*send) (modbus_t *ctx, const uint8_t *req, int req_length);
    int (*receive) (modbus_t *ctx, uint8_t *req);
    ssize_t (*recv) (modbus_t *ctx, uint8_t *rsp, int rsp_length);
    int (*check_integrity) (modbus_t *ctx, uint8_t *msg,
                            const int msg_length);
    int (*pre_check_confirmation) (modbus_t *ctx, const uint8_t *req,
                                   const uint8_t *rsp, int rsp_length);
    int (*connect) (modbus_t *ctx);
    void (*close) (modbus_t *ctx);
    int (*flush) (modbus_t *ctx);
    int (*select) (modbus_t *ctx, fd_set *rset, struct timeval *tv, int msg_length);
    void (*free) (modbus_t *ctx);
} modbus_backend_t;


struct _modbus {
    /* Slave address */
    int slave;
    /* Socket or file descriptor */
    int s;
    int debug;
    int error_recovery;
    struct timeval response_timeout;
    struct timeval byte_timeout;
    const modbus_backend_t *backend;
    void *backend_data;

    modbus_interrupt_core_t * interrupt_core;
};

void _modbus_init_common(modbus_t *ctx);
void _error_print(modbus_t *ctx, const char *context);
int _modbus_receive_msg(modbus_t *ctx, uint8_t *msg, msg_type_t msg_type);

#ifndef HAVE_STRLCPY
size_t strlcpy(char *dest, const char *src, size_t dest_size);
#endif

/* MODBUS EXTENSION - begin */

typedef void (*interrupt_listener_t) (modbus_t*, int func, int addr, int nb, void*);
typedef struct _interrupt_trigger {
    uint16_t addr;
    uint16_t quantity;
    int pulled;
    interrupt_listener_t listener;
} modbus_interrupt_trigger_t;

typedef struct _interrupt_manager {
    modbus_list_t * triggers_list;  /* triggers list. */
    unsigned int start;             /* the 1st address of the sequence. */
    unsigned int nb;                /* the number of address in the sequence. */
} modbus_interrupt_manager_t;

typedef struct _modbus_ext_msg
{
    uint8_t data[MODBUS_MAX_ADU_LENGTH];
    size_t size;
} modbus_ext_msg_t;

struct _interrupt_core {
    modbus_interrupt_manager_t * coils_triggers;
    modbus_interrupt_manager_t * input_bits_triggers;
    modbus_interrupt_manager_t * holding_regs_triggers;
    modbus_interrupt_manager_t * input_regs_triggers;
    modbus_ext_msg_t last_response_message;
    modbus_list_t * received_interrupt_queue;
};

static const int index_not_found = -1;

struct _modbus_list {
    void * data;
    size_t element_size;
    size_t capacity;
    size_t size;
    unsigned int begin;
    unsigned int end;
};

modbus_interrupt_manager_t * _modbus_ext_new_interrupt_manager(unsigned int start, unsigned int nb);
modbus_interrupt_core_t * _modbus_interrupt_core_new(   unsigned int start_bits, unsigned int nb_bits,
                                                        unsigned int start_input_bits, unsigned int nb_input_bits,
                                                        unsigned int start_registers, unsigned int nb_registers,
                                                        unsigned int start_input_registers,
                                                        unsigned int nb_input_registers);
void _modbus_ext_free_interrupt_core(modbus_interrupt_core_t * core);
void _modbus_ext_free_interrupt_manager(modbus_interrupt_manager_t * m);

//#define _MODBUS_LOG_

#ifdef _MODBUS_LOG_

#define MACRO_TO_STR_FILENAME(id) #id ".log"

#define _MODBUS_LOG_BEGIN(id) FILE * id = ___modbus_log_new(MACRO_TO_STR_FILENAME(id))

FILE * ___modbus_log_new(char * filename);
int _MODBUS_LOG_PRINTF(FILE * log, char * fmt,...);
void _MODBUS_LOG_PRINT_LIST(FILE * f, modbus_list_t * list);
void _MODBUS_LOG_END(FILE * f);
#else

#define _MODBUS_LOG_BEGIN(id)
#define _MODBUS_LOG_PRINTF(id, fmtstr,...)
//#define _MODBUS_LOG_PRINTF(id, msg)
#define _MODBUS_LOG_END(id)
#define _MODBUS_LOG_PRINT_LIST(id,list)

#endif

/*
#define _MODBUS_LIST_MOVE_BLOCK(list,dest,src,elements)\
if (elements >= 0) {\
    void * ptr_src  = (char *)list->data + (list->element_size * (src));\
    void * ptr_dest = (char *)list->data + (list->element_size * (dest));\
    size_t bytes = (elements) * list->element_size;\
    memmove(ptr_dest, ptr_src, bytes);\
}*/

MODBUS_END_DECLS

#endif  /* MODBUS_PRIVATE_H */
