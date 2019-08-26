/*
 * Copyright © 2001-2011 Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * SPDX-License-Identifier: LGPL-2.1+
 *
 * This library implements the Modbus protocol.
 * http://libmodbus.org/
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>
#include <limits.h>
#include <time.h>
#ifndef _MSC_VER
#include <unistd.h>
#endif

#include <config.h>

#include "modbus.h"
#include "modbus-private.h"

/* Internal use */
#define MSG_LENGTH_UNDEFINED -1

/* Exported version */
const unsigned int libmodbus_version_major = LIBMODBUS_VERSION_MAJOR;
const unsigned int libmodbus_version_minor = LIBMODBUS_VERSION_MINOR;
const unsigned int libmodbus_version_micro = LIBMODBUS_VERSION_MICRO;

/* Max between RTU and TCP max adu length (so TCP) */
#define MAX_MESSAGE_LENGTH 260

/* 3 steps are used to parse the query */
typedef enum {
    _STEP_FUNCTION,
    _STEP_META,
    _STEP_DATA
} _step_t;

/* MODBUS EXTENSION: prototypes */
static void _receive_interrupts_and_last_rsp_msg(modbus_t * ctx, msg_type_t msg_type);
static void _dequeue_and_dispatch_interrupts(modbus_t * ctx);
static void _modbus_set_interrupt(modbus_interrupt_manager_t * manager, int mapping_address, int nb,
                                    uint8_t trigger_time);
static int _modbus_get_interrupt_trigger(modbus_interrupt_manager_t * manager,
                                            uint16_t addr,
                                            modbus_interrupt_trigger_t * trigger);


/* MODBUS EXTENSION: end */

const char *modbus_strerror(int errnum) {
    switch (errnum) {
    case EMBXILFUN:
        return "Illegal function";
    case EMBXILADD:
        return "Illegal data address";
    case EMBXILVAL:
        return "Illegal data value";
    case EMBXSFAIL:
        return "Slave device or server failure";
    case EMBXACK:
        return "Acknowledge";
    case EMBXSBUSY:
        return "Slave device or server is busy";
    case EMBXNACK:
        return "Negative acknowledge";
    case EMBXMEMPAR:
        return "Memory parity error";
    case EMBXGPATH:
        return "Gateway path unavailable";
    case EMBXGTAR:
        return "Target device failed to respond";
    case EMBBADCRC:
        return "Invalid CRC";
    case EMBBADDATA:
        return "Invalid data";
    case EMBBADEXC:
        return "Invalid exception code";
    case EMBMDATA:
        return "Too many data";
    case EMBBADSLAVE:
        return "Response not from requested slave";
    default:
        return strerror(errnum);
    }
}

void _error_print(modbus_t *ctx, const char *context)
{
    if (ctx->debug) {
        fprintf(stderr, "ERROR %s", modbus_strerror(errno));
        if (context != NULL) {
            fprintf(stderr, ": %s\n", context);
        } else {
            fprintf(stderr, "\n");
        }
    }
}

static void _sleep_response_timeout(modbus_t *ctx)
{
    /* Response timeout is always positive */
#ifdef _WIN32
    /* usleep doesn't exist on Windows */
    Sleep((ctx->response_timeout.tv_sec * 1000) +
          (ctx->response_timeout.tv_usec / 1000));
#else
    /* usleep source code */
    struct timespec request, remaining;
    request.tv_sec = ctx->response_timeout.tv_sec;
    request.tv_nsec = ((long int)ctx->response_timeout.tv_usec) * 1000;
    while (nanosleep(&request, &remaining) == -1 && errno == EINTR) {
        request = remaining;
    }
#endif
}

int modbus_flush(modbus_t *ctx)
{
    int rc;

    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    rc = ctx->backend->flush(ctx);
    if (rc != -1 && ctx->debug) {
        /* Not all backends are able to return the number of bytes flushed */
        fprintf(stderr, "Bytes flushed (%d)\n", rc);
    }
    return rc;
}

/* Computes the length of the expected response */
static unsigned int compute_response_length_from_request(modbus_t *ctx, uint8_t *req)
{
    int length;
    const int offset = ctx->backend->header_length;

    switch (req[offset]) {
    case MODBUS_FC_READ_COILS:
    case MODBUS_FC_READ_DISCRETE_INPUTS: {
        /* Header + nb values (code from write_bits) */
        int nb = (req[offset + 3] << 8) | req[offset + 4];
        length = 2 + (nb / 8) + ((nb % 8) ? 1 : 0);
    }
        break;
    case MODBUS_FC_WRITE_AND_READ_REGISTERS:
    case MODBUS_FC_READ_HOLDING_REGISTERS:
    case MODBUS_FC_READ_INPUT_REGISTERS:
        /* Header + 2 * nb values */
        length = 2 + 2 * (req[offset + 3] << 8 | req[offset + 4]);
        break;
    case MODBUS_FC_READ_EXCEPTION_STATUS:
        length = 3;
        break;
    case MODBUS_FC_REPORT_SLAVE_ID:
        /* The response is device specific (the header provides the
           length) */
        return MSG_LENGTH_UNDEFINED;
    case MODBUS_FC_MASK_WRITE_REGISTER:
        length = 7;
        break;
    default:
        length = 5;
    }

    return offset + length + ctx->backend->checksum_length;
}

/* Sends a request/response */
static int send_msg(modbus_t *ctx, uint8_t *msg, int msg_length)
{
    int rc;
    int i;

    _MODBUS_LOG_BEGIN(_send_msg);
    _MODBUS_LOG_PRINTF(_send_msg, "msg={");
    for (i = 0; i < msg_length; i++) {
        _MODBUS_LOG_PRINTF(_send_msg, "[%.2X]", msg[i]);
    }
    _MODBUS_LOG_PRINTF(_send_msg, "}\n");
    _MODBUS_LOG_PRINTF(_send_msg, "msg_length=%d\n", msg_length);

    msg_length = ctx->backend->send_msg_pre(msg, msg_length);

    if (ctx->debug) {
        for (i = 0; i < msg_length; i++)
            fprintf(stderr, "[%.2X]", msg[i]);
        fprintf(stderr, "\n");
    }

    /* In recovery mode, the write command will be issued until to be
       successful! Disabled by default. */
    do {
        _MODBUS_LOG_PRINTF(_send_msg, "Sending...");
        rc = ctx->backend->send(ctx, msg, msg_length);
        if (rc == -1) {
            _MODBUS_LOG_PRINTF(_send_msg, "ERROR!\n");

            _error_print(ctx, NULL);
            if (ctx->error_recovery & MODBUS_ERROR_RECOVERY_LINK) {
                int saved_errno = errno;

                if ((errno == EBADF || errno == ECONNRESET || errno == EPIPE)) {
                    _MODBUS_LOG_PRINTF(_send_msg, "Reconnecting...\n");
                    modbus_close(ctx);
                    _sleep_response_timeout(ctx);
                    modbus_connect(ctx);
                    _MODBUS_LOG_PRINTF(_send_msg, "Ok!\n");
                } else {
                    _sleep_response_timeout(ctx);
                    modbus_flush(ctx);
                    _MODBUS_LOG_PRINTF(_send_msg, "Stream flushed\n");
                }
                errno = saved_errno;
            }
        }
    } while ((ctx->error_recovery & MODBUS_ERROR_RECOVERY_LINK) &&
             rc == -1);

    if (rc > 0 && rc != msg_length) {
        errno = EMBBADDATA;
        _MODBUS_LOG_PRINTF(_send_msg, "ERROR: EMBBADDATA\n");
        _MODBUS_LOG_END(_send_msg);
        return -1;
    }

    _MODBUS_LOG_END(_send_msg);
    return rc;
}

int modbus_send_raw_request(modbus_t *ctx, uint8_t *raw_req, int raw_req_length)
{
    sft_t sft;
    uint8_t req[MAX_MESSAGE_LENGTH];
    int req_length;

    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    if (raw_req_length < 2 || raw_req_length > (MODBUS_MAX_PDU_LENGTH + 1)) {
        /* The raw request must contain function and slave at least and
           must not be longer than the maximum pdu length plus the slave
           address. */
        errno = EINVAL;
        return -1;
    }

    sft.slave = raw_req[0];
    sft.function = raw_req[1];
    /* The t_id is left to zero */
    sft.t_id = 0;
    /* This response function only set the header so it's convenient here */
    req_length = ctx->backend->build_response_basis(&sft, req);

    if (raw_req_length > 2) {
        /* Copy data after function code */
        memcpy(req + req_length, raw_req + 2, raw_req_length - 2);
        req_length += raw_req_length - 2;
    }

    return send_msg(ctx, req, req_length);
}

/*
 *  ---------- Request     Indication ----------
 *  | Client | ---------------------->| Server |
 *  ---------- Confirmation  Response ----------
 */

/* Computes the length to read after the function received */
static uint8_t compute_meta_length_after_function(int function,
                                                  msg_type_t msg_type)
{
    int length;

    if (msg_type == MSG_INDICATION) {
        if (function <= MODBUS_FC_WRITE_SINGLE_REGISTER) {
            length = 4;
        } else if (function == MODBUS_FC_WRITE_MULTIPLE_COILS ||
                   function == MODBUS_FC_WRITE_MULTIPLE_REGISTERS) {
            length = 5;
        } else if (function == MODBUS_FC_MASK_WRITE_REGISTER) {
            length = 6;
        } else if (function == MODBUS_FC_WRITE_AND_READ_REGISTERS) {
            length = 9;
        } else if (function >= MODBUS_FC_LISTEN_COILS && function <= MODBUS_FC_LISTEN_INPUT_REGISTERS) {
            length = 5;
        } else {
            /* MODBUS_FC_READ_EXCEPTION_STATUS, MODBUS_FC_REPORT_SLAVE_ID */
            length = 0;
        }
    }

    else {
        /* MSG_CONFIRMATION */
        /* MODBUS_EXTENSION: ... and MSG_INTERRUPTION */
        switch (function) {
        case MODBUS_FC_WRITE_SINGLE_COIL:
        case MODBUS_FC_WRITE_SINGLE_REGISTER:
        case MODBUS_FC_WRITE_MULTIPLE_COILS:
        case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
            length = 4;
            break;

        /* MODBUS_EXTENSION: begin */
        case MODBUS_FC_LISTEN_COILS:
        case MODBUS_FC_LISTEN_DISCRETE_INPUTS:
        case MODBUS_FC_LISTEN_HOLDING_REGISTERS:
        case MODBUS_FC_LISTEN_INPUT_REGISTERS:
            length = 3;
            break;
        /* MODBUS_EXTENSION: end */

        case MODBUS_FC_MASK_WRITE_REGISTER:
            length = 6;
            break;
        default:
            length = 1;
        }
    }

    return length;
}

/* Computes the length to read after the meta information (address, count, etc) */
static int compute_data_length_after_meta(modbus_t *ctx, uint8_t *msg,
                                          msg_type_t msg_type)
{
    int function = msg[ctx->backend->header_length];
    int length;

    if (msg_type == MSG_INDICATION) {
        switch (function) {
        case MODBUS_FC_WRITE_MULTIPLE_COILS:
        case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
            length = msg[ctx->backend->header_length + 5];
            break;
        case MODBUS_FC_WRITE_AND_READ_REGISTERS:
            length = msg[ctx->backend->header_length + 9];
            break;
        default:
            length = 0;
        }
    } else {
        /* MSG_CONFIRMATION */
        /* MODBUS_EXTENSION: ... and MSG_INTERRUPTION */
        if (function <= MODBUS_FC_READ_INPUT_REGISTERS ||
            function == MODBUS_FC_REPORT_SLAVE_ID ||
            function == MODBUS_FC_WRITE_AND_READ_REGISTERS) {
            length = msg[ctx->backend->header_length + 1];
        }

        /* MODBUS_EXTENSION: begin */
        else if(function >= MODBUS_FC_LISTEN_COILS && function <= MODBUS_FC_LISTEN_INPUT_REGISTERS) {
            length = msg[ctx->backend->header_length + 3];
        }
        /* MODBUS_EXTENSION: end */

        else {
            length = 0;
        }
    }

    length += ctx->backend->checksum_length;

    return length;
}


/* Waits a response from a modbus server or a request from a modbus client.
   This function blocks if there is no replies (3 timeouts).

   The function shall return the number of received characters and the received
   message in an array of uint8_t if successful. Otherwise it shall return -1
   and errno is set to one of the values defined below:
   - ECONNRESET
   - EMBBADDATA
   - EMBUNKEXC
   - ETIMEDOUT
   - read() or recv() error codes
*/

int _modbus_receive_msg(modbus_t *ctx, uint8_t *msg, msg_type_t msg_type)
{
    int rc;
    fd_set rset;
    struct timeval tv;
    struct timeval *p_tv;
    int length_to_read;
    int msg_length = 0;
    _step_t step;

    if (ctx->debug) {
        if (msg_type == MSG_INDICATION) {
            fprintf(stderr, "Checking for an indication...\n");
        }

        /* MODBUS_EXTENSION: begin */
        else if (msg_type == MSG_INTERRUPTION) {
            fprintf(stderr, "Checking for an interruption...\n");
        }
        /* MODBUS_EXTENSION: end */
        else {
            fprintf(stderr, "Waiting for a confirmation...\n");
        }

    }

    /* Add a file descriptor to the set */
    FD_ZERO(&rset);
    FD_SET(ctx->s, &rset);

    /* We need to analyse the message step by step.  At the first step, we want
     * to reach the function code because all packets contain this
     * information. */
    step = _STEP_FUNCTION;
    length_to_read = ctx->backend->header_length + 1;

    /* MODBUS_EXTENSION: included MSG_INTERRUPTION */
    if (msg_type == MSG_INDICATION || msg_type == MSG_INTERRUPTION) {
        /* Wait for a message, we don't know when the message will be
         * received */
        //p_tv = NULL;

        /* MODBUS EXTENSION: don't wait! */
        tv.tv_sec = 0;
        tv.tv_usec = 0;
        p_tv = &tv;
    } else {
        tv.tv_sec = ctx->response_timeout.tv_sec;
        tv.tv_usec = ctx->response_timeout.tv_usec;
        p_tv = &tv;
    }

    while (length_to_read != 0) {

        rc = ctx->backend->select(ctx, &rset, p_tv, length_to_read);

        if (rc == -1) {

            /* MODBUS EXTENSION: if the message is an indication or an interruption, no error occurs. */
            switch (msg_type) {
                case MSG_INDICATION:
                case MSG_INTERRUPTION:
                    return 0;

                case MSG_CONFIRMATION:
                    /* do nothing */
                    break;
            }

            _error_print(ctx, "select");
            if (ctx->error_recovery & MODBUS_ERROR_RECOVERY_LINK) {

                int saved_errno = errno;

                if (errno == ETIMEDOUT) {

                    _sleep_response_timeout(ctx);
                    modbus_flush(ctx);
                } else if (errno == EBADF) {

                    modbus_close(ctx);
                    modbus_connect(ctx);
                }
                errno = saved_errno;
            }

            return -1;
        }

        rc = ctx->backend->recv(ctx, msg + msg_length, length_to_read);
        if (rc == 0) {
            errno = ECONNRESET;
            rc = -1;
        }

        if (rc == -1) {
            _error_print(ctx, "read");
            if ((ctx->error_recovery & MODBUS_ERROR_RECOVERY_LINK) &&
                (errno == ECONNRESET || errno == ECONNREFUSED ||
                 errno == EBADF)) {
                int saved_errno = errno;
                modbus_close(ctx);
                modbus_connect(ctx);
                /* Could be removed by previous calls */
                errno = saved_errno;
            }
            return -1;
        }

        /* Display the hex code of each character received */
        if (ctx->debug) {
            int i;
            for (i=0; i < rc; i++)
                fprintf(stderr, "<%.2X>", msg[msg_length + i]);
        }

        /* Sums bytes received */
        msg_length += rc;
        /* Computes remaining bytes */
        length_to_read -= rc;

        if (length_to_read == 0) {
            switch (step) {
            case _STEP_FUNCTION:
                /* Function code position */
                length_to_read = compute_meta_length_after_function(
                    msg[ctx->backend->header_length],
                    msg_type);
                if (length_to_read != 0) {
                    step = _STEP_META;
                    break;
                } /* else switches straight to the next step */
            case _STEP_META:
                length_to_read = compute_data_length_after_meta(
                    ctx, msg, msg_type);
                if ((msg_length + length_to_read) > (int)ctx->backend->max_adu_length) {
                    errno = EMBBADDATA;
                    _error_print(ctx, "too many data");
                    return -1;
                }
                step = _STEP_DATA;
                break;
            default:
                break;
            }
        }

        if (length_to_read > 0 &&
            (ctx->byte_timeout.tv_sec > 0 || ctx->byte_timeout.tv_usec > 0)) {
            /* If there is no character in the buffer, the allowed timeout
               interval between two consecutive bytes is defined by
               byte_timeout */
            tv.tv_sec = ctx->byte_timeout.tv_sec;
            tv.tv_usec = ctx->byte_timeout.tv_usec;
            p_tv = &tv;
        }
        /* else timeout isn't set again, the full response must be read before
           expiration of response timeout (for CONFIRMATION only) */
    }

    if (ctx->debug)
        fprintf(stderr, "\n");

    return ctx->backend->check_integrity(ctx, msg, msg_length);
}

/* Receive the request from a modbus master */
int modbus_receive(modbus_t *ctx, uint8_t *req)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    return ctx->backend->receive(ctx, req);
}

/* Receives the confirmation.

   The function shall store the read response in rsp and return the number of
   values (bits or words). Otherwise, its shall return -1 and errno is set.

   The function doesn't check the confirmation is the expected response to the
   initial request.
*/
int modbus_receive_confirmation(modbus_t *ctx, uint8_t *rsp)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    return _modbus_receive_msg(ctx, rsp, MSG_CONFIRMATION);
}

static int check_confirmation(modbus_t *ctx, uint8_t *req,
                              uint8_t *rsp, int rsp_length)
{
    int rc;
    int rsp_length_computed;
    const int offset = ctx->backend->header_length;
    const int function = rsp[offset];

    if (ctx->backend->pre_check_confirmation) {
        rc = ctx->backend->pre_check_confirmation(ctx, req, rsp, rsp_length);
        if (rc == -1) {
            if (ctx->error_recovery & MODBUS_ERROR_RECOVERY_PROTOCOL) {
                _sleep_response_timeout(ctx);
                modbus_flush(ctx);
            }
            return -1;
        }
    }

    rsp_length_computed = compute_response_length_from_request(ctx, req);

    /* Exception code */
    if (function >= 0x80) {
        if (rsp_length == (offset + 2 + (int)ctx->backend->checksum_length) &&
            req[offset] == (rsp[offset] - 0x80)) {
            /* Valid exception code received */

            int exception_code = rsp[offset + 1];
            if (exception_code < MODBUS_EXCEPTION_MAX) {
                errno = MODBUS_ENOBASE + exception_code;
            } else {
                errno = EMBBADEXC;
            }
            _error_print(ctx, NULL);
            return -1;
        } else {
            errno = EMBBADEXC;
            _error_print(ctx, NULL);
            return -1;
        }
    }

    /* Check length */
    if ((rsp_length == rsp_length_computed ||
         rsp_length_computed == MSG_LENGTH_UNDEFINED) &&
        function < 0x80) {
        int req_nb_value;
        int rsp_nb_value;

        /* Check function code */
        if (function != req[offset]) {
            if (ctx->debug) {
                fprintf(stderr,
                        "Received function not corresponding to the request (0x%X != 0x%X)\n",
                        function, req[offset]);
            }
            if (ctx->error_recovery & MODBUS_ERROR_RECOVERY_PROTOCOL) {
                _sleep_response_timeout(ctx);
                modbus_flush(ctx);
            }
            errno = EMBBADDATA;
            return -1;
        }

        /* Check the number of values is corresponding to the request */
        switch (function) {
        case MODBUS_FC_READ_COILS:
        case MODBUS_FC_READ_DISCRETE_INPUTS:
            /* Read functions, 8 values in a byte (nb
             * of values in the request and byte count in
             * the response. */
            req_nb_value = (req[offset + 3] << 8) + req[offset + 4];
            req_nb_value = (req_nb_value / 8) + ((req_nb_value % 8) ? 1 : 0);
            rsp_nb_value = rsp[offset + 1];
            break;
        case MODBUS_FC_WRITE_AND_READ_REGISTERS:
        case MODBUS_FC_READ_HOLDING_REGISTERS:
        case MODBUS_FC_READ_INPUT_REGISTERS:
            /* Read functions 1 value = 2 bytes */
            req_nb_value = (req[offset + 3] << 8) + req[offset + 4];
            rsp_nb_value = (rsp[offset + 1] / 2);
            break;
        case MODBUS_FC_WRITE_MULTIPLE_COILS:
        case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
            /* N Write functions */
            req_nb_value = (req[offset + 3] << 8) + req[offset + 4];
            rsp_nb_value = (rsp[offset + 3] << 8) | rsp[offset + 4];
            break;
        case MODBUS_FC_REPORT_SLAVE_ID:
            /* Report slave ID (bytes received) */
            req_nb_value = rsp_nb_value = rsp[offset + 1];
            break;
        default:
            /* 1 Write functions & others */
            req_nb_value = rsp_nb_value = 1;
        }

        if (req_nb_value == rsp_nb_value) {
            rc = rsp_nb_value;
        } else {
            if (ctx->debug) {
                fprintf(stderr,
                        "Quantity not corresponding to the request (%d != %d)\n",
                        rsp_nb_value, req_nb_value);
            }

            if (ctx->error_recovery & MODBUS_ERROR_RECOVERY_PROTOCOL) {
                _sleep_response_timeout(ctx);
                modbus_flush(ctx);
            }

            errno = EMBBADDATA;
            rc = -1;
        }
    } else {
        if (ctx->debug) {
            fprintf(stderr,
                    "Message length not corresponding to the computed length (%d != %d)\n",
                    rsp_length, rsp_length_computed);
        }
        if (ctx->error_recovery & MODBUS_ERROR_RECOVERY_PROTOCOL) {
            _sleep_response_timeout(ctx);
            modbus_flush(ctx);
        }
        errno = EMBBADDATA;
        rc = -1;
    }

    return rc;
}

static int response_io_status(uint8_t *tab_io_status,
                              int address, int nb,
                              uint8_t *rsp, int offset)
{
    int shift = 0;
    /* Instead of byte (not allowed in Win32) */
    int one_byte = 0;
    int i;

    for (i = address; i < address + nb; i++) {
        one_byte |= tab_io_status[i] << shift;
        if (shift == 7) {
            /* Byte is full */
            rsp[offset++] = one_byte;
            one_byte = shift = 0;
        } else {
            shift++;
        }
    }

    if (shift != 0)
        rsp[offset++] = one_byte;

    return offset;
}

/* Build the exception response */
static int response_exception(modbus_t *ctx, sft_t *sft,
                              int exception_code, uint8_t *rsp,
                              unsigned int to_flush,
                              const char* template, ...)
{
    int rsp_length;

    /* Print debug message */
    if (ctx->debug) {
        va_list ap;

        va_start(ap, template);
        vfprintf(stderr, template, ap);
        va_end(ap);
    }

    /* Flush if required */
    if (to_flush) {
        _sleep_response_timeout(ctx);
        modbus_flush(ctx);
    }

    /* Build exception response */
    sft->function = sft->function + 0x80;
    rsp_length = ctx->backend->build_response_basis(sft, rsp);
    rsp[rsp_length++] = exception_code;

    return rsp_length;
}

/* Send a response to the received request.
   Analyses the request and constructs a response.

   If an error occurs, this function construct the response
   accordingly.
*/
int modbus_reply(modbus_t *ctx, const uint8_t *req,
                 int req_length, modbus_mapping_t *mb_mapping)
{
    _MODBUS_LOG_BEGIN(_modbus_reply);
    int offset;
    int slave;
    int function;
    uint16_t address;
    uint8_t rsp[MAX_MESSAGE_LENGTH];
    int rsp_length = 0;
    sft_t sft;

    if (ctx == NULL) {
        errno = EINVAL;
        _MODBUS_LOG_END(_modbus_reply);
        return -1;
    }

    offset = ctx->backend->header_length;
    slave = req[offset - 1];
    function = req[offset];
    address = (req[offset + 1] << 8) + req[offset + 2];

    sft.slave = slave;
    sft.function = function;
    sft.t_id = ctx->backend->prepare_response_tid(req, &req_length);

    /* MODBUS EXTENSION - begin */
    int is_listen;
    uint8_t trigger_time = 0;

    _MODBUS_LOG_PRINTF(_modbus_reply, "req_length=%d\n", req_length);
    _MODBUS_LOG_PRINTF(_modbus_reply, "function=%d\n", function);
    _MODBUS_LOG_PRINTF(_modbus_reply, "address=%d\n", address);
    switch(function)
    {
        case MODBUS_FC_LISTEN_COILS:
        case MODBUS_FC_LISTEN_DISCRETE_INPUTS:
        case MODBUS_FC_LISTEN_HOLDING_REGISTERS:
        case MODBUS_FC_LISTEN_INPUT_REGISTERS:
        {
            trigger_time = req[offset + 5];
            is_listen = TRUE;
            break;
        }

        default:
        {
            is_listen = FALSE;
            break;
        }
    }
    _MODBUS_LOG_PRINTF(_modbus_reply, "is_listen=%d\n", is_listen);
    _MODBUS_LOG_PRINTF(_modbus_reply, "trigger_time=%d\n", trigger_time);
    /* MODBUS EXTENSION - end */

    /* Data are flushed on illegal number of values errors. */
    switch (function) {
    case MODBUS_FC_LISTEN_COILS:
    case MODBUS_FC_LISTEN_DISCRETE_INPUTS:
    case MODBUS_FC_READ_COILS:
    case MODBUS_FC_READ_DISCRETE_INPUTS: {
        int is_input = (function == MODBUS_FC_READ_DISCRETE_INPUTS || function == MODBUS_FC_LISTEN_DISCRETE_INPUTS);
        int start_bits = is_input ? mb_mapping->start_input_bits : mb_mapping->start_bits;
        int nb_bits = is_input ? mb_mapping->nb_input_bits : mb_mapping->nb_bits;
        uint8_t *tab_bits = is_input ? mb_mapping->tab_input_bits : mb_mapping->tab_bits;
        const char * const name = is_input
                                ? (is_listen    ? "listen_input_bits"
                                                : "read_input_bits")
                                : (is_listen    ? "listen_bits"
                                                : "read_bits");
        int nb  = (req[offset + 3] << 8) + req[offset + 4];

        /* The mapping can be shifted to reduce memory consumption and it
           doesn't always start at address zero. */
        int mapping_address = address - start_bits;

        _MODBUS_LOG_PRINTF(_modbus_reply, "is_input=%d\n", is_input);
        _MODBUS_LOG_PRINTF(_modbus_reply, "start_bits=%d\n", start_bits);
        _MODBUS_LOG_PRINTF(_modbus_reply, "nb_bits=%d\n", nb_bits);
        _MODBUS_LOG_PRINTF(_modbus_reply, "name=%s\n", name);
        _MODBUS_LOG_PRINTF(_modbus_reply, "nb=%d\n", nb);

        if (nb < 1 || MODBUS_MAX_READ_BITS < nb) {
            rsp_length = response_exception(
                ctx, &sft, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp, TRUE,
                "Illegal nb of values %d in %s (max %d)\n",
                nb, name, MODBUS_MAX_READ_BITS);
        } else if (mapping_address < 0 || (mapping_address + nb) > nb_bits) {
            rsp_length = response_exception(
                ctx, &sft,
                MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp, FALSE,
                "Illegal data address 0x%0X in %s\n",
                mapping_address < 0 ? address : address + nb, name);
        } else {
            rsp_length = ctx->backend->build_response_basis(&sft, rsp);

            /* MODBUS EXTENSION - begin */
            if (is_listen)
            {
                /* the response to a listen request have a start address on 2nd and 3rd bytes */
                rsp[rsp_length++] = req[offset + 1];
                rsp[rsp_length++] = req[offset + 2];

                modbus_interrupt_manager_t * m = (is_input ? mb_mapping->interrupt_core->input_bits_triggers : mb_mapping->interrupt_core->coils_triggers);
                _MODBUS_LOG_PRINTF(_modbus_reply, "m=(%p)\n", m);
                _modbus_set_interrupt(m, address, nb, trigger_time);
            }
            /* MODBUS EXTENSION - end */

            rsp[rsp_length++] = (nb / 8) + ((nb % 8) ? 1 : 0);
            rsp_length = response_io_status(tab_bits, mapping_address, nb,
                                            rsp, rsp_length);
        }
    }
        break;
    case MODBUS_FC_LISTEN_HOLDING_REGISTERS:
    case MODBUS_FC_LISTEN_INPUT_REGISTERS:
    case MODBUS_FC_READ_HOLDING_REGISTERS:
    case MODBUS_FC_READ_INPUT_REGISTERS: {
        int is_input = (function == MODBUS_FC_READ_INPUT_REGISTERS|| function == MODBUS_FC_LISTEN_INPUT_REGISTERS);
        int start_registers = is_input ? mb_mapping->start_input_registers : mb_mapping->start_registers;
        int nb_registers = is_input ? mb_mapping->nb_input_registers : mb_mapping->nb_registers;
        uint16_t *tab_registers = is_input ? mb_mapping->tab_input_registers : mb_mapping->tab_registers;
        const char * const name = is_input ? (is_listen ? "listen_input_registers" : "read_input_registers") : (is_listen ? "listen_registers" : "read_registers");
        int nb = (req[offset + 3] << 8) + req[offset + 4];
        /* The mapping can be shifted to reduce memory consumption and it
           doesn't always start at address zero. */
        int mapping_address = address - start_registers;

        if (nb < 1 || MODBUS_MAX_READ_REGISTERS < nb) {
            rsp_length = response_exception(
                ctx, &sft, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp, TRUE,
                "Illegal nb of values %d in %s (max %d)\n",
                nb, name, MODBUS_MAX_READ_REGISTERS);
        } else if (mapping_address < 0 || (mapping_address + nb) > nb_registers) {
            rsp_length = response_exception(
                ctx, &sft, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp, FALSE,
                "Illegal data address 0x%0X in %s\n",
                mapping_address < 0 ? address : address + nb, name);
        } else {
            int i;

            rsp_length = ctx->backend->build_response_basis(&sft, rsp);

            /* MODBUS EXTENSION - begin */
            if (is_listen) {
                rsp[rsp_length++] = req[offset + 1];
                rsp[rsp_length++] = req[offset + 2];

                modbus_interrupt_manager_t * m = (is_input ? mb_mapping->interrupt_core->input_regs_triggers : mb_mapping->interrupt_core->holding_regs_triggers);

                _MODBUS_LOG_PRINTF(_modbus_reply, "m=(%p)\n", m);
                _modbus_set_interrupt(m, mapping_address, nb, trigger_time);
            }
            /* MODBUS EXTENSION - end */

            rsp[rsp_length++] = nb << 1;
            for (i = mapping_address; i < mapping_address + nb; i++) {
                rsp[rsp_length++] = tab_registers[i] >> 8;
                rsp[rsp_length++] = tab_registers[i] & 0xFF;
            }
        }
    }
        break;
    case MODBUS_FC_WRITE_SINGLE_COIL: {
        int mapping_address = address - mb_mapping->start_bits;

        if (mapping_address < 0 || mapping_address >= mb_mapping->nb_bits) {
            rsp_length = response_exception(
                ctx, &sft, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp, FALSE,
                "Illegal data address 0x%0X in write_bit\n",
                address);
        } else {
            int data = (req[offset + 3] << 8) + req[offset + 4];

            if (data == 0xFF00 || data == 0x0) {
                mb_mapping->tab_bits[mapping_address] = data ? ON : OFF;
                memcpy(rsp, req, req_length);
                rsp_length = req_length;
            } else {
                rsp_length = response_exception(
                    ctx, &sft,
                    MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp, FALSE,
                    "Illegal data value 0x%0X in write_bit request at address %0X\n",
                    data, address);
            }
        }
    }
        break;
    case MODBUS_FC_WRITE_SINGLE_REGISTER: {
        int mapping_address = address - mb_mapping->start_registers;

        if (mapping_address < 0 || mapping_address >= mb_mapping->nb_registers) {
            rsp_length = response_exception(
                ctx, &sft,
                MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp, FALSE,
                "Illegal data address 0x%0X in write_register\n",
                address);
        } else {
            int data = (req[offset + 3] << 8) + req[offset + 4];

            mb_mapping->tab_registers[mapping_address] = data;
            memcpy(rsp, req, req_length);
            rsp_length = req_length;
        }
    }
        break;
    case MODBUS_FC_WRITE_MULTIPLE_COILS: {
        int nb = (req[offset + 3] << 8) + req[offset + 4];
        int mapping_address = address - mb_mapping->start_bits;

        if (nb < 1 || MODBUS_MAX_WRITE_BITS < nb) {
            /* May be the indication has been truncated on reading because of
             * invalid address (eg. nb is 0 but the request contains values to
             * write) so it's necessary to flush. */
            rsp_length = response_exception(
                ctx, &sft, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp, TRUE,
                "Illegal number of values %d in write_bits (max %d)\n",
                nb, MODBUS_MAX_WRITE_BITS);
        } else if (mapping_address < 0 ||
                   (mapping_address + nb) > mb_mapping->nb_bits) {
            rsp_length = response_exception(
                ctx, &sft,
                MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp, FALSE,
                "Illegal data address 0x%0X in write_bits\n",
                mapping_address < 0 ? address : address + nb);
        } else {
            /* 6 = byte count */
            modbus_set_bits_from_bytes(mb_mapping->tab_bits, mapping_address, nb,
                                       &req[offset + 6]);

            rsp_length = ctx->backend->build_response_basis(&sft, rsp);
            /* 4 to copy the bit address (2) and the quantity of bits */
            memcpy(rsp + rsp_length, req + rsp_length, 4);
            rsp_length += 4;
        }
    }
        break;
    case MODBUS_FC_WRITE_MULTIPLE_REGISTERS: {
        int nb = (req[offset + 3] << 8) + req[offset + 4];
        int mapping_address = address - mb_mapping->start_registers;

        if (nb < 1 || MODBUS_MAX_WRITE_REGISTERS < nb) {
            rsp_length = response_exception(
                ctx, &sft, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp, TRUE,
                "Illegal number of values %d in write_registers (max %d)\n",
                nb, MODBUS_MAX_WRITE_REGISTERS);
        } else if (mapping_address < 0 ||
                   (mapping_address + nb) > mb_mapping->nb_registers) {
            rsp_length = response_exception(
                ctx, &sft, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp, FALSE,
                "Illegal data address 0x%0X in write_registers\n",
                mapping_address < 0 ? address : address + nb);
        } else {
            int i, j;
            for (i = mapping_address, j = 6; i < mapping_address + nb; i++, j += 2) {
                /* 6 and 7 = first value */
                mb_mapping->tab_registers[i] =
                    (req[offset + j] << 8) + req[offset + j + 1];
            }

            rsp_length = ctx->backend->build_response_basis(&sft, rsp);
            /* 4 to copy the address (2) and the no. of registers */
            memcpy(rsp + rsp_length, req + rsp_length, 4);
            rsp_length += 4;
        }
    }
        break;
    case MODBUS_FC_REPORT_SLAVE_ID: {
        int str_len;
        int byte_count_pos;

        rsp_length = ctx->backend->build_response_basis(&sft, rsp);
        /* Skip byte count for now */
        byte_count_pos = rsp_length++;
        rsp[rsp_length++] = _REPORT_SLAVE_ID;
        /* Run indicator status to ON */
        rsp[rsp_length++] = 0xFF;
        /* LMB + length of LIBMODBUS_VERSION_STRING */
        str_len = 3 + strlen(LIBMODBUS_VERSION_STRING);
        memcpy(rsp + rsp_length, "LMB" LIBMODBUS_VERSION_STRING, str_len);
        rsp_length += str_len;
        rsp[byte_count_pos] = rsp_length - byte_count_pos - 1;
    }
        break;
    case MODBUS_FC_READ_EXCEPTION_STATUS:
        if (ctx->debug) {
            fprintf(stderr, "FIXME Not implemented\n");
        }
        errno = ENOPROTOOPT;
        _MODBUS_LOG_END(_modbus_reply);
        return -1;
        break;
    case MODBUS_FC_MASK_WRITE_REGISTER: {
        int mapping_address = address - mb_mapping->start_registers;

        if (mapping_address < 0 || mapping_address >= mb_mapping->nb_registers) {
            rsp_length = response_exception(
                ctx, &sft, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp, FALSE,
                "Illegal data address 0x%0X in write_register\n",
                address);
        } else {
            uint16_t data = mb_mapping->tab_registers[mapping_address];
            uint16_t and = (req[offset + 3] << 8) + req[offset + 4];
            uint16_t or = (req[offset + 5] << 8) + req[offset + 6];

            data = (data & and) | (or & (~and));
            mb_mapping->tab_registers[mapping_address] = data;
            memcpy(rsp, req, req_length);
            rsp_length = req_length;
        }
    }
        break;
    case MODBUS_FC_WRITE_AND_READ_REGISTERS: {
        int nb = (req[offset + 3] << 8) + req[offset + 4];
        uint16_t address_write = (req[offset + 5] << 8) + req[offset + 6];
        int nb_write = (req[offset + 7] << 8) + req[offset + 8];
        int nb_write_bytes = req[offset + 9];
        int mapping_address = address - mb_mapping->start_registers;
        int mapping_address_write = address_write - mb_mapping->start_registers;

        if (nb_write < 1 || MODBUS_MAX_WR_WRITE_REGISTERS < nb_write ||
            nb < 1 || MODBUS_MAX_WR_READ_REGISTERS < nb ||
            nb_write_bytes != nb_write * 2) {
            rsp_length = response_exception(
                ctx, &sft, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp, TRUE,
                "Illegal nb of values (W%d, R%d) in write_and_read_registers (max W%d, R%d)\n",
                nb_write, nb, MODBUS_MAX_WR_WRITE_REGISTERS, MODBUS_MAX_WR_READ_REGISTERS);
        } else if (mapping_address < 0 ||
                   (mapping_address + nb) > mb_mapping->nb_registers ||
                   mapping_address < 0 ||
                   (mapping_address_write + nb_write) > mb_mapping->nb_registers) {
            rsp_length = response_exception(
                ctx, &sft, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp, FALSE,
                "Illegal data read address 0x%0X or write address 0x%0X write_and_read_registers\n",
                mapping_address < 0 ? address : address + nb,
                mapping_address_write < 0 ? address_write : address_write + nb_write);
        } else {
            int i, j;
            rsp_length = ctx->backend->build_response_basis(&sft, rsp);
            rsp[rsp_length++] = nb << 1;

            /* Write first.
               10 and 11 are the offset of the first values to write */
            for (i = mapping_address_write, j = 10;
                 i < mapping_address_write + nb_write; i++, j += 2) {
                mb_mapping->tab_registers[i] =
                    (req[offset + j] << 8) + req[offset + j + 1];
            }

            /* and read the data for the response */
            for (i = mapping_address; i < mapping_address + nb; i++) {
                rsp[rsp_length++] = mb_mapping->tab_registers[i] >> 8;
                rsp[rsp_length++] = mb_mapping->tab_registers[i] & 0xFF;
            }
        }
    }
        break;

    default:
        rsp_length = response_exception(
            ctx, &sft, MODBUS_EXCEPTION_ILLEGAL_FUNCTION, rsp, TRUE,
            "Unknown Modbus function code: 0x%0X\n", function);
        break;
    }

    _MODBUS_LOG_END(_modbus_reply);
    /* Suppress any responses when the request was a broadcast */
    return (slave == MODBUS_BROADCAST_ADDRESS) ? 0 : send_msg(ctx, rsp, rsp_length);
}

int modbus_reply_exception(modbus_t *ctx, const uint8_t *req,
                           unsigned int exception_code)
{
    int offset;
    int slave;
    int function;
    uint8_t rsp[MAX_MESSAGE_LENGTH];
    int rsp_length;
    int dummy_length = 99;
    sft_t sft;

    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    offset = ctx->backend->header_length;
    slave = req[offset - 1];
    function = req[offset];

    sft.slave = slave;
    sft.function = function + 0x80;;
    sft.t_id = ctx->backend->prepare_response_tid(req, &dummy_length);
    rsp_length = ctx->backend->build_response_basis(&sft, rsp);

    /* Positive exception code */
    if (exception_code < MODBUS_EXCEPTION_MAX) {
        rsp[rsp_length++] = exception_code;
        return send_msg(ctx, rsp, rsp_length);
    } else {
        errno = EINVAL;
        return -1;
    }
}

/* Reads IO status */
static int read_io_status(modbus_t *ctx, int function,
                          int addr, int nb, uint8_t *dest)
{
    int rc;
    int req_length;

    uint8_t req[_MIN_REQ_LENGTH];
    uint8_t rsp[MAX_MESSAGE_LENGTH];

    req_length = ctx->backend->build_request_basis(ctx, function, addr, nb, req);

    rc = send_msg(ctx, req, req_length);
    if (rc > 0) {
        int i, temp, bit;
        int pos = 0;
        int offset;
        int offset_end;

        /*  MODBUS EXTENSION:
            if the interrupt core is setted ...*/
        if (ctx->interrupt_core != NULL)
        {
            /* discard the last received response message (if it exists...). */
            ctx->interrupt_core->last_response_message.size = 0;

            _receive_interrupts_and_last_rsp_msg(ctx, MSG_CONFIRMATION);

            /* if a response message was received, then use it. */
            if (ctx->interrupt_core->last_response_message.size > 0)
            {
                memcpy(rsp, ctx->interrupt_core->last_response_message.data, ctx->interrupt_core->last_response_message.size);
                rc = ctx->interrupt_core->last_response_message.size;
            }

            /* error: nothing was received. */
            else
            {
                rc = -1;
            }
        }

        /* original mode */
        else
        {
            rc = _modbus_receive_msg(ctx, rsp, MSG_CONFIRMATION);
            if (rc == -1)
            {
                return -1;
            }
        }

        rc = check_confirmation(ctx, req, rsp, rc);

        if (rc == -1)
            return -1;

        offset = ctx->backend->header_length + 2;
        offset_end = offset + rc;
        for (i = offset; i < offset_end; i++) {
            /* Shift reg hi_byte to temp */
            temp = rsp[i];

            for (bit = 0x01; (bit & 0xff) && (pos < nb);) {
                dest[pos++] = (temp & bit) ? TRUE : FALSE;
                bit = bit << 1;
            }

        }
    }

    return rc;
}

/* Reads the boolean status of bits and sets the array elements
   in the destination to TRUE or FALSE (single bits). */
int modbus_read_bits(modbus_t *ctx, int addr, int nb, uint8_t *dest)
{
    int rc;

    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    if (nb > MODBUS_MAX_READ_BITS) {
        if (ctx->debug) {
            fprintf(stderr,
                    "ERROR Too many bits requested (%d > %d)\n",
                    nb, MODBUS_MAX_READ_BITS);
        }
        errno = EMBMDATA;
        return -1;
    }

    rc = read_io_status(ctx, MODBUS_FC_READ_COILS, addr, nb, dest);

    if (rc == -1)
        return -1;
    else
        return nb;
}


/* Same as modbus_read_bits but reads the remote device input table */
int modbus_read_input_bits(modbus_t *ctx, int addr, int nb, uint8_t *dest)
{
    int rc;

    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    if (nb > MODBUS_MAX_READ_BITS) {
        if (ctx->debug) {
            fprintf(stderr,
                    "ERROR Too many discrete inputs requested (%d > %d)\n",
                    nb, MODBUS_MAX_READ_BITS);
        }
        errno = EMBMDATA;
        return -1;
    }

    rc = read_io_status(ctx, MODBUS_FC_READ_DISCRETE_INPUTS, addr, nb, dest);

    if (rc == -1)
        return -1;
    else
        return nb;
}

/* Reads the data from a remove device and put that data into an array */
static int read_registers(modbus_t *ctx, int function, int addr, int nb,
                          uint16_t *dest)
{
    int rc;
    int req_length;
    uint8_t req[_MIN_REQ_LENGTH];
    uint8_t rsp[MAX_MESSAGE_LENGTH];

    if (nb > MODBUS_MAX_READ_REGISTERS) {
        if (ctx->debug) {
            fprintf(stderr,
                    "ERROR Too many registers requested (%d > %d)\n",
                    nb, MODBUS_MAX_READ_REGISTERS);
        }
        errno = EMBMDATA;
        return -1;
    }

    req_length = ctx->backend->build_request_basis(ctx, function, addr, nb, req);

    rc = send_msg(ctx, req, req_length);
    if (rc > 0) {
        int offset;
        int i;

        /*  MODBUS EXTENSION:
            if the interrupt core is setted ...*/
        if (ctx->interrupt_core != NULL)
        {
            /* discard the last received response message. */
            ctx->interrupt_core->last_response_message.size = 0;

            _receive_interrupts_and_last_rsp_msg(ctx, MSG_CONFIRMATION);

            if (ctx->interrupt_core->last_response_message.size > 0)
            {
                memcpy(rsp, ctx->interrupt_core->last_response_message.data, ctx->interrupt_core->last_response_message.size);
                rc = ctx->interrupt_core->last_response_message.size;
            }
            else
            {
                rc = -1;
            }
        }

        rc = _modbus_receive_msg(ctx, rsp, MSG_CONFIRMATION);
        if (rc == -1)
            return -1;

        rc = check_confirmation(ctx, req, rsp, rc);
        if (rc == -1)
            return -1;

        offset = ctx->backend->header_length;

        for (i = 0; i < rc; i++) {
            /* shift reg hi_byte to temp OR with lo_byte */
            dest[i] = (rsp[offset + 2 + (i << 1)] << 8) |
                rsp[offset + 3 + (i << 1)];
        }
    }

    return rc;
}

/* Reads the holding registers of remote device and put the data into an
   array */
int modbus_read_registers(modbus_t *ctx, int addr, int nb, uint16_t *dest)
{
    int status;

    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    if (nb > MODBUS_MAX_READ_REGISTERS) {
        if (ctx->debug) {
            fprintf(stderr,
                    "ERROR Too many registers requested (%d > %d)\n",
                    nb, MODBUS_MAX_READ_REGISTERS);
        }
        errno = EMBMDATA;
        return -1;
    }

    status = read_registers(ctx, MODBUS_FC_READ_HOLDING_REGISTERS,
                            addr, nb, dest);
    return status;
}

/* Reads the input registers of remote device and put the data into an array */
int modbus_read_input_registers(modbus_t *ctx, int addr, int nb,
                                uint16_t *dest)
{
    int status;

    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    if (nb > MODBUS_MAX_READ_REGISTERS) {
        fprintf(stderr,
                "ERROR Too many input registers requested (%d > %d)\n",
                nb, MODBUS_MAX_READ_REGISTERS);
        errno = EMBMDATA;
        return -1;
    }

    status = read_registers(ctx, MODBUS_FC_READ_INPUT_REGISTERS,
                            addr, nb, dest);

    return status;
}

/* Write a value to the specified register of the remote device.
   Used by write_bit and write_register */
static int write_single(modbus_t *ctx, int function, int addr, int value)
{
    int rc;
    int req_length;
    uint8_t req[_MIN_REQ_LENGTH];

    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    req_length = ctx->backend->build_request_basis(ctx, function, addr, value, req);

    rc = send_msg(ctx, req, req_length);
    if (rc > 0) {
        /* Used by write_bit and write_register */
        uint8_t rsp[MAX_MESSAGE_LENGTH];

        rc = _modbus_receive_msg(ctx, rsp, MSG_CONFIRMATION);
        if (rc == -1)
            return -1;

        rc = check_confirmation(ctx, req, rsp, rc);
    }

    return rc;
}

/* Turns ON or OFF a single bit of the remote device */
int modbus_write_bit(modbus_t *ctx, int addr, int status)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    return write_single(ctx, MODBUS_FC_WRITE_SINGLE_COIL, addr,
                        status ? 0xFF00 : 0);
}

/* Writes a value in one register of the remote device */
int modbus_write_register(modbus_t *ctx, int addr, int value)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    return write_single(ctx, MODBUS_FC_WRITE_SINGLE_REGISTER, addr, value);
}

/* Write the bits of the array in the remote device */
int modbus_write_bits(modbus_t *ctx, int addr, int nb, const uint8_t *src)
{
    int rc;
    int i;
    int byte_count;
    int req_length;
    int bit_check = 0;
    int pos = 0;
    uint8_t req[MAX_MESSAGE_LENGTH];

    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    if (nb > MODBUS_MAX_WRITE_BITS) {
        if (ctx->debug) {
            fprintf(stderr, "ERROR Writing too many bits (%d > %d)\n",
                    nb, MODBUS_MAX_WRITE_BITS);
        }
        errno = EMBMDATA;
        return -1;
    }

    req_length = ctx->backend->build_request_basis(ctx,
                                                   MODBUS_FC_WRITE_MULTIPLE_COILS,
                                                   addr, nb, req);
    byte_count = (nb / 8) + ((nb % 8) ? 1 : 0);
    req[req_length++] = byte_count;

    for (i = 0; i < byte_count; i++) {
        int bit;

        bit = 0x01;
        req[req_length] = 0;

        while ((bit & 0xFF) && (bit_check++ < nb)) {
            if (src[pos++])
                req[req_length] |= bit;
            else
                req[req_length] &=~ bit;

            bit = bit << 1;
        }
        req_length++;
    }

    rc = send_msg(ctx, req, req_length);
    if (rc > 0) {
        uint8_t rsp[MAX_MESSAGE_LENGTH];

        rc = _modbus_receive_msg(ctx, rsp, MSG_CONFIRMATION);
        if (rc == -1)
            return -1;

        rc = check_confirmation(ctx, req, rsp, rc);
    }


    return rc;
}

/* Write the values from the array to the registers of the remote device */
int modbus_write_registers(modbus_t *ctx, int addr, int nb, const uint16_t *src)
{
    int rc;
    int i;
    int req_length;
    int byte_count;
    uint8_t req[MAX_MESSAGE_LENGTH];

    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    if (nb > MODBUS_MAX_WRITE_REGISTERS) {
        if (ctx->debug) {
            fprintf(stderr,
                    "ERROR Trying to write to too many registers (%d > %d)\n",
                    nb, MODBUS_MAX_WRITE_REGISTERS);
        }
        errno = EMBMDATA;
        return -1;
    }

    req_length = ctx->backend->build_request_basis(ctx,
                                                   MODBUS_FC_WRITE_MULTIPLE_REGISTERS,
                                                   addr, nb, req);
    byte_count = nb * 2;
    req[req_length++] = byte_count;

    for (i = 0; i < nb; i++) {
        req[req_length++] = src[i] >> 8;
        req[req_length++] = src[i] & 0x00FF;
    }

    rc = send_msg(ctx, req, req_length);
    if (rc > 0) {
        uint8_t rsp[MAX_MESSAGE_LENGTH];

        rc = _modbus_receive_msg(ctx, rsp, MSG_CONFIRMATION);
        if (rc == -1)
            return -1;

        rc = check_confirmation(ctx, req, rsp, rc);
    }

    return rc;
}

int modbus_mask_write_register(modbus_t *ctx, int addr, uint16_t and_mask, uint16_t or_mask)
{
    int rc;
    int req_length;
    /* The request length can not exceed _MIN_REQ_LENGTH - 2 and 4 bytes to
     * store the masks. The ugly substraction is there to remove the 'nb' value
     * (2 bytes) which is not used. */
    uint8_t req[_MIN_REQ_LENGTH + 2];

    req_length = ctx->backend->build_request_basis(ctx,
                                                   MODBUS_FC_MASK_WRITE_REGISTER,
                                                   addr, 0, req);

    /* HACKISH, count is not used */
    req_length -= 2;

    req[req_length++] = and_mask >> 8;
    req[req_length++] = and_mask & 0x00ff;
    req[req_length++] = or_mask >> 8;
    req[req_length++] = or_mask & 0x00ff;

    rc = send_msg(ctx, req, req_length);
    if (rc > 0) {
        /* Used by write_bit and write_register */
        uint8_t rsp[MAX_MESSAGE_LENGTH];

        rc = _modbus_receive_msg(ctx, rsp, MSG_CONFIRMATION);
        if (rc == -1)
            return -1;

        rc = check_confirmation(ctx, req, rsp, rc);
    }

    return rc;
}

/* Write multiple registers from src array to remote device and read multiple
   registers from remote device to dest array. */
int modbus_write_and_read_registers(modbus_t *ctx,
                                    int write_addr, int write_nb,
                                    const uint16_t *src,
                                    int read_addr, int read_nb,
                                    uint16_t *dest)

{
    int rc;
    int req_length;
    int i;
    int byte_count;
    uint8_t req[MAX_MESSAGE_LENGTH];
    uint8_t rsp[MAX_MESSAGE_LENGTH];

    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    if (write_nb > MODBUS_MAX_WR_WRITE_REGISTERS) {
        if (ctx->debug) {
            fprintf(stderr,
                    "ERROR Too many registers to write (%d > %d)\n",
                    write_nb, MODBUS_MAX_WR_WRITE_REGISTERS);
        }
        errno = EMBMDATA;
        return -1;
    }

    if (read_nb > MODBUS_MAX_WR_READ_REGISTERS) {
        if (ctx->debug) {
            fprintf(stderr,
                    "ERROR Too many registers requested (%d > %d)\n",
                    read_nb, MODBUS_MAX_WR_READ_REGISTERS);
        }
        errno = EMBMDATA;
        return -1;
    }
    req_length = ctx->backend->build_request_basis(ctx,
                                                   MODBUS_FC_WRITE_AND_READ_REGISTERS,
                                                   read_addr, read_nb, req);

    req[req_length++] = write_addr >> 8;
    req[req_length++] = write_addr & 0x00ff;
    req[req_length++] = write_nb >> 8;
    req[req_length++] = write_nb & 0x00ff;
    byte_count = write_nb * 2;
    req[req_length++] = byte_count;

    for (i = 0; i < write_nb; i++) {
        req[req_length++] = src[i] >> 8;
        req[req_length++] = src[i] & 0x00FF;
    }

    rc = send_msg(ctx, req, req_length);
    if (rc > 0) {
        int offset;

        rc = _modbus_receive_msg(ctx, rsp, MSG_CONFIRMATION);
        if (rc == -1)
            return -1;

        rc = check_confirmation(ctx, req, rsp, rc);
        if (rc == -1)
            return -1;

        offset = ctx->backend->header_length;
        for (i = 0; i < rc; i++) {
            /* shift reg hi_byte to temp OR with lo_byte */
            dest[i] = (rsp[offset + 2 + (i << 1)] << 8) |
                rsp[offset + 3 + (i << 1)];
        }
    }

    return rc;
}

/* Send a request to get the slave ID of the device (only available in serial
   communication). */
int modbus_report_slave_id(modbus_t *ctx, int max_dest, uint8_t *dest)
{
    int rc;
    int req_length;
    uint8_t req[_MIN_REQ_LENGTH];

    if (ctx == NULL || max_dest <= 0) {
        errno = EINVAL;
        return -1;
    }

    req_length = ctx->backend->build_request_basis(ctx, MODBUS_FC_REPORT_SLAVE_ID,
                                                   0, 0, req);

    /* HACKISH, addr and count are not used */
    req_length -= 4;

    rc = send_msg(ctx, req, req_length);
    if (rc > 0) {
        int i;
        int offset;
        uint8_t rsp[MAX_MESSAGE_LENGTH];

        rc = _modbus_receive_msg(ctx, rsp, MSG_CONFIRMATION);
        if (rc == -1)
            return -1;

        rc = check_confirmation(ctx, req, rsp, rc);
        if (rc == -1)
            return -1;

        offset = ctx->backend->header_length + 2;

        /* Byte count, slave id, run indicator status and
           additional data. Truncate copy to max_dest. */
        for (i=0; i < rc && i < max_dest; i++) {
            dest[i] = rsp[offset + i];
        }
    }

    return rc;
}

void _modbus_init_common(modbus_t *ctx)
{
    /* Slave and socket are initialized to -1 */
    ctx->slave = -1;
    ctx->s = -1;

    ctx->debug = FALSE;
    ctx->error_recovery = MODBUS_ERROR_RECOVERY_NONE;

    ctx->response_timeout.tv_sec = 0;
    ctx->response_timeout.tv_usec = _RESPONSE_TIMEOUT;

    ctx->byte_timeout.tv_sec = 0;
    ctx->byte_timeout.tv_usec = _BYTE_TIMEOUT;

    ctx->interrupt_core = NULL;             // MODBUS EXTENSION: no interruption core by default.
}

/* Define the slave number */
int modbus_set_slave(modbus_t *ctx, int slave)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    return ctx->backend->set_slave(ctx, slave);
}

int modbus_set_error_recovery(modbus_t *ctx,
                              modbus_error_recovery_mode error_recovery)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    /* The type of modbus_error_recovery_mode is unsigned enum */
    ctx->error_recovery = (uint8_t) error_recovery;
    return 0;
}

int modbus_set_socket(modbus_t *ctx, int s)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    ctx->s = s;
    return 0;
}

int modbus_get_socket(modbus_t *ctx)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    return ctx->s;
}

/* Get the timeout interval used to wait for a response */
int modbus_get_response_timeout(modbus_t *ctx, uint32_t *to_sec, uint32_t *to_usec)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    *to_sec = ctx->response_timeout.tv_sec;
    *to_usec = ctx->response_timeout.tv_usec;
    return 0;
}

int modbus_set_response_timeout(modbus_t *ctx, uint32_t to_sec, uint32_t to_usec)
{
    if (ctx == NULL ||
        (to_sec == 0 && to_usec == 0) || to_usec > 999999) {
        errno = EINVAL;
        return -1;
    }

    ctx->response_timeout.tv_sec = to_sec;
    ctx->response_timeout.tv_usec = to_usec;
    return 0;
}

/* Get the timeout interval between two consecutive bytes of a message */
int modbus_get_byte_timeout(modbus_t *ctx, uint32_t *to_sec, uint32_t *to_usec)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    *to_sec = ctx->byte_timeout.tv_sec;
    *to_usec = ctx->byte_timeout.tv_usec;
    return 0;
}

int modbus_set_byte_timeout(modbus_t *ctx, uint32_t to_sec, uint32_t to_usec)
{
    /* Byte timeout can be disabled when both values are zero */
    if (ctx == NULL || to_usec > 999999) {
        errno = EINVAL;
        return -1;
    }

    ctx->byte_timeout.tv_sec = to_sec;
    ctx->byte_timeout.tv_usec = to_usec;
    return 0;
}

int modbus_get_header_length(modbus_t *ctx)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    return ctx->backend->header_length;
}

int modbus_connect(modbus_t *ctx)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    return ctx->backend->connect(ctx);
}

void modbus_close(modbus_t *ctx)
{
    if (ctx == NULL)
        return;

    ctx->backend->close(ctx);
}

void modbus_free(modbus_t *ctx)
{
    if (ctx == NULL)
        return;

    /* MODBUS EXTENSION - begin */
    _modbus_ext_free_interrupt_core(ctx->interrupt_core);
    /* MODBUS EXTENSION - end */

    ctx->backend->free(ctx);
}

int modbus_set_debug(modbus_t *ctx, int flag)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    ctx->debug = flag;
    return 0;
}

/* Allocates 4 arrays to store bits, input bits, registers and inputs
   registers. The pointers are stored in modbus_mapping structure.

   The modbus_mapping_new_ranges() function shall return the new allocated
   structure if successful. Otherwise it shall return NULL and set errno to
   ENOMEM. */
modbus_mapping_t* modbus_mapping_new_start_address(
    unsigned int start_bits, unsigned int nb_bits,
    unsigned int start_input_bits, unsigned int nb_input_bits,
    unsigned int start_registers, unsigned int nb_registers,
    unsigned int start_input_registers, unsigned int nb_input_registers)
{
    modbus_mapping_t *mb_mapping;

    mb_mapping = (modbus_mapping_t *)malloc(sizeof(modbus_mapping_t));
    if (mb_mapping == NULL) {
        return NULL;
    }

    /* 0X */
    mb_mapping->nb_bits = nb_bits;
    mb_mapping->start_bits = start_bits;
    if (nb_bits == 0) {
        mb_mapping->tab_bits = NULL;
    } else {
        /* Negative number raises a POSIX error */
        mb_mapping->tab_bits =
            (uint8_t *) malloc(nb_bits * sizeof(uint8_t));
        if (mb_mapping->tab_bits == NULL) {
            free(mb_mapping);
            return NULL;
        }

        memset(mb_mapping->tab_bits, 0, nb_bits * sizeof(uint8_t));
    }

    /* 1X */
    mb_mapping->nb_input_bits = nb_input_bits;
    mb_mapping->start_input_bits = start_input_bits;
    if (nb_input_bits == 0) {
        mb_mapping->tab_input_bits = NULL;
    } else {
        mb_mapping->tab_input_bits =
            (uint8_t *) malloc(nb_input_bits * sizeof(uint8_t));
        if (mb_mapping->tab_input_bits == NULL) {
            free(mb_mapping->tab_bits);
            free(mb_mapping);
            return NULL;
        }

        memset(mb_mapping->tab_input_bits, 0, nb_input_bits * sizeof(uint8_t));
    }

    /* 4X */
    mb_mapping->nb_registers = nb_registers;
    mb_mapping->start_registers = start_registers;
    if (nb_registers == 0) {
        mb_mapping->tab_registers = NULL;
    } else {
        mb_mapping->tab_registers =
            (uint16_t *) malloc(nb_registers * sizeof(uint16_t));
        if (mb_mapping->tab_registers == NULL) {
            free(mb_mapping->tab_input_bits);
            free(mb_mapping->tab_bits);
            free(mb_mapping);
            return NULL;
        }

        memset(mb_mapping->tab_registers, 0, nb_registers * sizeof(uint16_t));
    }

    /* 3X */
    mb_mapping->nb_input_registers = nb_input_registers;
    mb_mapping->start_input_registers = start_input_registers;
    if (nb_input_registers == 0) {
        mb_mapping->tab_input_registers = NULL;
    } else {
        mb_mapping->tab_input_registers =
            (uint16_t *) malloc(nb_input_registers * sizeof(uint16_t));
        if (mb_mapping->tab_input_registers == NULL) {
            free(mb_mapping->tab_registers);
            free(mb_mapping->tab_input_bits);
            free(mb_mapping->tab_bits);
            free(mb_mapping);
            return NULL;
        }
        memset(mb_mapping->tab_input_registers, 0,
               nb_input_registers * sizeof(uint16_t));
    }

    mb_mapping->interrupt_core = _modbus_interrupt_core_new(mb_mapping->start_bits, mb_mapping->nb_bits,
                                                            mb_mapping->start_input_bits, mb_mapping->nb_input_bits,
                                                            mb_mapping->start_registers, mb_mapping->nb_registers,
                                                            mb_mapping->start_input_registers,
                                                            mb_mapping->nb_input_registers);

    if (mb_mapping->interrupt_core == NULL)
    {
            free(mb_mapping->tab_input_registers);
            free(mb_mapping->tab_registers);
            free(mb_mapping->tab_input_bits);
            free(mb_mapping->tab_bits);
            free(mb_mapping);

            return NULL;
    }

    return mb_mapping;
}

modbus_mapping_t* modbus_mapping_new(int nb_bits, int nb_input_bits,
                                     int nb_registers, int nb_input_registers)
{
    return modbus_mapping_new_start_address(
        0, nb_bits, 0, nb_input_bits, 0, nb_registers, 0, nb_input_registers);
}

/* Frees the 4 arrays */
void modbus_mapping_free(modbus_mapping_t *mb_mapping)
{
    if (mb_mapping == NULL) {
        return;
    }

    _modbus_ext_free_interrupt_core(mb_mapping->interrupt_core);

    free(mb_mapping->tab_input_registers);
    free(mb_mapping->tab_registers);
    free(mb_mapping->tab_input_bits);
    free(mb_mapping->tab_bits);

    free(mb_mapping);
}

#ifndef HAVE_STRLCPY
/*
 * Function strlcpy was originally developed by
 * Todd C. Miller <Todd.Miller@courtesan.com> to simplify writing secure code.
 * See ftp://ftp.openbsd.org/pub/OpenBSD/src/lib/libc/string/strlcpy.3
 * for more information.
 *
 * Thank you Ulrich Drepper... not!
 *
 * Copy src to string dest of size dest_size.  At most dest_size-1 characters
 * will be copied.  Always NUL terminates (unless dest_size == 0).  Returns
 * strlen(src); if retval >= dest_size, truncation occurred.
 */
size_t strlcpy(char *dest, const char *src, size_t dest_size)
{
    register char *d = dest;
    register const char *s = src;
    register size_t n = dest_size;

    /* Copy as many bytes as will fit */
    if (n != 0 && --n != 0) {
        do {
            if ((*d++ = *s++) == 0)
                break;
        } while (--n != 0);
    }

    /* Not enough room in dest, add NUL and traverse rest of src */
    if (n == 0) {
        if (dest_size != 0)
            *d = '\0'; /* NUL-terminate dest */
        while (*s++)
            ;
    }

    return (s - src - 1); /* count does not include NUL */
}
#endif

/* MODBUS EXTENSION FUNCTIONS */

/* Trigger Time:
    4 bits (1->4) = 16 (0->15) => 0 a 15 unidades.
    2 bits (5->6) =  4 (0-> 3) => 0 a 3 múltiplos de 15 unidades (0, 15, 30, 45)
    2 bits (7->8) =  4 (0-> 3) => tipo da unidade:  00 -   1,6... ms (0 -> 100 ms)
                                                    01 -  16,6... ms (0 -> 1 s)
                                                    10 - 166,6... ms (0 -> 10 s)
                                                    11 -   1 s (0 -> 1 min)

*/

/* TODO: Trigger time isn't implemented yet.
enum trggrtm_unit_type {
    tut_1_6_MS      = 0b00,
    tut_16_6_MS     = 0b01,
    tut_166_6_MS    = 0b10,
    tut_1_S         = 0b11
};

static clock_t _trigger_time_to_clock(uint8_t trigger_time)
{
    uint8_t part1 = trigger_time & 0b00001111;
    uint8_t part2 = (trigger_time >> 4) & 0b0011;
    uint8_t type = (trigger_time >> 6) & 0b00000011;

    double mult;

    switch(type)
    {
        case tut_1_6_MS:
            mult = CLOCKS_PER_SEC / 40.0 / 15;
            break;
        case tut_16_6_MS:
            mult = CLOCKS_PER_SEC / 4.0 / 15;
            break;
        case tut_166_6_MS:
            mult = CLOCKS_PER_SEC / 2.5 / 15;
            break;
        case tut_1_S:
            mult = CLOCKS_PER_SEC;
            break;
    }

    return  (clock_t)((part1 + part2 * 15) * mult);

}
*/

static void _modbus_set_interrupt(modbus_interrupt_manager_t * manager, int addr, int nb,
                                    uint8_t trigger_time)
{
    _MODBUS_LOG_BEGIN(__modbus_set_interrupt);
    _MODBUS_LOG_PRINTF(__modbus_set_interrupt, "manager=(%p)\n",manager);
    _MODBUS_LOG_PRINTF(__modbus_set_interrupt, "addr=(%d)\n",addr);
    _MODBUS_LOG_PRINTF(__modbus_set_interrupt, "nb=(%d)\n",nb);
    _MODBUS_LOG_PRINTF(__modbus_set_interrupt, "trigger_time=(%d)\n",trigger_time);

    // TODO: confirmar campos addr e quantaty, se são utilizados e se estão recebendo os valores corretos.
    modbus_interrupt_trigger_t trigger;

    int index = _modbus_get_interrupt_trigger(manager, addr, &trigger);
    int r;

    _MODBUS_LOG_PRINTF(__modbus_set_interrupt, "index=(%d)\n",index);

    trigger.quantity = nb;
    trigger.pulled = FALSE;

    if (index >= 0)
    {
        assert(index <= (int)modbus_list_get_size(manager->triggers_list));
        r = modbus_list_set_element(manager->triggers_list, &trigger, index);
    }
    else
    {

        trigger.addr = addr;
        r = modbus_list_add_element(manager->triggers_list, &trigger);
    }

    assert(r != -1);

    _MODBUS_LOG_PRINTF(__modbus_set_interrupt, "r=(%d)\n",r);
    _MODBUS_LOG_END(__modbus_set_interrupt);
}

static void _receive_interrupts_and_last_rsp_msg(modbus_t * ctx, msg_type_t msg_type)
{
    _MODBUS_LOG_BEGIN(__receive_interrupts_and_last_rsp_msg);

    modbus_ext_msg_t msg;
    while( (int) (msg.size = _modbus_receive_msg(ctx, msg.data, msg_type)) != 0 )
    {
        _MODBUS_LOG_PRINTF(__receive_interrupts_and_last_rsp_msg, "Received message");
        _MODBUS_LOG_PRINTF(__receive_interrupts_and_last_rsp_msg, "(%d)", msg.data[ctx->backend->header_length]);
        switch (msg.data[ctx->backend->header_length])
        {
            case MODBUS_FC_LISTEN_COILS:
            case MODBUS_FC_LISTEN_DISCRETE_INPUTS:
            case MODBUS_FC_LISTEN_HOLDING_REGISTERS:
            case MODBUS_FC_LISTEN_INPUT_REGISTERS:
            {
                if (ctx->debug) {
                    fprintf(stderr, "A interrupt was received (%d).\n", msg.data[ctx->backend->header_length]);
                }
                MODBUS_LIST_ENQUEUE(ctx->interrupt_core->received_interrupt_queue, &msg);
                break;
            }

            default:
            {
                if (ctx->debug) {
                    fprintf(stderr, "A response message was received (%d).\n", msg.data[ctx->backend->header_length]);
                }
                /* store de last response message and return*/
                memcpy(&ctx->interrupt_core->last_response_message, &msg, sizeof(modbus_ext_msg_t));
                return;
            }
        }
    }

    /* There is no last response message. */
    ctx->interrupt_core->last_response_message.size = 0;

    _MODBUS_LOG_END(__receive_interrupts_and_last_rsp_msg);
}

static modbus_interrupt_manager_t * _modbus_get_interrupt_manager(modbus_interrupt_core_t * core, int func)
{
    modbus_interrupt_manager_t * manager;
    switch(func)
    {
        case MODBUS_FC_LISTEN_COILS:
        {
            manager = core->coils_triggers;
        }
        break;

        case MODBUS_FC_LISTEN_DISCRETE_INPUTS:
        {
            manager = core->input_bits_triggers;
        }
        break;

        case MODBUS_FC_LISTEN_HOLDING_REGISTERS:
        {
            manager = core->holding_regs_triggers;
        }
        break;

        case MODBUS_FC_LISTEN_INPUT_REGISTERS:
        {
            manager = core->input_regs_triggers;
        }
        break;

        default:
            return NULL;
    }

    return manager;
}

static int _modbus_get_interrupt_trigger(modbus_interrupt_manager_t * manager, uint16_t addr,
                                         modbus_interrupt_trigger_t * trigger)
{
    _MODBUS_LOG_BEGIN(__modbus_get_interrupt_trigger);
    _MODBUS_LOG_PRINTF(__modbus_get_interrupt_trigger,"manager={start=%d, nb=%d}\n", manager->start, manager->nb);
    _MODBUS_LOG_PRINTF(__modbus_get_interrupt_trigger,"addr=%d\n", addr);

    if (addr < manager->start || addr > manager->start + manager->nb)
    {
        _MODBUS_LOG_PRINTF(__modbus_get_interrupt_trigger,"addr < manager->start || addr > manager->start + manager->nb\n");
        _MODBUS_LOG_END(__modbus_get_interrupt_trigger);
        return -1;
    }

    int i,e;

    for(i=0, e=modbus_list_get_size(manager->triggers_list); i<e; i++)
    {
        modbus_interrupt_trigger_t tr;
        if (modbus_list_get_element(manager->triggers_list, &tr, i) && addr >= tr.addr && addr <= tr.addr + tr.quantity)
        {
            memcpy(trigger, &tr, sizeof(tr));

            _MODBUS_LOG_PRINTF(__modbus_get_interrupt_trigger,"i=%d -> trigger={addr=%d, quantity=%d, pulled=%d}\n",i, trigger->addr, trigger->quantity, trigger->pulled);
            _MODBUS_LOG_END(__modbus_get_interrupt_trigger);
            return i;
        }
    }

    _MODBUS_LOG_PRINTF(__modbus_get_interrupt_trigger,"No trigger founded!\n");
    _MODBUS_LOG_END(__modbus_get_interrupt_trigger);
    return -1;
}

static void _dispatch_interrupt(modbus_t * ctx, const modbus_ext_msg_t * msg)
{
    _MODBUS_LOG_BEGIN(__dispatch_interrupt);
    if(msg->size == 0)
    {
        fprintf(stderr, "_dispatch_interrupt - ERROR: empty message\n");
        _MODBUS_LOG_PRINTF(__dispatch_interrupt, "ERROR: empty message\n");
        _MODBUS_LOG_END(__dispatch_interrupt);
        return;
    }

    const int func = msg->data[ctx->backend->header_length];
    const int addr = ((uint16_t)msg->data[ctx->backend->header_length+1] << 1) + msg->data[ctx->backend->header_length+2];
    modbus_interrupt_manager_t * manager = _modbus_get_interrupt_manager(ctx->interrupt_core, func);
    if (manager == NULL)
    {
        fprintf(stderr, "ERROR: cannot allocate memory for a modbus_interrupt_manager_t\n");
        _MODBUS_LOG_PRINTF(__dispatch_interrupt, "ERROR: cannot allocate memory for a modbus_interrupt_manager_t\n");
        _MODBUS_LOG_END(__dispatch_interrupt);
        return;
    }

    modbus_interrupt_trigger_t trigger;
    if (_modbus_get_interrupt_trigger(manager, addr, &trigger) == -1)
    {
        fprintf(stderr, "_dispatch_interrupt - ERROR: cannot find trigger for address %d\n", addr);
        _MODBUS_LOG_PRINTF(__dispatch_interrupt, "_ERROR: cannot find trigger for address %d\n");
        _MODBUS_LOG_END(__dispatch_interrupt);
        return;
    }

    switch(func) {
        case MODBUS_FC_LISTEN_COILS:
        case MODBUS_FC_LISTEN_DISCRETE_INPUTS:
        {
            // TODO: testar!!!!!!
            uint8_t data[MODBUS_MAX_READ_BITS];
            int offset = ctx->backend->header_length + 4;
            int offset_end = msg->size;
            int pos = 0;
            int i;
            for (i = offset; i < offset_end; i++) {
                /* Shift reg hi_byte to temp */
                int temp = msg->data[i];
                int bit;

                for (bit = 0x01; (bit & 0xff) && (pos < trigger.quantity);) {
                    data[pos++] = (temp & bit) ? TRUE : FALSE;
                    bit = bit << 1;
                }
            }

            _MODBUS_LOG_PRINTF(__dispatch_interrupt, "Executing the listener... ");
            trigger.listener(ctx, func, addr, pos, data);
            break;
        }

        case MODBUS_FC_LISTEN_HOLDING_REGISTERS:
        case MODBUS_FC_LISTEN_INPUT_REGISTERS:
        {
            // TODO: testar!!!!
            uint16_t data[MODBUS_MAX_READ_REGISTERS];
            int offset = ctx->backend->header_length;
            int i, e = msg->data[offset+3] / 2;
            offset += 2;
            for (i = 0; i < e; i++) {
                offset += 2;
                /* shift reg hi_byte to temp OR with lo_byte */
                data[i] = MODBUS_GET_INT16_FROM_INT8(msg->data, offset);
            }

            _MODBUS_LOG_PRINTF(__dispatch_interrupt, "Executing the listener... ");
            trigger.listener(ctx, func, addr, i, data);
            break;
        }
    }

    _MODBUS_LOG_PRINTF(__dispatch_interrupt, "Done!");

    _MODBUS_LOG_END(__dispatch_interrupt);
}

static void _dequeue_and_dispatch_interrupts(modbus_t * ctx)
{
    _MODBUS_LOG_BEGIN(__dequeue_and_dispatch_interrupts);
    /*if (ctx->debug) {
        fprintf(stderr, "Start to dequeue and dispatch interrupts...\n");
    }*/
    modbus_ext_msg_t m;
    while( modbus_list_get_element(ctx->interrupt_core->received_interrupt_queue, &m, 0) )
    {
        _MODBUS_LOG_PRINTF(__dequeue_and_dispatch_interrupts, "_dispatch_interrupt(m)\n");
        _dispatch_interrupt(ctx, &m);
        MODBUS_LIST_DEQUEUE(ctx->interrupt_core->received_interrupt_queue);
    }
    /*if (ctx->debug) {
        fprintf(stderr, "Finish of dequeue and dispatch interrupts.\n");
    }*/
    _MODBUS_LOG_END(__dequeue_and_dispatch_interrupts);
}

void modbus_ext_try_listen_interrupt(modbus_t * ctx)
{
    /* dispatch immediately the enqueued interrupts */
    _dequeue_and_dispatch_interrupts(ctx);

    /* try to receive interrupts and dispatch then */
    _receive_interrupts_and_last_rsp_msg(ctx, MSG_INTERRUPTION);
    _dequeue_and_dispatch_interrupts(ctx);
}

static int _send_interrupt_request(modbus_t * ctx, int function, uint16_t addr,
                                    int nb, uint8_t trigger_time)
{
    int req_length;

    uint8_t req[MAX_MESSAGE_LENGTH];

    req_length = ctx->backend->build_request_basis(ctx, function, addr, nb, req);

    req[req_length] = trigger_time;
    req_length++;

    return send_msg(ctx, req, req_length);
}





static int _modbus_ext_remove_interrupt_trigger(modbus_t * ctx, int func, uint16_t addr)
{
    modbus_interrupt_manager_t * manager = _modbus_get_interrupt_manager(ctx->interrupt_core, func);

    modbus_interrupt_trigger_t trigger;
    int index = _modbus_get_interrupt_trigger(manager, addr, &trigger);

    if (index == -1)
    {
        return -1;
    }

    if (! modbus_list_remove_element(manager->triggers_list, index) )
    {
        return -1;
    }

    return _send_interrupt_request(ctx, func, addr, 0, 0);
}


/* add a trigger on the master */
static int _modbus_ext_add_interrupt_trigger(modbus_t * ctx, int func, uint16_t addr, int nb,
                                                uint8_t trigger_time, interrupt_listener_t listener)
{
    if (listener == NULL)
    {
        return _modbus_ext_remove_interrupt_trigger(ctx, func, addr);
    }

    modbus_interrupt_manager_t * manager;

    switch(func)
    {
        case MODBUS_FC_LISTEN_COILS:
        {
            manager = ctx->interrupt_core->coils_triggers;
        }
        break;

        case MODBUS_FC_LISTEN_DISCRETE_INPUTS:
        {
            manager = ctx->interrupt_core->input_bits_triggers;
        }
        break;

        case MODBUS_FC_LISTEN_HOLDING_REGISTERS:
        {
            manager = ctx->interrupt_core->holding_regs_triggers;
        }
        break;

        case MODBUS_FC_LISTEN_INPUT_REGISTERS:
        {
            manager = ctx->interrupt_core->input_regs_triggers;
        }
        break;

        default:
            _error_print(ctx,"_modbus_ext_add_interrupt_trigger: function is not a interrupt listener code.");
            return -1;
    }


    if ( addr < manager->start || addr > manager->start+manager->nb )
    {
        char msg[200];
        sprintf(msg, "_modbus_ext_add_interrupt_trigger: address %d is not allocated for function %d.", addr, func);
        _error_print(ctx, msg);
        return -1;
    }

    modbus_interrupt_trigger_t trigger;
    int index = _modbus_get_interrupt_trigger(manager, addr, &trigger);

    if ( index >= 0 )
    {
        /* delete the previous trigger */
        index = _modbus_ext_remove_interrupt_trigger(ctx, func, addr);
        assert(index != -1);
    }

    trigger.addr = addr;
    trigger.quantity = nb;
    trigger.listener = listener;

    index = modbus_list_add_element(manager->triggers_list, &trigger);
    assert(index != -1);

    return _send_interrupt_request(ctx, func, addr, nb, trigger_time);
}

modbus_interrupt_manager_t * _modbus_ext_new_interrupt_manager(unsigned int start, unsigned int nb)
{
    if (nb <= 0)
    {
        return NULL;
    }

    modbus_interrupt_manager_t * m = (modbus_interrupt_manager_t *) malloc(sizeof(modbus_interrupt_manager_t));
    if (m == NULL)
    {
        return NULL;
    }

    m->triggers_list = modbus_list_new(sizeof(modbus_interrupt_trigger_t), MODBUS_DEFAULT_LIST_CAPACITY);
    if (m->triggers_list == NULL)
    {
        free(m);
        return NULL;
    }

    m->start = start;
    m->nb = nb;

    return m;
}

modbus_interrupt_core_t * _modbus_interrupt_core_new(
    unsigned int start_bits, unsigned int nb_bits,
    unsigned int start_input_bits, unsigned int nb_input_bits,
    unsigned int start_registers, unsigned int nb_registers,
    unsigned int start_input_registers, unsigned int nb_input_registers)
{
    modbus_interrupt_core_t * interrupt_core = (modbus_interrupt_core_t *) malloc(sizeof(modbus_interrupt_core_t));
    if (interrupt_core == NULL)
    {
        return NULL;
    }

    memset(interrupt_core, 0, sizeof(modbus_interrupt_core_t));

    // coils / bits
    interrupt_core->coils_triggers = _modbus_ext_new_interrupt_manager(start_bits, nb_bits);
    if (interrupt_core->coils_triggers == NULL)
    {
        _modbus_ext_free_interrupt_core(interrupt_core);
        return NULL;
    }

    // input bits
    interrupt_core->input_bits_triggers = _modbus_ext_new_interrupt_manager(start_input_bits, nb_input_bits);
    if (interrupt_core->input_bits_triggers == NULL)
    {
        _modbus_ext_free_interrupt_core(interrupt_core);
        return NULL;
    }

    // holding registers
    interrupt_core->holding_regs_triggers = _modbus_ext_new_interrupt_manager(start_registers, nb_registers);
    if (interrupt_core->holding_regs_triggers == NULL)
    {
        _modbus_ext_free_interrupt_core(interrupt_core);
        return NULL;
    }

    // input registers
    interrupt_core->input_regs_triggers = _modbus_ext_new_interrupt_manager(start_input_registers, nb_input_registers);
    if (interrupt_core->input_regs_triggers == NULL)
    {
        _modbus_ext_free_interrupt_core(interrupt_core);
        return NULL;
    }

    interrupt_core->received_interrupt_queue = modbus_list_new(sizeof(modbus_ext_msg_t), 100);
    if (interrupt_core->received_interrupt_queue == NULL)
    {
        _modbus_ext_free_interrupt_core(interrupt_core);
        return NULL;
    }

    interrupt_core->last_response_message.size = 0;

    return interrupt_core;
}

void _modbus_ext_free_interrupt_core(modbus_interrupt_core_t * interrupt_core)
{
    if (interrupt_core != NULL)
    {
        if (interrupt_core->coils_triggers != NULL) _modbus_ext_free_interrupt_manager(interrupt_core->coils_triggers);
        if (interrupt_core->input_bits_triggers != NULL) _modbus_ext_free_interrupt_manager(interrupt_core->input_bits_triggers);
        if (interrupt_core->holding_regs_triggers != NULL) _modbus_ext_free_interrupt_manager(interrupt_core->holding_regs_triggers);
        if (interrupt_core->input_regs_triggers != NULL) _modbus_ext_free_interrupt_manager(interrupt_core->input_regs_triggers);
        if (interrupt_core->received_interrupt_queue != NULL) modbus_list_free(interrupt_core->received_interrupt_queue);

        free(interrupt_core);
    }
}


void _modbus_ext_free_interrupt_manager(modbus_interrupt_manager_t * m)
{
    if (m == NULL)
    {
        return;
    }

    modbus_list_free(m->triggers_list);
    free(m);
}

int modbus_ext_set_interrupt_core(modbus_t * ctx,
                                 int start_bits, int nb_bits,
                                 int start_input_bits, int nb_input_bits,
                                 int start_registers, int nb_registers,
                                 int start_input_registers, int nb_input_registers)
{
    ctx->interrupt_core = _modbus_interrupt_core_new(start_bits, nb_bits,
                                                     start_input_bits, nb_input_bits,
                                                     start_registers, nb_registers,
                                                     start_input_registers, nb_input_registers);
    return (ctx->interrupt_core != NULL);
}

void modbus_ext_unset_interrupt_core(modbus_t * ctx)
{
    _modbus_ext_free_interrupt_core(ctx->interrupt_core);

    ctx->interrupt_core = NULL;
}


int modbus_ext_listen_bits(modbus_t *ctx, int addr, int nb, uint8_t trigger_time, interrupt_listener_8bits_t listener)
{
    return _modbus_ext_add_interrupt_trigger(ctx, MODBUS_FC_LISTEN_COILS, addr, nb, trigger_time, (interrupt_listener_t)listener);
}

int modbus_ext_listen_input_bits(modbus_t *ctx, int addr, int nb, uint8_t trigger_time, interrupt_listener_8bits_t * listener)
{
    return _modbus_ext_add_interrupt_trigger(ctx, MODBUS_FC_LISTEN_DISCRETE_INPUTS, addr, nb, trigger_time, (interrupt_listener_t)listener);
}

int modbus_ext_listen_registers(modbus_t *ctx, int addr, int nb, uint8_t trigger_time, interrupt_listener_16bits_t * listener)
{
    return _modbus_ext_add_interrupt_trigger(ctx, MODBUS_FC_LISTEN_HOLDING_REGISTERS, addr, nb, trigger_time, (interrupt_listener_t)listener);
}

int modbus_ext_listen_input_registers(modbus_t *ctx, int addr, int nb, uint8_t trigger_time, interrupt_listener_16bits_t * listener)
{
    return _modbus_ext_add_interrupt_trigger(ctx, MODBUS_FC_LISTEN_INPUT_REGISTERS, addr, nb, trigger_time, (interrupt_listener_t)listener);
}

static int _mapping_set_io_bit(modbus_mapping_t * mb_map, int func, uint8_t * tab, int start, int addr, uint8_t value)
{
    _MODBUS_LOG_BEGIN(log_mapping_set_io_bit);
    _MODBUS_LOG_PRINTF(log_mapping_set_io_bit, "_mapping_set_io_bit(mb_map, func=%d, tab={...}, start=%d, addr=%d, value=%d)\n", func, start, addr, value);
    if (start > addr)
    {
        _MODBUS_LOG_PRINTF(log_mapping_set_io_bit, "ERROR: start > addr\n");
        _MODBUS_LOG_END(log_mapping_set_io_bit);
        return -1;
    }

    _MODBUS_LOG_PRINTF(log_mapping_set_io_bit, "tab[%d]=%d\n", addr-start, tab[addr-start]);

    modbus_interrupt_manager_t * m = _modbus_get_interrupt_manager(mb_map->interrupt_core, func);
    if (m == NULL) {
        _MODBUS_LOG_END(log_mapping_set_io_bit);
        return -1;
    }

    modbus_interrupt_trigger_t trigger;
    int index = _modbus_get_interrupt_trigger(m, addr, &trigger);
    int pulled = index != -1 && tab[addr-start] != value;

    _MODBUS_LOG_PRINTF(log_mapping_set_io_bit,"pulled=%d\n", pulled);

    if (pulled)
    {
        tab[addr-start] = value;
        trigger.pulled = TRUE;
        int r = modbus_list_set_element(m->triggers_list, &trigger, index);
        assert(r != -1);
        _MODBUS_LOG_PRINTF(log_mapping_set_io_bit,"m->triggers_list(%p)[%d]=trigger\n", m->triggers_list, index);
        _MODBUS_LOG_PRINTF(log_mapping_set_io_bit,"trigger.pulled=%d\n", trigger.pulled);
        _MODBUS_LOG_PRINTF(log_mapping_set_io_bit,"tab[%d] = %d\n", addr-start, tab[addr-start]);
    }

    _MODBUS_LOG_END(log_mapping_set_io_bit);
    return pulled;
}

static int _mapping_set_io_reg(modbus_mapping_t * mb_map, int func, uint16_t * tab, int start, int addr, uint16_t value)
{
    _MODBUS_LOG_BEGIN(log_mapping_set_io_reg);
    _MODBUS_LOG_PRINTF(log_mapping_set_io_reg, "log_mapping_set_io_reg(mb_map, func=%d, tab={...}, start=%d, addr=%d, value=%d)\n", func, start, addr, value);
    if (start > addr)
    {
        _MODBUS_LOG_PRINTF(log_mapping_set_io_reg, "ERROR: start > addr\n");
        _MODBUS_LOG_END(log_mapping_set_io_reg);
        return -1;
    }

    _MODBUS_LOG_PRINTF(log_mapping_set_io_reg, "tab[%d]=%d\n", addr-start, tab[addr-start]);

    modbus_interrupt_manager_t * m = _modbus_get_interrupt_manager(mb_map->interrupt_core, func);
    if (m == NULL) {
        _MODBUS_LOG_END(log_mapping_set_io_reg);
        return -1;
    }

    modbus_interrupt_trigger_t trigger;
    int index = _modbus_get_interrupt_trigger(m, addr, &trigger);
    int pulled = index != -1 && tab[addr-start] != value;

    _MODBUS_LOG_PRINTF(log_mapping_set_io_reg,"pulled=%d\n", pulled);

    if (pulled)
    {
        trigger.pulled = TRUE;
        tab[addr-start] = value;
        int r = modbus_list_set_element(m->triggers_list, &trigger, index);
        assert(r != -1);
        _MODBUS_LOG_PRINTF(log_mapping_set_io_reg,"m->triggers_list(%p)[%d]=trigger\n", m->triggers_list, index);
        _MODBUS_LOG_PRINTF(log_mapping_set_io_reg,"trigger.pulled=%d\n", trigger.pulled);
        _MODBUS_LOG_PRINTF(log_mapping_set_io_reg,"tab[%d] = %d\n", addr-start, tab[addr-start]);
    }

    _MODBUS_LOG_END(log_mapping_set_io_reg);
    return pulled;
}

int modbus_ext_mapping_set_bit(modbus_mapping_t * mb_map, int addr, int value)
{
	int r = _mapping_set_io_bit(mb_map, MODBUS_FC_LISTEN_COILS, mb_map->tab_bits, mb_map->start_bits, addr, value);
    return r;
}

int modbus_ext_mapping_set_input_bit(modbus_mapping_t * mb_map, int addr, int value)
{
    return _mapping_set_io_bit(mb_map, MODBUS_FC_LISTEN_DISCRETE_INPUTS, mb_map->tab_input_bits,
                               mb_map->start_input_bits, addr, value);
}

int modbus_ext_mapping_set_reg(modbus_mapping_t * mb_map, int addr, int value)
{
    return _mapping_set_io_reg(mb_map, MODBUS_FC_LISTEN_HOLDING_REGISTERS, mb_map->tab_registers,
                               mb_map->start_registers, addr, value);
}

int modbus_ext_mapping_set_input_reg(modbus_mapping_t * mb_map, int addr, int value)
{
    return _mapping_set_io_reg(mb_map, MODBUS_FC_LISTEN_INPUT_REGISTERS, mb_map->tab_input_registers,
                               mb_map->start_input_registers, addr, value);
}

static void _modbus_check_interrupt_manager(modbus_t * ctx, int func, modbus_mapping_t * map)
{
    _MODBUS_LOG_BEGIN(_check_interrupt_manager);
    _MODBUS_LOG_PRINTF(_check_interrupt_manager, "func=%d\n", func);
    modbus_interrupt_manager_t * m;

    switch(func)
    {
        case MODBUS_FC_LISTEN_COILS:
            m = map->interrupt_core->coils_triggers;
            break;
        case MODBUS_FC_LISTEN_DISCRETE_INPUTS:
            m = map->interrupt_core->input_bits_triggers;
            break;
        case MODBUS_FC_LISTEN_HOLDING_REGISTERS:
            m = map->interrupt_core->holding_regs_triggers;
            break;
        case MODBUS_FC_LISTEN_INPUT_REGISTERS:
            m = map->interrupt_core->input_regs_triggers;
            break;
        default:
            _error_print(ctx, "_modbus_check_interrupt_manager::func: invalid value.");
            _MODBUS_LOG_END(_check_interrupt_manager);
            return;
    }

    int i;
    int e = modbus_list_get_size(m->triggers_list);

    _MODBUS_LOG_PRINTF(_check_interrupt_manager, "----------\n");
    _MODBUS_LOG_PRINTF(_check_interrupt_manager, "triggers_list[0..%d]:\n", e-1);
    for(i=0; i<e; i++)
    {
        modbus_interrupt_trigger_t trigger;
        if ( modbus_list_get_element(m->triggers_list, &trigger, i) )
        {
            _MODBUS_LOG_PRINTF(_check_interrupt_manager, "\ttrigger(%d/%d).pulled=%s\n", i, e-1, trigger.pulled ? "TRUE" : "FALSE");
            if (trigger.pulled)
            {
                 _MODBUS_LOG_PRINTF(_check_interrupt_manager, "\t\tBuilding message...\n");
                uint8_t rsp[MAX_MESSAGE_LENGTH];
                sft_t sft;

                sft.t_id = 0;
                sft.function = func;
                sft.slave = ctx->slave;

                int rsp_length = ctx->backend->build_response_basis(&sft, rsp);
                int offset = ctx->backend->header_length;

                rsp[offset++] = (uint8_t)func;
                rsp[offset++] = MODBUS_GET_HIGH_BYTE(trigger.addr);
                rsp[offset++] = MODBUS_GET_LOW_BYTE(trigger.addr);

                switch (func)
                {
                    case MODBUS_FC_LISTEN_COILS:
                    {
                        rsp[offset++] = trigger.quantity / 8 + (trigger.quantity % 8 ? 1 : 0);
                        rsp_length = response_io_status(map->tab_bits, trigger.addr - map->start_bits, rsp_length, rsp, offset);
                        break;
                    }
                    case MODBUS_FC_LISTEN_DISCRETE_INPUTS:
                    {
                        rsp[offset++] = trigger.quantity / 8 + (trigger.quantity % 8 ? 1 : 0);
                        rsp_length = response_io_status(map->tab_input_bits, trigger.addr - map->start_input_bits, rsp_length, rsp, offset);
                        break;
                    }
                    case MODBUS_FC_LISTEN_HOLDING_REGISTERS:
                    {
                        rsp_length = offset;
                        rsp[rsp_length++] = trigger.quantity << 1;
                        for (i = trigger.addr - map->start_registers; i < trigger.addr - map->start_registers + trigger.quantity; i++) {
                            rsp[rsp_length++] = map->tab_registers[i] >> 8;
                            rsp[rsp_length++] = map->tab_registers[i] & 0xFF;
                        }
                        //TODO: incorreto! Deve ser o tamanho completo da mensagem.
                        //rsp_length = trigger.quantity * 2;
                        break;
                    }
                    case MODBUS_FC_LISTEN_INPUT_REGISTERS:
                    {
                        _MODBUS_LOG_PRINTF(_check_interrupt_manager, "MODBUS_FC_LISTEN_INPUT_REGISTERS:");
                        rsp_length = offset;
                        rsp[rsp_length++] = trigger.quantity << 1;
                        for (i = trigger.addr - map->start_input_registers; i < trigger.addr - map->start_input_registers + trigger.quantity; i++)
                        {
                            rsp[rsp_length++] = map->tab_input_registers[i] >> 8;
                            _MODBUS_LOG_PRINTF(_check_interrupt_manager, " %02X", rsp[rsp_length-1]);
                            rsp[rsp_length++] = map->tab_input_registers[i] & 0xFF;
                            _MODBUS_LOG_PRINTF(_check_interrupt_manager, " %02X", rsp[rsp_length-1]);
                        }
                        _MODBUS_LOG_PRINTF(_check_interrupt_manager, "\n");

                        //TODO: incorreto! Deve ser o tamanho completo da mensagem.
                        //rsp_length = trigger.quantity * 2;
                        break;
                    }
                }

                 _MODBUS_LOG_PRINTF(_check_interrupt_manager, "\t\tSending (data length=%d)... ", rsp_length);

                int rc = send_msg(ctx, rsp, rsp_length);
                if (rc != -1) {
                    _MODBUS_LOG_PRINTF(_check_interrupt_manager, "OK! Data length: %d.", rc);
                    trigger.pulled = FALSE;
                    int r = modbus_list_set_element(m->triggers_list, &trigger, i);
                    assert(r != -1);
                }
                else {
                    _MODBUS_LOG_PRINTF(_check_interrupt_manager, "ERROR!");
                    _error_print(ctx, "ERROR: Failed to send the interrupt message.\n");
                }
            }
        }
    }

    _MODBUS_LOG_END(_check_interrupt_manager);
}


void modbus_ext_check_interrupt(modbus_t * ctx, modbus_mapping_t * map)
{
    _modbus_check_interrupt_manager(ctx, MODBUS_FC_LISTEN_COILS, map);
    _modbus_check_interrupt_manager(ctx, MODBUS_FC_LISTEN_DISCRETE_INPUTS, map);
    _modbus_check_interrupt_manager(ctx, MODBUS_FC_LISTEN_HOLDING_REGISTERS, map);
    _modbus_check_interrupt_manager(ctx, MODBUS_FC_LISTEN_INPUT_REGISTERS, map);
}

/* MODBUS EXTENSION: modbus_list_t */

static void _modbus_list_move_block(modbus_list_t * list, int dest, int src, int elements) {
    if (elements >= 0) {
        void * ptr_src  = (char *)list->data + (list->element_size * src);
        void * ptr_dest = (char *)list->data + (list->element_size * dest);
        size_t bytes = elements * list->element_size;
        memmove(ptr_dest, ptr_src, bytes);
    }
}

modbus_list_t * modbus_list_new(size_t element_size, size_t capacity)
{
    _MODBUS_LOG_BEGIN(log_list);
    _MODBUS_LOG_PRINTF(log_list, "modbus_list_new!\n\telement_size=%u\n", element_size);
    _MODBUS_LOG_PRINTF(log_list, "\tcapacity=%u\n", capacity);
    modbus_list_t * l = (modbus_list_t *) malloc(sizeof(modbus_list_t));
    if (l == NULL)
    {
        _MODBUS_LOG_PRINTF(log_list, "ERROR: Memory allocation for modbus_list_t failed!\n");
        _MODBUS_LOG_END(log_list);
        return NULL;
    }

    l->data = malloc(element_size * capacity);
    if (l->data == NULL)
    {
        _MODBUS_LOG_PRINTF(log_list, "ERROR: Memory allocation for data (%u) failed!\n", element_size * capacity);
        _MODBUS_LOG_END(log_list);
        free(l);
        return NULL;
    }

    l->capacity = capacity;
    l->element_size = element_size;
    l->begin = 0;
    l->end = 0;
    l->size = 0;

    _MODBUS_LOG_PRINTF(log_list, "modbus_list_t allocated in address %p!\n", l);
    _MODBUS_LOG_END(log_list);
    return l;
}

size_t modbus_list_get_size(const modbus_list_t * list)
{
    return list->size;
}

size_t modbus_list_get_capacity(const modbus_list_t * list)
{
    return list->capacity;
}

int modbus_list_reorganize(modbus_list_t * list)
{
    if (list->size > 0)
    {
        if (list->begin > 0)
        {
            if (list->end <= list->begin && list->end > 0)
            {
                void * new_data = malloc(list->end * list->element_size);
                if (new_data == NULL)
                {
                    return FALSE;
                }
                memcpy(new_data, list->data, list->end * list->element_size);

                size_t quantity = list->capacity - list->begin;
                _modbus_list_move_block(list, 0, list->begin, quantity);
                memcpy((char*)list->data + (quantity * list->element_size), new_data, list->end * list->element_size);

                free(new_data);
            }
            else
            {
                size_t quantity = (list->capacity + list->end) % list->capacity - list->begin;
                _modbus_list_move_block(list, 0, list->begin, quantity);
            }
        }
    }

    list->begin = 0;
    list->end = list->size % list->capacity;

    return TRUE;
}

int modbus_list_set_capacity(modbus_list_t * list, size_t new_capacity)
{
    _MODBUS_LOG_BEGIN(log_list);
    _MODBUS_LOG_PRINTF(log_list,"modbus_list_set_capacity!\n\tlist=%p\n", list);
    _MODBUS_LOG_PRINTF(log_list,"\tnew_capacity=%u\n", new_capacity);
    _MODBUS_LOG_PRINT_LIST(log_list,list);

    if (list->begin > 0 && list->begin + list->size > new_capacity)
    {
        _MODBUS_LOG_PRINTF(log_list,"Reorganizing elements... ");
        if (! modbus_list_reorganize(list) )
        {
            _MODBUS_LOG_PRINTF(log_list,"ERROR: Cannot reorganize list.\n");
            _MODBUS_LOG_END(log_list);
            return FALSE;
        }
        _MODBUS_LOG_PRINTF(log_list,"Ok!\n");
        _MODBUS_LOG_PRINT_LIST(log_list,list);
    }

    _MODBUS_LOG_PRINTF(log_list,"Call realloc(list->data, %lu) [new elements=%lu]... ",new_capacity * list->element_size, new_capacity);
    void * ptr = realloc(list->data, new_capacity * list->element_size);

    if (ptr == NULL)
    {
        _MODBUS_LOG_PRINTF(log_list,"ERROR: Cannot realloc memory.\n");
        _MODBUS_LOG_END(log_list);
        return FALSE;
    }

    _MODBUS_LOG_PRINTF(log_list,"Ok!\n");

    list->data = ptr;
    list->capacity = new_capacity;

    if (list->size > new_capacity)
    {
        _MODBUS_LOG_PRINTF(log_list,"new_capacity(%lu) is less than list->size(%lu), so: list->size=%lu\n",new_capacity, list->size, new_capacity);
        list->size = new_capacity;
    }

    list->end = (list->begin + list->size) % list->capacity;
    _MODBUS_LOG_PRINT_LIST(log_list,list);
    _MODBUS_LOG_END(log_list);
    return TRUE;
}

int modbus_list_add_element(modbus_list_t * list, const void * element)
{
    _MODBUS_LOG_BEGIN(log_list);
    _MODBUS_LOG_PRINTF(log_list,"modbus_list_add_element!\n\tlist=%p\n\telement=%d\n", list, *(int*)element);
    _MODBUS_LOG_PRINT_LIST(log_list,list);
    if (list->size == list->capacity) {
        _MODBUS_LOG_PRINTF(log_list, "Call modbus_list_set_capacity(list, %lu)\n", list->capacity + MODBUS_DEFAULT_LIST_CAPACITY);
        if (! modbus_list_set_capacity(list, list->capacity + MODBUS_DEFAULT_LIST_CAPACITY))
        {
            _MODBUS_LOG_PRINTF(log_list,"modbus_list_set_capacity() fails.\n");
            _MODBUS_LOG_END(log_list);
            return -1;
        }
        _MODBUS_LOG_PRINT_LIST(log_list, list);
    }

    void * dest = (char*)list->data + (list->element_size * list->end);
    memcpy(dest, element, list->element_size);

    list->end = (list->end + 1) % list->capacity;
    list->size++;
    _MODBUS_LOG_PRINT_LIST(log_list,list);
    _MODBUS_LOG_END(log_list);

    return list->size;
}

int modbus_list_insert_element(modbus_list_t * list, const void * element, unsigned int position)
{
    _MODBUS_LOG_BEGIN(log_list);
    _MODBUS_LOG_PRINTF(log_list,"modbus_list_insert_element!\n\tlist=%p\n\telement=%d\n\tposition=%d\n", list, *(int*)element, position);
    _MODBUS_LOG_PRINT_LIST(log_list,list);
    /*  If the postion is out of range or the position is the last one, then add the element at the tail */
    if (position < 0 || position >= list->size) {
        _MODBUS_LOG_PRINTF(log_list, "Call modbus_list_add_element(list,element)\n");
        int r = modbus_list_add_element(list, element);
        _MODBUS_LOG_PRINT_LIST(log_list,list);
        _MODBUS_LOG_END(log_list);
        return r;
    }

    if (list->size == list->capacity) {
        _MODBUS_LOG_PRINTF(log_list,"Call modbus_list_set_capacity(list, %lu)\n",list->capacity + MODBUS_DEFAULT_LIST_CAPACITY);
        if (! modbus_list_set_capacity(list, list->capacity + MODBUS_DEFAULT_LIST_CAPACITY) )
        {
            _MODBUS_LOG_PRINTF(log_list,"modbus_list_set_capacity() fails.\n");
            _MODBUS_LOG_END(log_list);
            return -1;
        }
    }

    /* Here, the list isn't full neither empty */

    unsigned int abs_pos;

    /* If the insertion is before the first ...*/
    if (position == 0)
    {
        /* Create a vacant position before the first. */
        list->begin = (list->capacity + list->begin - 1) % list->capacity;
        abs_pos = list->begin;
    }
    else
    {
        abs_pos = (list->capacity + list->begin + position) % list->capacity;

        size_t quantity;
        if (list->end < abs_pos)
        {
            /* move front forward */
            _modbus_list_move_block(list, 0, 1, list->end);
            /* move tail to front. */
            _modbus_list_move_block(list, list->capacity-1, 0, 1);
            quantity = list->capacity - abs_pos - 1;
        }
        else
        {
            quantity = list->end - abs_pos;
        }

        /* create the vacant position */
        _modbus_list_move_block(list, abs_pos+1, abs_pos, quantity);

        list->end = (list->end + 1) % list->capacity;
    }

    /* insert the element. */
    void * dest = (char *)list->data + (abs_pos * list->element_size);
    memcpy(dest, element, list->element_size);
    list->size++;

    _MODBUS_LOG_PRINT_LIST(log_list,list);
    _MODBUS_LOG_END(log_list);
    return list->size;
}


int modbus_list_remove_element(modbus_list_t * list, unsigned int position)
{
    _MODBUS_LOG_BEGIN(log_list);
    _MODBUS_LOG_PRINTF(log_list,"modbus_list_remove_element!\n", list);
    _MODBUS_LOG_PRINTF(log_list,"\tlist=%p\n", list);
    _MODBUS_LOG_PRINTF(log_list,"\tposition=%d\n", position);
    _MODBUS_LOG_PRINT_LIST(log_list,list);

    if (list->size == 0)
    {
        _MODBUS_LOG_END(log_list);
        return -1;
    }

    /* if the element is the first one, just dequeue it. */
    if (position <= 0)
    {
        list->begin = (list->begin+1) % list->capacity;
        list->size--;
        _MODBUS_LOG_PRINT_LIST(log_list,list);
        _MODBUS_LOG_END(log_list);
        return list->size;
    }

    /*  if the position is after the last one, then it is the last one.
        if the element is the last one, just pop it. */
    if (position >= list->size-1)
    {
        list->end = (list->capacity + list->end - 1) % list->capacity;
        list->size--;
        _MODBUS_LOG_PRINT_LIST(log_list,list);
        _MODBUS_LOG_END(log_list);
        return list->size;
    }

    /* if the element is neither the first nor the last one, then continue to the complex process. */

    int abs_pos = (int)( (list->begin + position)) % list->capacity;
    int quantity = ((int)(list->end <= list->begin) ? list->capacity : list->end) - abs_pos - 1;
    _MODBUS_LOG_PRINTF(log_list,"abs_pos=%d, quantity=%d\n", abs_pos, quantity);

    _modbus_list_move_block(list, abs_pos, abs_pos+1, quantity);

    _MODBUS_LOG_PRINT_LIST(log_list,list);

    if (list->end <= list->begin && list->end > 0)
    {
        _modbus_list_move_block(list, ((int)list->capacity-1), 0, 1);
        _modbus_list_move_block(list, 0, 1, ((int)list->end - 1));
    }

    list->end = (list->capacity + list->end - 1) % list->capacity;
    list->size--;

    _MODBUS_LOG_PRINT_LIST(log_list,list);
    _MODBUS_LOG_END(log_list);

    return list->size;
}

int modbus_list_set_element(modbus_list_t * list, const void * element, unsigned int position)
{
    /* TODO: implement this function replacing the element, without remove and insert it.*/
    return (modbus_list_remove_element(list, position) != -1 && modbus_list_insert_element(list, element, position) != -1);
}

size_t modbus_list_get_element_size(const modbus_list_t * list)
{
    return list->element_size;
}

int modbus_list_get_element(const modbus_list_t * list, void * element, unsigned int position)
{
    if (list->size == 0)
    {
        return FALSE;
    }

    if (position < 0 || position >= list->size)
    {
        position = list->size - 1;
    }

    unsigned int abs_pos = (list->begin + position) % list->capacity;
    void * src = (char *)list->data + (list->element_size * abs_pos);

    memcpy(element, src, list->element_size);

    return TRUE;
}

void modbus_list_clean(modbus_list_t * list)
{
    _MODBUS_LOG_BEGIN(log_list);
    _MODBUS_LOG_PRINTF(log_list,"modbus_list_clean!\n");
    _MODBUS_LOG_PRINTF(log_list,"\tlist=%p\n", list);
    list->begin = 0;
    list->end = 0;
    list->size = 0;
    _MODBUS_LOG_END(log_list);
}


void modbus_list_free(modbus_list_t * list)
{
    if (list != NULL)
    {
        free(list->data);
        free(list);
    }
}

#ifdef _MODBUS_LOG_
FILE * ___modbus_log_new(char * filename)
{
    FILE * f = fopen(filename, "a");
    if(f == NULL)
    {
        fprintf(stderr, "ERROR MODBUS_LOG: [%d] %s", errno, strerror(errno));
    }
    else
    {
        fprintf(f,"\n******************************** BEGIN LOG ********************************\n");
        //fflush(f);
    }
    return f;
}

int _MODBUS_LOG_PRINTF(FILE * log, char * fmt,...)
{
    if (log == NULL) {
        return EOF;
    }
    va_list args;

    va_start(args, fmt);

    int r = vfprintf(log, fmt, args );
    //fflush(log);

    va_end(args);
    return r;
}

void _MODBUS_LOG_PRINT_LIST(FILE * f, modbus_list_t * list)
{
    if (f == NULL) {
        return;
    }
    _MODBUS_LOG_PRINTF(f, "--- LIST --------------------------------------------------------------------\n");
    _MODBUS_LOG_PRINTF(f,"list->begin=%lu, list->end=%lu, list->size=%lu, list->capacity=%lu\n",
        list->begin, list->end, list->size, list->capacity);
    _MODBUS_LOG_PRINTF(f, "list->data={");
    unsigned int i;
    if(list->begin + list->size <= list->capacity)
    {
        for(i=list->begin; i<list->size; i++)
        {
            _MODBUS_LOG_PRINTF(f, "[%u]=%d ", i, ((char *)list->data + list->element_size * i));
        }
    }
    else
    {
        for(i=list->begin; i<list->capacity; i++)
        {
            _MODBUS_LOG_PRINTF(f, "[%u]=%d ", i, ((char *)list->data + list->element_size * i));
        }
        for(i=0; i<list->end; i++)
        {
            _MODBUS_LOG_PRINTF(f, "[%u]=%d ", i, ((char *)list->data + list->element_size * i));
        }
    }
    _MODBUS_LOG_PRINTF(f, "}\n");
    _MODBUS_LOG_PRINTF(f, "-----------------------------------------------------------------------------\n");
}

void _MODBUS_LOG_END(FILE * f)
{
    if (f == NULL) {
        return;
    }
    fprintf(f, "\n********************************  END LOG  ********************************\n");
    fclose(f);
}

#endif
