#include <stdio.h>
#include <memory.h>
#include <assert.h>
#include <ncurses.h>
#include <modbus/modbus.h>
#include <modbus/modbus-tcp.h>
#include "common.h"

static void listenerInputBits(modbus_t * ctx, int func, int addr, int nb, uint8_t * data);
static void listenerCoils(modbus_t * ctx, int func, int addr, int nb, uint8_t * data);
static void listenerInputRegs(modbus_t * ctx, int func, int addr, int nb, uint16_t * data);
static void listenerHoldRegs(modbus_t * ctx, int func, int addr, int nb, uint16_t * data);

static void process_alarms(modbus_t * ctx, ModbusAddress * address);
static void process_lighting(modbus_t * ctx, ModbusAddress * address);
static void process_changes(modbus_t * ctx, const ModbusAddress * bkp, const ModbusAddress * mb_address);

ModbusAddress mb_address;

int main(void)
{
    clrscr();
    printf("Creating connection to %s:%d... ", IP, PORTA);
    modbus_t * ctx = modbus_new_tcp(IP, PORTA);
    if (ctx == NULL)
    {
        fprintf(stderr, "Não foi possível alocar contexto.\n");
        return -1;
    }
    printf("Ok!\n");
    //modbus_set_debug(ctx, TRUE);

    printf("Connecting... ");
    if (modbus_connect(ctx) == -1)
    {
        fprintf(stderr, "Não foi possível conectar: %s:%d.\n", IP, PORTA);
        modbus_free(ctx);
        return -1;
    }
    printf("Ok!\n");

    printf("Allocating interrupt core... ");
    if (! modbus_ext_set_interrupt_core(ctx, 0, n_coils,
                                             0, n_input_bits,
                                             0, n_holding_registers,
                                             0, n_input_registers) )
    {
        fprintf(stderr, "Não foi possível alocar núcleo de interrupções.\n");
        modbus_free(ctx);
        return -1;
    }
    printf("Ok!\n");

    mb_address = EMPTY_MODBUS_ADDRESS;

    modbus_ext_listen_input_bits(ctx, 0, n_input_bits, TRIGGER_TIME, listenerInputBits);
    modbus_ext_listen_bits(ctx, 0, n_coils, TRIGGER_TIME, listenerCoils);
    modbus_ext_listen_input_registers(ctx, 0, n_input_registers, TRIGGER_TIME, listenerInputRegs);
    modbus_ext_listen_registers(ctx, 0, n_holding_registers, TRIGGER_TIME, listenerHoldRegs);

    for(;;)
    {
        modbus_ext_try_listen_interrupt(ctx);

        ModbusAddress bkp = mb_address;
        char k = menu("MASTER", &mb_address, FALSE);

        process_changes(ctx, &bkp, &mb_address);

        process_alarms(ctx, &mb_address);
        process_lighting(ctx, &mb_address);

        // ESC key was pressed ...
        if ( k == 27 )
        {
            break;
        }
    }

    modbus_free(ctx);

    return 0;
}

void process_alarms(modbus_t * ctx, ModbusAddress * mb_address)
{
    int buzzer;

    if( mb_address->coils[coil_move_alarm] && mb_address->ibits[ibit_movimento] )
    {
        buzzer = mb_address->holds[hold_buzzer] | MOVEMENT_ALARM_BIT;
    }
    else
    {
        buzzer = mb_address->holds[hold_buzzer] & ~MOVEMENT_ALARM_BIT;
    }


    if (mb_address->iregs[ireg_gases] >= 75)
    {
        buzzer |= FIRE_ALARM_BIT;
    }
    else
    {
        buzzer &=  ~FIRE_ALARM_BIT;
    }


    if (buzzer != mb_address->holds[hold_buzzer])
    {
        refresh_menu();
        mb_address->holds[hold_buzzer] = buzzer;
        if (modbus_write_register(ctx, hold_buzzer, buzzer) == -1)
        {
            fprintf(stderr, "modbus_write_register: error\n");
        }
    }
}

void process_lighting(modbus_t * ctx, ModbusAddress * mb_address)
{
    int led = (!mb_address->coils[coil_move_alarm] && mb_address->ibits[ibit_movimento]
               && mb_address->iregs[ireg_luminosidade] >= 75);

    if ( led != mb_address->coils[coil_led])
    {
        refresh_menu();
        mb_address->coils[coil_led] = led;
        if(modbus_write_bit(ctx, coil_led, led) == -1)
        {
            fprintf(stderr, "modbus_write_bit: error\n");
        }
    }
}

void listenerInputBits(modbus_t * ctx, int func, int addr, int nb, uint8_t * data)
{
    if (addr != 0)
    {
        fprintf(stderr, "listenerInputBits::addr: invalid value (%d).", addr);
        return;
    }
    if (func != MODBUS_FC_LISTEN_DISCRETE_INPUTS)
    {
        fprintf(stderr, "listenerInputBits::func: invalid value (%d).", func);
        return;
    }
    if (nb != n_input_bits)
    {
        fprintf(stderr, "listenerInputBits::nb: invalid value (%d).", nb);
        return;
    }

    fprintf(stderr, "data={");
    int i;
    for(i=0; i<nb; i++)
    {
        fprintf(stderr, "%02X ", data[i]);
    }
    fprintf(stderr, "}\n");

    memcpy(mb_address.ibits, data, nb);
    refresh_menu();
}
void listenerCoils(modbus_t * ctx, int func, int addr, int nb, uint8_t * data)
{
    if (addr != 0)
    {
        fprintf(stderr, "listenerCoils::addr: invalid value (%d).", addr);
        return;
    }
    if (func != MODBUS_FC_LISTEN_COILS)
    {
        fprintf(stderr, "listenerCoils::func: invalid value (%d).", func);
        return;
    }
    if (nb != n_coils)
    {
        fprintf(stderr, "listenerCoils::nb: invalid value (%d).", nb);
        return;
    }

    fprintf(stderr, "data={");
    int i;
    for(i=0; i<nb; i++)
    {
        fprintf(stderr, "%02X ", data[i]);
    }
    fprintf(stderr, "}\n");

    memcpy(mb_address.coils, data, nb);
    refresh_menu();
}
void listenerInputRegs(modbus_t * ctx, int func, int addr, int nb, uint16_t * data)
{
    if (addr != 0)
    {
        fprintf(stderr, "listenerInputRegs::addr: invalid value (%d).", addr);
        return;
    }
    if (func != MODBUS_FC_LISTEN_INPUT_REGISTERS)
    {
        fprintf(stderr, "listenerInputRegs::func: invalid value (%d).", func);
        return;
    }
    if (nb != n_input_registers)
    {
        fprintf(stderr, "listenerInputRegs::nb: invalid value (%d).", nb);
        return;
    }

    fprintf(stderr, "data={");
    int i;
    for(i=0; i<nb; i++)
    {
        fprintf(stderr, "%04X ", data[i]);
    }
    fprintf(stderr, "}\n");

    memcpy(mb_address.iregs, data, nb * 2);
    refresh_menu();
}
void listenerHoldRegs(modbus_t * ctx, int func, int addr, int nb, uint16_t * data)
{
    if (addr != 0)
    {
        fprintf(stderr, "listenerHoldRegs::addr: invalid value (%d).", addr);
        return;
    }
    if (func != MODBUS_FC_LISTEN_HOLDING_REGISTERS)
    {
        fprintf(stderr, "listenerHoldRegs::func: invalid value (%d).", func);
        return;
    }
    if (nb != n_holding_registers)
    {
        fprintf(stderr, "listenerHoldRegs::nb: invalid value (%d).", nb);
        return;
    }

    fprintf(stderr, "data={");
    int i;
    for(i=0; i<nb; i++)
    {
        fprintf(stderr, "%04X ", data[i]);
    }
    fprintf(stderr, "}\n");

    memcpy(mb_address.holds, data, nb * 2);
    refresh_menu();
}

static void process_changes(modbus_t * ctx, const ModbusAddress * bkp, const ModbusAddress * mb_address)
{
    int i;
    bool changed = false;
    for(i=0; i<n_coils && ! changed; i++)
    {
        changed = (bkp->coils[i] != mb_address->coils[i]);
    }
    if(changed)
    {
        modbus_write_bits(ctx, MODBUS_ADDRESS_FIRST_COILS, n_coils, mb_address->coils);
    }

    changed = false;
    for(i=0; i<n_holding_registers && ! changed; i++)
    {
        changed = (bkp->holds[i] != mb_address->holds[i]);
    }
    if(changed)
    {
        modbus_write_registers(ctx, MODBUS_ADDRESS_FIRST_HOLD_REGS, n_holding_registers, mb_address->holds);
    }
}
