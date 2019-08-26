#ifndef COMMON_INCLUDED
#define COMMON_INCLUDED

#include <modbus/modbus.h>
#include <stdbool.h>

#define IP "127.0.0.1"
#define PORTA 1502

#define TRIGGER_TIME ((uint8_t) 0)

#define BOOL_TO_STR(b) (b ? "TRUE" : "FALSE")
#define TOGGLE_BOOL(b) (b = !(uint8_t)b)
#define ADD_32BITS(v,i) (v = (uint16_t)((v + (uint16_t)i + 65536) % 65536))
#define MOVEMENT_ALARM_BIT  0b010
#define FIRE_ALARM_BIT      0b100

#define SHOW_ERRORF(fmt,...) ((mvprintw(15,0,"> "), printw(fmt,__VA_ARGS__)))
#define SHOW_ERROR(msg) SHOW_ERRORF("%s",msg)

#define MODBUS_ADDRESS_FIRST_INPUT_BITS 0
#define MODBUS_ADDRESS_FIRST_COILS      0
#define MODBUS_ADDRESS_FIRST_INPUT_REGS 0
#define MODBUS_ADDRESS_FIRST_HOLD_REGS  0

typedef enum {
    ibit_movimento = MODBUS_ADDRESS_FIRST_INPUT_BITS,
    n_input_bits
} InputBit;

typedef enum {
    coil_move_alarm = MODBUS_ADDRESS_FIRST_COILS,
    coil_led,
    n_coils
} Coil;

typedef enum {
    ireg_gases = MODBUS_ADDRESS_FIRST_INPUT_REGS,
    ireg_luminosidade,
    n_input_registers
} InputRegister;

typedef enum {
    hold_gases = MODBUS_ADDRESS_FIRST_HOLD_REGS,
    hold_luminosidade,
    hold_buzzer,
    n_holding_registers
} HoldingRegister;


typedef enum {
    at_ibits,
    at_coils,
    at_iregs,
    at_holds
} AddressType;

typedef struct {
    uint8_t coils[n_coils];
    uint8_t ibits[n_input_bits];
    uint16_t holds[n_holding_registers];
    uint16_t iregs[n_input_registers];
} ModbusAddress;

extern const ModbusAddress EMPTY_MODBUS_ADDRESS;

char menu(const char title[], ModbusAddress * addr, bool modify_input);
void refresh_menu(void);

int kbhit(void);
void clrscr(void);


#endif // COMMON_INCLUDED
