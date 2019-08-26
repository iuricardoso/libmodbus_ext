#include <stdio.h>
#include <memory.h>
#include <ncurses.h>
#include <modbus/modbus.h>
#include <modbus/modbus-tcp.h>
#include "common.h"

void copyAddressToMap(ModbusAddress * mb_address, modbus_mapping_t * map);
void copyMapToAddress(modbus_mapping_t * map, ModbusAddress * mb_address);

int main(void)
{
    clrscr();
    printf("Creating connection... ");
    modbus_t * ctx = modbus_new_tcp(IP, PORTA);
    if (ctx == NULL)
    {
        fprintf(stderr, "Não foi possível alocar contexto.\n");
        return -1;
    }
    printf("Ok!\n");
    //modbus_set_debug(ctx, TRUE);

    printf("Creating a server socket... ");
    int rc = modbus_tcp_listen(ctx, 1);

    if (rc == -1)
    {
        fprintf(stderr, "Não foi possível criar socket servidor: %s:%d.\n", IP, PORTA);
        modbus_free(ctx);
        return -1;
    }
    printf("Ok!\n");

    printf("Listening connections... ");
    if (modbus_tcp_accept(ctx, &rc) == -1)
    {
        fprintf(stderr, "Não foi possível aceitar conexão: %s:%d.\n", IP, PORTA);
        modbus_free(ctx);
        return -1;
    }
    printf("Accepted!\n");

    modbus_mapping_t * map = modbus_mapping_new(n_coils, n_input_bits, n_holding_registers, n_input_registers);

    if (map == NULL)
    {
        fprintf(stderr, "Não foi possível alocar mapa de endereços.\n");
        modbus_free(ctx);
        return -1;
    }

    ModbusAddress mb_address = EMPTY_MODBUS_ADDRESS;
    int i=0;
    for(;;)
    {
        uint8_t data[MODBUS_MAX_ADU_LENGTH];

        copyAddressToMap(&mb_address, map);

        int len = modbus_receive(ctx, data);
        if (len == -1)
        {
            fprintf(stderr, "Erro ao receber mensagem.\n");
            modbus_free(ctx);
            return -1;
        }

        if (len > 0)
        {
            if (modbus_reply(ctx, data, len, map) == -1)
            {
                fprintf(stderr, "Erro ao responder mensagem.\n");
                modbus_free(ctx);
                return -1;
            }
            copyMapToAddress(map, &mb_address);
        }

        modbus_ext_check_interrupt(ctx, map);

        char k = menu("SLAVE", &mb_address, TRUE);

        // ESC key was pressed ...
        if ( k == 27 )
        {
            break;
        }

        i++;
        if (i % 100 == 0)
        {
            refresh_menu();
        }
    }

    modbus_free(ctx);

    return 0;
}

void copyAddressToMap(ModbusAddress * mb_address, modbus_mapping_t * map)
{
    int addr;

    for(addr=0; addr<n_input_bits; addr++)
    {
        if (modbus_ext_mapping_set_input_bit(map, addr, mb_address->ibits[addr]) == 1) {
            fprintf(stderr, "modbus_ext_mapping_set_input_bit(map, %d, %d)\n", addr, mb_address->ibits[addr]);
        }
    }

    for(addr=0; addr<n_coils; addr++)
    {
        if (modbus_ext_mapping_set_bit(map, addr, mb_address->coils[addr]) == 1) {
            fprintf(stderr, "modbus_ext_mapping_set_bit(map, %d, %d)\n", addr, mb_address->coils[addr]);
        }
    }

    for(addr=0; addr<n_input_registers; addr++)
    {
        if (modbus_ext_mapping_set_input_reg(map, addr, mb_address->iregs[addr]) == 1) {
            fprintf(stderr, "modbus_ext_mapping_set_input_reg(map, %d, %d)\n", addr, mb_address->iregs[addr]);
        }
    }

    for(addr=0; addr<n_holding_registers; addr++)
    {
        if (modbus_ext_mapping_set_reg(map, addr, mb_address->holds[addr]) == 1) {
            fprintf(stderr, "modbus_ext_mapping_set_reg(map, %d, %d)\n", addr, mb_address->holds[addr]);
        }
    }
}

void copyMapToAddress(modbus_mapping_t * map, ModbusAddress * mb_address)
{
    memcpy(mb_address->ibits, map->tab_input_bits, n_input_bits * sizeof(uint8_t));
    memcpy(mb_address->coils, map->tab_bits, n_coils * sizeof(uint8_t));
    memcpy(mb_address->iregs, map->tab_input_registers, n_input_registers * sizeof(uint16_t));
    memcpy(mb_address->holds, map->tab_registers, n_holding_registers * sizeof(uint16_t));
}
