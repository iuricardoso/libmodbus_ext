//#include <ncurses.h>
#include <stdio.h>
#include <stdlib.h>
#include <modbus/modbus.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "common.h"

const ModbusAddress EMPTY_MODBUS_ADDRESS =  {
                                                .coils = {0},
                                                .ibits = {0},
                                                .holds = {0},
                                                .iregs = {0}
                                            };

static const AddressType address_type[] = {at_ibits, at_coils, at_coils,
                                    at_iregs, at_iregs,
                                    at_holds, at_holds, at_holds};

static bool _refreshed = FALSE;

void refresh_menu(void)
{
    _refreshed = FALSE;
}

static char * at_to_str(int i_menu)
{
    if (i_menu < 0 || i_menu > sizeof(address_type) / sizeof(AddressType))
    {
        return "";
    }

    switch(address_type[i_menu])
    {
        case at_ibits: return "(input bits)";
        case at_coils: return "(coil)";
        case at_iregs: return "(input regs)";
        case at_holds: return "(hold regs)";
    }

    return "";
}

char menu(const char title[], ModbusAddress * addr, bool modify_input)
{


    if (! _refreshed)
    {
        system("clear");
        printf("%s\n", title);
        printf("\n  %-12s Sensor Movimento .....: %-5s", at_to_str(0), BOOL_TO_STR(addr->ibits[ibit_movimento]));
        if (modify_input)
        {
            printf(" [O/o]");
        }
        printf("\n  %-12s Alarme de movimento ..: %-5s [A/a]", at_to_str(1), BOOL_TO_STR(addr->coils[coil_move_alarm]));
        printf("\n  %-12s Led ..................: %-5s [L/l]", at_to_str(2), BOOL_TO_STR(addr->coils[coil_led]));
        printf("\n  %-12s Sensor de Gases ......: %5d", at_to_str(3), addr->iregs[ireg_gases]);
        if (modify_input)
        {
            printf(" [G+/g-]");
        }
        printf("\n  %-12s Sensor de Luminosidade: %5d", at_to_str(4), addr->iregs[ireg_luminosidade]);
        if (modify_input)
        {
            printf(" [U+/u-]");
        }
        printf("\n  %-12s Display Gases ........: %5d [S+/s-]", at_to_str(5), addr->holds[hold_gases]);
        printf("\n  %-12s Display Luminosidade .: %5d [M+/m-]", at_to_str(6), addr->holds[hold_luminosidade]);
        printf("\n  %-12s Buzzer ...............: %5d [B+/b-]", at_to_str(7), addr->holds[hold_buzzer]);

        _refreshed = TRUE;
    }


    if (! kbhit()) {
        return EOF;
    }

    _refreshed = FALSE;

    char k = getchar();
    printf("\n\n  > ");

    switch(k)
    {
        case 'A':
        case 'a':
            TOGGLE_BOOL(addr->coils[coil_move_alarm]);
            break;
        case 'L':
        case 'l':
            TOGGLE_BOOL(addr->coils[coil_led]);
            break;
        case 'S':
            ADD_32BITS(addr->holds[hold_gases], 1);
            break;
        case 's':
            ADD_32BITS(addr->holds[hold_gases], -1);
            break;
        case 'M':
            ADD_32BITS(addr->holds[hold_luminosidade], 1);
            break;
        case 'm':
            ADD_32BITS(addr->holds[hold_luminosidade], -1);
            break;
        case 'B':
            ADD_32BITS(addr->holds[hold_buzzer], +1);
            break;
        case 'b':
            ADD_32BITS(addr->holds[hold_buzzer], -1);
            break;
        default:
            if (modify_input)
            {
                switch(k)
                {
                    case 'O':
                    case 'o':
                        TOGGLE_BOOL(addr->ibits[ibit_movimento]);
                        break;

                    case 'G':
                        ADD_32BITS(addr->iregs[ireg_gases], 1);
                        break;

                    case 'g':
                        ADD_32BITS(addr->iregs[ireg_gases], -1);
                        break;

                    case 'U':
                        ADD_32BITS(addr->iregs[ireg_luminosidade], 1);
                        break;

                    case 'u':
                        ADD_32BITS(addr->iregs[ireg_luminosidade], -1);
                        break;
                }
            }
            break;
    }

    return k;

}

int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

void clrscr(void)
{
    system("clear");
}
