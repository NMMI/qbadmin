//=================================================================     includes

#include "../../qbAPI/src/qbmove_communications.h"
#include "definitions.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <unistd.h>

#if !(defined(_WIN32) || defined(_WIN64))
    #include <termios.h>
#endif

//=============================================================     declarations

int port_selection(char*);
int open_port(char*);

int pilot_pos_stiff();
int set_pos_stiff(short int*, short int*);

//==================================================================     globals

int device_id;
comm_settings comm_settings_t;

//==============================================================================
//                                                                          main
//==============================================================================

int main(int argc, char **argv){
    char port[255];
    device_id = BROADCAST_ID;

    assert(port_selection(port));

    assert(open_port(port));

    assert(pilot_pos_stiff());

    closeRS485(&comm_settings_t);

    return 1;
}

//==========================================================     other functions


int port_selection(char* my_port){
    int i;
    int aux_int;
    int num_ports = 0;
    char ports[10][255];

    while(1) {
        num_ports = RS485listPorts(ports);
          
        if(num_ports) {
            puts("\nChoose the serial port for your QB:\n");

            for(i = 0; i < num_ports; ++i) {
                printf("[%d] - %s\n\n", i+1, ports[i]);
            }

            printf("Serial port: ");
            scanf("%d", &aux_int);

            if( aux_int && (aux_int <= num_ports) ) {
                strcpy(my_port, ports[aux_int - 1]);
            } else {
                puts("Choice not available");
                continue;
            }
            return 1;
        } else {
            puts("No serial port available.");
            return 0;
        }
    }
}


int open_port(char* port_s) {
    printf("Opening serial port...");
    fflush(stdout);

    openRS485(&comm_settings_t, port_s);
    
    if(comm_settings_t.file_handle == INVALID_HANDLE_VALUE)
    {
        puts("Couldn't connect to the serial port.");
        return 0;
    }
    usleep(500000);
    printf("Done.\n");
    return 1;
}


int pilot_pos_stiff(){
    char c = ' ';
    short int pos, stiff;

    // Activate motors
    commActivate(&comm_settings_t, device_id, 1);

    // Instructions
    printf("\nUse Arrow Keys (type 'q' to terminate):\n");
    printf("\n            + Stiff\n");
    printf("                |\n");
    printf("- Ref. Pos. <-------> + Ref. Pos.\n");
    printf("                |\n");
    printf("            - Stiff\n\n");

    pos = 0;
    stiff = 0;

    #if !(defined(_WIN32) || defined(_WIN64))
    //---- tty inizialization ---- BEGIN

    static struct termios oldt, newt;

    /*tcgetattr gets the parameters of the current terminal
    STDIN_FILENO will tell tcgetattr that it should write the settings
    of stdin to oldt*/
    tcgetattr( STDIN_FILENO, &oldt);
    /*now the settings will be copied*/
    newt = oldt;

    /*ICANON normally takes care that one line at a time will be processed
    that means it will return if it sees a "\n" or an EOF or an EOL*/
    newt.c_lflag &= ~(ICANON);
    newt.c_lflag &= ~(ICANON | ECHO);       

    /*Those new settings will be set to STDIN
    TCSANOW tells tcsetattr to change attributes immediately. */
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);
    
    //---- tty inizialization ---- END
    #endif

    while(c != 'q') {
        c = getchar();
        switch(c) {
            // case 27:
            //  break;
            // case 91:
            //  break;
            case 65:
                //printf("up\n");
                stiff ++;
                break;
            case 66:
                //printf("down\n");
                stiff --;
                break;
            case 67:
                //printf("right\n");
                pos += DEFAULT_INCREMENT * DEG_TICK_MULTIPLIER;
                break;
            case 68:
                //printf("left\n");
                pos -= DEFAULT_INCREMENT * DEG_TICK_MULTIPLIER;
                break;
            default:
                //do nothing
                break;
        }

        set_pos_stiff(&pos, &stiff);
        printf("                            \r");
        printf("Pos: %d, Stiff %d\t\t\r", (int)(pos / DEG_TICK_MULTIPLIER), (int)(stiff));
        //commSetInputs(&comm_settings_t, device_id, current_ref);
    }

    pos = 0;
    stiff = 0;

    set_pos_stiff(&pos, &stiff);
    usleep(2000000);

    // Deactivate motors
    commActivate(&comm_settings_t, device_id, 0);

    #if !(defined(_WIN32) || defined(_WIN64))
    // Restore the old tty settings
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
    #endif

    return 1;
}

int set_pos_stiff(short int* pos, short int* stiff) {
    short int curr_ref[NUM_OF_MOTORS];

    if (*pos > (DEFAULT_SUP_LIMIT - DEFAULT_STIFFNESS * DEG_TICK_MULTIPLIER)) {
        *pos = (DEFAULT_SUP_LIMIT - DEFAULT_STIFFNESS * DEG_TICK_MULTIPLIER);
    } else if (*pos < (DEFAULT_INF_LIMIT + DEFAULT_STIFFNESS * DEG_TICK_MULTIPLIER)) {
        *pos = (DEFAULT_INF_LIMIT + DEFAULT_STIFFNESS * DEG_TICK_MULTIPLIER);
    }

    if (*stiff > 100) {
        *stiff = 100;
    } else if (*stiff < -100) {
        *stiff = -100;
    }
 
    // Position
    curr_ref[0] = *pos;
    // Stiffness
    curr_ref[1] = (*stiff) * 32767 / 100;

    commSetPosStiff(&comm_settings_t, device_id, curr_ref);

    return 1;
}

/* END OF FILE */