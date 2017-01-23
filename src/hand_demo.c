// BSD 3-Clause License

// Copyright (c) 2017, qbrobotics
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/**
* \file         hand_demo.c
*
* \brief        qbHand demonstration file
*
* \details      With this file is possible to see a brief demonstration of qbHand
*               opening and closing.
*
* \copyright    (C)  qbrobotics. All rights reserved.
*/

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
int hand_move();
void set_input(short int);
void get_input(short int *);
void print_current();

//==================================================================     globals

int device_id;
comm_settings comm_settings_t;
const int def_inc = 100;

//==============================================================================
//                                                                          main
//==============================================================================

int main(int argc, char **argv){
    char port[255];
    device_id = BROADCAST_ID;

    assert(port_selection(port));

    assert(open_port(port));

    assert(hand_move());

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


int hand_move(){
    char c = ' ';
    short int pos;

    // Activate motors
    commActivate(&comm_settings_t, device_id, 1);

    // Instructions
    printf("\nUse Arrow Keys (type 'q' to terminate):\n");
    printf("Ref. -- <-------> Ref. ++\n");

    pos = 0;

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

        get_input(&pos);

        switch(c) {
            case 67:
                //printf("right\n");
                pos += def_inc;
                break;

            case 68:
                //printf("left\n");
                pos -= def_inc;
                break;

            default:
                //do nothing
                break;
        }

        set_input(pos);

        printf("                            \r");
        printf("Actual input: %d   ", (int)(pos));

        print_current();
    }

    pos = 0;
    set_input(pos);

    usleep(2000000);

    // Deactivate motors
    commActivate(&comm_settings_t, device_id, 0);

    #if !(defined(_WIN32) || defined(_WIN64))
    // Restore the old tty settings
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
    #endif

    return 1;
}

void set_input(short int pos) {
    static short int inputs[2];

    inputs[0] = pos;
    inputs[1] = 0;

    commSetInputs(&comm_settings_t, device_id, inputs);
    return;
}

void get_input(short int *pos) {
    static short int inputs[2];

    commGetInputs(&comm_settings_t, device_id, inputs);

    *pos = inputs[0];

    return;
}

void print_current() {
    static short int currents[2];

    commGetCurrents(&comm_settings_t, device_id, currents);

    printf("current: %d\n", (int)currents[0]);

    return;
}

/* END OF FILE */