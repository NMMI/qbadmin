// ----------------------------------------------------------------------------
// BSD 3-Clause License

// Copyright (c) 2016, qbrobotics
// Copyright (c) 2017-2020, Centro "E.Piaggio"
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
// POSSIBILITY OF SUCH DAMAGE.
// ----------------------------------------------------------------------------

/**
* \file         nmmi_param_imu.c
*
* \brief        Command line tools file
* \author       _Centro "E.Piaggio"_
* \copyright    (C) 2012-2016 qbrobotics. All rights reserved.
* \copyright    (C) 2017-2020 Centro "E.Piaggio". All rights reserved.
*
* \details      With this file is possible to get or set IMU parameters.
*/

// --- INCLUDE ---
#include "../../qbAPI/src/qbmove_communications.h"
#include "../../qbAPI/src/cp_communications.h"
#include "definitions.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <stdint.h>

// function declaration
int port_selection();
int open_port();
int initMemory();
void printMainMenu();
void printVersion();
int calibrate();

// global variables
char get_or_set;
comm_settings comm_settings_t;
uint8_t device_id = BROADCAST_ID;

/** Baudrate functions
 */
int baudrate_reader();

// --- MAIN ---
int main(int argc, char **argv) {
    int i,j,k;
    char c_choice;

    uint8_t aux_string[5000] = "";

    int value_size;
    int num_of_values;
    int num_of_params;
    int menu_number[50];
    int index;

    int data_type[50];
    int data_dim[50];
    int data_size[50];
    char data_string[20];
    char tmp_string[150] = "";

    int8_t aux_int8[4];
    uint8_t aux_uint8[4];
    int16_t aux_int16[4];
    uint16_t aux_uint16[4];
    int32_t aux_int32[4];
    uint32_t aux_uint32[4];
    float aux_float[4]; 
    double aux_double[4];
    uint8_t temp_char[4];








	// Get device ID
	if (argc > 1)
    {
        sscanf(argv[1],"%d",&device_id);
		printf("Communicating with device %d\n", device_id);
    }

    if(device_id)
        printf("\nUsing nmmi_param_imu with ID: %hhu\n\n", device_id);
    else
        printf("\nUsing nmmi_param_imu in broadcast.\n\n");

    printVersion();

    if (!open_port()) {
        assert(port_selection());
        assert(open_port());
    }


//===============================================================     MAIN MENU

    printMainMenu();
    scanf("%c", &c_choice);

    switch(c_choice) {
        case 'g':
            get_or_set = c_choice;
            break;
        case 's':
            get_or_set = c_choice;
            break;
        case 'm':
            initMemory();
            break;
        case 'c':
            calibrate();
            break;

        default:
            break;
    }

    if (get_or_set == 'g' || get_or_set == 's') {

        //==============================================================     SET
        printf("\nDevice parameters: \n");

        // When commGetIMUParamList is called with index = 0, it will return in aux_string
        // a packet containing the parameters' values and description
        index = 0;
        value_size = 0;
        num_of_values = 0;
        commGetIMUParamList(&comm_settings_t, device_id, index, NULL, value_size, num_of_values, aux_string);
        

        // The packet returned in aux_string is composed as follows
        // [':'][':'][ID][LEN][CMD][PARAM_NUM][...]






        num_of_params = aux_string[5];

        // The data, for a single parameter, is packed as follows
        // [DATA_TYPE][DATA_DIMENSION][..DATA..][..DESCRIPTION..] 
        // Data_type is a enum that tells the parameter type (int, float, short int)
        // Data_dimension is the number of values included in that parameter. For position pid is equal to 3. 

        for(i = 0; i < num_of_params; i++) {
            data_type[i] = aux_string[i * PARAM_BYTE_SLOT + 6];
            data_dim[i] = aux_string[i * PARAM_BYTE_SLOT + 7];
            for(j = 0; j < 4; j++) {
                aux_int8[j] = 0; aux_uint8[j] = 0; temp_char[j] = 0; aux_int16[j] = 0;
                aux_uint16[j] = 0; aux_int32[j] = 0; aux_uint32[j] = 0; aux_float[j] = 0.0;
                aux_double[j] = 0.0;
            }

            // For each parameter is associated a size, which is the number of bytes of that parameter, depending on its type
            switch(data_type[i]){
                case TYPE_FLAG:
                    data_size[i] = 1;
                    for(k = 0; k < data_dim[i]; k++) {
                        for(j = 0; j < data_size[i]; j++) {
                            aux_uint8[k] += aux_string[i*PARAM_BYTE_SLOT + 8 + k * data_size[i] + data_size[i] - j - 1] << (8 * j);  
                        }
                        /*sprintf(data_string, " %hhu", aux_uint8[k]);
                        strcat(tmp_string, data_string);*/
                    }
                case TYPE_INT8:
                    data_size[i] = 1;
                    for(k = 0; k < data_dim[i]; k++) {
                        for(j = 0; j < data_size[i]; j++) {
                            aux_int8[k] += aux_string[i*PARAM_BYTE_SLOT + 8 + k * data_size[i] + data_size[i] - j - 1] << (8 * j);
                        }
                        sprintf(data_string, " %hhd", aux_int8[k]);
                        strcat(tmp_string, data_string);
                    }
                break;
                case TYPE_UINT8:
                    data_size[i] = 1;
                    for(k = 0; k < data_dim[i]; k++) {
                        for(j = 0; j < data_size[i]; j++) {
                            aux_uint8[k] += aux_string[i*PARAM_BYTE_SLOT + 8 + k * data_size[i] + data_size[i] - j - 1] << (8 * j);  
                        }
                        if (aux_uint8[k] == 255){ 	// Not IMU Connected
                        	sprintf(data_string, " %c", '-');
                        }
                        else {
                        	sprintf(data_string, " %hhu", aux_uint8[k]);
                        }
                        strcat(tmp_string, data_string); 
                    }
                break;
                case TYPE_INT16:
                    data_size[i] = 2;
                    for(k = 0; k < data_dim[i]; k++) {
                        for(j = 0; j < data_size[i]; j++) {
                            aux_int16[k] += aux_string[i*PARAM_BYTE_SLOT + 8 + k * data_size[i] + data_size[i] - j - 1] << (8 * j);
                        }
                        sprintf(data_string, " %hd", aux_int16[k]);
                        strcat(tmp_string, data_string); 
                    }
                break;
                case TYPE_UINT16:
                    data_size[i] = 2;
                    for(k = 0; k < data_dim[i]; k++) {
                        for(j = 0; j < data_size[i]; j++) {
                            aux_uint16[k] += aux_string[i*PARAM_BYTE_SLOT + 8 + k * data_size[i] + data_size[i] - j - 1] << (8 * j);
                        }
                        sprintf(data_string, " %hu", aux_uint16[k]);
                        strcat(tmp_string, data_string); 
                    }
                break;
                case TYPE_INT32:
                    data_size[i] = 4;
                    for(k = 0; k < data_dim[i]; k++) {
                        for(j = 0; j < data_size[i]; j++) {
                            aux_int32[k] += aux_string[i*PARAM_BYTE_SLOT + 8 + k * data_size[i] + data_size[i] - j - 1] << (8 * j);
                        }
                        sprintf(data_string, " %d", aux_int32[k]);
                        strcat(tmp_string, data_string); 
                    }
                break;
                case TYPE_UINT32:
                    data_size[i] = 4;
                    for(k = 0; k < data_dim[i]; k++) {
                        for(j = 0; j < data_size[i]; j++) {
                            aux_uint32[k] += aux_string[i*PARAM_BYTE_SLOT + 8 + k * data_size[i] + data_size[i] - j - 1] << (8 * j);   
                        }
                        sprintf(data_string, " %u", aux_uint32[k]);
                        strcat(tmp_string, data_string);
                    }
                break;
                case TYPE_FLOAT:
                    data_size[i] = 4;
                    for(k = 0; k < data_dim[i]; k++) {
                        for(j = 0; j < data_size[i]; j++) {
                            temp_char[j] = aux_string[i*PARAM_BYTE_SLOT + 8 + k * data_size[i] + data_size[i] - j - 1];
                        }
                        aux_float[k] = (*(float *) &temp_char);
                        sprintf(data_string, " %f", aux_float[k]);

                        strcat(tmp_string, data_string);
                    }
                break;
                case TYPE_DOUBLE:
                    data_size[i] = 4;
                    for(k = 0; k < data_dim[i]; k++) {
                        for(j = 0; j < data_size[i]; j++) {
                            temp_char[j] = aux_string[i*PARAM_BYTE_SLOT + 8 + k * data_size[i] + data_size[i] - j - 1];
                        }
                        aux_double[k] = (*(double *) &temp_char);
                        sprintf(data_string, " %f", aux_double[k]);
                        strcat(tmp_string, data_string);
                    }
                break;

            }
            // The parameter description is printed character by character until it reaches the end of the string
            for(k = 0; aux_string[i * PARAM_BYTE_SLOT + 6 + data_size[i] * data_dim[i] + 2 + k] != '\0'; k++) {
                printf("%c", aux_string[i * PARAM_BYTE_SLOT + 6 + data_size[i] * data_dim[i] + 2 + k]);
            }
            
            // If the parameter is a TYPE_FLAG it has a printable menu at the end of the packet. 
            // Their order is written at the end of each parameter description

            if(data_type[i] == TYPE_FLAG)
                menu_number[i] = aux_string[i * PARAM_BYTE_SLOT + 6 + data_size[i] * data_dim[i] + 2 + k + 1];
            else
                menu_number[i] = -1;
            // If the parameter has associated a -1 to its menu_number, the parameter has not a menu.

            if(data_type[i] == TYPE_FLAG)
                printf("\n");
            else
                printf("%s\n", tmp_string);
            
            strcpy(tmp_string, "");
        }

        if (get_or_set == 's') {

            printf("\nSelect a parameter from the previous list\n");
            scanf("%d", &index);
            if(index <= 0 || index > num_of_params) {
                printf("Invalid choice\n");
                return 0;
            }

            for(j = 0; j < 4; j++) {
                    aux_int8[j] = 0; aux_uint8[j] = 0; aux_int16[j] = 0; aux_uint16[j] = 0;
                    aux_int32[j] = 0; aux_uint32[j] = 0; aux_float[j] = 0.0; aux_double[j] = 0.0;
            }
            // Depending on the number of values of the parameter and its type
            // different and/or multiple readings must be done
            printf("Insert new parameters values\n");
            for(i = 0; i < data_dim[index - 1]; i++) {
                if(data_type[index - 1] != TYPE_FLAG) {
                    printf("Insert %d° parameter: \n", i+1);
                    switch(data_type[index - 1]) {
                        case TYPE_INT8:
                            scanf("%hhd", &aux_int8[i]);
                        break;

                        case TYPE_UINT8:
                            scanf("%hhu", &aux_uint8[i]);
                        break;

                        case TYPE_INT16:
                            scanf("%hd", &aux_int16[i]);
                        break;

                        case TYPE_UINT16:
                            scanf("%hu", &aux_uint16[i]);
                        break;

                        case TYPE_INT32:
                            scanf("%d", &aux_int32[i]);
                        break;

                        case TYPE_UINT32:
                            scanf("%u", &aux_uint32[i]);
                        break;

                        case TYPE_FLOAT:
                            scanf("%f", &aux_float[i]);
                        break;

                        case TYPE_DOUBLE:
                            scanf("%lf", &aux_double[i]);
                        break;
                    }
                }
                else {      // TYPE_FLAG is a uint8 but with a menu
                    for(k = 0; aux_string[num_of_params * PARAM_BYTE_SLOT + 6 + (menu_number[index - 1] - 1) * PARAM_MENU_SLOT + k] != '\0'; k++)
                        printf("%c", aux_string[num_of_params * PARAM_BYTE_SLOT + 6 + (menu_number[index - 1] - 1) * PARAM_MENU_SLOT + k]);
                    scanf("%hhu", aux_uint8);
                }

            }
            switch(data_type[index - 1]) {
                    case TYPE_FLAG:
                        commGetIMUParamList(&comm_settings_t, device_id, index, aux_uint8, data_size[index - 1], data_dim[index - 1], NULL);
                    break;
                    case TYPE_INT8:
                        commGetIMUParamList(&comm_settings_t, device_id, index, aux_int8, data_size[index - 1], data_dim[index - 1], NULL);
                    break;
                    case TYPE_UINT8:
                        commGetIMUParamList(&comm_settings_t, device_id, index, aux_uint8, data_size[index - 1], data_dim[index - 1], NULL);
                    break;
                    case TYPE_INT16:
                        commGetIMUParamList(&comm_settings_t, device_id, index, aux_int16, data_size[index - 1], data_dim[index - 1], NULL);
                    break;
                    case TYPE_UINT16:
                        commGetIMUParamList(&comm_settings_t, device_id, index, aux_uint16, data_size[index - 1], data_dim[index - 1], NULL);
                    break;
                    case TYPE_INT32:
                        commGetIMUParamList(&comm_settings_t, device_id, index, aux_int32, data_size[index - 1], data_dim[index - 1], NULL);
                    break;
                    case TYPE_UINT32:
                        commGetIMUParamList(&comm_settings_t, device_id, index, aux_uint32, data_size[index - 1], data_dim[index - 1], NULL);
                    break;
                    case TYPE_FLOAT:
                        commGetIMUParamList(&comm_settings_t, device_id, index, aux_float, data_size[index - 1], data_dim[index - 1], NULL);
                    break;
                    case TYPE_DOUBLE:
                        commGetIMUParamList(&comm_settings_t, device_id, index, aux_double, data_size[index - 1], data_dim[index - 1], NULL);
                    break;
                }

            usleep(100000);
            commStoreParams(&comm_settings_t, device_id);
            usleep(100000);
        }

    }

    return 1;
}

int port_selection() {
    int i;
    int aux_int;
    int num_ports = 0;
    char my_port[255];
    char ports[20][255];
    FILE *file;

    while(1) {
        num_ports = RS485listPorts(ports);

        if(num_ports) {
            puts("\nChoose the serial port for your NMMI board:\n");

            for(i = 0; i < num_ports; ++i) {
                printf("[%d] - %s\n\n", i+1, ports[i]);
            }
            printf("Serial port: ");
            scanf("%d", &aux_int);
            getchar();

            if( aux_int && (aux_int <= num_ports) ) {
                strcpy(my_port, ports[aux_int - 1]);
            } else {
                puts("Choice not available");
                continue;
            }

            file = fopen(QBMOVE_FILE, "w+");
            if (file == NULL) {
                printf("Cannot open qbmove.conf\n");
            }
            fprintf(file,"serialport %s\n", my_port);
            fclose(file);
            return 1;

        } else {
            puts("No serial port available.");
            return 0;
        }
    }
}

int calibrate() {
    printf("Calibrating...");
    fflush(stdout);
    if(!commCalibrate(&comm_settings_t, device_id)) {
        printf("DONE\n");
        return 1;
    } else {
        printf("FAILED\n");
        return 1;
    }
}


int open_port() {
    FILE *file;
    char port[255];
    int br = baudrate_reader();

    file = fopen(QBMOVE_FILE, "r");

    if (file == NULL) {
        printf("Error opening file %s\n", QBMOVE_FILE);
        return 0;
    }

    fscanf(file, "serialport %s\n", port);

    fclose(file);

    #if !(defined(_WIN32) || defined(_WIN64)) && !(defined(__APPLE__)) //only for linux

        if (br == BAUD_RATE_T_460800)
            openRS485(&comm_settings_t, port , B460800);
        else
            openRS485(&comm_settings_t, port , B2000000);
    #else
        if (br == BAUD_RATE_T_460800)
           openRS485(&comm_settings_t, port , 460800);
        else
            openRS485(&comm_settings_t, port , 2000000);
    #endif


    if(comm_settings_t.file_handle == INVALID_HANDLE_VALUE)
    {
        puts("Couldn't connect to the serial port.");
        return 0;
    }
    usleep(100000);

    return 1;
}


//==============================================================================
//                                                               baudrate_reader
//==============================================================================

int baudrate_reader(){

    int br = 0;
    FILE* file;

    file = fopen(QBMOVE_FILE_BR, "r");

    if (file == NULL) {
        printf("Error operning file %s\n", QBMOVE_FILE_BR);
        return 0;
    }

    fscanf(file, "baudrate %d\n", &br);

    fclose(file);

    if (br == 460800)
        return BAUD_RATE_T_460800;
    else
        return BAUD_RATE_T_2000000;
}

//==============================================================================
//                                                                   memory init
//==============================================================================

int initMemory() {
    char choice;

    getchar();
    printf("WARNING: Restore initial memory settings? [y/N]\n");
    choice = getchar();

    if (choice != 'y' && choice != 'Y') {
        return 0;
    }

    printf("Initializing memory...");

    if (!commInitMem(&comm_settings_t, device_id)) {
        printf("DONE\n");
        return 1;
    }

    printf("Failed\n");
    return 0;
}


void printVersion() {
    printf("==============================================\n");
    printf("nmmi_param_imu version: %s\n", NMMI_PARAM_VERSION);
    printf("==============================================\n");
}

void printMainMenu() {
    printf("g: getParam\n");
    printf("s: setParam\n");
    printf("m: initMemory\n");
    printf("c: calibrate\n");
    printf("==============================================\n");
}

/* END OF FILE */