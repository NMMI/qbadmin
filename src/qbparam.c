// --- INCLUDE ---
#include "../../qbAPI/src/qbmove_communications.h"
#include "definitions.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
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
/** Baudrate functions
 */
int baudrate_reader();

// --- MAIN ---
int main() {
    int i,j,k;
    char c_choice;

    uint8_t aux_string[2000];
    int value_size;
    int num_of_values;
    int num_of_params;
    int menu_number[50];
    int index;
    int data_type[50];
    int data_dim[50];
    int data_size[50];
    char data_string[20];
    char tmp_string[100];

    int8_t aux_int8[4];
    uint8_t aux_uint8[4];
    int16_t aux_int16[4];
    uint16_t aux_uint16[4];
    int32_t aux_int32[4];
    uint32_t aux_uint32[4];
    float aux_float[4]; 
    double aux_double[4];
    uint8_t temp_char[4];

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

        // When commGetParamList is called with index = 0, it will return in aux_string
        // a packet containing the parameters' values and description
        index = 0;
        value_size = 0;
        num_of_values = 0;
        commGetParamList(&comm_settings_t, BROADCAST_ID, index, NULL, value_size, num_of_values, aux_string);
        
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
                        sprintf(data_string, " %hhu", aux_uint8[k]);
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
                        commGetParamList(&comm_settings_t, BROADCAST_ID, index, aux_uint8, data_size[index - 1], data_dim[index - 1], NULL);
                    break;
                    case TYPE_INT8:
                        commGetParamList(&comm_settings_t, BROADCAST_ID, index, aux_int8, data_size[index - 1], data_dim[index - 1], NULL);
                    break;
                    case TYPE_UINT8:
                        commGetParamList(&comm_settings_t, BROADCAST_ID, index, aux_uint8, data_size[index - 1], data_dim[index - 1], NULL);
                    break;
                    case TYPE_INT16:
                        commGetParamList(&comm_settings_t, BROADCAST_ID, index, aux_int16, data_size[index - 1], data_dim[index - 1], NULL);
                    break;
                    case TYPE_UINT16:
                        commGetParamList(&comm_settings_t, BROADCAST_ID, index, aux_uint16, data_size[index - 1], data_dim[index - 1], NULL);
                    break;
                    case TYPE_INT32:
                        commGetParamList(&comm_settings_t, BROADCAST_ID, index, aux_int32, data_size[index - 1], data_dim[index - 1], NULL);
                    break;
                    case TYPE_UINT32:
                        commGetParamList(&comm_settings_t, BROADCAST_ID, index, aux_uint32, data_size[index - 1], data_dim[index - 1], NULL);
                    break;
                    case TYPE_FLOAT:
                        commGetParamList(&comm_settings_t, BROADCAST_ID, index, aux_float, data_size[index - 1], data_dim[index - 1], NULL);
                    break;
                    case TYPE_DOUBLE:
                        commGetParamList(&comm_settings_t, BROADCAST_ID, index, aux_double, data_size[index - 1], data_dim[index - 1], NULL);
                    break;
                }

            usleep(100000);
            commStoreParams(&comm_settings_t, BROADCAST_ID);
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
    char ports[10][255];
    FILE *file;

    while(1) {
        num_ports = RS485listPorts(ports);

        if(num_ports) {
            puts("\nChoose the serial port for your QB:\n");

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
    if(!commCalibrate(&comm_settings_t, BROADCAST_ID)) {
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


    openRS485(&comm_settings_t, port);

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

    if (!commInitMem(&comm_settings_t, BROADCAST_ID)) {
        printf("DONE\n");
        return 1;
    }

    printf("Failed\n");
    return 0;
}


void printVersion() {
    printf("==============================================\n");
    printf("qbparam version: %s\n", QBADMIN_VERSION);
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