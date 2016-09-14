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

int open_port();
int retrieve_id();
int retrieve_serial();
int retrieve_offsets();
int read_conf_file();
int create_file();
int write_file();
int close_file();


//==================================================================     globals

int device_id;
char port[255];
char* serial;
comm_settings comm_settings_t;
short int offsets[4];
FILE* filep;
char backup_folder[512];
uint8_t aux_string[2000];
int sensor_num = 0;
short int temp_meas[4];

//==============================================================================
//                                                                          main
//==============================================================================

int main() {

    assert(open_port());

    assert(retrieve_id());

    sensor_num = commGetMeasurements(&comm_settings_t, device_id, temp_meas);

    assert(retrieve_serial());

    assert(retrieve_offsets());

    assert(read_conf_file());

    assert(create_file());

    assert(write_file());

    assert(close_file());

    closeRS485(&comm_settings_t); //port close

    printf("Configuration saved!\n");

    return 1;
}

//==========================================================     other functions

int open_port() {
    FILE *file;

    file = fopen(QBMOVE_FILE, "r");

    if (file == NULL) {
        printf("Could not open configuration file\n");
        return 0;
    }

    fscanf(file, "serialport %s\n", port);

    fclose(file);


    printf("Opening port...");
    fflush(stdout);

    openRS485(&comm_settings_t, port);
    
    if(comm_settings_t.file_handle == INVALID_HANDLE_VALUE)
    {
        puts("Couldn't connect to the serial port.");
        return 0;
    }
    usleep(500000);

    printf("DONE\n");
    return 1;
}

int retrieve_id() {

    printf("Retrieving ID...");
    fflush(stdout);

    int index = 1;      //The index equal to one is relative to the ID of the board

    commGetParamList(&comm_settings_t, BROADCAST_ID, 0, NULL, 0, 0, aux_string);
    device_id = aux_string[(index - 1) * PARAM_BYTE_SLOT + 8];


    if (device_id == 0) {
        printf("Could not retrieve a good ID\n");
        return 0;
    }
    usleep(500000);

    printf("DONE\n");

    return 1;
}

int retrieve_serial() {

    serial = strtok(port, "-");
    serial = strtok(NULL, "-");
    if (serial == NULL) {
        printf("Could not retrieve serial\n");
        return 0;
    }
    printf("Current serial: %s\n", serial);

    return 1;
}

int retrieve_offsets() {
    int i,j;
    int index = 8;          //The index relative to the measurements offsets is 8
    int data_size = 2;      //The measurements offsets are 2 bytes wide 

    printf("Retrieving offsets...");
    fflush(stdout);

    for(i = 0; i < sensor_num; i++) {
        for(j = 0; j < data_size; j++) {
            offsets[i] += aux_string[(index - 1)* PARAM_BYTE_SLOT + 8 + i * data_size + data_size - j - 1] << (8 * j);
        }
    }

    for (i = 0; i < sensor_num; i++) {
        if (offsets[i] != 0)
            break;
        if (i == (sensor_num - 1)) {
            printf("Offsets seems not to be set properly\n");
            return 0;
        }
    }

    usleep(500000);

    printf("DONE\n");

    printf("Offsets: ");
    for (i = 0; i < sensor_num; i++) {
        printf("%d\t", offsets[i]);
    }
    printf("\n");

    return 1;
}

int read_conf_file() {
    FILE* conf_file;

    conf_file = fopen(QBBACKUP_FILE, "r");
    if (conf_file == NULL) {
        printf("Cannot open conf file\n");
        return 0;
    }

    fscanf(conf_file, "backup_folder: %s\n", backup_folder);

    fclose(conf_file);

    return 1;
}

int create_file() {
    char filename[512];
    char reply;


    strcpy(filename, backup_folder);
    strcat(filename, "/backup_");
    strcat(filename, serial);
    strcat(filename, ".bkp");

    if(!access(filename, W_OK)) {
        reply = 'N';
        printf("File already exists. Do you want to overwrite it? [y,N]\n");
        reply = getchar();

        if ((reply != 'Y') && (reply != 'y')) {
            return 0;
        }
    }
    filep = fopen(filename, "w");
    if (filep == NULL) {
        printf("Cannot open file\n");
        return 0;
    }
    return 1;
}

int write_file() {
    int i;
    printf("Writing current configuration...");
    fflush(stdout);

    for (i = 0; i < sensor_num; i++) {
        fprintf(filep, "%d\n", offsets[i]);    
    }
    usleep(500000);

    printf("DONE\n");

    return 1;
}

int close_file() {
    fclose(filep);
    return 1;
}

/* END OF FILE */