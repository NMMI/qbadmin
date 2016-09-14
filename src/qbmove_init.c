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

#if (defined(_WIN32) || defined(_WIN64))
    #include <conio.h>
    #include <Windows.h>
    #define usleep(x) Sleep((x) / 1000)
#endif

//=============================================================     declarations

int mode_selection();
int port_selection(char*);
int open_port(char*);
int init_params();
int change_id();
int set_resolution(int);
int set_pid_parameters();
int adjust_zeros();
int get_info();
int test();
int backup();
int calibrate();


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

    assert(init_params());

    assert(adjust_zeros());

    commRestoreParams(&comm_settings_t, device_id);

    assert(get_info());

    assert(calibrate());

    assert(get_info()); 

    closeRS485(&comm_settings_t); //port close

    assert(test());

    #if !(defined(_WIN32) || defined(_WIN64))
        assert(backup());
    #endif

    printf("===== Configuration completed. =====\n");

    return 1;
}

//==========================================================     other functions

int port_selection(char* my_port){
    int i;
    int aux_int;
    int num_ports = 0;
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

int init_params() {
    int i;
    printf("Restoring factory settings...");
    fflush(stdout);

    i = 1;
    while(commInitMem(&comm_settings_t, device_id)) {
        printf("Try #%d\n", i++);
        usleep(500000);
    }
    
    printf("Done.\n");
    return 1;   
}

int adjust_zeros(){
    int i;
    char aux_char;
    char c = ' ';
    short int measurements[4];
    short int measurements_off[4];
    short int current_ref[NUM_OF_MOTORS];
    short int zeros[4];
    int sensor_num = 0;

    // Setting current offset to zero
    for (i = 0; i < 4; i++) {
        zeros[i] = 0;
        measurements_off[i] = 0;
    }
    
    //Set the offsets equal to the actual posistion to drive the motors from the initial position of 0
    sensor_num = commGetMeasurements(&comm_settings_t, device_id, measurements);
    for(i = 0; i< sensor_num; i++)
        measurements_off[i] = -measurements[i];

    commSetZeros(&comm_settings_t, device_id, measurements_off, sensor_num);
    commStoreDefaultParams(&comm_settings_t, device_id);


    // Activate motors
    while (1) {
        // set
        commActivate(&comm_settings_t, device_id, 1);
        usleep(100000);

        // check
        commGetActivate(&comm_settings_t, device_id, &aux_char);

        if (aux_char == 0x03)
            break;
    }


    // Instructions
    printf("\nTo move the motors use letters 'q', 'a', 'w', 's'.\n");
    printf("To move the output shaft (two motors toghether) use letters 'e', 'd':\n");
    printf("When you are done type 'x' to set zero for current position\n\n");
    printf("    | M1 | M2 | OUT |\n");
    printf("----|----|----|-----|\n");
    printf(" -> | q  | w  |  e  |\n");
    printf(" <- | a  | s  |  d  |\n");

    for (i = 0; i < NUM_OF_MOTORS; i++) {
        current_ref[i] = 0;
    }

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

    while(c != 'x') {
        #if !(defined(_WIN32) || defined(_WIN64))
            c = getchar();
        #else
            c = getch();
        #endif
        switch(c) {
            case 'q':
                current_ref[0] += DEFAULT_INCREMENT * DEG_TICK_MULTIPLIER;
                break;
            case 'a':
                current_ref[0] -= DEFAULT_INCREMENT * DEG_TICK_MULTIPLIER;
                break;
            case 'w':
                current_ref[1] += DEFAULT_INCREMENT * DEG_TICK_MULTIPLIER;
                break;
            case 's':
                current_ref[1] -= DEFAULT_INCREMENT * DEG_TICK_MULTIPLIER;
                break;
            case 'e':
                current_ref[0] += DEFAULT_INCREMENT * DEG_TICK_MULTIPLIER;
                current_ref[1] += DEFAULT_INCREMENT * DEG_TICK_MULTIPLIER;
                break;
            case 'd':
                current_ref[0] -= DEFAULT_INCREMENT * DEG_TICK_MULTIPLIER;
                current_ref[1] -= DEFAULT_INCREMENT * DEG_TICK_MULTIPLIER;
                break;
            case 'x':
                break;
            default:
                printf("Use only q, a, w, s, e, d in lowercase\n");
                break;
        }

        commSetInputs(&comm_settings_t, device_id, current_ref);
    }

	#if !(defined(_WIN32) || defined(_WIN64))
    // Restore the old tty settings
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
	#endif


    // Deactivate motors
    while (1) {
        //set
        commActivate(&comm_settings_t, device_id, 0);
        usleep(100000);

        //check
        commGetActivate(&comm_settings_t, device_id, &aux_char);

        if (aux_char == 0x00)
            break;
    }

    // Reset the offsets to zero to get the correct measurements
    commSetZeros(&comm_settings_t, device_id, zeros, sensor_num);
    commStoreDefaultParams(&comm_settings_t, device_id);

    usleep(100000);

    commGetMeasurements(&comm_settings_t, device_id, measurements);  
    // Prepare offsets for setting zero
    for (i = 0; i < sensor_num; i++) {
        measurements_off[i] = -measurements[i];
    }

    if(commSetZeros(&comm_settings_t, device_id, measurements_off, sensor_num)) {
        printf("\nAn error occurred while setting measurements offsets. Retry.\n");
        exit(1);
    }

    if(commStoreDefaultParams(&comm_settings_t, device_id)) {
        printf("Error while saving measurements offsets\n");
        exit(1);
    }

    // Setting zero for current position
    printf("Setting new zero for pulleys and shaft...");
    fflush(stdout);

    usleep(500000);

    printf("DONE\n");

    return 1;
}


int test() {
    char c;

    fflush(stdin);
    printf("Do you want to perform a test cycle? [Y,n]\n");
    c = getchar();
    if (c == 'n' || c == 'N') {
        return 1;
    }

    #if !(defined(_WIN32) || defined(_WIN64))
        system("./qbtest -r 1");
    #else
        system("qbtest -r 1");
    #endif

    return 1;
}

int get_info() {
    printf("Displaying informations:\n\n");
    char aux_string[2000];

    RS485GetInfo(&comm_settings_t,  aux_string);
    puts(aux_string);

    return 1;
}


int calibrate() {
    char c;
    int i;

    fflush(stdin);
    printf("Do you want to auto-calibrate stiffness? [Y,n]\n");
    c = getchar();
    if (c == 'n' || c == 'N') {
        return 1;
    }

    printf("Calibrating...");
    fflush(stdout);
    if(!commCalibrate(&comm_settings_t, BROADCAST_ID)) {
        printf("EXECUTING: wait for the end of calibration\n");
        for (i = 0; i < 100; i++) {
            usleep(150000);
        }
        return 1;
    } else {
        printf("FAILED\n");
        return 0;
    }
}

int backup() {

    #if !(defined(_WIN32) || defined(_WIN64))
        system("./qbbackup");
    #else
        system("qbbackup");
    #endif

    return 1;
}
/* END OF FILE */