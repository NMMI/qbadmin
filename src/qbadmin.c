// ----------------------------------------------------------------------------
// BSD 3-Clause License

// Copyright (c) 2016, qbrobotics
// Copyright (c) 2017-2018, Centro "E.Piaggio"
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
* \file         qbadmin.c
*
* \brief        Command line tools file
* \author       _Centro "E.Piaggio"_
* \copyright    (C) 2012-2016 qbrobotics. All rights reserved.
* \copyright    (C) 2017-2018 Centro "E.Piaggio". All rights reserved.
*
* \details      With this file is possible to command a terminal device.
*/

 /**
* \mainpage     Command line tools
*
* \brief        Those functions allows to use the board through a serial port
*
* \author       _Centro "E.Piaggio"_
* \copyright    (C) 2012-2016 qbrobotics. All rights reserved.
* \copyright    (C) 2017-2018 Centro "E.Piaggio". All rights reserved.
*
* \date         May 03, 2018
*
* \details      This is a set of functions that allows to use the boards 
*               via a serial port.
*
*/
//==================================================================     defines


//=================================================================     includes

#include "../../qbAPI/src/qbmove_communications.h"
#include "definitions.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <getopt.h>
#include <string.h>
#include <sys/time.h>
#include <math.h>
#include <signal.h>
#include <assert.h>

#if defined(_WIN32) || defined(_WIN64)
    #include <windows.h>
	#define sleep(x) Sleep(1000 * x)
#endif

//===============================================================     structures


static const struct option longOpts[] = {
    { "set_inputs", required_argument, NULL, 's' },
    { "get_measurements", no_argument, NULL, 'g' },
    { "activate", no_argument, NULL, 'a' },
    { "deactivate", no_argument, NULL, 'd' },
    { "ping", no_argument, NULL, 'p' },
    { "serial_port", no_argument, NULL, 't' },
    { "verbose", no_argument, NULL, 'v' },
    { "help", no_argument, NULL, 'h' },
    { "file", required_argument, NULL, 'f'},
    { "log", no_argument, NULL, 'l'},
    { "get_emg", no_argument, NULL, 'q'},
    { "set_zeros", no_argument, NULL, 'z'},
    { "use_gen_sin", no_argument, NULL, 'y'},
    { "get_currents", no_argument, NULL, 'c'},
    { "calibration", no_argument, NULL, 'k'},
    { "bootloader", no_argument, NULL, 'b'},
    { "joystick", no_argument, NULL, 'j'},
    { "external drive", no_argument, NULL, 'x'},
    { "set_pos_stiff", required_argument, NULL, 'e'},
    { "get_velocities", no_argument, NULL, 'i'},
    { "get_accelerations", no_argument, NULL, 'o'},
    { "set_cuff_inputs", no_argument, NULL, 'u'},
   // { "set_baudrate", required_argument, NULL, 'R'},
    { "baudrate", required_argument, NULL, 'B'},
    { "set_watchdog", required_argument, NULL, 'W'},
    { "polling", no_argument, NULL, 'P'},
    { NULL, no_argument, NULL, 0 }
};

static const char *optString = "s:adgptvh?f:ljqxzkycbe:uoiW:PB:";

struct global_args {
    int device_id;
    int flag_set_inputs;            ///< ./qbadmin -s option 
    int flag_get_measurements;      ///< ./qbadmin -g option 
    int flag_activate;              ///< ./qbadmin -a option 
    int flag_deactivate;            ///< ./qbadmin -d option 
    int flag_ping;                  ///< ./qbadmin -p option 
    int flag_serial_port;           ///< ./qbadmin -t option 
    int flag_verbose;               ///< ./qbadmin -v option 
    int flag_file;                  ///< ./qbadmin -f option 
    int flag_log;                   ///< ./qbadmin -l option
    int flag_get_emg;               ///< ./qbadmin -q option to get the EMG sensors measurements 
    int flag_set_zeros;             ///< ./qbadmin -z option 
    int flag_use_gen_sin;           ///< ./qbadmin -y option
    int flag_calibration;           ///< ./qbadmin -k option to start a series of hand closures and openings 
    int flag_get_currents;          ///< ./qbadmin -c option 
    int flag_bootloader_mode;       ///< ./qbadmin -b option 
    int flag_set_pos_stiff;         ///< ./qbadmin -e option 
    int flag_get_velocities;        ///< ./qbadmin -i option 
    int flag_get_accelerations;     ///< ./qbadmin -o option
    int flag_set_cuff_inputs;       ///< ./qbadmin -u option
    int flag_set_baudrate;          ///< ./qbadmin -R option 
    int flag_set_watchdog;          ///< ./qbadmin -W option 
    int flag_polling;               ///< ./qbadmin -P option 
    int flag_baudrate;              ///< ./qbadmin -B option 
    int flag_get_joystick;          ///< ./qbadmin -j option
    int flag_ext_drive;             ///< ./qbadmin -x option

    short int inputs[NUM_OF_MOTORS];
    short int measurements[4];
    short int velocities[4];
    short int accelerations[4];
    short int measurement_offset[4];
    short int currents[NUM_OF_MOTORS];
    char filename[255];
    char log_file[255];
    short int calib_speed;             ///< Calibration speed
    short int calib_repetitions;       ///< Calibration repetitions
    short int emg[NUM_OF_EMGS];        ///< Emg sensors values read from the device
    short int joystick[2];             ///< Analog joystick measurements
    short int ext_drive;

    short int BaudRate;
    int save_baurate;
    short int WDT;

    FILE* emg_file;
    FILE* log_file_fd;
} global_args;  //multiple boards on multiple usb

struct position {
    float prec;
    float act;
} p1, p2;


//==========================================================    global variables

uint8_t resolution[4];         // sensors resolution set on the board

int ret;                                    //utility variable to store return values
int aux_int;

comm_settings comm_settings_1;


//=====================================================     function declaration

int open_port();
int port_selection();
int polling();


/** Display program usage, and exit.
 */
void display_usage( void );

/** Parse csv input file with values to be sent to the motors
 */
float** file_parser(char*, int*, int*);

/** CTRL-c handler 1
 */
void int_handler(int sig);

/** CTRL-c handler 2
 */
void int_handler_2(int sig);

/** CTRL-c handler 3
 */
void int_handler_3(int sig);

/** Baudrate functions
 */
int baudrate_reader();
int baudrate_writer(const int);




//==============================================================================
//                                                                     main loop
//==============================================================================


/** main loop
 */
int main (int argc, char **argv)
{

    int  i = 0;             // global counters
    int  k = 0;

    char aux_string[10000]; // used to store PING reply
    int  aux[3];             // used to store input during set_inputs

    int  option;             // used for processing options
    int  longIndex = 0;

    int sensor_num = 0;

    char aux_char;

    // initializations

    global_args.device_id               = 0;
    global_args.flag_serial_port        = 0;
    global_args.flag_ping               = 0;
    global_args.flag_verbose            = 0;
    global_args.flag_activate           = 0;
    global_args.flag_deactivate         = 0;
    global_args.flag_get_measurements   = 0;
    global_args.flag_set_inputs         = 0;
    global_args.flag_file               = 0;
    global_args.flag_log                = 0;
    global_args.flag_set_zeros          = 0;
    global_args.flag_use_gen_sin        = 0;
    global_args.flag_bootloader_mode    = 0;
    global_args.flag_calibration        = 0;
    global_args.flag_set_pos_stiff      = 0;
    global_args.flag_get_velocities     = 0;
    global_args.flag_get_accelerations  = 0;
    global_args.flag_set_baudrate       = 0;
    global_args.flag_set_watchdog       = 0;
    global_args.flag_polling            = 0;
    global_args.flag_baudrate           = 0;
    global_args.flag_set_cuff_inputs    = 0;
    global_args.flag_get_joystick       = 0;
    global_args.flag_ext_drive          = 0;
    global_args.flag_get_emg            = 0;

    global_args.BaudRate                = baudrate_reader();

    //===================================================     processing options

    while ((option = getopt_long( argc, argv, optString, longOpts, &longIndex )) != -1)
    {
        switch (option)
        {
            case 's':
                sscanf(optarg,"%d,%d", &aux[0], &aux[1]);
                global_args.inputs[0] = (short int) aux[0];
                global_args.inputs[1] = (short int) aux[1];
                global_args.flag_set_inputs = 1;
                break;
            case 'e':
                sscanf(optarg,"%d,%d", &aux[0], &aux[1]);
                global_args.inputs[0] = (short int) aux[0];
                global_args.inputs[1] = (short int) aux[1];
                global_args.flag_set_pos_stiff = 1;
                break;
            case 'g':
                global_args.flag_get_measurements = 1;
                break;
            case 'a':
                global_args.flag_activate = 1;
                break;
            case 'd':
                global_args.flag_deactivate = 1;
                break;
            case 't':
                port_selection();
                break;
            case 'p':
                global_args.flag_ping = 1;
                break;
            case 'v':
                global_args.flag_verbose = 1;
                break;
            case 'f':
                sscanf(optarg, "%s", global_args.filename);
                global_args.flag_file = 1;
                break;
            case 'l':
                global_args.flag_log = 1;
                break;
            case 'q':
                global_args.flag_get_emg = 1;
                break;
            case 'y':
                global_args.flag_use_gen_sin = 1;
                break;
            case 'z':
                global_args.flag_set_zeros = 1;
                break;
            case 'c':
                global_args.flag_get_currents = 1;
                break;
            case 'b':
                global_args.flag_bootloader_mode = 1;
                break;
            case 'i':
                global_args.flag_get_velocities = 1;
                break;
            case 'o':
                global_args.flag_get_accelerations = 1;
                break;
            case 'k':
                printf("Specify speed [0 - 200]: ");
                scanf("%d", &aux_int);
                global_args.calib_speed = (short int)aux_int;
                printf("Specify repetitions [0 - 32767]: ");
                scanf("%d", &aux_int);
                global_args.calib_repetitions = (short int)aux_int;
                global_args.flag_calibration = 1;
                break;
            case 'u':
                global_args.flag_set_cuff_inputs = 1;
                break;
            case 'j':
                global_args.flag_get_joystick = 1;
                break;
            case 'x':
                global_args.flag_ext_drive = 1;
                break;
            case 'R':
                sscanf(optarg,"%d", &aux[0]);
                global_args.BaudRate = (short int) aux[0];

                global_args.flag_set_baudrate = 1;
                break;
            case 'W':
                sscanf(optarg,"%d", &aux[0]);
                global_args.WDT = (short int) aux[0];
                global_args.flag_set_watchdog = 1;
                break;
            case 'P':
                global_args.flag_polling = 1;
                break;
            case 'B':
                sscanf(optarg,"%d", &aux[0]);
                global_args.flag_baudrate = 1;
                global_args.save_baurate = (int) aux[0];
                break;
            case 'h':
            case '?':
            default:
                display_usage();
                return 0;
                break;
        }
    }

    if((optind == 1) | (global_args.flag_verbose & (optind == 2)))
    {
        display_usage();
        return 0;
    }

    //==================================================================     polling
    
    if (global_args.flag_polling)
        assert(polling());
    else{     
        if (!open_port()) {
            assert(port_selection());
     
            if (global_args.flag_polling)
                assert(polling());
            else
                assert(open_port());
        }
    }

    //====================================================     getting device id

    if (argc - optind == 1)
    {
        sscanf(argv[optind++],"%d",&global_args.device_id);
        if(global_args.flag_verbose)
            printf("Device ID:%d\n", global_args.device_id);
    }
    else if(global_args.flag_verbose)
        puts("No device ID was chosen. Running in broadcasting mode.");

    //=================================================================     ping

    // If ping... then DOESN'T PROCESS OTHER COMMANDS

    if(global_args.flag_ping)
    {
        if(global_args.flag_verbose)
            puts("Pinging serial port.");

        if(global_args.device_id) {
            commGetInfo(&comm_settings_1, global_args.device_id, INFO_ALL, aux_string);
        } else {
            RS485GetInfo(&comm_settings_1,  aux_string);
        }

       puts(aux_string);

        if(global_args.flag_verbose)
            puts("Closing the application.");

        return 0;
    }


//===============================================================     set inputs

    if(global_args.flag_set_inputs)
    {
        if(global_args.flag_verbose)
            printf("Setting inputs to %d and %d.\n", global_args.inputs[0], global_args.inputs[1]);

        commSetInputs(&comm_settings_1, global_args.device_id, global_args.inputs);
    }


//============================================================     set pos stiff

    if(global_args.flag_set_pos_stiff)
    {
        if(global_args.flag_verbose)
            printf("Setting pos to %d and stiffness to %d.\n", global_args.inputs[0], global_args.inputs[1]);

        commSetPosStiff(&comm_settings_1, global_args.device_id, global_args.inputs);
    }

//===========================================================     set Cuff inputs

    if(global_args.flag_set_cuff_inputs)
    {
        /*
        if(global_args.flag_verbose)
            printf("Setting Cuff working modality\n");
        printf("Do you want to Activate [1] or Deactivate [0] the Cuff device?\n");
        scanf("%c", &aux_char);
        if(aux_char == '1')
            commSetCuffInputs(&comm_settings_1, global_args.device_id, 1);
        else
            commSetCuffInputs(&comm_settings_1, global_args.device_id, 0);
        */
    }


//=========================================================     get measurements

    if(global_args.flag_get_measurements)
    {
        if(global_args.flag_verbose)
            puts("Getting measurements.");

        while(1) {
            sensor_num = commGetMeasurements(&comm_settings_1, global_args.device_id, global_args.measurements);

            if(sensor_num < 0 || sensor_num > 4) {
                printf("An error occurred or the device is not supported\n");
                break;
            }
            else {
                printf("measurements:     ");
                for (i = 0; i < sensor_num; i++) {
                    printf("%d  ", (int)global_args.measurements[i]);
                }
                printf("\n");

                usleep(100000);
            }
        }
    }


//===========================================================     get velocities

    if(global_args.flag_get_velocities)
    {
        if(global_args.flag_verbose)
            puts("Getting velocities.");


        while(1) {
            sensor_num = commGetVelocities(&comm_settings_1, global_args.device_id, global_args.velocities);

            if(sensor_num < 0 || sensor_num > 4) {
                printf("An error occurred or the device is not supported\n");
                break;
            }
            else {
                printf("velocities:     ");
                for (i = 0; i < sensor_num; i++) {
                    printf("%d  ", (int)global_args.velocities[i]);
                }
                printf("\n");

                usleep(100000);
            }
        }
    }

//===========================================================     get accelerations

    if(global_args.flag_get_accelerations)
    {
        if(global_args.flag_verbose)
            puts("Getting accelerations.");

        while(1) {
            sensor_num = commGetAccelerations(&comm_settings_1, global_args.device_id, global_args.accelerations);

            if(sensor_num < 0 || sensor_num > 4) {
                printf("An error occurred or the device is not supported\n");
                break;
            }
            else {
                printf("accelerations:     ");
                for (i = 0; i < sensor_num; i++) {
                    printf("%hd  ", global_args.accelerations[i]);
                }
                printf("\n");

                usleep(100000);
            }
        }
    }

//==============================================================     joystick

    if(global_args.flag_get_joystick) {
        while(1) {
            ret = commGetJoystick(&comm_settings_1, global_args.device_id, global_args.joystick);
            
            if(ret < 0) {
                printf("An error occurred or the device has no joystick\n");
                break;
            }
            else {
                printf("Joystick Meas - X:%hd  Y:%hd\n", global_args.joystick[0], global_args.joystick[1]);
                usleep(100000);
            }
        }
    }

//==========================================================     bootloader mode

    if(global_args.flag_bootloader_mode)
    {
        printf("Are you sure you want to enter in bootloader mode? y/[N]\n");
        scanf("%c", &aux_char);
        if(aux_char == 'y' || aux_char == 'Y') {
            printf("Entering bootloader mode\n");
            if(commBootloader(&comm_settings_1, global_args.device_id) >= 0)
                printf("DONE\n");
            else
                printf("An error occurred.\nRetry.\n");
        }
    }

//==============================================================     calibration

    if (global_args.flag_calibration) {

        if(commHandCalibrate(&comm_settings_1, global_args.device_id, global_args.calib_speed, global_args.calib_repetitions) < 0)
            puts("An error occured or the device does not supports this calibration");
        else
            printf("Speed: %d     Repetitions: %d\n", global_args.calib_speed, global_args.calib_repetitions);
        usleep(100000);

    }

//==============================================================     ext_drive

    if(global_args.flag_ext_drive) {
        printf("Activate [1] or Deactivate [0] external drive command?\n");
        scanf("%c", &aux_char);
        if(aux_char == '1') {
            ret = commExtDrive(&comm_settings_1, global_args.device_id, 1);
            if(ret < 0)
                printf("An error occurred or the device does not support this functionality\n");
        }
        else {
            ret = commExtDrive(&comm_settings_1, global_args.device_id, 0);
            if(ret < 0)
                printf("An error occurred or the device does not support this functionality\n");
        }
    } 

//==========================================================     get_currents

    if(global_args.flag_get_currents)
    {
        if(global_args.flag_verbose)
            puts("Getting currents.");

        while(1) {
            commGetCurrents(&comm_settings_1, global_args.device_id, global_args.currents);

            printf("Current 1: %hd\t Current 2: %hd\n", global_args.currents[0], global_args.currents[1]);
            fflush(stdout);
            usleep(100000);
        }
    }

//==========================================================     get_emg

    struct timeval t_act, begin;
    struct timezone foo;
    int delta_t = 1500; //in microseconds

    i = 0;

    if(global_args.flag_get_emg) {
        if(global_args.flag_verbose) {
            puts("Getting emg signals.");
        }

        signal(SIGINT, int_handler_3);
        global_args.emg_file = fopen(EMG_SAVED_VALUES, "w");

        // Initialize begin time
        gettimeofday(&begin, &foo);

        while(1) {
            i++;

            // Getting EMG values
            if(commGetEmg(&comm_settings_1, global_args.device_id, global_args.emg) < 0) {
                puts("An error occurred or the device has no EMG functionality");
                break;
            }
            if(global_args.flag_verbose) {
                printf("Signal 1: %d\t Signal 2: %d\n", global_args.emg[0], global_args.emg[1]);
            }

#define REFERENCES

#ifdef REFERENCES

            // Getting current references
            commGetInputs(&comm_settings_1, global_args.device_id, global_args.inputs);
            fprintf(global_args.emg_file, "%d,%d,%d,%d\n", global_args.emg[0],
                    global_args.emg[1], global_args.inputs[0],
                    (int)timevaldiff(&begin, &t_act));

#else //current position

            // Getting current position
            commGetMeasurements(&comm_settings_1, global_args.device_id, global_args.measurements);
            fprintf(global_args.emg_file, "%d,%d,%d,%d\n", global_args.emg[0],
                    global_args.emg[1], global_args.measurements[0],
                    (int)timevaldiff(&begin, &t_act));

#endif

            // Wait for actual time to be grater than begin + i * delta_t
            while(1) {
                gettimeofday(&t_act, &foo);
                if (timevaldiff(&begin, &t_act) >= i * delta_t) {
                    break;
                }
            }

        }
    }


//=================================================================     activate

    if(global_args.flag_activate)
    {
        if(global_args.flag_verbose)
            puts("Turning device on.\n");
        
        commActivate(&comm_settings_1, global_args.device_id, 1);
        usleep(1000);
        commGetActivate(&comm_settings_1, global_args.device_id, &aux_char);

        printf("%c %d\n", aux_char, (int)aux_char);
    }


//===============================================================     deactivate

    if(global_args.flag_deactivate)
    {
        if(global_args.flag_verbose)
           puts("Turning device off.\n");

        commActivate(&comm_settings_1, global_args.device_id, 0);
    }

//===============================================================     baudrate

    if(global_args.flag_baudrate)
    {
        if(global_args.flag_verbose)
           puts("Save BaudRate.\n");

        if ((global_args.save_baurate != 460800) && (global_args.save_baurate != 2000000)){
            puts("Set default BaudRate, 2000000.");
            global_args.save_baurate = 2000000;
        }

        
        baudrate_writer(global_args.save_baurate);
    }


//=================================================================     watchdog

    if(global_args.flag_set_watchdog)
    {
        if(global_args.flag_verbose)
            puts("Set Watchdog timer to \n");

        if (global_args.WDT > MAX_WATCHDOG_TIME){
            printf("Watchdog Saturated to %d.\n", MAX_WATCHDOG_TIME);
            global_args.WDT = MAX_WATCHDOG_TIME;
        }
        else
            if (global_args.flag_verbose && (global_args.WDT <= 0)){
                puts("Watchdog DISABLED.\n");
                global_args.WDT = 0;
            }

        commSetWatchDog(&comm_settings_1, global_args.device_id, global_args.WDT);

    }


//==============================================================     use_gen_sin

    if(global_args.flag_use_gen_sin)
    {
        //variable declaration
        float delta_t;                      // milliseconds between values
        float amplitude_1, amplitude_2;     // sinusoid amplitude
        float bias_1, bias_2;               // sinusoid bias
        float freq_1, freq_2;               // sinusoid frequency
        float period_1, period_2;           // sinusoid period = 1/frequency
        float phase_shift;                  // angular shift between sinusoids
        float total_time;                   // total execution time (if 0 takes
                                            //   number of values as parameter)
        int num_values;                     // number of values (ignored if
                                            //   total time != 0)

        float angle_1 = 0;                  // actual angle
        float angle_2 = 0;                  // actual angle
        float inc_1, inc_2;                 // angle increment for every step

        struct timeval t_act, begin, end;
        struct timezone foo;

        int error_counter = 0;
        int do_once = 1;
        int sensor_num = 0;

        if(global_args.flag_log) {
            strcpy(global_args.log_file, "sin_log.csv");
            global_args.log_file_fd = fopen(global_args.log_file, "w");
        }

        // CTRL-C handler
        signal(SIGINT, int_handler);

        if(global_args.flag_verbose) {
           puts("Generate sinusoidal inputs\n");
        }

        // opening file
        FILE* filep;
        filep = fopen(SIN_FILE, "r");
        if (filep == NULL) {
            printf("Failed opening file\n");
        }

        fscanf(filep, "delta_t %f\n", &delta_t);
        fscanf(filep, "amplitude_1 %f\n", &amplitude_1);
        fscanf(filep, "amplitude_2 %f\n", &amplitude_2);
        fscanf(filep, "bias_1 %f\n", &bias_1);
        fscanf(filep, "bias_2 %f\n", &bias_2);
        fscanf(filep, "freq_1 %f\n", &freq_1);
        fscanf(filep, "freq_2 %f\n", &freq_2);
        fscanf(filep, "phase_shift %f\n", &phase_shift);
        fscanf(filep, "total_time %f\n", &total_time);
        fscanf(filep, "num_values %d\n", &num_values);

        // closing file
        fclose(filep);

        // if total_time set, calculate num_values
        if (total_time != 0) {
            num_values = (total_time*1000)/delta_t;
            printf("Num_values: %d\n", num_values);
        }

        // calculate periods
        period_1 = 1/freq_1;
        period_2 = 1/freq_2;

        // deg to rad
        phase_shift = phase_shift * PI / 180.0;

        // calculate increment for every step
        inc_1 = (2 * PI) / (period_1 / (delta_t / 1000.0));
        inc_2 = (2 * PI) / (period_2 / (delta_t / 1000.0));

        // activate motors
        commActivate(&comm_settings_1, global_args.device_id, 1);

        // retrieve begin time
        gettimeofday(&begin, &foo);

        for(i=0; i<num_values; i++) {
            // wait for next value
            while (1) {
                gettimeofday(&t_act, &foo);
                if (timevaldiff(&begin, &t_act) >= i * delta_t * 1000) {
                    break;
                }
            }
            printf("Time: %ld\n", timevaldiff(&begin, &t_act));
            // update measurements
            sensor_num = commGetMeasurements(&comm_settings_1, global_args.device_id,
                    global_args.measurements);
            if (sensor_num < 0) {
                error_counter++;
            }

            // update inputs
            global_args.inputs[0] = (sin(angle_1)*amplitude_1 + bias_1);
            global_args.inputs[1] = (sin(angle_2 + phase_shift)*amplitude_2 + bias_2);

            // set new inputs
            commSetInputs(&comm_settings_1, global_args.device_id, global_args.inputs);

            // update angle position
            angle_1 += inc_1;
            angle_2 += inc_2;

            //Get currents to save in log file
            commGetCurrents(&comm_settings_1, global_args.device_id, global_args.currents);
            //log file
            if (global_args.flag_log) {
                if (do_once){
                    fprintf(global_args.log_file_fd, "t(ms)\t I[0],\tI[1],\tM[0],\tM[1],\tM[2],\tC[0],\tC[1]\n");
                    do_once = 0;
                }
		fprintf(global_args.log_file_fd, "%ld\t", timevaldiff(&begin, &t_act)/1000);
                fprintf(global_args.log_file_fd, "%d,\t%d\t",
                    global_args.inputs[0], global_args.inputs[1]);
                for (k = 0; k < sensor_num; k++) {
                    fprintf(global_args.log_file_fd, "%d,\t", global_args.measurements[k]);
                }
                fprintf(global_args.log_file_fd, "%d,\t%d\n",
                    global_args.currents[0], global_args.currents[1]);
            }

        }

        // get time at the end of for cycle
        gettimeofday(&end, &foo);

        // reset motor  position
        global_args.inputs[0] = 0;
        global_args.inputs[1] = 0;
        commSetInputs(&comm_settings_1, global_args.device_id,
                global_args.inputs);

        printf("total time (millisec): %f\n", timevaldiff(&begin, &end)/1000.0);
        printf("Error counter: %d\n", error_counter);

    }


//===============================================================     input file

    if(global_args.flag_file)
    {
        // variable declaration
        struct timeval t_ref, t_act, begin, end;
        struct timezone foo;
        int deltat = 0; //time interval in millisecs
        int num_values = 0;
        float** array;
        char filename[255];
        char* extension;
        char* name;
        int error_counter = 0;
        int sensor_num = 0;

        // VERBOSE ONLY
        if(global_args.flag_verbose) {
            printf("Parsing file %s\n", global_args.filename);
        }

        // parsing file
        array = file_parser(global_args.filename, &deltat, &num_values);

        // VERBOSE ONLY
        if(global_args.flag_verbose)
            printf("Sending %d values with Dt = %d\n", num_values, deltat);

        //if log enabled, open file for logging
        if(global_args.flag_log) {
            strcpy(filename, global_args.filename);
            name = strtok(filename, ".");
            extension = strtok(NULL, ".");
            strcpy(global_args.log_file, name);
            strcat(global_args.log_file, "_log.");
            strcat(global_args.log_file, extension);
            global_args.log_file_fd = fopen(global_args.log_file, "w");
        }

        //retrieve current time
        gettimeofday(&t_ref, &foo);
        gettimeofday(&begin, &foo);

        signal(SIGINT, int_handler);

        
        sensor_num = commGetMeasurements(&comm_settings_1, global_args.device_id, global_args.measurements);
        // write first line of log file
        if (global_args.flag_log) {
            for (k = 0; k < sensor_num; k++) {
                    fprintf(global_args.log_file_fd, "sensor_%d,\t", (k + 1));
                }
            fprintf(global_args.log_file_fd, "input_1,\tinput_2,\t");
            fprintf(global_args.log_file_fd, "current_1,\tcurrent_2\n");
        }

        for(i=0; i<num_values; i++) {
            while (1) {
                gettimeofday(&t_act, &foo);
                if (timevaldiff(&begin, &t_act) >= i*deltat*1000) {
                    break;
                }
            }

            sensor_num = commGetMeasurements(&comm_settings_1, global_args.device_id, global_args.measurements);

            // update measurements
            if (sensor_num < 0) {
                error_counter++;
            }


            // update inputs
            global_args.inputs[0] = array[0][i];
            global_args.inputs[1] = array[1][i];

            // set new inputs
            commSetInputs(&comm_settings_1, global_args.device_id,
                    global_args.inputs);

            // write measurements in log file
            if (global_args.flag_log) {
                for (k = 0; k < sensor_num; k++) {
                    fprintf(global_args.log_file_fd, "%d,\t", global_args.measurements[k]);
                }
                fprintf(global_args.log_file_fd, "%f,\t%f,\t",
                    array[0][i], array[1][i]);
                fprintf(global_args.log_file_fd, "%d,\t%d\n",
                    global_args.currents[0], global_args.currents[1]);
            }
        }

        //get time at the end to verify correct execution
        gettimeofday(&end, &foo);


        //free dynamically allocated memory in parser
        free(array[0]);
        free(array[1]);
        free(array);

        //if necessary close log file
        if (global_args.flag_log) {
            fclose(global_args.log_file_fd);
        }

        //at the end, set motors to 0
        usleep(500000);
        global_args.inputs[0] = 0;
        global_args.inputs[1] = 0;
        commSetInputs(&comm_settings_1, global_args.device_id,
                global_args.inputs);

        printf("total time (usec): %d\n", (int)timevaldiff(&begin, &end));
        printf("Error counter %d\n", error_counter);
    }


    //============================================================     set zeros
    if(global_args.flag_set_zeros)
    {
        struct timeval t_prec, t_act;
        struct timezone foo;

        signal(SIGINT, int_handler_2);
        printf("Press CTRL-C to set Zero Position\n\n");
        printf("Press return to proceed\n");
        getchar();

        sensor_num = commGetMeasurements(&comm_settings_1, global_args.device_id, global_args.measurements);

        // Deactivate device to avoid motor movements
        commActivate(&comm_settings_1, global_args.device_id, 0);

        // Reset all the offsets
        for (i = 0; i < sensor_num; i++) {
            global_args.measurement_offset[i] = 0;
        }

        commSetZeros(&comm_settings_1, global_args.device_id, 
                    global_args.measurement_offset, sensor_num);


        //Display current values until CTRL-C is pressed
        gettimeofday(&t_prec, &foo);
        gettimeofday(&t_act, &foo);
        while(1) {
            while (1) {
                gettimeofday(&t_act, &foo);
                if (timevaldiff(&t_prec, &t_act) >= 200000) {
                    break;
                }
            }
            commGetMeasurements(&comm_settings_1, global_args.device_id,
                    global_args.measurements);
            for (i = 0; i < sensor_num; i++) {
                printf("%d\t", global_args.measurements[i]);
            }
            printf("\n");

            gettimeofday(&t_prec, &foo);
        }
    }
    //============================================================     baudrate

    if (global_args.flag_set_baudrate){


        if (((int) global_args.BaudRate == BAUD_RATE_T_460800) || ((int) global_args.BaudRate == BAUD_RATE_T_2000000))
            commSetBaudRate(&comm_settings_1, global_args.device_id, global_args.BaudRate);
        else
            printf("BaudRate request not supported. \n 0 -> 2000000 \n 1 -> 460800\n");

    }


//==========================     closing serial port and closing the application


    closeRS485(&comm_settings_1);

    if(global_args.flag_verbose)
        puts("Closing the application.");

#ifdef PHIDGETS_BRIDGE
    CPhidget_close((CPhidgetHandle)bridge);
    CPhidget_delete((CPhidgetHandle)bridge);
#endif

    return 1;
}


//==============================================================================
//                                                                port_selection
//==============================================================================

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

//==============================================================================
//                                                               baudrate_writer
//==============================================================================

int baudrate_writer(const int baudrate) {

    FILE *file;

    file = fopen(QBMOVE_FILE_BR, "w+");
    
    if (file == NULL) 
        printf("Cannot open qbmove.conf\n");

    fprintf(file,"baudrate %d\n", baudrate);
    fclose(file);
    
    return 1;

}


//==============================================================================
//                                                               baudrate_reader
//==============================================================================

int baudrate_reader(){

    int br = 0;
    FILE* file;

    if(global_args.flag_verbose)
        puts("Reading BaudRate configuration files.");

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
//                                                                     open_port
//==============================================================================

int open_port() {
    FILE *file;
    char port[255];

    file = fopen(QBMOVE_FILE, "r");

    if (file == NULL) {
        printf("Error opening file %s\n", QBMOVE_FILE);
        return 0;
    }

    fscanf(file, "serialport %s\n", port);

    fclose(file);


    #if !(defined(_WIN32) || defined(_WIN64)) && !(defined(__APPLE__)) //only for linux

        if (global_args.BaudRate == BAUD_RATE_T_460800)
            openRS485(&comm_settings_1, port , B460800);
        else
            openRS485(&comm_settings_1, port , B2000000);
    #else
        if (global_args.BaudRate == BAUD_RATE_T_460800)
           openRS485(&comm_settings_1, port , 460800);
        else
            openRS485(&comm_settings_1, port , 2000000);
    #endif

    if(comm_settings_1.file_handle == INVALID_HANDLE_VALUE)
    {
        puts("Couldn't connect to the serial port.");
        return 0;
    }
    usleep(100000);

    return 1;
}


//==============================================================================
//                                                                       polling
//==============================================================================

int polling() {
    FILE *file;
    char port[255];
    bool device_number = false;
    short int meas[3];
    int i, id;
    int sensor_num = 0;

    file = fopen(QBMOVE_FILE, "r");

    if (file == NULL) {
        printf("Error opening file %s\n", QBMOVE_FILE);
        return 0;
    }

    fscanf(file, "serialport %s\n", port);

    fclose(file);

#if !(defined(_WIN32) || defined(_WIN64)) && !(defined(__APPLE__)) //only for linux
    openRS485(&comm_settings_1, port , B460800);
#else
    openRS485(&comm_settings_1, port , 460800);
#endif

        // Scan for 460800 Baud Rate devices
        printf("Devices Connect: BaudRate = 460800\n");
        printf("ID\tPos1\tPos2\tPosL\n");
        printf("=============================\n");

        for (id = 1; id < 128; ++id){
            sensor_num = commGetMeasurements(&comm_settings_1, id, meas);
            if (sensor_num > 0){

                printf("%d\t", id);
                for (i = 0; i < sensor_num; ++i) 
                    printf("%d\t", (int) meas[i]);

                printf("\n");

                device_number = true;
            }
        }
        if (device_number)
            printf("-----------------------------\n");
        else
            printf("NO DEVICE FOUND!\n\n");

    closeRS485(&comm_settings_1);

#if !(defined(_WIN32) || defined(_WIN64)) && !(defined(__APPLE__)) //only for linux
    openRS485(&comm_settings_1, port , B2000000);
#else
    openRS485(&comm_settings_1, port , 2000000);
#endif

        device_number = false;

        // Scan for 2000000 Baud Rate devices
        printf("Devices Connect: BaudRate = 2000000\n");
        printf("ID\tPos1\tPos2\tPosL\n");
        printf("=============================\n");

        for (id = 1; id < 128; ++id){
            sensor_num = commGetMeasurements(&comm_settings_1, id, meas);
            if (sensor_num > 0){

                printf("%d\t", id);
                for (i = 0; i < sensor_num; ++i) 
                    printf("%d\t", (int) meas[i]);

                printf("\n");

                device_number = true;
            }
        }
        if (device_number)
            printf("-----------------------------\n");
        else
            printf("NO DEVICE FOUND!\n\n");
    
    closeRS485(&comm_settings_1);

    if(comm_settings_1.file_handle == INVALID_HANDLE_VALUE)
    {
        puts("Couldn't connect to the serial port.");
        return 0;
    }
    usleep(100000);

    return 1;
}


//==============================================================================
//                                                                   file_parser
//==============================================================================

/** Parse CSV file and return a pointer to a matrix of float dinamically
 *  allocated.  Remember to use free(pointer) in the caller
 */

float** file_parser( char* filename, int* deltat, int* num_values )
{
    FILE* filep;
    float** array = NULL;
    int i;
    filep = fopen(filename, "r");
    if (filep == NULL) perror ("Error opening file");
    else {
        //read first line
        fscanf(filep, "%d,%d", deltat, num_values);

        //alloc memory for the arrays
        array = (float**)malloc(2*sizeof(float*));
        array[0] = (float*)malloc(*num_values*sizeof(float));
        array[1] = (float*)malloc(*num_values*sizeof(float));

        //read num_values line of file and store them in array
        for(i=0; i<*num_values; i++) {
            fscanf(filep, "%f,%f", &array[0][i], &array[1][i]);
        }
    fclose(filep);
    }
    return array;
}

//==============================================================================
//                                                          CTRL-C interruptions
//==============================================================================

/** handle CTRL-C interruption 1
*/
void int_handler(int sig) {
    printf("\nForced quit!!!\n");

    //if necessary close log file
    if (global_args.flag_log) {
        fclose(global_args.log_file_fd);

        // erase last line of log file  /////////////BEGIN
        const char *tmpfilename = "tmpfile~~~";
        char line[1000];
        char command[256];
        FILE *thefile = fopen(global_args.log_file, "r");
        FILE *tmpfile = fopen(tmpfilename, "w");

        while (fgets(line, sizeof(line), thefile))
          if (!feof(thefile))
            fputs(line, tmpfile);

        fclose(tmpfile);
        fclose(thefile);

        strcpy(command, "mv ");
        strcat(command, tmpfilename);
        strcat(command, " ");
        strcat(command, global_args.log_file);

        system(command);
        // erase last line of log file  ///////////////END
    }

    // set motors to 0,0
    global_args.inputs[0] = 0;
    global_args.inputs[1] = 0;
    commSetInputs(&comm_settings_1, global_args.device_id, global_args.inputs);


    exit(1);
}

/** handle CTRL-C interruption 2
*/
void int_handler_2(int sig) {
    int i;
    short int temp_meas[4];
    int sensor_num  = 0;

    sensor_num = commGetMeasurements(&comm_settings_1, global_args.device_id, temp_meas);

    if(sensor_num > 0 && sensor_num < 4) {
        printf("\n\nSetting zero position\n");

        //Set the offsets equal to minus current positions
        for (i = 0; i < sensor_num; i++) {
            global_args.measurement_offset[i] = -global_args.measurements[i];
        }

        if(commSetZeros(&comm_settings_1, global_args.device_id, global_args.measurement_offset, sensor_num)) {
            printf("\nAn error occurred while setting measurements offsets. Retry.\n");
            exit(1);
        }

        if (commStoreParams(&comm_settings_1, global_args.device_id)) {
            printf("Error saving params\n");
            exit(1);
        }

        sleep(1);

        // set motors to 0,0
        global_args.inputs[0] = 0;
        global_args.inputs[1] = 0;
        commSetInputs(&comm_settings_1, global_args.device_id, global_args.inputs);
    }
    else
        printf("Number of sensors not supported\n");

    exit(1);
}

/** Handles the ctrl+c interruption to save the emg sensors measurements into a file
*/

void int_handler_3(int sig) {
    printf("Closing file and quitting application...\n");

    fclose(global_args.emg_file);

    printf("DONE\n");

    exit(1);
}

//==============================================================================
//                                                                 display usage
//==============================================================================

/** Display program usage, and exit.
*/

void display_usage( void )
{
    puts("================================================================================");
    printf("qbadmin version: %s\n", QBADMIN_VERSION);
    puts("================================================================================");
    puts("usage: qbadmin [id] [OPTIONS]" );
    puts("--------------------------------------------------------------------------------");
    puts("Options:");
    puts("");
    puts("================================================================================");
    puts("General commands");
    puts("================================================================================");
    puts(" -h, --help                       Shows this information.");
    puts(" -p, --ping                       Get info on the device.");
    puts(" -t, --serial_port                Set up serial port.");
    puts(" -W, --set_watchdog               Set up Watchdog ");
    puts("                                  [0 - 500] with step rate of 2 [cs]).");
    puts(" -P, --polling                    Call a polling search.");
    puts(" -B, --baudrate <value>           Set Baudrate communication "); 
    puts("                                  [460800 or 2000000].");
    puts(" -b, --bootloader                 Enter bootloader mode to update firmware.");
    puts(" -v, --verbose                    Verbose mode.");
    puts(" -s, --set_inputs <value,value>   Send reference inputs to the board.");
    puts(" -g, --get_measurements           Get measurements from the board.");
    puts(" -i, --get_velocities             Get velocities from the board.");
    puts(" -o, --get_accelerations          Get accelerations for the board");
    puts(" -c, --get_currents               Get motor currents");
    puts(" -a, --activate                   Activate the QB Move.");
    puts(" -d, --deactivate                 Deactivate the QB Move.");
    puts(" -z, --set_zeros                  Set zero position for all sensors");
    puts(" -e, --set_pos_stiff <pos,stiff>  Set position (degree) and stiffness (\%)");
    puts(" -y, --use_gen_sin                Sinusoidal inputs using sin.conf file");
    puts(" -f, --file <filename>            Pass a CSV file as input");
    puts("                                  File is in the form:");
    puts("                                  millisecs,num_rows");
    puts("                                  input1_1,input2_1");
    puts("                                  input1_2,input2_2");
    puts("                                  input1_3,input2_3");
    puts("                                  ...        ...");
    puts("                                  input1_num_rows,input2_num_rows");
    puts(" -l, --log                        Use in combination with -f to");
    puts("                                  save a log of the positions in");
    puts("                                  a file named filename_log");
    //puts(" -u, --set_cuff_modality          Activates the Cuff modality if the device is a");
    //puts("                                  Cuff");
	puts(" -k, --calibration                Makes a series of opening and closing.");
    puts(" -q, --get_emg                    Get EMG values and save them in a file");
    puts("                                  defined in \"definitions.h\". Use -v option");
    puts("                                  to display values in the console too.");
    puts(" -x, --ext_drive                  Reads measurements and drives a second board.");
    puts(" -j, --get_joystick               Get joystick measurements.");
    puts("");
    puts("--------------------------------------------------------------------------------");
    puts("Examples:");
    puts("");
    puts("  qbadmin -p                      Get info on whatever device is connected.");
    puts("  qbadmin -t                      Set up serial port.");
    puts("  qbadmin 65 -s 10,10             Set inputs of device 65 to 10 and 10.");
    puts("  qbadmin 65 -g                   Get measurements from device 65.");
    puts("  qbadmin 65 -g -s 10,10          Set inputs of device 65 to 10");
    puts("                                  and 10, and get measurements.");
    puts("  qbadmin 65 -a                   Turn device 65 on.");
    puts("  qbadmin 65 -a                   Turn device 65 off.");
    puts("  qbadmin 65 -f filename          Pilot device 65 using file 'filename'");
    puts("================================================================================");
    /* ... */
    exit( EXIT_FAILURE );
}
/* [] END OF FILE */
