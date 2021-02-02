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
* \file         qbadmin.c
*
* \brief        Command line tools file
* \author       _Centro "E.Piaggio"_
* \copyright    (C) 2012-2016 qbrobotics. All rights reserved.
* \copyright    (C) 2017-2020 Centro "E.Piaggio". All rights reserved.
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
* \copyright    (C) 2017-2020 Centro "E.Piaggio". All rights reserved.
*
* \date         March 19th, 2020
*
* \details      This is a set of functions that allows to use the boards 
*               via a serial port.
*
*/
//==================================================================     defines


//=================================================================     includes

#include "../../qbAPI/src/qbmove_communications.h"
#include "../../qbAPI/src/cp_communications.h"
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
#include <io.h>

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
	{ "reading_ping", no_argument, NULL, 'r' },
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
	{"get_imu_readings", no_argument, NULL, 'Q'},
	{"get_adc_raw", no_argument, NULL, 'A'},
	{"get_encoder_raw", no_argument, NULL, 'E'},
	{"get_SD_files", no_argument, NULL, 'S'},
    {"get_SD_filesystem", no_argument, NULL, 'X'},
    { NULL, no_argument, NULL, 0 }
};

static const char *optString = "s:adgprtvh?f:ljqxzkycbe:uoiW:PB:QAESX";

struct global_args {
    int device_id;
    int flag_set_inputs;            ///< ./qbadmin -s option 
    int flag_get_measurements;      ///< ./qbadmin -g option 
    int flag_activate;              ///< ./qbadmin -a option 
    int flag_deactivate;            ///< ./qbadmin -d option 
    int flag_ping;                  ///< ./qbadmin -p option 
	int flag_reading_ping;			///< ./qbmove -r option
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
	int flag_get_imu_readings;		///< Additional -Q option
	int flag_get_adc_raw;			///< Additional -A option
	int flag_get_encoder_raw;		///< Additional -E option
	int flag_get_SD_files;			///< Additional -S option
    int flag_get_SD_filesystem;     ///< Additional -X option

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

	int n_imu;
	uint8_t* ids;
	uint8_t* imu_table;
	uint8_t* mag_cal;
    short int BaudRate;
    int save_baurate;
    short int WDT;
	
	short int* adc_raw;
	FILE* SD_param_file;
	FILE* SD_data_file;
	
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
	char aux_imu[3];

    // initializations

    global_args.device_id               = 0;
    global_args.flag_serial_port        = 0;
    global_args.flag_ping               = 0;
	global_args.flag_reading_ping       = 0;
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
	global_args.flag_get_imu_readings   = 0;
	global_args.flag_get_adc_raw   		= 0;
	global_args.flag_get_encoder_raw	= 0;
	global_args.flag_get_SD_files		= 0;
    global_args.flag_get_SD_filesystem  = 0;

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
			case 'r':
                global_args.flag_reading_ping = 1;
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
			case 'Q':
				global_args.flag_get_imu_readings = 1;
				break;
			case 'A':
				global_args.flag_get_adc_raw = 1;
				break;
			case 'E':
				global_args.flag_get_encoder_raw = 1;
				break;	
			case 'S':
				global_args.flag_get_SD_files = 1;
				break;
            case 'X':
                global_args.flag_get_SD_filesystem = 1;
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

		}
		else {
			RS485GetInfo(&comm_settings_1,  aux_string);

		}
		
		puts(aux_string);


        if(global_args.flag_verbose)
            puts("Closing the application.");

        return 0;
    }
	
	if(global_args.flag_reading_ping)
    {
        if(global_args.flag_verbose)
            puts("Pinging serial port.");

		
		commGetInfo(&comm_settings_1, global_args.device_id, INFO_READING, aux_string);
		
		puts(aux_string);

        if(global_args.flag_verbose)
            puts("Closing the application.");

        return 0;
    }
		
	if (global_args.flag_get_SD_files)
	{
		char str_param[10000] = "";
		
		if(global_args.flag_verbose)
            puts("Getting SD parameters file.");
		
		commGetInfo(&comm_settings_1, global_args.device_id, GET_SD_PARAM, str_param);
		
		//puts(str_param);
		
		global_args.SD_param_file = fopen(SD_PARAM_FILE, "w");
		fprintf(global_args.SD_param_file, "%s", str_param);
		fclose(global_args.SD_param_file);
		
		printf("SD current parameters have been saved in %s file\n", SD_PARAM_FILE);
		
		char str_data[10000] = "";
		
		if(global_args.flag_verbose)
            puts("Getting SD data file.");
		
		commGetInfo(&comm_settings_1, global_args.device_id, GET_SD_DATA, str_data);
		
		//puts(str_data);
		
		global_args.SD_data_file = fopen(SD_DATA_FILE, "w");
		fprintf(global_args.SD_data_file, "%s", str_data);
		fclose(global_args.SD_data_file);
		
		printf("SD current data have been saved in %s file\n", SD_DATA_FILE);
		
		if(global_args.flag_verbose)
            puts("Closing the application.");
		
		return 0;
	}

    if (global_args.flag_get_SD_filesystem)
    {
        char str_folder_tree[10000] = "";
        
        if(global_args.flag_verbose)
            puts("Getting SD filesystem.");

        // Called with PING command
        // Get all the folders with path and contained files number in the CSV-like format
        // e.g. rows like [USER\YYYY\MM\DD, number_of_files]
        fprintf(stdout, "Getting the SD card filesystem structure ...");
        fflush(stdout);
        commGetInfo(&comm_settings_1, global_args.device_id, GET_SD_FS_TREE, str_folder_tree);
        //printf("%s\n", str_folder_tree);
        fprintf(stdout, " OK\n");

        if (mkdir(SD_FS_FOLDER) == 0){
    
            // Parse
            char path[100] = "";
            int n_files = 0;
            int n_bytes = 0;
            int n_bytes_local = 0;
            char local_str[100] = "";
            char user[10] = "";
            char year[10] = "";
            char month[10] = "";
            char day[10] = "";
        
            strncpy(local_str, str_folder_tree, strlen(str_folder_tree));
            do {
                sscanf(local_str, "%100[^,],%d\r\n%n", path, &n_files, &n_bytes_local);
                sscanf(path, "\\%10[^\\]\\%10[^\\]\\%10[^\\]\\%10[^\\]", user, year, month, day);
                //printf("%s %d %d\n", path, n_files, n_bytes_local);    
                //printf("%s %s %s %s %d %d\n", user, year, month, day, n_files, n_bytes_local);    

                // Create each folder and move inside it
                char f_mkdir_path[100] = "";
                strcpy(f_mkdir_path, SD_FS_FOLDER);

                strcat(f_mkdir_path, user);
                mkdir(f_mkdir_path);            // create user folder
                strcat(f_mkdir_path, "\\");
                strcat(f_mkdir_path, year);
                mkdir(f_mkdir_path);            // create year folder
                strcat(f_mkdir_path, "\\");
                strcat(f_mkdir_path, month);
                mkdir(f_mkdir_path);            // create month folder
                strcat(f_mkdir_path, "\\");
                strcat(f_mkdir_path, day);
                mkdir(f_mkdir_path);            // create year folder

                // Download every file in the folder
                for (int i = 0; i < n_files; i++){

                    char str_data[20000] = "";
                    char fw_path[100] = "";
                    char filename_path[1000] = "";
                    char filename[20] = "";
                    strcpy(fw_path,  path);

                    if (i %2 == 0){
                        sprintf(filename, "\\Param_%d.csv", i/2);    
                    }
                    else {
                        sprintf(filename, "\\UseStats_%d.csv", i/2);
                    }
                    strcat(fw_path, filename);
                   
                    fprintf(stdout, "Getting the file %s ", fw_path);
                    fflush(stdout);

                    strcpy(str_data, "");
                    int ret;
                    do{
                        ret = commGetSDFile(&comm_settings_1, global_args.device_id, fw_path, str_data);
                        usleep(500000);
                        fprintf(stdout, ".");
                        fflush(stdout);
                    } while(!strcmp(str_data, ""));

                    fprintf(stdout, " OK\n");

                    if (!ret){

                        //printf("File content: %s\n", str_data);
        
                        strcat(filename_path, f_mkdir_path);
                        strcat(filename_path, filename);
                        global_args.SD_data_file = fopen(filename_path, "w");
                        fprintf(global_args.SD_data_file, "%s", str_data);
                        fclose(global_args.SD_data_file);

                    }
                    else
                        break;
                }
    
                n_bytes += n_bytes_local;
                strncpy(local_str, str_folder_tree + n_bytes, strlen(str_folder_tree) - n_bytes);            

            } while(n_bytes_local > 0);
                
            printf("SD filesystem has been saved in %s folder\n", SD_FS_FOLDER);
        }
        else {
            printf("Error in creating SD filesystem folder. Maybe you have to remove the old SD_card folder in qbadmin path first");   
        }                   

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

	
	//=========================================================  get imu readings

    if(global_args.flag_get_imu_readings)
    {
		uint8_t aux_string[5000];
		uint8_t PARAM_SLOT_BYTES = 50;
//		uint8_t NUM_SF_PARAMS = 3;
		int num_of_params;
		float* imu_values;
		uint8_t num_imus_id_params = 7;
		uint8_t num_mag_cal_params = 0;
		uint8_t first_imu_parameter = 2;
		uint8_t new_board = 1;
		int i = 0;
		
		if (commGetIMUParamList(&comm_settings_1, global_args.device_id, 0, NULL, 0, 0, aux_string) < 0){
			// If commGetIMUParamList returns -1, the connected board is a PSoC3 board instead of a STM32 or PSoC5 board
			// so call the commGetParamList instead
			new_board = 0;
			commGetParamList(&comm_settings_1, global_args.device_id, 0, NULL, 0, 0, aux_string);
			num_imus_id_params = 6;
		}
		
		num_of_params = aux_string[5];
		
		//aux_string[6] <-> packet_data[2] on the firmware
		global_args.n_imu = aux_string[8];
		printf("Number of connected IMUs: %d\n", global_args.n_imu);
		
		// Compute number of read parameters depending on global_args.n_imu and
		// update packet_length
		num_mag_cal_params = (global_args.n_imu / 2);
		if ( (global_args.n_imu - num_mag_cal_params*2) > 0 ) num_mag_cal_params++;

		global_args.ids = (uint8_t *) calloc(global_args.n_imu, sizeof(uint8_t));
		i = 0;
		for (int k = 1; k <= num_imus_id_params; k++){
			if (aux_string[k*PARAM_SLOT_BYTES + 8] != 255) {
				global_args.ids[i] = aux_string[k*PARAM_SLOT_BYTES + 8];
				i++;
			}
			if (aux_string[k*PARAM_SLOT_BYTES + 9] != 255) {
				global_args.ids[i] = aux_string[k*PARAM_SLOT_BYTES + 9];
				i++;
			}
			if (aux_string[k*PARAM_SLOT_BYTES + 10] != 255) {
				global_args.ids[i] = aux_string[k*PARAM_SLOT_BYTES + 10];
				i++;
			}
		}
		
		// Retrieve magnetometer calibration parameters
		global_args.mag_cal = (uint8_t *) calloc(global_args.n_imu, 3*sizeof(uint8_t));
		i = 0;
		for (int k=1; k <= num_mag_cal_params; k++) {
			global_args.mag_cal[3*i + 0] = aux_string[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 8];
			global_args.mag_cal[3*i + 1] = aux_string[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 9];
			global_args.mag_cal[3*i + 2] = aux_string[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 10];
			printf("MAG PARAM: %d %d %d\n", global_args.mag_cal[3*i + 0], global_args.mag_cal[3*i + 1], global_args.mag_cal[3*i + 2]);
			i++;
			
			if (aux_string[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 7] == 6) {
				global_args.mag_cal[3*i + 0] = aux_string[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 11];
				global_args.mag_cal[3*i + 1] = aux_string[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 12];
				global_args.mag_cal[3*i + 2] = aux_string[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 13];
				printf("MAG PARAM: %d %d %d\n", global_args.mag_cal[3*i + 0], global_args.mag_cal[3*i + 1], global_args.mag_cal[3*i + 2]);
				i++;
			}
		}
	
		first_imu_parameter = 1 + num_imus_id_params + num_mag_cal_params + 1;
		global_args.imu_table = (uint8_t *) calloc(global_args.n_imu, 5*sizeof(uint8_t));
		for (int i=0; i< global_args.n_imu; i++){
			global_args.imu_table[5*i + 0] = aux_string[first_imu_parameter*PARAM_SLOT_BYTES + 8 + 50*i];
			global_args.imu_table[5*i + 1] = aux_string[first_imu_parameter*PARAM_SLOT_BYTES + 9 + 50*i];
			global_args.imu_table[5*i + 2] = aux_string[first_imu_parameter*PARAM_SLOT_BYTES + 10 + 50*i];
			global_args.imu_table[5*i + 3] = aux_string[first_imu_parameter*PARAM_SLOT_BYTES + 11 + 50*i];
			global_args.imu_table[5*i + 4] = aux_string[first_imu_parameter*PARAM_SLOT_BYTES + 12 + 50*i];
			printf("ID: %d - %d, %d, %d, %d, %d\n", global_args.ids[i], global_args.imu_table[5*i + 0], global_args.imu_table[5*i + 1], global_args.imu_table[5*i + 2], global_args.imu_table[5*i + 3], global_args.imu_table[5*i + 4]);
			
		}
		
		// Imu values is a (3 sensors x 3 axes + 4 + 1) x n_imu values
		imu_values = (float *) calloc(global_args.n_imu, 3*3*sizeof(float)+4*sizeof(float)+sizeof(float));
		
		if (!new_board && global_args.n_imu > 1){
			int idx = 0;
			for (i = 0; i < global_args.n_imu; i++) {
				if (global_args.imu_table[5*i + 3]){
					idx++;
				}
				if (idx > 0) {
					printf("\n[WARNING] Quaternion will not be read as it is computed only if there is ONLY 1 IMU connected to the board.\n\n");
					return -1;
				}
			}
		}

		while(1){
			
			commGetImuReadings(&comm_settings_1, global_args.device_id, global_args.imu_table, global_args.mag_cal, global_args.n_imu, imu_values);

			for (i = 0; i < global_args.n_imu; i++) {
		
				printf("IMU: %d\n", global_args.ids[i]);
				usleep(10000);
				
				if (global_args.imu_table[5*i + 0]){
					printf("Accelerometer\n");
					printf("%f, %f, %f\n", imu_values[(3*3+4+1)*i], imu_values[(3*3+4+1)*i+1], imu_values[(3*3+4+1)*i+2]);
				}
				if (global_args.imu_table[5*i + 1]){
					printf("Gyroscope\n");
					printf("%f, %f, %f\n", imu_values[(3*3+4+1)*i+3], imu_values[(3*3+4+1)*i+4], imu_values[(3*3+4+1)*i+5]);
				}
				if (global_args.imu_table[5*i + 2] ){
					printf("Magnetometer\n");
					printf("%f, %f, %f\n", imu_values[(3*3+4+1)*i+6], imu_values[(3*3+4+1)*i+7], imu_values[(3*3+4+1)*i+8]);
				}
				if (global_args.imu_table[5*i + 3] ){
					printf("Quaternion\n");
					printf("%f, %f, %f, %f\n", imu_values[(3*3+4+1)*i+9], imu_values[(3*3+4+1)*i+10], imu_values[(3*3+4+1)*i+11], imu_values[(3*3+4+1)*i+12]);
				}
				if (global_args.imu_table[5*i + 4] ){
					printf("Temperature\n");
					printf("%f\n", imu_values[(3*3+4+1)*i+13]);
				}
				
				printf("\n");				
			}
		}
		
	}
	
//=========================================================     get emg raw

    if(global_args.flag_get_adc_raw)
    {
		uint8_t adc_map[100];
		uint8_t tot_adc_channels = 0;
		uint8_t used_adc_channels = 0;
		int idx;
		
        if(global_args.flag_verbose)
            puts("Getting adc raw values.");

		commGetADCConf(&comm_settings_1, global_args.device_id, &tot_adc_channels, adc_map);
		usleep(100000);
		
		printf("Number of ADC channels: %d\n", tot_adc_channels);
		
		for (int i=0; i< tot_adc_channels; i++) {
			if (adc_map[i] == 1) {
				used_adc_channels++;
			}
		}
		
		global_args.adc_raw = (short int *) calloc(used_adc_channels, sizeof(short int));
		
        while(1) {
            if (commGetADCRawValues(&comm_settings_1, global_args.device_id, used_adc_channels, global_args.adc_raw) < 0) {
                printf("An error occurred or the device is not supported\n");
                break;
            }
            else {
				idx = 0;
                for (int i = 0; i < tot_adc_channels; i++) {
					if (adc_map[i] == 1) {
						printf("Raw %d: %d\n", i, (int)global_args.adc_raw[idx]);
						idx++;
					}
                }
                printf("\n");

                usleep(100000);
            }
        }
    }
	
	
//=========================================================  get encoder raw

    if(global_args.flag_get_encoder_raw)
    {
		uint8_t enc_map[100];
		uint16_t* encoder_values;
		uint8_t num_encoder_lines = 2;
		uint8_t num_encoder_per_line = 5;
		uint8_t num_encoder_conf_total = 0;
		int idx;
		
		if(global_args.flag_verbose)
            puts("Getting encoder raw values.");
		
		commGetEncoderConf(&comm_settings_1, global_args.device_id, &num_encoder_lines, &num_encoder_per_line, enc_map);
		usleep(100000);
		
		printf("Number of Connected Encoder lines: %d\n", num_encoder_lines);
		printf("Number of Connected Encoder per line: %d\n", num_encoder_per_line);

		for (int i=0; i< num_encoder_lines; i++) {
			for (int j=0; j < num_encoder_per_line; j++) {
				num_encoder_conf_total += enc_map[i*num_encoder_per_line + j];
				//printf("Line %d, Encoder %d -> %d\n", i, j+1, enc_map[i*num_encoder_per_line + j]);
			}
		}
		
		encoder_values = (uint16_t *) calloc(num_encoder_conf_total, sizeof(uint16_t));
	
		while(1){
			
			if (commGetEncoderRawValues(&comm_settings_1, global_args.device_id, num_encoder_conf_total, encoder_values) < 0) {
				printf("An error occurred or the device is not supported\n");
                break;
			}
			else {
				idx = 0;
                for (int i=0; i< num_encoder_lines; i++) {
					for (int j=0; j < num_encoder_per_line; j++) {
						if (enc_map[i*num_encoder_per_line + j] == 1) {
							printf("Line %d, Encoder %d -> %d\n", i, j+1, (int)encoder_values[idx]);
							idx++;
						}
					}
                }
				printf("\n");
				usleep(100000);
			
			}
		}		
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
    puts("");
    puts("================================================================================");
    puts("qbMove exclusive commands");
    puts("================================================================================");
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
    puts("");
    puts("================================================================================");
    puts("Hand exclusive commands");
    puts("================================================================================");
    puts(" -k, --calibration                Makes a series of opening and closing.");
    puts(" -q, --get_emg                    Get EMG values and save them in a file");
    puts("                                  defined in \"definitions.h\". Use -v option");
    puts("                                  to display values in the console too.");
    puts(" -x, --ext_drive                  Reads measurements and drives a second board.");
    puts(" -j, --get_joystick               Get joystick measurements.");
    puts("");
    puts("================================================================================");
    puts("Additional commands");
    puts("================================================================================");
	puts(" -Q, --get_imu_readings           Retrieve accelerometers, gyroscopes and magnetometers readings");
	puts(" -m, --get_emg_raw				Retrieve emg raw values");
	puts(" -E, --get_encoder_raw			Retrieve encoder raw values");
    puts(" -S, --get_SD_files               Retrieve current used SD parameters and data file");
    puts(" -X, --get_SD_filesystem          Retrieve all the SD card filesystem");
	puts("");
    puts("--------------------------------------------------------------------------------");
    puts("Examples:");
    puts("");
    puts("  qbadmin -p                      Get info on whatever device is connected.");
	puts("  qbadmin -r 						Get cycles info.");
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
