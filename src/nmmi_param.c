// Copyright (c) 2019-2020, Mattia Poggiani.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// - Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
* \file         nmmi_param.c
*
* \brief        Command line tools file
* \author       _Centro "E.Piaggio"_
* \copyright    (C) 2019-2020 Centro "E.Piaggio". All rights reserved.
*
* \details      With this file is possible to get or set firmware parameters 
*				with a new interface based on old qbparam tool.
*/

// --- INCLUDE ---
#include "../../qbAPI/src/qbmove_communications.h"
#include "definitions.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <stdint.h>
#include <stdlib.h>

#define NUM_OF_MAX_PARAMS	100

// function declaration
int port_selection();
int open_port();
int initMemory();
void printMainMenu();
void printVersion();
void sort_params_asc(int*, int*, int*, int);
void retrieve_section_str(int, char*);

// global variables
char get_or_set;
comm_settings comm_settings_t;
uint8_t device_id = BROADCAST_ID;

// holds the address of the array of which the sorted index order needs to be found
int *base_arr;
int* param_idx_arr;

/** Baudrate functions
 */
int baudrate_reader();

// --- MAIN ---
int main(int argc, char **argv) {
    int i,j,k;
    char c_choice;

    uint8_t aux_string[10000] = "";

    int value_size;
    int num_of_values;
    int num_of_params;
	int num_of_sections;
	int num_of_av_sections = 1;
    int menu_number[NUM_OF_MAX_PARAMS];
	int struct_number[NUM_OF_MAX_PARAMS];
    int index;
	int section_index;
	int current_index;

    int data_type[NUM_OF_MAX_PARAMS];
    int data_dim[NUM_OF_MAX_PARAMS];
    int data_size[NUM_OF_MAX_PARAMS];
    char data_string[20];
    char tmp_string[150] = "";
	
	char param_string[NUM_OF_MAX_PARAMS][50];
	char aux_p_str[100] = "";
	char values_string[NUM_OF_MAX_PARAMS][100];
	int ordered_idx[NUM_OF_MAX_PARAMS];
	int param_idx[NUM_OF_MAX_PARAMS];
	int show_section = -1;		// show every section
	int max_p_idx_section[50];	// max number of param for section
	int is_sec_av[50];		// is section available
	
    int8_t aux_int8[4];
    uint8_t aux_uint8[4];
    int16_t aux_int16[4];
    uint16_t aux_uint16[4];
    int32_t aux_int32[4];
    uint32_t aux_uint32[4];
    float aux_float[4]; 
    double aux_double[4];
    uint8_t temp_char[4];
	uint8_t aux_str[100] = "";			// custom string

	// Get device ID
	if (argc > 1)
    {
        sscanf(argv[1],"%d",&device_id);
		
		printf("Communicating with device %d\n", device_id);
		
		// Parse submenu type and show only related parameters
		char submenu[100] = "";
		if (!device_id) {
			sscanf(argv[1],"%s",&submenu);
		}
		else if (argc > 2){
			sscanf(argv[2],"%s",&submenu);
		}
		else {
			strcpy(submenu, "all");
		}
		
		if (!strcmp(submenu, "all")){
			show_section = -1;
		}
		else if (!strcmp(submenu, "dev") || !strcmp(submenu, "device")){
			show_section = ST_DEVICE;
		}
		else if (!strcmp(submenu, "mot") || !strcmp(submenu, "motor")){
			show_section = ST_MOTOR;
		}
		else if (!strcmp(submenu, "enc") || !strcmp(submenu, "encoder")){
			show_section = ST_ENCODER;
		}
		else if (!strcmp(submenu, "emg")){
			show_section = ST_EMG;
		}
		else if (!strcmp(submenu, "imu")){
			show_section = ST_IMU;
		}
		else if (!strcmp(submenu, "exp") || !strcmp(submenu, "expansion")){
			show_section = ST_EXPANSION;
		}
		else if (!strcmp(submenu, "usr") || !strcmp(submenu, "user")){
			show_section = ST_USER;
		}
		else if (!strcmp(submenu, "SH") || !strcmp(submenu, "softhand")){
			show_section = ST_SH_SPEC;
		}
        else if (!strcmp(submenu, "FB") || !strcmp(submenu, "feedback")){
            show_section = ST_FB_SPEC;
        }
		else {		// option not recognized
			printf("Parameters section not recognized\n\n");
			printf("[USAGE]: nmmi_param device_id section\n\n");
			printf("Use one of the following allowed sections to get or set related parameters:\n");
			printf("Device\t\t'dev' or 'device'\n");
			printf("Motor\t\t'mot' or 'motor'\n");
			printf("Encoder\t\t'enc' or 'encoder'\n");
			printf("EMG\t\t'emg'\n");
			printf("IMU\t\t'imu'\n");
			printf("Expansion port\t'exp' or 'expansion'\n");
			printf("User\t\t'usr' or 'user'\n");
			printf("SoftHand\t'SH' or 'softhand'\n");
            printf("Feedback\t'FB' or 'feedback'\n");
			return -1;
		}
	}

    if(device_id)
        printf("\nUsing nmmi_param with ID: %hhu\n\n", device_id);
    else
        printf("\nUsing nmmi_param in broadcast.\n\n");

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
        commGetParamList(&comm_settings_t, device_id, index, NULL, value_size, num_of_values, aux_string);
        

        // The packet returned in aux_string is composed as follows
        // [':'][':'][ID][LEN][CMD][PARAM_NUM][...]

		//for (int i=0; i<5000; i++){
		//	printf("%c", aux_string[i]);
		//}
		//printf("\n");

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
				case TYPE_STRING:								// Custom data type
                    data_size[i] = 1;
					strcat(tmp_string, " "); 
                    for(k = 0; k < data_dim[i]; k++) {
                        for(j = 0; j < data_size[i]; j++) {
                            aux_uint8[k] += aux_string[i*PARAM_BYTE_SLOT + 8 + k * data_size[i] + data_size[i] - j - 1] << (8 * j);  
                        }
                        sprintf(data_string, "%c", aux_uint8[k]);
                        strcat(tmp_string, data_string); 
                    }
                break;
				
            }
            // The parameter description is printed character by character until it reaches the end of the string
            for(k = 0; aux_string[i * PARAM_BYTE_SLOT + 8 + data_size[i] * data_dim[i] + k] != '\0'; k++) {
				aux_p_str[k] = aux_string[i * PARAM_BYTE_SLOT + 8 + data_size[i] * data_dim[i] + k];
            }
			aux_p_str[k] = '\0';
			
			// Parse tmp_string to extract parameter index and string
			sscanf(aux_p_str, "%d - %[^\t\n]", &param_idx[i], param_string[i]);
			
            
            // If the parameter is a TYPE_FLAG it has a printable menu at the end of the packet. 
            // Their order is written at the end of each parameter description

            if(data_type[i] == TYPE_FLAG)
                menu_number[i] = aux_string[i * PARAM_BYTE_SLOT + 8 + data_size[i] * data_dim[i] + k + 1];
            else
                menu_number[i] = -1;
            // If the parameter has associated a -1 to its menu_number, the parameter has not a menu.

			// Retrieve struct number after an empty bit
			if(data_type[i] == TYPE_FLAG)
                struct_number[i] = aux_string[i * PARAM_BYTE_SLOT + 8 + data_size[i] * data_dim[i] + k + 3];
            else
                struct_number[i] = aux_string[i * PARAM_BYTE_SLOT + 8 + data_size[i] * data_dim[i] + k + 1];

			strcpy(values_string[i], tmp_string);				
			strcpy(tmp_string, "");
        }

		// Sort parameters by struct index
		sort_params_asc(struct_number, param_idx, ordered_idx, num_of_params);

		// Print get parameters
		int ref_st_idx = 0;
		int prev_st_idx = struct_number[ordered_idx[0]];
		int curr_p_idx = 1;
		int idx_section = 1;
		int curr_sections = 0;
		char a_str[100];
		for (int i=0; i< num_of_params; i++) {
			if (struct_number[ordered_idx[i]] - prev_st_idx >= 1){
				prev_st_idx = struct_number[ordered_idx[i]];
				// Print also struct name
				ref_st_idx = ((int)(struct_number[ordered_idx[i]]/10))*10;
				curr_p_idx = 1;
				idx_section++;
			}
			if (show_section < 0 || show_section == ref_st_idx) {
				
				is_sec_av[idx_section-1] = 1;	// Section is available

				if (curr_p_idx == 1) {		// print section name before related parameters
					strcpy(a_str, "");
					retrieve_section_str(struct_number[ordered_idx[i]], a_str);
					printf("\n%d) %s\n", curr_sections+1, a_str);
					curr_sections++;
				}
				if(data_type[ordered_idx[i]] == TYPE_FLAG)
					printf("%d - %s\n", curr_p_idx, param_string[ordered_idx[i]]);		// Only index and string
				else
					printf("%d - %s%s\n", curr_p_idx, param_string[ordered_idx[i]], values_string[ordered_idx[i]]);	// Index, string and values
			}
				
			max_p_idx_section[idx_section-1] = curr_p_idx;		// at the end of the loop it holds the max param idx of every section
			curr_p_idx++;			
		}
		
		num_of_sections = idx_section;
		
		if (show_section < 0){
			num_of_av_sections = num_of_sections;		// all sections
		} 
		else {
			num_of_av_sections = curr_sections;
		}
		//printf("Number of available sections: %d\n", num_of_av_sections);
		//for (int i=0; i< num_of_sections; i++){
		//	printf("Section %d available: %s - Number of param in section:%d\n", i, is_sec_av[i]?"YES":"NO", max_p_idx_section[i]);
		//}
		
        if (get_or_set == 's') {

			if (num_of_av_sections > 1) {
				printf("\nSelect a section from the previous list\n");
				scanf("%d", &section_index);
				if(section_index <= 0 || section_index > num_of_av_sections) {
					printf("Invalid choice\n");
					return 0;
				}
			}
			else {
				section_index = 1;
			}
			
			// Retrieve index of chosen section
			int chosen_idx_sec;
			int var = 0;
			for (j = 0; j < num_of_sections; j++){
				if (is_sec_av[j]) {
					var++;
				}
				if (var == section_index){
					break;
				}
			}
			chosen_idx_sec = j;
			
            printf("\nSelect a parameter from the previous list\n");
            scanf("%d", &current_index);
            if(current_index <= 0 || current_index > max_p_idx_section[chosen_idx_sec]) {
                printf("Invalid choice\n");
                return 0;
            }
			
			// Retrieve index in all parameters list
			int idx = 0;
			if (show_section < 0){
				for (j = 0; j < section_index-1; j++){	// skip all sections previous than that interested
					idx += max_p_idx_section[j];
				}
			}
			else {
				j = 0;
				for (j = 0; j < chosen_idx_sec; j++){
					idx += max_p_idx_section[j];
				}
			}
			index = param_idx[ordered_idx[idx+(current_index-1)]];
			//printf("Chosen index: %d\n", index);
			printf("\n");
			
            for(j = 0; j < 4; j++) {
                    aux_int8[j] = 0; aux_uint8[j] = 0; aux_int16[j] = 0; aux_uint16[j] = 0;
                    aux_int32[j] = 0; aux_uint32[j] = 0; aux_float[j] = 0.0; aux_double[j] = 0.0;
            }
            // Depending on the number of values of the parameter and its type
            // different and/or multiple readings must be done
            printf("Insert new parameters values\n");
            for(i = 0; i < data_dim[index - 1]; i++) {
                if(data_type[index - 1] != TYPE_FLAG && data_type[index - 1] != TYPE_STRING) {
                    printf("Insert %d%\370 parameter: \n", i+1);
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
                else {      
					if(data_type[index - 1] == TYPE_FLAG){
						// TYPE_FLAG is a uint8 but with a menu
						for(k = 0; aux_string[num_of_params * PARAM_BYTE_SLOT + 6 + (menu_number[index - 1] - 1) * PARAM_MENU_SLOT + k] != '\0'; k++)
							printf("%c", aux_string[num_of_params * PARAM_BYTE_SLOT + 6 + (menu_number[index - 1] - 1) * PARAM_MENU_SLOT + k]);
						scanf("%hhu", aux_uint8);
					}
                }
            }
			
			if (data_type[index - 1] == TYPE_STRING) {		// custom data type		
				scanf("%s", &aux_str);
			}

            switch(data_type[index - 1]) {
                    case TYPE_FLAG:
                        commGetParamList(&comm_settings_t, device_id, index, aux_uint8, data_size[index - 1], data_dim[index - 1], NULL);
                    break;
                    case TYPE_INT8:
                        commGetParamList(&comm_settings_t, device_id, index, aux_int8, data_size[index - 1], data_dim[index - 1], NULL);
                    break;
                    case TYPE_UINT8:
                        commGetParamList(&comm_settings_t, device_id, index, aux_uint8, data_size[index - 1], data_dim[index - 1], NULL);
                    break;
                    case TYPE_INT16:
                        commGetParamList(&comm_settings_t, device_id, index, aux_int16, data_size[index - 1], data_dim[index - 1], NULL);
                    break;
                    case TYPE_UINT16:
                        commGetParamList(&comm_settings_t, device_id, index, aux_uint16, data_size[index - 1], data_dim[index - 1], NULL);
                    break;
                    case TYPE_INT32:
                        commGetParamList(&comm_settings_t, device_id, index, aux_int32, data_size[index - 1], data_dim[index - 1], NULL);
                    break;
                    case TYPE_UINT32:
                        commGetParamList(&comm_settings_t, device_id, index, aux_uint32, data_size[index - 1], data_dim[index - 1], NULL);
                    break;
                    case TYPE_FLOAT:
                        commGetParamList(&comm_settings_t, device_id, index, aux_float, data_size[index - 1], data_dim[index - 1], NULL);
                    break;
                    case TYPE_DOUBLE:
                        commGetParamList(&comm_settings_t, device_id, index, aux_double, data_size[index - 1], data_dim[index - 1], NULL);
                    break;
					case TYPE_STRING:			// custom data type
                        commGetParamList(&comm_settings_t, device_id, index, aux_str, data_size[index - 1], data_dim[index - 1], NULL);
                    break;
                }

            usleep(100000);
            commStoreParams(&comm_settings_t, device_id);
            usleep(100000);
        }

    }

    return 1;
}

 
// Note how the compare function compares the values of the
// array to be sorted. The passed value to this function
// by `qsort' are actually the `idx' array elements.
static int compar (const void *a, const void *b)
{
  int aa = *((int *) a), bb = *((int *) b);
  if (base_arr[aa] < base_arr[bb])
    return -1;
  if (base_arr[aa] == base_arr[bb]){
    if (param_idx_arr[aa] < param_idx_arr[bb]){
		return -1;
	}
	if (param_idx_arr[aa] == param_idx_arr[bb]){
		return 0;
	}
	if (param_idx_arr[aa] > param_idx_arr[bb]){
		return 1;
	}
  }
  if (base_arr[aa] > base_arr[bb])
    return 1;
}

void sort_params_asc(int* values, int* p_idx, int* indices, int n){
	// Sort parameters ascending (by struct value)
	int i;
 
	// initialize initial index permutation of unmodified `arr'
	for (i = 0; i < n; i++){
      indices[i] = i;
	}
 
	// Assign the address of out original array to the static global 
	// pointer, this will be used by the compare function to index 
	// into the original array using `indices' values
	base_arr = values;
	param_idx_arr = p_idx;
 
	qsort (indices, n, sizeof (int), compar);
 
	//printf ("\nOriginal list: ");
	//for (i = 0; i < n; i++) {
    //  printf ("%d ", values[i]);
    //}
 
	//printf ("\nSorted index: ");
	//for (i = 0; i < n; i++) {
    //  printf ("%d ", indices[i]);
    //}
}

void retrieve_section_str(int sec_idx, char* res_str){
	int ref_st_idx = ((int)(sec_idx/10))*10;
	
	switch(ref_st_idx){
		case ST_DEVICE:
			strcpy(res_str, "GENERAL DEVICE PARAMETERS");
			break;
		case ST_MOTOR:
			sprintf(res_str, "MOTOR %d PARAMETERS", sec_idx - ref_st_idx + 1);
			break;
		case ST_ENCODER:
			sprintf(res_str, "ENCODER %d (%s) PARAMETERS", sec_idx - ref_st_idx, (sec_idx - ref_st_idx == 0)?"RIGHT SIDE":"LEFT SIDE");
			break;
		case ST_EMG:
			strcpy(res_str, "EMG PARAMETERS");
			break;
		case ST_IMU:
			strcpy(res_str, "IMU PARAMETERS");
			break;
		case ST_EXPANSION:
			strcpy(res_str, "EXPANSION PORT PARAMETERS");
			break;
		case ST_USER:
			sprintf(res_str, "USER %d PARAMETERS", sec_idx - ref_st_idx);
			break;
		case ST_SH_SPEC:
			strcpy(res_str, "SOFTHAND SPECIFIC PARAMETERS");
			break;			
        case ST_FB_SPEC:
            strcpy(res_str, "FEEDBACK SPECIFIC PARAMETERS");
            break;                      
		default:
			break;
	}
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
    printf("nmmi_param version: %s\n", NMMI_PARAM_VERSION);
    printf("==============================================\n");
}

void printMainMenu() {
    printf("g: getParam\n");
    printf("s: setParam\n");
    printf("m: initMemory\n");
    printf("==============================================\n");
}

/* END OF FILE */