# compiler
COMPILER = g++

# flags
CFLAGS = -c -Wall
LMFLAGS = -lm


ifeq "$(OS)"  "Windows_NT"

# folders
BIN_FOLDER = ..\bin_win
OBJS_FOLDER = ..\objs_win
LIB_FOLDER = ..\..\qbAPI\lib_win

else

# folders
BIN_FOLDER = ../bin_unix
OBJS_FOLDER = ../objs_unix
LIB_FOLDER = ../../qbAPI/lib_unix

endif

all:qbadmin qbparam nmmi_param nmmi_param_imu 


qbadmin:$(OBJS_FOLDER)/qbadmin.o $(BIN_FOLDER)
	$(COMPILER) $(OBJS_FOLDER)/qbadmin.o      $(LIB_FOLDER)/libqbmove_comm.a -o $(BIN_FOLDER)/qbadmin $(LMFLAGS)

qbparam:$(OBJS_FOLDER)/qbparam.o $(BIN_FOLDER)
	$(COMPILER) $(OBJS_FOLDER)/qbparam.o     $(LIB_FOLDER)/libqbmove_comm.a -o $(BIN_FOLDER)/qbparam $(LMFLAGS)
	
nmmi_param:$(OBJS_FOLDER)/nmmi_param.o $(BIN_FOLDER)
	$(COMPILER) $(OBJS_FOLDER)/nmmi_param.o     $(LIB_FOLDER)/libqbmove_comm.a -o $(BIN_FOLDER)/nmmi_param $(LMFLAGS)

nmmi_param_imu:$(OBJS_FOLDER)/nmmi_param_imu.o $(BIN_FOLDER)
	$(COMPILER) $(OBJS_FOLDER)/nmmi_param_imu.o     $(LIB_FOLDER)/libqbmove_comm.a -o $(BIN_FOLDER)/nmmi_param_imu $(LMFLAGS)

$(OBJS_FOLDER)/qbadmin.o:qbadmin.c $(OBJS_FOLDER)
	$(COMPILER) $(CFLAGS) qbadmin.c -o      $(OBJS_FOLDER)/qbadmin.o 

$(OBJS_FOLDER)/qbparam.o:qbparam.c $(OBJS_FOLDER)
	$(COMPILER) $(CFLAGS) qbparam.c -o     $(OBJS_FOLDER)/qbparam.o
	
$(OBJS_FOLDER)/nmmi_param.o:nmmi_param.c $(OBJS_FOLDER)
	$(COMPILER) $(CFLAGS) nmmi_param.c -o     $(OBJS_FOLDER)/nmmi_param.o	

$(OBJS_FOLDER)/nmmi_param_imu.o:nmmi_param_imu.c $(OBJS_FOLDER)
	$(COMPILER) $(CFLAGS) nmmi_param_imu.c -o     $(OBJS_FOLDER)/nmmi_param_imu.o

clean:
ifeq "$(OS)"  "Windows_NT"
	rmdir /Q /S $(OBJS_FOLDER)
	rmdir /Q /S $(BIN_FOLDER)
else
	rm -rf $(BIN_FOLDER) $(OBJS_FOLDER)
endif

$(OBJS_FOLDER):
	mkdir $(OBJS_FOLDER)

$(BIN_FOLDER):
	mkdir $(BIN_FOLDER)
