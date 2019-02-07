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

all:qbadmin qbparam qbparam_imu


qbadmin:$(OBJS_FOLDER)/qbadmin.o $(BIN_FOLDER)
	$(COMPILER) $(OBJS_FOLDER)/qbadmin.o      $(LIB_FOLDER)/libqbmove_comm.a -o $(BIN_FOLDER)/qbadmin $(LMFLAGS)

qbparam:$(OBJS_FOLDER)/qbparam.o $(BIN_FOLDER)
	$(COMPILER) $(OBJS_FOLDER)/qbparam.o     $(LIB_FOLDER)/libqbmove_comm.a -o $(BIN_FOLDER)/qbparam $(LMFLAGS)

qbparam_imu:$(OBJS_FOLDER)/qbparam_imu.o $(BIN_FOLDER)
	$(COMPILER) $(OBJS_FOLDER)/qbparam_imu.o     $(LIB_FOLDER)/libqbmove_comm.a -o $(BIN_FOLDER)/qbparam_imu $(LMFLAGS)

$(OBJS_FOLDER)/qbadmin.o:qbadmin.c $(OBJS_FOLDER)
	$(COMPILER) $(CFLAGS) qbadmin.c -o      $(OBJS_FOLDER)/qbadmin.o 

$(OBJS_FOLDER)/qbparam.o:qbparam.c $(OBJS_FOLDER)
	$(COMPILER) $(CFLAGS) qbparam.c -o     $(OBJS_FOLDER)/qbparam.o

$(OBJS_FOLDER)/qbparam_imu.o:qbparam_imu.c $(OBJS_FOLDER)
	$(COMPILER) $(CFLAGS) qbparam_imu.c -o     $(OBJS_FOLDER)/qbparam_imu.o

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
