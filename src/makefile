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

all:qbadmin qbmove_init qbmove_test qbmove_pos_stiff_demo qbmove_backup qbparam hand_demo


qbadmin:$(OBJS_FOLDER)/qbadmin.o $(BIN_FOLDER)
	$(COMPILER) $(OBJS_FOLDER)/qbadmin.o      $(LIB_FOLDER)/libqbmove_comm.a -o $(BIN_FOLDER)/qbadmin $(LMFLAGS)

qbmove_init:$(OBJS_FOLDER)/qbmove_init.o $(BIN_FOLDER)
	$(COMPILER) $(OBJS_FOLDER)/qbmove_init.o      $(LIB_FOLDER)/libqbmove_comm.a -o $(BIN_FOLDER)/qbmove_init $(LMFLAGS)

qbmove_test:$(OBJS_FOLDER)/qbmove_test.o $(BIN_FOLDER)
	$(COMPILER) $(OBJS_FOLDER)/qbmove_test.o      $(LIB_FOLDER)/libqbmove_comm.a -o $(BIN_FOLDER)/qbmove_test $(LMFLAGS)

qbmove_pos_stiff_demo:$(OBJS_FOLDER)/qbmove_pos_stiff_demo.o $(BIN_FOLDER)
	$(COMPILER) $(OBJS_FOLDER)/qbmove_pos_stiff_demo.o $(LIB_FOLDER)/libqbmove_comm.a -o $(BIN_FOLDER)/qbmove_pos_stiff_demo $(LMFLAGS)

qbmove_backup:$(OBJS_FOLDER)/qbmove_backup.o $(BIN_FOLDER)
	$(COMPILER) $(OBJS_FOLDER)/qbmove_backup.o    $(LIB_FOLDER)/libqbmove_comm.a -o $(BIN_FOLDER)/qbmove_backup $(LMFLAGS)

qbparam:$(OBJS_FOLDER)/qbparam.o $(BIN_FOLDER)
	$(COMPILER) $(OBJS_FOLDER)/qbparam.o     $(LIB_FOLDER)/libqbmove_comm.a -o $(BIN_FOLDER)/qbparam $(LMFLAGS)

hand_demo:$(OBJS_FOLDER)/hand_demo.o $(BIN_FOLDER)
	$(COMPILER) $(OBJS_FOLDER)/hand_demo.o     $(LIB_FOLDER)/libqbmove_comm.a -o $(BIN_FOLDER)/hand_demo     $(LMFLAGS)


$(OBJS_FOLDER)/qbadmin.o:qbadmin.c $(OBJS_FOLDER)
	$(COMPILER) $(CFLAGS) qbadmin.c -o      $(OBJS_FOLDER)/qbadmin.o 

$(OBJS_FOLDER)/qbmove_init.o:qbmove_init.c $(OBJS_FOLDER)
	$(COMPILER) $(CFLAGS) qbmove_init.c -o      $(OBJS_FOLDER)/qbmove_init.o

$(OBJS_FOLDER)/qbmove_test.o:qbmove_test.c $(OBJS_FOLDER)
	$(COMPILER) $(CFLAGS) qbmove_test.c -o      $(OBJS_FOLDER)/qbmove_test.o

$(OBJS_FOLDER)/qbmove_pos_stiff_demo.o:qbmove_pos_stiff_demo.c $(OBJS_FOLDER)
	$(COMPILER) $(CFLAGS) qbmove_pos_stiff_demo.c -o $(OBJS_FOLDER)/qbmove_pos_stiff_demo.o

$(OBJS_FOLDER)/qbmove_backup.o:qbmove_backup.c $(OBJS_FOLDER)
	$(COMPILER) $(CFLAGS) qbmove_backup.c -o    $(OBJS_FOLDER)/qbmove_backup.o

$(OBJS_FOLDER)/qbparam.o:qbparam.c $(OBJS_FOLDER)
	$(COMPILER) $(CFLAGS) qbparam.c -o     $(OBJS_FOLDER)/qbparam.o

$(OBJS_FOLDER)/hand_demo.o:hand_demo.c $(OBJS_FOLDER)
	$(COMPILER) $(CFLAGS) $(PFLAGS) hand_demo.c -o $(OBJS_FOLDER)/hand_demo.o


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
