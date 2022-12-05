# Command line tools for qbmoves and hands

### IMPORTANT: Versioning:
Each of the repositories in the following table has a version number
composed of 3 numbers in the form: **v x.y.z**

|  Tools  [T]     |  Libraries [L] |  Firmware [F]            |
|-----------------|----------------|--------------------------|
| qbmove simulink | qbAPI          | qbmove firmware          |
| qbadmin         |                | qbmove advanced firmware |
|                 |                | hand firmware micro      |

E.g. **xT** is the **x** number of the Tools whereas **yF** is the **y** number of the Firmware version.

- Every change in the number **z** means a change in the respective repo which not implies changes in other repos.
- Every change in the number **y** in a repo is backward compatible reading the table from right to left. This means that
if you have some new feature in a Firmware, you can still use old Libraries and Tools for management, but of course
you will not be able to use the new features. In this case the rule is **yF >= yL >= yT**.
- Every change in the number **x** means a change which is not backward compatible, hence you will need to update
everything to use it. In this case the rule is **xF = xL = xT**.

Summarising
- **z** independent
- **yF >= yL >= yT**
- **xF = xL = xT**

E.g.

| Tools          | Libraries    | Firmware               | Compatible |
|----------------|--------------|------------------------|------------|
| qbadmin v4.2.3 | qbAPI v4.5.0 | qbmove firmware v4.6.7 | YES        |
| qbadmin v3.2.3 | qbAPI v4.5.0 | qbmove firmware v4.6.7 | NO         |
| qbadmin v4.2.3 | qbAPI v4.5.7 | qbmove firmware v4.5.0 | YES        |


## Installation requirements

The simplest way to compile these libraries is by using the command line utility *make*.
To do so, you will need also the *gcc/g++* compiler installed on your system.

> NOTE: if it is the first time you use qbrobotics devices on your computer, it is possible that
> you need to download the drivers to let your computer see the port correctly.
> To do that visit [FTDI driver](http://www.ftdichip.com/Drivers/VCP.htm) webpage -> VCP drivers
> and download the proper driver depending on your OS

### Unix
You should have both *gcc/g++* and *make* installed. 

### MacOSX
Download *XCode* from Mac App Store, this will install *gcc/g++* and *make* utility.

### Windows
Download [MinGW](https://sourceforge.net/projects/mingw/) and install it. Open MinGW Installation
Manager, from the left panel select basic setup, then from the right panel select
mingw32-base and mingw32-gcc-g++, then click on `Installation -> Apply Changes`.
This will install the *gcc/g++* compiler. To use it from the command line you need
to provide to windows the binary path to the executable. Go to System Properties
and click on Environment Variables. In the System Variable windows, look for `path`,
select it and click `edit`. Go to the end of the Variable Value field, add a `;`
separator and add the path to the binary folder for gcc (usually it is in `C:\MinGW\bin`).

Download the *make* utility from
[here](http://gnuwin32.sourceforge.net/packages/make.htm). Follow the installation
instruction. In the end you will need to add the binary path to the Environment
Variables. To do that follow the previous steps. (Usually the binary folder
for the make utility is in `C:\Program Files (x86)\GnuWin32\bin`).

>NOTE: if you have the CMD already opened when performing the installation,
>you probably will need to reopen a new one to be able to use the utilities.

## Compile the software

Fisrt of all you need to download the
[*qbAPI*](https://github.com/NMMI/qbapi)
repository and be sure to have a file tree organised like this (remove the `-master` suffix):

* your_workingcopy
    * qbAPI
    * qbadmin

Then you will need to compile the libraries.
To do that, go to `qbAPI/src` and type `make`.
Then, to compile the tools, go to `qbadmin/src` folder and type `make`.

If everything is ok, depending on your OS, you should see a folder tree like this:

* qbadmin
   * bin_win
   * conf_files
   * objs_win
   * src

or this:

* qbadmin
   * bin_unix
   * conf_files
   * objs_unix
   * src

## Using the tools

Using a terminal, go to the binary folder and execute what you need e.g.:

Windows
`name_of_the_bin.exe`

UNIX
`./name_of_the_bin`

> By executing the tools you will receive the usage instruction

## Use with a Bluetooth device

### Windows
After pairing your Bluetooth device to the computer, go to the Bluetooth option devices settings to find the right COM port, following the procedure
> Click 'Start', then 'Settings' (You can also access Settings with the Windows Key + I shortcut), click 'Bluetooth & devices', click 'Devices', then scroll down, and click 'More Bluetooth settings'. Under the 'COM Ports' tab find the OUTPUT/OUTGOING port related to your BT device and use this one with qbadmin.exe

### Linux
After pairing your Bluetooth device to the computer, open a terminal and type rfcomm command to bind it to a device filename:
`sudo rfcomm bind /dev/rfcommX MAC_ADDRESS channel` (e.g. `sudo rfcomm bind /dev/rfcomm0 20:71:9B:19:3E:41 1`)
> Now the device is recognized under the name `/dev/rfcomm0`

### MacOS
After pairing your Bluetooth device to the computer, it will be recognized by the system with a name similar to
`/dev/tty.NMMIDev001-BTCOMPORT`
