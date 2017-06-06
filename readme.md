# Lepton Grabber #

## Objective ##
This code upon execution captures Y16 pixelformat images from multiple leptons sensor based boards near simultaneously.

## Dependencies ##
### Python ###
Python 3.6

### Linux ###
This code has been tested with Ubuntu 16.10 and Arch Linux.

### libuvc ###
The libuvc library is sourced from https://github.com/ktossell/libuvc
Follow build instructions there and place the built libuvc.so file under "libs" subfolder accompanying the script.

### pypng (python) ###
PyPng is required for writing output to .png format https://pythonhosted.org/pypng/

### numpy (python) ###
Numpy is used for converting grabbed data to structured arrays and subsequently write to csv files.

## Program Execution ##
Command: python3 frame_grabber.py --dbg_interval 60 --dbg_capture_count 10
Would capture once every 60 seconds for 10 capture events.

## Program behaviour ##
The program upon execution does the following for each of the connected lepton sensors:
- Captures images based on the inteval values specified by the following globals: SLEEP_INTERVAL and CAPTURE_COUNT
- Write out a CSV file with octal values
- Write out a PNG file

## Sample Images ##
TODO: Append some sample images here.
