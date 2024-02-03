#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr  3 13:11:07 2022

@author: user
"""

"""
A simple example of streaming data from 1 nRF24L01 transceiver to another.
This example was written to be used on 2 devices acting as 'nodes'.
"""
import sys
import argparse
import time
from RF24 import RF24, RF24_PA_LOW


########### USER CONFIGURATION ###########
# See https://github.com/TMRh20/RF24/blob/master/pyRF24/readme.md
# Radio CE Pin, CSN Pin, SPI Speed
# CE Pin uses GPIO number with BCM and SPIDEV drivers, other platforms use
# their own pin numbering
# CS Pin addresses the SPI bus number at /dev/spidev<a>.<b>
# ie: RF24 radio(<ce_pin>, <a>*10+<b>); spidev1.0 is 10, spidev1.1 is 11 etc..

# Generic:
radio = RF24(22, 0)
################## Linux (BBB,x86,etc) #########################
# See http://nRF24.github.io/RF24/pages.html for more information on usage
# See http://iotdk.intel.com/docs/master/mraa/ for more information on MRAA
# See https://www.kernel.org/doc/Documentation/spi/spidev for more
# information on SPIDEV

# Specify the number of bytes in the payload. This is also used to
# specify the number of payloads in 1 stream of data
SIZE = 32  # this is the default maximum payload size

# initialize the nRF24L01 on the spi bus
if not radio.begin():
    raise RuntimeError("radio hardware is not responding")

# For this example, we will use different addresses
# An address need to be a buffer protocol object (bytearray)
address = [b"1Node", b"2Node"]
# It is very helpful to think of an address as a path instead of as
# an identifying device destination

print(sys.argv[0])  # print example name

# to use different addresses on a pair of radios, we need a variable to
# uniquely identify which address this radio will use to transmit
# 0 uses address[0] to transmit, 1 uses address[1] to transmit

radio_number = 0

radio.setAutoAck(False)
# set the Power Amplifier level to -12 dBm since this test example is
# usually run with nRF24L01 transceivers in close proximity of each other
radio.setPALevel(RF24_PA_LOW)  # RF24_PA_MAX is default

# set the TX address of the RX node into the TX pipe
radio.openWritingPipe(address[radio_number])  # always uses pipe 0

# set the RX address of the TX node into a RX pipe
radio.openReadingPipe(1, address[not radio_number])  # using pipe 1

# To save time during transmission, we'll set the payload size to be only
# what we need. For this example, we'll be using the default maximum 32
radio.payloadSize = SIZE

# for debugging, we have 2 options that print a large block of details
# (smaller) function that prints raw register values
# radio.printDetails()
# (larger) function that prints human readable data
# radio.printPrettyDetails()


def master(buffer, count=10):
    """Uses all 3 levels of the TX FIFO to send a stream of data
    :param int count: how many times to transmit the stream of data.
    """
    radio.stopListening()  # put radio in TX mode
    radio.flush_tx()  # clear the TX FIFO so we can use all 3 levels
    failures = 0  # keep track of manual retries
    start_timer = time.monotonic_ns()  # start timer
    for multiplier in range(count):  # repeat transmit the same data stream

        print(buffer)

        if not radio.writeFast(buffer):  # transmission failed
            failures += 1  # increment manual retry count
            if failures > count - 1:
                # we need to prevent an infinite loop
                print(
                    "Too many failures detected. Aborting at payload ",
                    buffer[0]
                )

                break  # exit the while loop
            radio.reUseTX()  # resend payload in top level of TX FIFO
        else:  # transmission succeeded
            print("Data sent!")
            
    end_timer = time.monotonic_ns()  # end timer
    print(
        "Time to transmit data = {} us. Detected {} failures.".format(
            (end_timer - start_timer) / 1000,
            failures
        )
    )



if __name__ == "__main__":

    
    buffer = b"LAX1500Y1700O180BX1360Y1230O172E"
    
    master(buffer)
 
