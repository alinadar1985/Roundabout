#!/usr/bin/env python

import os
import sys
import optparse
import subprocess
import json
import socket
import random
import math
import numpy
from geopy.distance import geodesic


# ********************************************************* Iterate through Vclass:passenger only  ***********************************************************
def sendCPM(ID,lat,lon,timestamp):
    # GET data from CARLA
    # first, get station data (the car itself)
    # second,  get detected objects from the car
    # x, y = traci.vehicle.getPosition(ID)
    #        lon, lat = traci.simulation.convertGeo(x, y)
    #        lat = numpy.around(lat,7)*10000000
    #        lon = numpy.around(lon,7)*10000000
    #        lat = int(lat)
    #        lon = int(lon)

    CPM_message = {
        "type": "cpm",
        "origin": "EURECOM_Geoserver_Client",
        "version": "1.0.0",
        "source_uuid": ID,
        "timestamp": timestamp,
        "message": {
            "protocol_version": 1,
            "station_id": float(ID),
            "generation_delta_time": 1,
            "management_container": {
                "station_type": 5,
                "reference_position": {
                    "latitude": lat,
                    "longitude": lon,
                    "altitude": 0},
                "confidence": {
                    "position_confidence_ellipse": {
                        "semi_major_confidence": 100,
                        "semi_minor_confidence": 50,
                        "semi_major_orientation": 180},
                    "altitude": 3
                }
            }
        }
    }
    json_data = json.dumps(CPM_message)
    print(json_data)
    # print("THE SIZE OF WHAT I SEND IS:",sys.getsizeof(json_data))
    server_address_port = ("127.0.0.1", 5003)
    print('connecting to {}'.format(server_address_port))
    print('Sending data for PASSENGER TIME_STEP: {}'.format(timestamp))
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.sendto(json_data.encode(), server_address_port)
    client.close()
    print("-------------------------- Recieved -------------------------------------------")
    del json_data

    # ******************************************************************************************************************************************

    # main entry point
    if __name__ == "__main__":
        sendCPM = sendCPM()

