import sys
import time
from utils.NatNetClient import NatNetClient
import utils.DataDescriptions as DataDescriptions
import utils.MoCapData as MoCapData

def my_parse_args(arg_list, args_dict):
    # set up base values
    arg_list_len=len(arg_list)
    if arg_list_len>1:
        args_dict["serverAddress"] = arg_list[1]
        if arg_list_len>2:
            args_dict["clientAddress"] = arg_list[2]
        if arg_list_len>3:
            if len(arg_list[3]):
                args_dict["use_multicast"] = True
                if arg_list[3][0].upper() == "U":
                    args_dict["use_multicast"] = False

    return args_dict

class Natnet_blimp:
    def __init__(self, my_ip, server_ip):
        optionsDict = {}
        optionsDict["clientAddress"] = my_ip
        optionsDict["serverAddress"] = server_ip
        optionsDict["use_multicast"] = False
        optionsDict = my_parse_args(sys.argv, optionsDict)
        # This will create a new NatNet client
        self.streaming_client = NatNetClient()
        self.streaming_client.set_client_address(optionsDict["clientAddress"])
        self.streaming_client.set_server_address(optionsDict["serverAddress"])
        self.streaming_client.set_use_multicast(optionsDict["use_multicast"])
    
