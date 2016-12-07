#!/usr/bin/env python
# -*- coding: utf-8 -*-

from tinyrpc.protocols.jsonrpc import JSONRPCProtocol
from tinyrpc.transports.http import HttpPostClientTransport
from tinyrpc import RPCClient

rpc_client = RPCClient(
    JSONRPCProtocol(),
    HttpPostClientTransport('http://127.0.0.1:4040/')
)

remote_server = rpc_client.get_proxy()


## Cutting test: slice a particular region

cut_a_slice = {'action_cores': [{'action_core_name': 'Cutting', 'action_roles': [{'role_name': 'utensil', 'role_value': 'knife'}, {'role_name': 'unit', 'role_value': 'slice'}, {'role_name': 'obj_to_be_cut', 'role_value': 'bread'}, {'role_name': 'action_verb', 'role_value': 'cut'}, {'role_name': 'amount', 'role_value': '4'}]}]}
cut_test = [cut_a_slice]

# call a method called 'reverse_string' with a single string argument
#result = remote_server.prac2cram_client(cut_test)
result = remote_server.prac2cram_client({"clientId": "testClient", "tasks": cut_test})

print "Server answered:" 
print result

