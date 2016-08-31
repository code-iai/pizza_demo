#!/usr/bin/env python
# -*- coding: utf-8 -*-

from tinyrpc.protocols.jsonrpc import JSONRPCProtocol
from tinyrpc.transports.http import HttpPostClientTransport
from tinyrpc import RPCClient

rpc_client = RPCClient(
    JSONRPCProtocol(),
    HttpPostClientTransport('http://127.0.0.1:5050/')
)

remote_server = rpc_client.get_proxy()


## Cutting test: slice a particular region

#[{'action_cores': [{'action_core_name': 'Cutting',
#                     'action_roles': [{'role_name': 'action_verb',
#                                       'role_value': 'cut.v.01'},
#                                      {'role_name': 'amount',
#                                       'role_value': 'two.n.01'},
#                                      {'role_name': 'unit',
#                                       'role_value': 'piece.n.08'},
#                                      {'role_name': 'obj_to_be_cut',
#                                       'role_value': 'pizza.n.01'}]}]}]


cut_a_slice = {'action_cores': [{'action_core_name': 'Cutting', 'action_roles': [{'role_name': 'unit', 'role_value': 'piece.n.08'}, {'role_name': 'obj_to_be_cut', 'role_value': 'pizza.n.01'}, {'role_name': 'action_verb', 'role_value': 'cut.v.01'}, {'role_name': 'amount', 'role_value': 'two.n.01'}]}]}
cut_test = [cut_a_slice]

# call a method called 'reverse_string' with a single string argument
result = remote_server.prac2cram_client(cut_test)

print "Server answered:" 
print result

