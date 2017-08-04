#!/usr/bin/env python
# -*- coding: utf-8 -*-

from tinyrpc.protocols.jsonrpc import JSONRPCProtocol
from tinyrpc.transports.http import HttpPostClientTransport
from tinyrpc import RPCClient

import sys

rpc_client = RPCClient(
    JSONRPCProtocol(),
    # HttpPostClientTransport('http://134.102.206.96:4040/')
    HttpPostClientTransport('http://127.0.0.1:4040/')
)

remote_server = rpc_client.get_proxy()

command = {'clientId': '111', 'childId': sys.argv[1]}

result = remote_server.cancel_simulation(command)

print "Server answered:" 
print result

