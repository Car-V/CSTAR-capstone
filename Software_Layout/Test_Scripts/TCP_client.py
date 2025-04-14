# Networks 2540
print( 'Client program started.' )

# Import socket package
from socket import *

# Initialize important variables
server_address = '192.168.41.121'
server_port_number = 12000
server_identifier = ( server_address, server_port_number)

# Create client socket
client_socket = socket( AF_INET, SOCK_STREAM )
print( 'Client socket created.' )

# Connect client socket
client_socket.connect( server_identifier )
print( 'Client socket connected.' )

# Prepare first message
coordinate = (12.65, 13.45)
extra_val = True

# string_to_server = '12-23-56-5-95-3-55-24-9-34'
# Convert input string to message
#message_to_server = string_to_server.encode()
message_to_server = (coordinate, extra_val)

# Send message to server
client_socket.send( message_to_server )
print( 'Client sent message to server at: ' , server_identifier)

# Start communication loop

# Receive message from server
#message_from_server = client_socket.recv( 2048 )
#print( 'Client received message from server at:', server_identifier)

# Convert message to string
#string_from_server = message_from_server.decode()

#Determine response string
# if string_from_server = 'RESULT:316':
#	print("Good job! The answer is correct!")
#else:
#	print('mmm...I received a message but it looks incorrect!')

#Close the client socket
client_socket.close()
print( 'Client socket closed.' )
print( 'Client program terminated.' )
