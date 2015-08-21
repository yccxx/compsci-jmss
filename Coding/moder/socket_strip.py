__author__ = 'conroycheers'

import thread, socket, time, sys

class Carrier:
    def __init__(self, altitude, velocity, heading):
        self.altitude = altitude
        self.velocity = velocity
        self.heading = heading

    def infostring(self):
        return "alt=" + str(self.altitude) + ",vel=" + str(self.velocity) + ",head=" + str(self.heading) + "\n"

def trigger_drop():
    print('GOGOGOGOGOGOGOGOGO')

#Function for handling connections. This will be used to create threads
def clientthread(conn, c, flow_object=None):
    while True:
        try:
            #Sending message to connected client
            conn.send(c.infostring()) #send only takes string
            #send_data = str(flow_object.update())
            #conn.sendall(send_data)
            data = conn.recv(1024)
            if data:
                datastr = str(data).strip()
                print(data)
                if datastr == "exit":
                    break
                if datastr == "drop":
                    trigger_drop()
            else:
                continue
            time.sleep(0.01)
        except Exception as e:
            if '35' in str(e):
                pass
            else:
                print(e)
                break
    #came out of loop
    conn.close()

def sock_loop(s, c):
    while True:
        # obtain socket connection
        #wait to accept a connection - blocking call
        conn, addr = s.accept()
        print('Connected with ' + addr[0] + ':' + str(addr[1]))
        #start new thread takes 1st argument as a function name to be run, second is the tuple of arguments to the function.
        thread.start_new_thread(clientthread ,(conn, c))

# Socket-related code
HOST = ''   # Symbolic name meaning all available interfaces
PORT = 7898 # Arbitrary non-privileged port

c = Carrier(0,0,0)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print('Socket created')
#Bind socket to local host and port
try:
    s.bind((HOST, PORT))
except socket.error , msg:
    print('Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1])
    sys.exit()
print('Socket bind complete')
s.setblocking(False)
#Start listening on socket
s.listen(10)
print('Socket now listening')

while True:
    # obtain socket connection
    #wait to accept a connection - blocking call
    while True:
        try:
            conn, addr = s.accept()
            break
        except socket.error:
            continue
    print('Connected with ' + addr[0] + ':' + str(addr[1]))
    #start new thread takes 1st argument as a function name to be run, second is the tuple of arguments to the function.
    thread.start_new_thread(clientthread ,(conn, c))