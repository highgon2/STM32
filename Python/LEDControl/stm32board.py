import queue
import socket
import threading

class STM32LED:
    def __init__(self, label, state, voltage):
        self.__label   = label
        self.__state   = state
        self.__voltage = voltage

    @property
    def label(self):
        return self.__label

    @property
    def state(self):
        return self.__state

    @property
    def voltage(self):
        return self.__voltage

    @state.setter
    def state(self, state):
        self.__state = state

    @voltage.setter
    def voltage(self, voltage):
        self.__voltage = voltage

class STM32Board(threading.Thread):
    def __init__(self, queue):
        threading.Thread.__init__(self)

        self.daemon     = True
        self.__run      = False
        self.__queue    = queue
        self.__socket   = None
        self.__led_list = list()

    def connect(self, ip, port):
        try :
            self.__socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.__socket.settimeout(0.1)
            self.__socket.connect((ip, port));
        except Exception as e:
            print(e)
            return False
        return True

    def disconnect(self):
        self.__run = False

    def request_command(self, json_string):
        send_message = "JSON" + json_string + "\r\n"
        self.__socket.sendall(send_message.encode())
        
    def run(self):
        self.__run = True
        while(self.__run):
            try:
                data = self.__socket.recv(1024)
                self.__queue.put(data.decode())
                # print("received :", data.decode())
            except socket.timeout:
                pass
        self.__socket.close()

    def get_led(self, label):
        for led in self.__led_list:
            if label == led.label:
                return led
        return None

    def set_led(self, label, state):
        for led in self.__led_list:
            if led.label == label:
                led.state = state

    def append_led(self, label, state, voltage):
        self.__led_list.append(STM32LED(label, state, voltage))
