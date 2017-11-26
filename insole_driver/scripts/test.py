import pygatt
import time
from threading import Thread
from binascii import hexlify
import logging

logging.basicConfig()
logging.getLogger('pygatt').setLevel(logging.DEBUG)

def littleEndian(_orig):
    return ''.join(sum([(c, d, a, b) for (a, b, c, d) in zip(*[iter(_orig)] * 4)], ()))

def callback_func1(handle, rawData):
    binaryData = '{}'.format(hexlify(str(rawData)))
    bD_lE = littleEndian(binaryData)
    print "dev1"
    print bD_lE



class Person(object):

    def schrei(self, text):
       while 1:
           print(text+": aahhhhhh")

liste = list()
liste.append("Hans")
liste.append("Peter")

liste2 = list()

for person in liste:

    personObject = Person()
    t = Thread(target=personObject)
    liste2.append(t)


for thr in liste2:
    thr.start()

while 1:
  pass