import zmq
import json
import numpy as np


class Signaler:
  
  def __init__(self):
    self.ctx = zmq.Context()
    self.sock = self.ctx.socket(zmq.REQ)
    
    self.sock.connect("tcp://localhost:3563")
    
  def updateComputer(self, posx, posy, velx, vely):
    json_str = json.dumps({
      "posx": posx,
      "posy": posy,
      "velx": velx,
      "vely": vely
    })
    self.sock.send_string(json_str)
    
    res_str = self.sock.recv_string()
    res_json = json.loads(res_str)
    
    return np.array([res_json["Fx"], res_json["Fy"]])