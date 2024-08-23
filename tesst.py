from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException
import time
import pika
import logging
import json
import logging.handlers
import requests
from requests.packages.urllib3.exceptions import InsecureRequestWarning
from datetime import datetime,timedelta
import signal

global should_exit

should_exit = False

requests.packages.urllib3.disable_warnings(InsecureRequestWarning)

rfh = logging.handlers.RotatingFileHandler(
    filename='../log/door_client.log', 
    mode='a',
    maxBytes=50*1024*1024,
    backupCount=5,
    encoding=None,
    delay=0
)
def handle_sigint(signum, frame):
    global should_exit
    should_exit = True
    logging.info("Nhận tín hiệu SIGINT (Ctrl+C)! Dừng chương trình.")
    exit(0)

# Đăng ký hàm xử lý cho tín hiệu SIGINT
signal.signal(signal.SIGINT, handle_sigint)

logging.basicConfig( level=logging.INFO, #filename='./backend/log/master_control.log',
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                    handlers=[rfh])

class AMQP(object):

    def __init__(self,host: str, port:int, username: str, password:str, vr_host = '/myvhost'):
        credentials = pika.PlainCredentials(username = username, password = password)

        connect_param = pika.ConnectionParameters(
            host= host, port = port,
            credentials=credentials, virtual_host= "/myvhost", heartbeat= 0
        )
        self.connection = pika.BlockingConnection(connect_param)
        self.channel = self.connection.channel()


class AgvStatus:
    def __init__(self) -> None:
        self.host = "127.0.0.1"
        self.port_agv = 8500
        self.port_map = 8501
        self.url_agv= "https://"+ self.host + ":"+ str(self.port_agv) +"/AGV"
        self.url_map =  "https://"+self.host+ ":"+ str(self.port_map) + "/map"

    def get_agv_status(self,map_name):
        try:
            res= requests.get(url = self.url_agv + "/get_all_map_devices_status/" + map_name, timeout=1,verify=False)
            if(res.status_code==200):
                return res.json()
            else:
                return None
        except Exception as e:
            logging.error(f"Error when get agv status: {e}")
            return None
    def get_map(self):
        try:
            res= requests.get(url=self.url_map + "/get_monitor_map", timeout=1,verify=False)

            if(res.status_code==200):
                return res.json()["map_name"]
            else:
                return "None"
        except Exception as e:
            logging.error(f"Error when get map: {e}")
            return "None"
class PauseServer:
    def __init__(self) -> None:
        self.host = "127.0.0.1"
        self.port = 9998
        self.url_agv= "http://"+ self.host + ":"+ str(self.port) +"/fleet/rtcService/sendPause"


    def send_pause(self,map,robot):
        mission={"mission_name" : "Pause",
                 "robot_name":robot,
                 "map_name":map,
                 "type":"door"}
        try:
            res= requests.post(url = self.url_agv,json=mission, timeout=1)
            logging.info(res.json())
            if(res.status_code==200):
                return res.json()
            else:
                return None
        except Exception as e:
            logging.error(f"Error when get agv status: {e}")
            return None
    def send_continue(self,map,robot):
        mission={"mission_name" : "Continue",
                 "robot_name":robot,
                 "map_name":map,
                 "type":"door"}
        try:
            res= requests.post(url = self.url_agv,json=mission, timeout=1)
            logging.info(res.json())
            if(res.status_code==200):
                return res.json()
            else:
                return None
        except Exception as e:
            logging.error(f"Error when get agv status: {e}")
            return None

class ModbusClient:
    def __init__(self) -> None:
        self.SERVER_IP = '192.168.0.120'
        self.SERVER_PORT = 28899
        self.rabbit_host = "127.0.0.1"
        self.rabbit_port = 5672
        self.rabbit_user = "thang"
        self.rabbit_pass = "12345"
        self.rabbit_vhost = "/myvhost"
        self.map_monitor = "None"

        self.client = ModbusTcpClient(self.SERVER_IP, port=self.SERVER_PORT)

        self.agv_mission = AMQP(self.rabbit_host,self.rabbit_port,self.rabbit_user,self.rabbit_pass,self.rabbit_vhost)
        logging.info(f"Connect to Rabbitmq: {self.rabbit_host}")
        self.agv_mission.channel.exchange_declare(exchange= "robotmission",exchange_type= "direct")
        self.agv_mission.channel.basic_qos(prefetch_count=1)
        self.agvstatus_class= AgvStatus()
        self.pause_server = PauseServer()
        
    def run(self):
        global should_exit
        try:
            connection = self.client.connect()
            if connection:
                time_now = datetime.now()
                self.map_monitor = self.agvstatus_class.get_map()

                while should_exit == False:
                    
                    if datetime.now() - time_now >  timedelta(seconds=10):
                        self.map_monitor = self.agvstatus_class.get_map()
                        time_now = datetime.now() 
                    if self.map_monitor == "None":
                        self.map_monitor = self.agvstatus_class.get_map()

                    result=self.client.read_holding_registers(address= 0x20, count =4,slave=0x11)
                    agv_status= self.agvstatus_class.get_agv_status(self.map_monitor)

                    
                    if not result.isError():
                        print("Gia tri thanh ghi la: ", result.registers)
                        if agv_status is not None:
                            
                            list_agv = []
                            # print("size:",len(agv_status))
                            for agv in agv_status:
                                # print("agv:",agv)
                                data_agv = {"state": "",
                                        "name": ""}
                                data_agv["state"] = agv[0]["state"]
                                data_agv["name"] = agv[0]["name"]
                                list_agv.append(data_agv)

                            self.check_door(result.registers,list_agv)
                            # print("Trạng thái của agv: ", list_agv)
                            
                        else:
                            logging.error("Không thể lấy được trạng thái của agv")
                    else:
                        print("Đọc thanh ghi thất bại:", result)
                    time.sleep(1)
            else:
                print("Không thể kết nối tới server.")

        except ModbusException as e:
            print(f"Đã xảy ra lỗi khi kết nối hoặc giao tiếp với server: {e}")

        finally:
            self.client.close()

    def send_pause(self,agv_name):
        mission_payload = self.Mission_payload("Pause_door","Pause", self.map_monitor, agv_name, "", mission_data = "")
        self.agv_mission.channel.basic_publish(exchange= "robotmission", routing_key = "_mis"+agv_name,body = mission_payload)
        logging.info("Send Pause to agv: "+agv_name+" because door is open")

    def send_continue(self,agv_name):
        mission_payload = self.Mission_payload("Continue_Door","Continue", self.map_monitor,agv_name, "", mission_data = "")
        self.agv_mission.channel.basic_publish(exchange= "robotmission", routing_key = "_mis"+agv_name,body = mission_payload)
        logging.info("Send Continue to agv: "+agv_name+" because door is close")

    def check_door(self,result,agv_status):

        if (result[0]!=0) or (result[0]==0 and result[1]==0):
            logging.info("result door is:" + str(result[0])+ " " +str(result[1])+ " " +str(result[2])+ " " +str(result[3]))
            # print ("result 1:", result[1])
            for agv in agv_status:
                if agv["state"] not in ["Pause","Charging","Offline","None"]:
                    self.pause_server.send_pause(self.map_monitor,agv["name"])
                    # self.send_pause(agv["name"])
            print("Door open")

        if result[1]!=0 :
            for agv in agv_status:
                if agv["state"] =="Pause":
                    # self.send_continue(agv["name"])
                    self.pause_server.send_continue(self.map_monitor,agv["name"])
            print("Door close")

    def Mission_payload(self,mission_id, mission_name ,map_name, robot_name ,id_goods, mission_data):
        mission_payload = {
                                "nameid":mission_id,
                                "mission_name": mission_name,
                                "map_id": map_name,
                                "robot": robot_name,
                                "id_goods" : id_goods,
                                "Data": mission_data
                            }
        return json.dumps(mission_payload)

def main():
    global should_exit

    process= ModbusClient()
    
    while (should_exit == False):
        process.run()
        time.sleep(1)

if __name__=="__main__":
    main()

