from django.http import HttpResponse
from django.shortcuts import render#导入render模块
from django.utils.safestring import mark_safe 
import paho.mqtt.client as mqtt
import time

MQTTHOST = "47.242.51.78"
MQTTPORT = 1883
mqttClient = mqtt.Client()
 
strlist = []
data_list1 = []
data_list2 = []
# 连接MQTT服务器
def on_mqtt_connect():
    mqttClient.connect(MQTTHOST, MQTTPORT, 60)
    mqttClient.loop_start()
	
on_mqtt_connect() 
 
# publish 消息
def on_publish(topic, payload, qos):
    mqttClient.publish(topic, payload, qos)
 
# 消息处理函数
def on_message_come(lient, userdata, msg):
#list = [{'name':'','data':''},{'name':'','data':''}]
  global strlist
  global data_list1,data_list2
  str1 = str(msg.payload)
  str2 = str1.replace("b'","")
  strlist = str2.split('|')
  try:
	  print(strlist[6])
	  localtime = time.asctime( time.localtime(time.time()))
	  data_list1 = [{'name':localtime,'data':strlist[0]},{'name':localtime,'data':strlist[1]},{'name':localtime,'data':strlist[2]},{'name':localtime,'data':strlist[3]},{'name':localtime,'data':strlist[4]},{'name':localtime,'data':strlist[5]},{'name':localtime,'data':strlist[6]},{'name':localtime,'data':strlist[7]},{'name':localtime,'data':strlist[8]},{'name':localtime,'data':strlist[9]}]#将设备发来的数据格式化
	  print(strlist)
	  print(msg.topic + " " + ":" + str(msg.payload))
  except Exception as e:
	  mqttClient.reconnect()  # 必须重连将 client._state 从断开状态切换为初始化状态
      #mqttClient.loop_start()
 
 
# subscribe 消息
def on_subscribe():
    mqttClient.subscribe("PineconePi/Watch", 1)
    mqttClient.on_message = on_message_come # 消息到来处理函数
 
 
def main():
    on_publish("PineconePi/Watch", "z", 1)#发送"z"命令读取设备信息
    on_subscribe()
    #mqttClient.disconnect()
    #mqttClient.loop_stop()
#    while True:
#        pass
 
#if __name__ == '__main__':
#  main()
def index(request):
  main()#每刷新一次网页，则通过MQTT发送读取命令
  
  return render(request,'index.html.txt',{"form1":data_list1,"form2":data_list2})
