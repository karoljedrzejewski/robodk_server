# You can also use the new version of the API:
# taskset -c l python serwer_aktualne.py

from robodk import robolink    # RoboDK API
from robodk import robomath    # Robot toolbox
import pickle
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import socket
import sys
import math
import datetime
import threading


# funkcja do sprawdzania obecnej konfiguracji robota
def check_config(item):

    global touching_off
    pose = item.Joints()
    conf_RLF = item.JointsConfig(pose).list()

    rear  = conf_RLF[0] # 1 if Rear , 0 if Front
    lower = conf_RLF[1] # 1 if Lower, 0 if Upper (elbow)
    flip  = conf_RLF[2] # 1 if Flip , 0 if Non flip (Flip is usually when Joint 5 is negative)

    if rear == 1 and lower == 1 and flip == 1:
        if pose.list()[4] < -91 or pose.list()[4] > -89:
            touching_off = True
        else:
            touching_off = False
    else:
        
        with open('error_logger.txt', 'a+') as f:
            now = datetime.datetime.now()
            dt_string = now.strftime("%d/%m/%Y %H:%M:%S")
            f.write(dt_string + '\n')
            f.write(f'Konfiguracja robota nieprawidlowa: {conf_RLF}\n')
            f.write(f'Pose: {pose}\n')
            f.write(f'Joints: {item.Joints().list()}\n')
            f.write(f'JointsConfig: {item.JointsConfig(pose).list()}\n\n')

        #raise Exception('Konfiguracja robota nieprawidlowa')

        

# funkcja do znikania wybranego pudla i ustawiania kolejnych tacek jako widoczne
def box_out_trays_in(box):
    
    box.setVisible(False)
    stack_name = box.Name()
    
    stack_name_list = list(stack_name)
    
    if stack_name_list[7] == '0':
        stack_name_list[6] = '0'
        stack_name_list[7] = '9'
    else:
        x = int(stack_name_list[7])
        x -= 1
        stack_name_list[7] = str(x)

    stack_name = ''.join(stack_name_list)

    for i in range(1, 5):
        tray_name = stack_name + 'T' + str(i)
        tray = RDK.Item(tray_name)
        tray.setVisible(True)

# wybor strony w ktora ma byc zwrocona tacka przyczepiona do ssawki
def choose_picked(tacka):
    reversed = ['SCF02', 'SCF03', 'SCF04', 'SCF05', 'SCF06', 'SCF08']
    
    if tacka.Name()[0:5] in reversed:
        if tacka.Name().endswith('T1') or tacka.Name().endswith('T2'):
            return 'tacka2'
        elif tacka.Name().endswith('T3') or tacka.Name().endswith('T4'):
            return 'tacka'
        
    elif tacka.Name()[0:5] == 'SCF11':
        return 'tacka2'
    
    else:
        if tacka.Name().endswith('T1') or tacka.Name().endswith('T2'):
            return 'tacka'
        elif tacka.Name().endswith('T3') or tacka.Name().endswith('T4'):
            return 'tacka2'


#sprawdza czy występuje kolizja, działa zarówno do wykrywania niechcianych kolizji jak i tych potrzebnych do podniesienia tacek i pudeł
def check_if_collision():
    global inprogress
    i = 0
    items = RDK.CollisionItems()
    # if len(items) > 2:
    #     raise Exception('Za dużo kolizji')
    for item in items:

        i+=1

        if item.Name().startswith('SCF') or item.Name().startswith('tacka') or item.Name().startswith('cont1'):
            
            global gripper_full

            if len(item.Name()) == 10 and gripper_full == False:
                item.setVisible(visible = 0)
                tray_to_show = choose_picked(item)
                RDK.Item(tray_to_show).setVisible(True)
                
            elif len(item.Name()) == 8 and gripper_full == False:
                box_out_trays_in(item)
                RDK.Item('cont1').setVisible(True)

            gripper_full = True
            inprogress = False
            return
        
    inprogress = False

        # elif i == 2:
        #     raise Exception('Kolizja spowodowana nie przez pudełko/tackę')    
        

# funkcja do puszczenia chwyconej tacki/pudełka
def pump_off():

    global gripper_full
    pump_item_names = ['tacka', 'tacka2', 'cont1']
    list(map(lambda x: RDK.Item(x).setVisible(False), pump_item_names))
    gripper_full = False
    print('pompa wylaczona')


RDK = robolink.Robolink()
item = RDK.Item('Janusz5')


server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server.bind(('', 12346))

spawns1 = RDK.Item('tacka')
spawns2 = RDK.Item('tacka2')
spawns3 = RDK.Item('cont1')
robot = RDK.Item('Janusz5')
chwytak = RDK.Item('Chwytak_RoboDK_4')

# wyłączenie kolizji przy starcie serwera
RDK.setCollisionActivePair(check_state=0, item1=spawns1, item2=chwytak)
RDK.setCollisionActivePair(check_state=0, item1=spawns2, item2=chwytak)
RDK.setCollisionActivePair(check_state=0, item1=spawns3, item2=chwytak)
RDK.setCollisionActivePair(check_state=0, item1=robot, item2=chwytak, id1=5)
RDK.setCollisionActivePair(check_state=0, item1=robot, item2=chwytak, id1=6)

i = 0
gripper_full = False
touching_off = False
inprogress = False

# joints = list(map(lambda x: round(x, 4) ,(item.Joints().list())))
# print(joints)
_joints_in_deg = item.Joints().list()
joints = list(map(lambda x: round(math.radians(x), 6) ,(_joints_in_deg)))

while True:
    data, adress = server.recvfrom(1024)
    decoded_data = pickle.loads(data)
    
    if not touching_off and not inprogress:
        inprogress = True
        coll_thread = threading.Thread(target=check_if_collision)
        coll_thread.start()

    if decoded_data == 2: # get_position
        joints_in_rad = list(map(lambda x: round(math.radians(x), 6) ,(_joints_in_deg)))
        return_data = (2, joints_in_rad)
        now = datetime.datetime.now()

        conf_thread = threading.Thread(target=check_config, args=(item,))
        conf_thread.start()
        
        print(f"wysylam {return_data}, {now.strftime('%Y-%m-%d %H:%M:%S.%f')}")
        server.sendto(pickle.dumps(return_data), adress)

    elif decoded_data[0] == 1: # set_position
        now = datetime.datetime.now()
        print(f"odbieram {decoded_data}, {now.strftime('%Y-%m-%d %H:%M:%S.%f')}")
        pose_r = decoded_data[1]
        pose = list(map(lambda x: math.degrees(x), pose_r))
        __joints_in_deg = item.Joints()
        _joints_in_deg = __joints_in_deg.list()
        item.setJoints(pose)
        return_data = (1, True)
        server.sendto(pickle.dumps(return_data), adress)


    elif decoded_data[0] == 3: # pump_off
        print(f'wylaczam pompe')
        pump_off()
        return_data = True
        server.sendto(pickle.dumps(return_data), adress)