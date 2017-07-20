import xml.etree.cElementTree as ET
import os, errno

PWD = os.path.dirname(os.path.abspath(__file__))

def pd(path):
    return os.path.abspath(os.path.join(path, os.pardir))

KEY_FILE_PATH = os.path.join(pd(pd(PWD)), 'key.xml')
INFO_FILE_PATH = os.path.join(pd(pd(PWD)), 'map_info.xml')

def get_key():
    xmlroot = ET.parse(KEY_FILE_PATH).getroot()
    return xmlroot.text.strip()

def get_default():
    xmlroot = ET.parse(INFO_FILE_PATH).getroot()
    return xmlroot.attrib['default']

def get_gps_dict():
    gps_dict = {}
    xmlroot = ET.parse(INFO_FILE_PATH).getroot()
    for xmlnode in xmlroot.findall('map'):
        name = xmlnode.attrib['name']
        lat = float(str(xmlnode.find('lat').text))
        lon = float(str(xmlnode.find('lon').text))
        zoom = int(str(xmlnode.find('zoom').text))
        gps_dict[name] = [[lat, lon], zoom]
    return gps_dict

def get_typed_waypoints(map_name, folder_name):
    wp_file_path = os.path.join(PWD,'resources','wp_data',folder_name,'%s_%s.txt' % (map_name, folder_name))
    return get_waypoints(wp_file_path)

def get_windspeed_components():
    ws_file_path = os.path.join(PWD, 'resources', 'wp_data', 'drop_wps', '_WINDSPEED_.txt')
    try:
        with open(ws_file_path, 'r') as ws_file:
            ws_t = ws_file.read().split()
            Vwind_n = float(ws_t[0])
            Vwind_e = float(ws_t[1])
        return [Vwind_n, Vwind_e]
    except:
        return [0.0, 0.0]
