import math, urllib, os, shutil, time, sys
import xml.etree.cElementTree as ET
from map_info_parser import get_key
from PyQt5.QtGui import QImage, QPainter
from PyQt5.QtCore import QPoint

QString = type("")

_default_radius_m = 1000            # * radius of map coverage, in meters
zooms = [17,18,19,20]               # 0-22

_PWD = os.path.dirname(os.path.abspath(__file__))

def pd(path):
    return os.path.abspath(os.path.join(path, os.pardir))

_INFO_FILE_PATH = os.path.join(pd(pd(_PWD)), 'map_info.xml')

if get_key():
    _KEY = '&key=' + get_key()
else:
    _KEY = ''

TILEWIDTH = 640       # Larget tile dimension you can grab without paying
TILEHEIGHT = 615      # Effective height to cut off Google logo
_GOOGLESTRIP = 25      # Height of the Google logo, which will be cut off
_GRABRATE = 4          # Fastest rate at which we can download tiles without paying
_EARTHPIX = 268435456  # Number of pixels in half the earth's circumference at zoom = 21
_DEGREE_PRECISION = 4  # Number of decimal places for rounding coordinates
_pixrad = _EARTHPIX / math.pi

_MAPS_CACHE_PATH = os.path.expanduser('~/.local/share/mapscache')

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def startProgress(title):
    global progress_x
    sys.stdout.write(title + ": [" + "-" * 40 + "]" + chr(8) * 41)
    sys.stdout.flush()
    progress_x = 0

def progress(x):
    global progress_x
    x = int(x * 40 // 100)
    sys.stdout.write("#" * (x - progress_x))
    sys.stdout.flush()
    progress_x = x

def endProgress():
    sys.stdout.write("#" * (40 - progress_x) + "]\n")
    sys.stdout.flush()

def clearContents(folder):
    shutil.rmtree(folder)
    os.makedirs(folder)

def round_to(val, digits):
    return int(val * 10**digits) / 10.**digits

def pixels_to_degrees(pixels, zoom):
    return pixels * 2 ** (21 - zoom)

def pix_to_lon(j, lon, ntiles, tile_size, zoom):
    lonpix = _EARTHPIX + lon * math.radians(_pixrad)
    return math.degrees((lonpix + pixels_to_degrees(((j)-ntiles/2)*tile_size, zoom) - _EARTHPIX) / _pixrad)

def pix_to_lat(k, lat, ntiles, tile_size, zoom):
    sinlat = math.sin(math.radians(lat))
    latpix = _EARTHPIX - _pixrad * math.log((1 + sinlat)/(1 - sinlat)) / 2.0
    return math.degrees(math.pi/2 - 2 * math.atan(math.exp(((latpix + pixels_to_degrees((k-ntiles/2)*tile_size, zoom)) - _EARTHPIX) / _pixrad)))

# for fetching tiles from google
urlbase = 'https://maps.googleapis.com/maps/api/staticmap?center=%f,%f&zoom=%d&maptype=%s&size=%dx%d&format=jpg'
urlbase += _KEY

# extract info for each map, for comparing and compiling
print bcolors.BOLD + 'Parsing map_info.xml...' + bcolors.ENDC
map_dict = {}
try:
    xmlroot = ET.parse(_INFO_FILE_PATH).getroot()
    for xmlnode in xmlroot.findall('map'):
        name = xmlnode.attrib['name']
        map_dict[name] = {}
        map_dict[name]['lat'] = float(str(xmlnode.find('lat').text))
        map_dict[name]['lon'] = float(str(xmlnode.find('lon').text))
        if not xmlnode.find('radius_m'):
            map_dict[name]['r_m'] = _default_radius_m
        else:
            map_dict[name]['r_m'] = int(str(xmlnode.find('radius_m').text))
        map_dict[name]['fetch'] = False
except:
    print bcolors.BOLD + bcolors.FAIL + 'ERROR: Incorrectly formatted xml file!' + bcolors.ENDC

# check to see which maps need to be fetched or updated
for mapname in map_dict:
    print bcolors.OKGREEN + 'Processing maps for %s...' % mapname + bcolors.ENDC
    folder_path = os.path.join(_MAPS_CACHE_PATH, mapname)
    log_path = os.path.join(folder_path, '_log.txt')
    if os.path.exists(folder_path):
        if os.path.exists(log_path):
            with open(log_path, 'r') as logfile:
                loginfo = logfile.read().split('\n')
                try:
                    if not (float(loginfo[0]) == map_dict[mapname]['lat'] and \
                            float(loginfo[1]) == map_dict[mapname]['lon'] and \
                            int(loginfo[2]) == map_dict[mapname]['r_m']):
                        map_dict[mapname]['fetch'] = True
                except:
                    map_dict[mapname]['fetch'] = True
        else:
            map_dict[mapname]['fetch'] = True
    else:
        os.makedirs(folder_path)
        map_dict[mapname]['fetch'] = True

    # fetch tiles from the internet, if needed
    if map_dict[mapname]['fetch']:
        clearContents(folder_path)
        print 'Downloading maps for %s...' % mapname
        latitude = round_to(map_dict[mapname]['lat'], _DEGREE_PRECISION)
        longitude = round_to(map_dict[mapname]['lon'], _DEGREE_PRECISION)
        radius_meters = map_dict[mapname]['r_m']

        for zoom in zooms:
            zoom_folder_path = os.path.join(folder_path, str(zoom))
            os.makedirs(zoom_folder_path)
            startProgress('\tAt zoom level = %d' % zoom)
            # the following conversion formula comes from an employee at google:
            # https://groups.google.com/forum/#!topic/google-maps-js-api-v3/hDRO4oHVSeM
            pixels_per_meter = 2**zoom / (156543.03392 * math.cos(math.radians(latitude)))
            # number of tiles required to go from center latitude to desired radius in meters
            ntiles_x = int(round(2.0 * pixels_per_meter * radius_meters / TILEWIDTH))
            ntiles_y = int(round(2.0 * pixels_per_meter * radius_meters / TILEHEIGHT))

            if zoom <= 19:
                bigimage = QImage(ntiles_x * TILEWIDTH, ntiles_y * TILEHEIGHT, 4) # -----------

            with open(os.path.join(zoom_folder_path, 'info.txt'), 'w') as info_file:
                for i in range(ntiles_x):
                    lon = pix_to_lon(i, longitude, ntiles_x, TILEWIDTH, zoom)
                    for j in range(ntiles_y):
                        lat = pix_to_lat(j, latitude, ntiles_y, TILEHEIGHT, zoom)
                        specs = lat, lon, zoom, 'satellite', TILEWIDTH, TILEWIDTH
                        filename = os.path.join(zoom_folder_path, ('%d_%d' % (i, j)) + '.jpg')
                        url = urlbase % specs
                        result = urllib.urlopen(url).read()
                        tile = QImage()
                        tile.loadFromData(result)
                        tile.save(QString(filename))
                        info_file.write('%d %d %f %f\n' % (i, j, lat, lon))

                        progress(int((1.0 * ntiles_y * i + j) / (ntiles_x * ntiles_y) * 100))
                        time.sleep(1.0 / _GRABRATE)
                        if zoom <= 19:
                            destPos = QPoint(i*TILEWIDTH, j*TILEHEIGHT) # ---------
                            painter = QPainter(bigimage) # --------------------------
                            painter.drawImage(destPos, tile) # ----------------------
                            painter.end() # -----------------------------------------

                endProgress()
            if zoom <= 19:
                bigfilename = os.path.join(folder_path, 'ZOOM %d.jpg' % zoom) # --------------
                bigimage.save(QString(bigfilename)) # ----------------------------------------
            #+++++++++++++++++++++++++++++++++++++
        with open(log_path, 'w') as logfile:
            logfile.write(str(map_dict[mapname]['lat']) + '\n')
            logfile.write(str(map_dict[mapname]['lon']) + '\n')
            logfile.write(str(map_dict[mapname]['r_m']) + '\n')
    else:
        print 'Downloaded maps for %s already up to date.' % mapname
