from FETCH_MAPS import *
from collections import defaultdict
from PyQt5.QtCore import QRect, Qt

TILEWIDTH = 640       # Larget tile dimension you can grab without paying
TILEHEIGHT = 615      # Effective height to cut off Google logo
_EARTHPIX = 268435456  # Number of pixels in half the earth's circumference at zoom = 21
_pixrad = _EARTHPIX / math.pi
_MAPS_CACHE_PATH = os.path.expanduser('~/.local/share/mapscache')

class LatLon():
    def __init__(self, lat = 0.0, lon = 0.0):
        self.lat = lat
        self.lon = lon

class MapZoomObj():
    def __init__(self, mapname, zoom):
        filename = os.path.join(_MAPS_CACHE_PATH, mapname, str(zoom), 'info.txt')
        self.tiles = defaultdict(dict)

        with open(filename, 'r') as info_file:
            for line in info_file:
                items = line.split(' ')
                self.tiles[int(items[0])][int(items[1])] = LatLon(float(items[2]), float(items[3].strip()))

        self.cols = len(self.tiles)
        self.rows = len(self.tiles[0])

        # calculate min and max latlon of MapZoomObj
        min_latlon_center = self.tiles[0][self.rows-1]
        max_latlon_center = self.tiles[self.cols-1][0]

        min_lon = GoogleMapPlotter.pix_to_rel_lon(min_latlon_center.lon, int(-TILEWIDTH/2.0), zoom)
        max_lon = GoogleMapPlotter.pix_to_rel_lon(max_latlon_center.lon, int(TILEWIDTH/2.0), zoom)
        min_lat = GoogleMapPlotter.pix_to_rel_lat(min_latlon_center.lat, -int(TILEWIDTH/2.0 - TILEHEIGHT), zoom)
        max_lat = GoogleMapPlotter.pix_to_rel_lat(max_latlon_center.lat, -int(TILEWIDTH/2.0), zoom)

        self.min_latlon = LatLon(min_lat, min_lon)
        self.max_latlon = LatLon(max_lat, max_lon)

class GoogleMapPlotter():
    def __init__(self, mapdict, width, height, mapname, blankname):
        # grab all info objects, all at once
        self.mapname = mapname
        self.blankname = blankname
        self.mapdict = mapdict
        self.mz_objs = defaultdict(dict)
        for mapname in list(self.mapdict.keys()):
            if not mapname == self.blankname:
                for zoom in zooms:
                    self.mz_objs[mapname][zoom] = MapZoomObj(mapname, zoom)

        # grab objects for rendering
        self.width = width
        self.height = height
        self.center = LatLon(self.mapdict[self.mapname][0][0], self.mapdict[self.mapname][0][1])

        self.zoom = self.mapdict[self.mapname][1]
        if self.zoom > max(zooms):
            self.zoom = max(zooms)
        elif self.zoom < min(zooms):
            self.zoom = min(zooms)
        self.zoom_index = zooms.index(self.zoom)

        self.mz_obj = self.mz_objs[self.mapname][self.zoom]

        # kickoff fetching and updating
        self.x_offset = 0.0
        self.y_offset = 0.0
        self.window_img = self.new_image(self.width, self.height)
        self.fetch_and_update()

    def GetImage(self):
        return self.window_img

    def UpdateSize(self, new_width, new_height):
        self.width = new_width
        self.height = new_height
        self.window_img = self.new_image(self.width, self.height)
        self.fetch_and_update()

    def UpdateView(self, new_lat, new_lon):
        self.center.lat = new_lat
        self.center.lon = new_lon
        self.fetch_and_update()

    def UpdateZoom(self, zoom_increment):
        self.zoom_index = self.sat(self.zoom_index + zoom_increment, 0, len(zooms) - 1)
        self.zoom = zooms[self.zoom_index]
        self.fetch_and_update()

    def UpdateMap(self, mapname):
        self.mapname = mapname
        self.center.lat = self.mapdict[self.mapname][0][0]
        self.center.lon = self.mapdict[self.mapname][0][1]
        self.zoom = self.mapdict[self.mapname][1]
        self.fetch_and_update()

    def fetch_and_update(self):
        self.compute_region()
        if self.mapname == self.blankname:
            self.blank_update()
        else:
            self.mz_obj = self.mz_objs[self.mapname][self.zoom]
            self.fetch()
            self.update()

    @staticmethod
    def pix_to_rel_lon(centerlon, pix, zoom): # positive pix = right
        centerlonpix = _EARTHPIX + centerlon * math.radians(_pixrad)
        return math.degrees((centerlonpix + pixels_to_degrees(pix, zoom) - _EARTHPIX) / _pixrad)

    @staticmethod
    def rel_lon_to_rel_pix(centerlon, delta_lon, zoom):
        centerlonpix = _EARTHPIX + centerlon * math.radians(_pixrad)
        zoom_factor = 2**(21-zoom)
        return (math.pi/180.0*_pixrad*delta_lon + _EARTHPIX - centerlonpix)/zoom_factor

    @staticmethod
    def pix_to_rel_lat(centerlat, pix, zoom): # positive pix = down (!)
        sinlat = math.sin(math.radians(centerlat))
        centerlatpix = _EARTHPIX - _pixrad * math.log((1 + sinlat)/(1 - sinlat)) / 2.0
        return math.degrees(math.pi/2 - 2 * math.atan(math.exp(((centerlatpix + pixels_to_degrees(pix, zoom)) - _EARTHPIX) / _pixrad)))

    # you need a lat point (with a known y pix position) and the lat of the target point
    @staticmethod
    def rel_lat_to_rel_pix(centerlat, delta_lat, zoom):
        sinlat = math.sin(math.radians(centerlat))
        centerlatpix = _EARTHPIX - _pixrad * math.log((1 + sinlat)/(1 - sinlat)) / 2.0
        A = math.log(math.tan(math.pi/4 - math.pi/360.0*delta_lat))
        zoom_factor = 2**(21-zoom)
        return (_pixrad*A + _EARTHPIX - centerlatpix)/zoom_factor

    def fetch(self):
        self.img = self.fetch_tiles()

    def update(self):
        painter = QPainter()
        painter.begin(self.window_img)
        painter.fillRect(QRect(0, 0, self.width, self.height), Qt.black)
        painter.end()
        self.paste(self.window_img, self.img, -self.x_offset, -self.y_offset)

    def blank_update(self):
        painter = QPainter()
        painter.begin(self.window_img)
        painter.fillRect(QRect(0, 0, self.width, self.height), Qt.lightGray)
        painter.end()

    def new_image(self, width, height):
        # Format Value 4 corresponds to QImage.Format_RGB32
        # see http://pyqt.sourceforge.net/Docs/PyQt4/qimage.html#QImage-3
        # see http://pyqt.sourceforge.net/Docs/PyQt4/qimage.html#Format-enum
        return QImage(width, height, 4)

    def paste(self, big_image, small_image, upper_left_x, upper_left_y):
        destPos = QPoint(upper_left_x, upper_left_y)
        painter = QPainter(big_image)
        painter.drawImage(destPos, small_image)

    def grab_tile(self, i, j):
        filename = os.path.join(_MAPS_CACHE_PATH, self.mapname, str(self.zoom), '%d_%d.jpg' % (i, j))
        tile = None

        if os.path.isfile(filename):
            tile = QImage(QString(filename))

        return tile

    def localize_point(self, latlon, min_latlon, max_latlon):
        # which column?
        i = int((latlon.lon - min_latlon.lon) / (max_latlon.lon - min_latlon.lon) * self.mz_obj.cols)
        i = self.sat(i, 0, self.mz_obj.cols - 1)

        # which row? (calculated up-positive, but will be retrieved down-positive)
        j = self.mz_obj.rows - 1 - int((latlon.lat - min_latlon.lat) / (max_latlon.lat - min_latlon.lat) * self.mz_obj.rows)
        j = self.sat(j, 0, self.mz_obj.rows - 1)

        return i, j

    def sat(self, val, minval, maxval):
        if val < minval:
            return minval
        elif val > maxval:
            return maxval
        else:
            return val

    def compute_region(self):
        self.west = GoogleMapPlotter.pix_to_rel_lon(self.center.lon, -self.width/2, self.zoom)
        self.east = GoogleMapPlotter.pix_to_rel_lon(self.center.lon, self.width/2, self.zoom)
        self.north = GoogleMapPlotter.pix_to_rel_lat(self.center.lat, -self.height/2, self.zoom)
        self.south = GoogleMapPlotter.pix_to_rel_lat(self.center.lat, self.height/2, self.zoom)

        self.northeast = LatLon(self.north, self.east)
        self.southwest = LatLon(self.south, self.west)

    def fetch_tiles(self):
        # find out which i, j values correspond to each corner
        min_i, max_j = self.localize_point(self.southwest, self.mz_obj.min_latlon, self.mz_obj.max_latlon)
        max_i, min_j = self.localize_point(self.northeast, self.mz_obj.min_latlon, self.mz_obj.max_latlon)

        # fetch and paste images onto a big canvas
        bigsize_x = (max_i - min_i + 1) * TILEWIDTH
        bigsize_y = (max_j - min_j + 1) * TILEHEIGHT
        bigimage = self.new_image(bigsize_x, bigsize_y)

        for i in range(min_i, max_i + 1):
            for j in range(min_j, max_j + 1):
                tile = self.grab_tile(i, j)
                self.paste(bigimage, tile, (i-min_i)*TILEWIDTH, (j-min_j)*TILEHEIGHT)

        upper_left_center = self.mz_obj.tiles[min_i][min_j]
        upper_left_lon = GoogleMapPlotter.pix_to_rel_lon(upper_left_center.lon, int(-TILEWIDTH/2), self.zoom)
        upper_left_lat = GoogleMapPlotter.pix_to_rel_lat(upper_left_center.lat, int(-TILEWIDTH/2), self.zoom)

        self.x_offset = GoogleMapPlotter.rel_lon_to_rel_pix(upper_left_lon, self.west, self.zoom)
        self.y_offset = GoogleMapPlotter.rel_lat_to_rel_pix(upper_left_lat, self.north, self.zoom)

        return bigimage
