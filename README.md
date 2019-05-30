# ros_groundstation
RQT plugin for interacting with byu-magicc ROSplane

## Installation Instructions:

1. Install pip (if you don't already have it) and the python geographiclib library.
```
sudo apt-get install python-pip
sudo pip install geographiclib
```
2. Clone this repository into the source directory of a workspace. ROSplane and ROSflight are mandatory dependencies.

3. Build and source the workspace.

**IMPORTANT NOTE**: see the following section about map_info.xml, key.xml, and default_topics.yaml  **before running for the first time!**
4. Run with the command:
```
roslaunch ros_groundstation gs.launch
```
OR
```
rqt
```
- If running through rqt, the ground station will be available as a new plugin under ROSplane -> GroundStation.
## Running for the first time: map_info.xml and key.xml
When you run the ground station for the first time, the plugin will parse through the file **map_info.xml**, located in the top directory. Using the uncommented information, it will proceed to download map tiles for each map listed in ~/.local/share/mapscache.
In this file, each map field must provide a name, center latitude and longitude, and a meter-radius value to tell the parser how much map to download. A normal meter-radius is 1000 meters. The default displayed map is also defined in this file.
By default, only *Brigham Young University* is uncommented as an available map. Downloading the tiles for a single map will take upwards of 7-8 minutes, so keep this in mind when running for the first time. After the initial download process, no subsequent access to the internet will ever be needed to use the ground station.

Each map requires about 1000 images, or 100 MB of space, to render all four zoom levels. In order to download multiple maps in one sitting, the user must obtain an API key from https://developers.google.com/maps/documentation/staticmaps/#api (it takes about 30 seconds to do so).
The key must then be pasted within the file **key.xml**. Doing so will allow for downloading up to 25,000 images in a single day without incurring any charge.

To change which topics are subscribed to be default, modify the contents of **default_topics.yaml**, located in the params folder.

## In case of plugin issues:
Sometimes, rqt experiences conflicts with a new plugin if it appears to be overriding a previous one. This is only really an issue if building the ground station in multiple workspaces. To overcome this behavior, use the following command:
```
rm ~/.config/ros.org/rqt_gui.ini
```
Then, after sourcing the new workspace, the plugin should be available under ROSplane -> GroundStation in the rqt menu.
