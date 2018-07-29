#!/usr/bin/env python3.5
import argparse
from pathlib import Path
import rospy
import numpy
import gdal
import osr
from nav_msgs.msg import OccupancyGrid
from rospy.numpy_msg import numpy_msg


PROJ4_SIMPLE = '+proj=tmerc \
                +lat_0=0 \
                +lon_0=0 \
                +k=1 \
                +x_0=0 \
                +y_0=0 \
                +ellps=WGS84 \
                +towgs84=0,0,0,0,0,0,0 \
                +units=m \
                +no_defs'


if __name__ == '__main__':
    rospy.init_node('topic_to_geotiff', anonymous=True)

    parser = argparse.ArgumentParser(description='Get message from an OccupancyGrid topic and save it as a geotiff.')
    parser.add_argument('topic', type=str)
    parser.add_argument('output', type=Path)

    args = parser.parse_args()

    msg = rospy.wait_for_message(args.topic, numpy_msg(OccupancyGrid), 10)

    rows = msg.info.height
    cols = msg.info.width
    resolution = msg.info.resolution
    bands = 1
    data = numpy.reshape(msg.data, (rows, cols))
    originX = msg.info.origin.position.x
    originY = msg.info.origin.position.y
    driver = gdal.GetDriverByName('GTiff')
    outRaster = driver.Create(str(args.output), cols, rows, bands, gdal.GDT_Byte)
    outRaster.SetGeoTransform((originX, resolution, 0, originY, 0, resolution))  # TODO Explain magic numbers.
    outBand = outRaster.GetRasterBand(1)
    outBand.WriteArray(data)
    outRasterSRS = osr.SpatialReference()
    outRasterSRS.ImportFromProj4(PROJ4_SIMPLE)
    outRaster.SetProjection(outRasterSRS.ExportToWkt())
    outBand.FlushCache()
    outRaster = None  # Free up underlying C++ memory.
