#!/usr/bin/env python3.5
import argparse
from pathlib import Path
import numpy
import gdal
import osr
import json


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
    parser = argparse.ArgumentParser(description='Get message from an OccupancyGrid topic and save it as a geotiff.')
    parser.add_argument('input', type=Path)
    parser.add_argument('output', type=Path)

    args = parser.parse_args()

    with args.input.open('r') as f:
        data = json.load(f)

    rows = data['height']
    cols = data['width']
    resolution = data['resolution']
    bands = 1
    originX = data['origin_x']
    originY = data['origin_y']

    npData = numpy.reshape(data['data'], (rows, cols))
    driver = gdal.GetDriverByName('GTiff')
    outRaster = driver.Create(str(args.output), cols, rows, bands, gdal.GDT_Byte)
    outRaster.SetGeoTransform((originX, resolution, 0, originY, 0, resolution))  # TODO Explain magic numbers.
    outBand = outRaster.GetRasterBand(1)
    outBand.WriteArray(npData)
    outRasterSRS = osr.SpatialReference()
    outRasterSRS.ImportFromProj4(PROJ4_SIMPLE)
    outRaster.SetProjection(outRasterSRS.ExportToWkt())
    outBand.FlushCache()
    outRaster = None  # Free up underlying C++ memory.
