from qgis.core import QgsCoordinateReferenceSystem


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


simpleCrs = QgsCoordinateReferenceSystem('PROJ4:' + PROJ4_SIMPLE)
