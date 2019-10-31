import numpy as np
from shapely import ops
from shapely.geometry import MultiPoint,Point,MultiLineString,LineString
import matplotlib.pyplot as plt
from math import sin, cos, sqrt, atan2, radians,pi, acos
from geographiclib.geodesic import Geodesic
import geopandas as gpd
import pandas as pd
import os
from gym_jsbsim.catalogs import utils


def get_bearing(p1, p2):
    """
    :param p1: (long,lat)
    :param p2: (long,lat)
    :return: bearing in degrees [0,360]
    """
    long1, lat1 = p1
    long2, lat2 = p2
    brng = Geodesic.WGS84.Inverse(lat1, long1, lat2, long2)['azi1']
    return (brng+360)%360


def get_distance(p1, p2):
    """
    :param p1: (long,lat)
    :param p2: (long,lat)
    :return: distance in km
    """
    lat1 = radians(p1[1])
    lon1 = radians(p1[0])
    lat2 = radians(p2[1])
    lon2 = radians(p2[0])
    dist = 6371.01 * acos(sin(lat1) * sin(lat2) + cos(lat1) * cos(lat2) * cos(lon1 - lon2))
    return dist


class taxi_path(object):
    """
    methods to use:
    1. self.update(ref_pts,aircraft_heading_deg)  -> list[[(x,y),distance,heading,(long,lat)],[.....]]
    2. self.shortest_dist  -> returns the shortest (normal) distance to target path.
    3. self.plot() -> to visualize the airport, current path, next target points and aircraft position.
    """

    def __init__(self, path_to_points="../points.json", number_of_points_to_use=10):
        """
        :param ambd_folder_path: path to folder with airport shapefiles
        :param number_of_points_to_use: number of next points to return
        """

        # path_to_points = '/home/jyotsna/Documents/RL/learning-to-fly-master/code/notebooks/points.json'
        points_df = pd.read_json(path_to_points, orient='index', precise_float=True, )
        points_df.columns = ['long', 'lat']
        points_df['points'] = points_df.apply(lambda row: (row.long, row.lat), axis=1)
        points_df.drop(['long', 'lat'], axis=1, inplace=True)
        # add relative properties of points
        relative_distance = [0]
        relative_distance[1:] = [
            float("{:.4f}".format(get_distance(points_df.points.iloc[i], points_df.points.iloc[i - 1]))) for i in
            range(1, points_df.shape[0])]
        relative_bearing = [0]
        relative_bearing[1:] = [
            float("{:.4f}".format(get_bearing(points_df.points.iloc[i - 1], points_df.points.iloc[i]))) for i in
            range(1, points_df.shape[0])]

        points_df['rel_distance'] = relative_distance
        points_df['rel_bearing'] = relative_bearing
        self.points_df = points_df

        #self.centerline = LineString(list(points_df.points.values[:]))
        self.centerlinepoints = [(1.372116667,43.618963889),(1.372016751,43.619059862),(1.371703332,43.619360912),(1.371468441,43.619586533),(1.371364003,43.619667262),(1.3713364,43.619694125),(1.37130529,43.619718469),(1.371264514,43.619743713),(1.37120238,43.619770899),(1.371118886,43.619800027),(1.371072296,43.619810362),(1.371023144,43.619817431),(1.370970262,43.619821007),(1.370916573,43.619820496),(1.370860832,43.619819009),(1.370821503,43.61981796),(1.370761085,43.619813929),(1.370612467,43.619800938),(1.370522764,43.619788652),(1.37045667,43.619775735),(1.370383703,43.619755679),(1.370301927,43.61972727),(1.370204167,43.619686049),(1.369601175,43.619353657),(1.369531563,43.619331444),(1.3694207,43.619295581),(1.369349807,43.619282057),(1.369278193,43.619272181),(1.369180567,43.619264384),(1.36908951,43.619263568),(1.369009184,43.619268641),(1.368927868,43.619279205),(1.36885231,43.619294298),(1.368770028,43.619316051),(1.368703441,43.619336744),(1.368655029,43.619355092),(1.368601227,43.619382152),(1.368512991,43.619436889),(1.368446579,43.619478038),(1.366613251,43.621239278),(1.365656765,43.622152509),(1.363194125,43.624522927),(1.363131769,43.624642165),(1.363093005,43.624726048),(1.363083983,43.624764916),(1.363078816,43.624811425),(1.363078256,43.624860737),(1.3630833,43.6249154),(1.363093304,43.624969465),(1.363108326,43.625022064),(1.363136418,43.625090717),(1.363169953,43.625148595),(1.363203392,43.625192078),(1.36324252,43.625233577),(1.363296192,43.625287064),(1.363368814,43.62534465),(1.363670323,43.625509935),(1.364148757,43.625772208),(1.364225859,43.62580404),(1.364295357,43.625826689),(1.364360195,43.625842476),(1.364418677,43.625852282),(1.364473189,43.625857386),(1.364528372,43.625858216),(1.364572787,43.625857483),(1.364625924,43.625856607),(1.364691409,43.625849909),(1.364761715,43.625838075),(1.36486516,43.625812385),(1.364998595,43.625738566),(1.365149163,43.625656419),(1.366265751,43.624583896),(1.366338612,43.624544876),(1.366413857,43.624508629),(1.366500295,43.624471593),(1.366589676,43.624437985),(1.366621944,43.624428423),(1.36665944,43.6244217),(1.366712025,43.624421474),(1.366776279,43.624425268),(1.366816972,43.624427671),(1.367575396,43.624490576),(1.367589398,43.624491737),(1.367938072,43.624526202),(1.369320077,43.6246513),(1.369545506,43.624699831),(1.369638492,43.624724924),(1.369761429,43.624769658),(1.369863379,43.624824998),(1.369929479,43.6248778),(1.3699905,43.624943075),(1.370027697,43.625024184),(1.370037264,43.625069112),(1.370050106,43.625129418),(1.370057279,43.625212843),(1.370040402,43.625318751),(1.370014952,43.625388918),(1.369980621,43.625456346),(1.369939016,43.625518892),(1.369889125,43.625578879),(1.368410081,43.627000142),(1.366862063,43.628487685),(1.365728904,43.629576575),(1.360897787,43.634218958),(1.359444835,43.635615148),(1.358260906,43.636752826),(1.358230274,43.636820125),(1.358204781,43.636888001),(1.358184425,43.636956453),(1.358169206,43.637025482),(1.358157712,43.637109241),(1.358153644,43.637193833),(1.358162541,43.637273575),(1.35817666,43.637328625),(1.358206765,43.637422815),(1.358236346,43.637493027),(1.35825572,43.637521319),(1.358279073,43.637548375),(1.358334499,43.637602154),(1.35837969,43.637643609),(1.35858529,43.637765079),(1.358774036,43.637867215),(1.359141278,43.638065941),(1.359210082,43.638120254),(1.359266105,43.63817605),(1.359296057,43.638216329),(1.359326237,43.638267923),(1.359350017,43.638317891),(1.359365007,43.638361739),(1.359373997,43.638408205),(1.359378156,43.638465341),(1.359377523,43.638515391),(1.359372408,43.638556992),(1.359357521,43.638611571),(1.359331926,43.638672098),(1.359308254,43.638715871),(1.359280258,43.638758569),(1.357498459,43.640466531),(1.355129812,43.642744778),(1.355045052,43.642826303),(1.354633907,43.643221283),(1.354223227,43.643615817),(1.354148935,43.643687754),(1.35359322,43.644225851),(1.353525284,43.644282047),(1.353436004,43.644343613),(1.35335666,43.644386874),(1.353297936,43.644410859),(1.353206937,43.644444408),(1.353100021,43.644476536),(1.3529893,43.644502317),(1.352357063,43.644663032),(1.349656119,43.645343782),(1.349446844,43.645399597),(1.349334701,43.64542844),(1.349257673,43.645442488),(1.349189535,43.645449513),(1.349087552,43.645453401),(1.349003724,43.645451909),(1.348906821,43.64544332),(1.34882388,43.645429567),(1.348757833,43.645414029),(1.348695224,43.645395278),(1.348632016,43.64537451),(1.348524577,43.645322961),(1.348034353,43.645051118),(1.346698677,43.644315851),(1.346653261,43.644281044),(1.346597238,43.644225248),(1.346557722,43.644173336),(1.346524033,43.644126279),(1.346495199,43.644073395),(1.346473611,43.644022753),(1.346448677,43.6439438),(1.346433041,43.643869594),(1.346429701,43.643795926),(1.346435475,43.64373977),(1.346448488,43.643683932),(1.346468739,43.643628411),(1.346496229,43.643573208),(1.347101543,43.642991783),(1.348421632,43.641723792),(1.349226438,43.640950747)]
        self.centerline = LineString(self.centerlinepoints)
        #print(self.centerline)
        self.n = number_of_points_to_use
        # to update in reward function
        self.shortest_dist = None

    def update_path(self, aircraft_loc, aircraft_heading):
        """
        :param ref_pts: aircraft (longitude,latitude)
        :param ac_heading:
        :return: list[[(long,lat),distance,heading],[.....]]
        """

        n = self.n
        self.aircraft_loc = aircraft_loc
        line = MultiPoint(list(self.points_df.points.values[:]))
        nearest_point = ops.nearest_points(line, Point(aircraft_loc))[0]
        idx = self.points_df[self.points_df.points == nearest_point.coords[:2][0]].index[0]
        shortest_dist = float("{:.4f}".format(get_distance(aircraft_loc, self.points_df.points.loc[idx])))
        target_heading = (float("{:.4f}".format(get_bearing(aircraft_loc, self.points_df.points.loc[idx]))) + 360) % 360
        delta_heading = utils.reduce_reflex_angle_deg(aircraft_heading - target_heading)
        
        
        if abs(delta_heading) > 90:
            idx = idx + 1
            shortest_dist = float("{:.4f}".format(get_distance(aircraft_loc, self.points_df.points.loc[idx])))
            target_heading = (float("{:.4f}".format(get_bearing(aircraft_loc, self.points_df.points.loc[idx]))) + 360) % 360
        
        #print(target_heading)


        output = [[nearest_point.coords[:2][0], shortest_dist, target_heading]]
        output2 = [[self.points_df.points.loc[i], self.points_df.rel_distance.loc[i], self.points_df.rel_bearing.loc[i]]
                   for i in range(idx + 1, idx + n)]
        output += output2

        self.shortest_dist = self.centerline.distance(Point(aircraft_loc))*100000 #Point((1.3578, 43.587434)).distance(Point((1.3577, 43.587288)))*100000 # = 17m

        #print(f'{aircraft_loc[0]},{aircraft_loc[1]}, 10,')#, self.shortest_dist)
        #print(f'{self.shortest_dist}')

        return output

    def update_path2(self, aircraft_loc, aircraft_heading, id_path, nb_point):
        """
        :param ref_pts: aircraft (longitude,latitude)
        :param ac_heading:
        :return: list[[(long,lat),distance,heading],[.....]]
        """
        next_point = False
        output = []
        id_path=min(id_path, len(self.centerlinepoints)-1)
        
        angle_basic = aircraft_heading - get_bearing(aircraft_loc, self.centerlinepoints[id_path])
        angle_basic360 = (abs(angle_basic)+360)%360
        angle_ac_nextpoint = min(angle_basic360, 360-angle_basic360)
        if Point(aircraft_loc).distance(Point(self.centerlinepoints[id_path]))*100000 < 1 or angle_ac_nextpoint > 60:
            #print(id_path, Point(aircraft_loc).distance(Point(self.centerlinepoints[id_path]))*100000, aircraft_loc, self.centerlinepoints[id_path], "next_p", get_bearing(aircraft_loc, self.centerlinepoints[id_path]), "air_head", aircraft_heading, angle_basic, angle_ac_nextpoint)
            # I move to the next centerline point
            next_point = True
            # I keep my next 4 points
            my_points_state = self.centerlinepoints[id_path+1:min(id_path+nb_point+1, len(self.centerlinepoints)-1)]
        else:
            # I keep my next 4 points
            my_points_state = self.centerlinepoints[id_path:min(id_path+nb_point, len(self.centerlinepoints)-1)]
        # I compute heading and distance of my next 4 points
        for p in my_points_state:
            output.append([p, Point(aircraft_loc).distance(Point(p))*100000, get_bearing(aircraft_loc, p)])
        
        # Compute the shortest distance to the centerline
        self.shortest_dist = self.centerline.distance(Point(aircraft_loc))*100000 #Point((1.3578, 43.587434)).distance(Point((1.3577, 43.587288)))*100000 # = 17m

        return output, next_point


#--------------------------------------------------------------
# FUNCTION TO GENERATE points.json file used in taxi_path class
#--------------------------------------------------------------
def get_path_points(ambd_folder_path="../amdb", save_folder=os.getcwd()):
    path_id_numbers = [1977, 1974, 1973, 2001, 2202, 2203, 2453, 2204, 2206, 2000, 1996, 1968, 1969,
                       1971, 2020, 2019, 2018, 1931, 1929, 1925, 1926, 2031, 2042, 2134, 2091, 2099,
                       2103, 2385, 2105, 2136, 1978, 1947, 1949]
    number_of_points_to_use = 5
    shapefile_dir = ambd_folder_path  # 'amdb' folder path
    id_str = ['%d' % x for x in path_id_numbers]
    centerline = 'AM_AsrnEdge.shp'
    n = number_of_points_to_use
    shortest_dist = None

    # get linestrings in the right order
    df = gpd.read_file(shapefile_dir + '/' + centerline)
    df = df[df['idnumber'].isin(id_str)][['idnumber', 'geometry']]
    order = {}
    i = 1
    for id_n in id_str:
        order[id_n] = i
        i += 1
    df['order'] = df.idnumber.map(order)
    df.index = df.order
    df.sort_index(inplace=True)

    # get all path points in order
    all_lines = [df.geometry.values[i] for i in range(df.shape[0])]
    multi_line = MultiLineString(all_lines)
    merged_line = ops.linemerge(multi_line)
    """ TO DO: check if line is reversed"""
    reverse = True
    if reverse:
        merged_line = LineString(ops.linemerge(multi_line).coords[::-1])
    coords = [Point(merged_line.coords[i][:2]) for i in range(len(merged_line.coords))]
    points_df = gpd.GeoDataFrame(coords, columns=['geometry'])
    points_df['points'] = points_df.geometry.apply(lambda x: x.coords[:2][0])
    del df
    path_to_save = save_folder + '/points.csv'
    points_df.points.to_csv(path_to_save)
