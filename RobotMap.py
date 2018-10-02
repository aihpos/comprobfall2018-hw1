#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors, pyplot;
import re
from collections import deque, namedtuple
import collections
'''
from shapely import geometry
from matplotlib import pyplot as plt
from shapely.geometry.polygon import Polygon
from descartes import PolygonPatch
from operator import itemgetter
'''

'''
	PLEASE READ THIS THANK U 

	** Things that still need to be done **
		- Fixing the code to connect polygon vertices
			- general idea that I have is Dijkstra's Algorithm to find an optimal path between two vertices, and then flag all of the cells on the path as unsafe
			- if i don't finish this before i sleep just a heads up that you need to take the offset/resolution into account
				- the offset_from_center or offset_from_origin methods will compute that 
		- filling in the polygons
			- sophia's idea seemed good but the boundaries of the world are a bit of a problem with knowing whether or not to fill in cells
				- might want to mark the world borders as 2 instead of 1, and then normalize all nonzero entries to 1 afterwards
		- visibility graph
			- we have all of the vertices for the polygons so it shouldn't be too difficult to implement this
				- need to come up with a way to determine whether two vertices should be connected though which might be difficult
			- someone who's good at data visualization might wanna write a general skeleton for this and then we can put all of the vertices together
		- orientation fix which is just an issue of zero indexing but tbh im pretty sure we can just reflect the array across an axis

	if we get this done before 11am tomorrow u r both welcome to livi apartments and ill order hansel n griddle for us go team

	NOTES:
		- i have a connect_points function that kind of worked for a bit but the issue seems to lie in the resolution/offsets and determining if 
			the two vertices lie in the same cell, which is why i think we should move to dijkstra's instead
		- there still needs to be a function that packages all of the graph initialization/loading into one function but i can write that bc it's just calling a bunch of other functions
		- i am terrible at python and programming pls forgive me


'''

'''
	Usability:
		initialize a RobotMap object with a map file (read the file first and pass the contents to this)
		run the initialization code (all of the load things just read the very bottom for the order)
		RobotMap.grid contains the 2d array afterwords w the free/unfree spaces
		pray to jesus that this somehow works
		scold me for bad python/programming in general

	TODO:
		fill insides of polygons
		fix orientation (might just transpose self.grid a few times)
		display visibility graph (shouldn't be too hard just need to connect points in the polygon)
			For polygons, go througb self.obstacles and grab the Obstacle objects which contain the vertices of each
'''


class Obstacle:

    def __init__(self, vertex_list):
        self.vertex_list = vertex_list;
        self.num_vertices = len(vertex_list);


class RobotMap:

    def __init__(self, map_file_contents):
        self.world_corners = [];  # list of the corners of the world
        self.obstacles = [];  # list of obstacles in the map
        self.start_goal_points = [];  # list of tuples (a,b) where a = (x_1, y_1) and b (x_2, y_2), a is start, b is goal
        self.map_file_contents = map_file_contents;  # contents of the map.txt file
        self.map_size = (0, 0);  # initialize map size (stores len/width to help with adjusting for resolution changes)
        self.resolution = 4;  # resolution is the number of grid squares per 1 meter
        self.center = (0, 0);  # initialize the center of the grid for offsets (this is updated later)
        self.grid = [];  # stores the grid


    def load_world_corners(self, world_corner_listings):
        # this is just a parsing function to store the world corners
        world_corners = world_corner_listings.split(" ");
        for i in range(0, len(world_corners)):
            self.world_corners.append(self.extract_coordinates_from_ordered_pair(world_corners[i]));

    def load_obstacles(self, obstacle_listings):
        # just a parsing function to store obstacle vertices
        obstacle_set = obstacle_listings.split("\n");
        for i in range(0, len(obstacle_set)):
            current_obstacle_vertices = str(obstacle_set[i]).split();
            if (len(current_obstacle_vertices)>5):
                del current_obstacle_vertices[-1]
            current_vertex_list = [];
            for j in range(0, len(current_obstacle_vertices)):
                current_vertices = current_obstacle_vertices[j];
                current_vertex_list.append(self.extract_coordinates_from_ordered_pair(current_vertices));
            current_obstacle = Obstacle(current_vertex_list);
            self.obstacles.append(current_obstacle);

    def load_start_goal_points(self, start_goal_point_listings):
        # also just a parsing function for storing start/goal points
        start_goal_point_list = start_goal_point_listings.split("\n");
        del start_goal_point_list[-1]
        #print(start_goal_point_list)
        for i in range(0, len(start_goal_point_list)):
            start_goal_pair = start_goal_point_list[i].split(" ");
            current_start = start_goal_pair[0];
            current_goal = start_goal_pair[1];
            self.start_goal_points.append((self.extract_coordinates_from_ordered_pair(current_start),
                                           self.extract_coordinates_from_ordered_pair(current_goal)));

    def extract_coordinates_from_ordered_pair(self, ordered_pair):
        # helper function for parsing to return a tuple of floats containing the points in an ordered pair
        coordinates = str(ordered_pair)[1:len(ordered_pair) - 1].split(",");
        if len(coordinates) != 2:
            print "uh oh spaghettio in extract_coordinates_from_ordered_pair";
            return (":^(", ":^(");
        else:
            return (float(coordinates[0]), float(coordinates[1]));

    def generate_grid(self):
        # initializes the 2d array for the grid representation
        length = self.map_size[0];
        width = self.map_size[1];
        # resolution is a bit weird since I interpreted a resolution of 2 to have 4 blocks inside of a 1 meter square
        # generally, a resolution of x means that there are x^2 points per 1 meter block
        self.grid = [[0] * (int(self.resolution * width) + 2) for _ in range(int(length * self.resolution) + 2)];
        adjusted_center = self.offset_from_center(0, 0);
        #print adjusted_center;  # this is just debugging to test the offset/resolution stuff we can remove this later
        self.load_walls();  # puts all of the world boundaries as unsafe positions

    def load_obstacles_into_grid(self):
        # !!! right now, this only loads obstacle vertices into the grid, we'll need to add the edge generation in
        print "# of obstacles:" + str(len(self.obstacles));
        for i in range(0, len(self.obstacles)):
            current_obstacle = self.obstacles[i];
            print(current_obstacle.vertex_list)
            for j in range(0, current_obstacle.num_vertices):
                current_vertex = current_obstacle.vertex_list[j];
                x = current_vertex[0];
                y = current_vertex[1];
                x_offset = self.offset_from_center(x, y)[0];  # adjusting points for offsets
                y_offset = self.offset_from_center(x, y)[1];
                #print str(x_offset) + "," + str(y_offset);
                self.grid[int(x_offset)][int(y_offset)] = (i + 2);
        for i in range(0, len(self.obstacles)):
            curr_obstacle = self.obstacles[i];
            for j in range(0, len(curr_obstacle.vertex_list) - 1):
                v1 = curr_obstacle.vertex_list[j];
                v2 = curr_obstacle.vertex_list[j + 1];
                self.connect_points(self.offset_from_center(v1[0], v1[1]),self.offset_from_center(v2[0], v2[1]));
            first_v = curr_obstacle.vertex_list[0];
            last_v = curr_obstacle.vertex_list[len(curr_obstacle.vertex_list) - 1];
            self.connect_points(self.offset_from_center(last_v[0], last_v[1]), self.offset_from_center(first_v[0], first_v[1]));

      
    def load_walls(self):
        # just marks off the boundaries of the array as unsafe
        for i in range(0, len(self.grid)):
            for j in range(0, len(self.grid[0])):
                if i == 0 or j == 0 or i == len(self.grid) - 1 or j == len(self.grid[0]) - 1:
                    self.grid[i][j] = 1;

    def connect_points(self, v1, v2):
        # redundant after dijkstra's implementation
        print "connects two points of a polygon together, goes clockwise by vertices, use recursively";
        if v1 == v2:
            return;
        print "connecting" + str(v1) + " with " + str(v2);
        x1 = v1[0];
        y1 = v1[1];
        x2 = v2[0];
        y2 = v2[1];
        x_direction = 0;
        y_direction = 0;
        if abs(x1 - x2) <= 1 and abs(y1 - y2) <= 1:
            return;
        if x1 < x2:
            x_direction = 1;
        if x1 > x2:
            x_direction = -1;
        if y1 < y2:
            y_direction = 1;  # adjust for the orientation change between 2d array and grid
        if y1 > y2:
            y_direction = -1;
        self.grid[int(x1 + x_direction)][int(y1 + y_direction)] = 1;
        print "bounds: " + str(int(len(self.grid))) + " x " + str(int(len(self.grid[0])));
        print "next step: " + str(x1 + x_direction) + "," + str(y1 + y_direction);
        self.connect_points((x1 + x_direction, y1 + y_direction), v2);

    def load_polygon_outline(self, vertices):
        # we'll need to toss a call to dijkstra's in here to connect the outline of the polygon
        # for this, we need to make sure that we connect all vertices
        # we can just use a for loop bc we need to connect the last vertex and the first vertex to close off the polygon
        print "loads outside path";
        curr_pos_1 = vertices[0];
        for i in range(1, len(vertices)):
            curr_pos_2 = vertices[i];
            self.connect_points(curr_pos_1, curr_pos_2);
            curr_pos_1 = curr_pos_2;

        pos_1 = vertices[1];
        pos_2 = vertices[2];

        self.connect_points(pos_1, pos_2); #defunct, throws a recursion error

    def load_obstacle_outline(self, vertices):
       for i in range(0, len(vertices) - 1):
        vertex_1 = self.offset_from_center(vertices[i][0], vertices[i][1]);
        vertex_2 = self.offset_from_center(vertices[i + 1][0], vertices[i + 1][1]);
        x_1 = vertex_1[0];
        y_1 = vertex_1[1];
        x_2 = vertex_2[0];
        y_2 = vertex_2[1];
        path = self.get_path_between_cells(self.grid, (x_1, y_1), (x_2, y_2));
        print "path: " + str(path);
        for i in range(0, len(path)):
            current_cell = path[i];
            x = current_cell[0];
            y = current_cell[1];
            self.grid[x][y] = 1;

    def get_path_between_cells(self, map, begin, end):
        print "finding path between " + str(begin) + " and " + str(end);
        queue = collections.deque([[begin]])
        visited = set([begin])
        width = len(self.grid);
        length = len(self.grid[0]);
        while queue:
            path = queue.popleft()
            x, y = path[-1]
            if map[int(y)][int(x)] == end:
                return path
            for x_next, y_next in ((x+1,y), (x-1,y), (x,y+1), (x,y-1)):
                if 0 <= x_next < width and 0 <= y_next < length and map[int(y_next)][int(x_next)] != 1 and (x_next, y_next) not in visited:
                    queue.append(path + [(x_next, y_next)])
                    visited.add((x_next, y_next))

    def fill_polygons(self):
        # fills the inside of the polygons
        print "todo";
        filling = False;
        for i in range(1, len(self.grid) - 1):
            for j in range(1, len(self.grid[0]) - 1):
                if self.grid[i][j] == 1;


    def should_vertices_connect(self, v1, v2):
        line_conn = [];
        for i in range(0, len(self.obstacles)):
            for j in range(0, len(self.obstacles)):
                ob1 = self.obstacles[i];
                ob2 = self.obstacles[j];
                for x in range(0, len(ob1.vertex_list)):
                    for y in range(0, len(ob2.vertex_list)):
                        v1 = ob1.vertex_list[x];
                        v2 = ob2.vertex_list[y];
                        if self.is_point_on_line(v1, v2):
                            line_conn.append(v1, v2);


    def is_point_on_line(self, v1, v2, p):
        #y1 - y2 = m(x1 - x2)
        x1 = v1[0];
        y1 = v1[1];
        x2 = v2[0];
        y2 = v2[1];
        m = (y1 - y2)/(x1 - x2);
        px = p[0];
        py = p[1];
        return (py - y2)/(px - x2) == m;


    def show_visibility_graph(self):
        # visualizes the visibility graph

        '''fig = plt.figure()
        for i in range(0, len(self.obstacles)):
            current_obstacle = self.obstacles[i];
            vertices_of_one_polygon = []
            for j in range (0, current_obstacle.num_vertices):
                current_vertex = current_obstacle.vertex_list[j];
                vertices_of_one_polygon.append(current_vertex)
            
            #print(vertices_of_one_polygon)
            new_polygon = Polygon(vertices_of_one_polygon)
            ax = fig.add_subplot(111)
            new_patch = PolygonPatch(new_polygon)
            ax.add_patch(new_patch)
        print(self.world_corners)
        xvals = []
        yvals = []
        for k in range (0, len(self.world_corners)):
            new_tuple = self.world_corners[k]
            xvals.append(new_tuple[0])
            yvals.append(new_tuple[1])
        xrange = [min(xvals), max(xvals)]
        yrange = [min(yvals), max(yvals)]
        print(yrange)


        ax.set_xlim(*xrange)
        ax.set_ylim(*yrange)
        ax.set_aspect(1)

        plt.show()'''



    def offset_from_center(self, x, y):
        # offsets a point from the center of the grid considering resolution
        width = self.map_size[0];
        length = self.map_size[1];
        return ((float(x) + width / 2) * self.resolution, (float(y) + length / 2) * self.resolution);

    def compute_map_size(self):
        # Returns a tuple containing the length and width of the map size in meters
        # this is needed because we need to consider the size of each block since we need to take the turtlebot's size into account
        start_goal_flattened = [];
        for i in range(0, len(self.start_goal_points)):
            curr_pair = self.start_goal_points[i];
            start_goal_flattened.append(curr_pair[0]);
            start_goal_flattened.append(curr_pair[1]);

        all_pos = self.world_corners + start_goal_flattened;
        first_coordinate = all_pos[0];
        least_x = first_coordinate[0];
        least_y = first_coordinate[1];
        greatest_x = first_coordinate[0];
        greatest_y = first_coordinate[1];
        for i in range(1, len(all_pos)):
            current_coordinate = all_pos[i];
            current_x = current_coordinate[0];
            current_y = current_coordinate[1];
            if current_x < least_x:
                least_x = current_x;
            else:
                if current_x > greatest_x:
                    greatest_x = current_x;
            if current_y < least_y:
                least_y = current_y;
            else:
                if current_y > greatest_y:
                    greatest_y = current_y;
        width = greatest_x - least_x;
        length = greatest_y - least_y;
        self.map_size = (float(width), float(length));
        adjusted_center_x = (width / 2);
        adjusted_center_y = (length / 2);
        self.center = (adjusted_center_x * self.resolution, adjusted_center_y * self.resolution);

    def init_map(self):
        # this is the function that actually takes the map file text and sends it to the other functions for parsing
        pattern = re.compile(r'\s+---\s+');  # regex to get rid of irregularities with spacing on the delimiters
        map_file_contents_normalized = re.sub(pattern, '*', self.map_file_contents);
        map_file_split = map_file_contents_normalized.split("*");
        print(len(map_file_split))
        if len(map_file_split) != 3:
            print "uh oh spaghettiooooo";
        else:
            self.load_world_corners(map_file_split[0]);
            self.load_start_goal_points(map_file_split[2]);
            self.load_obstacles(map_file_split[1]);
            self.compute_map_size();

    def print_grid(self):
        # debugging purposes, prints grid
        for i in range(0, int(len(self.grid))):
            curr_row = "";
            for j in range(0, int(len(self.grid[0]))):
                curr_row += str(self.grid[i][j]) + " ";
            print curr_row;
        print "Adjusted Center for Grid: " + str(self.center);

    def generate_report(self):
        # debugging purposes, generates a report on the map with size, obstacles, start/end points, and world corners
        if len(self.world_corners) == 0:
            print "Map has not been initialized yet.";
            return;
        print "Map Dimensions:";
        print "\tWidth: " + str(self.map_size[0]) + "m\tLength: " + str(self.map_size[1]) + "m";
        print "\tArea: " + str(self.map_size[0] * self.map_size[1]) + "m";
        for i in range(0, len(self.world_corners)):
            print "Corner " + str(i + 1) + " " + str(self.world_corners[i]);
        print "---";
        for i in range(0, len(self.obstacles)):
            print "Obstacle " + str(i + 1);
            current_obstacle = self.obstacles[i];
            for j in range(0, len(current_obstacle.vertex_list)):
                print "\tVertex " + str(i + 1) + " " + str(current_obstacle.vertex_list[j]);
        print "---";
        for i in range(0, len(self.start_goal_points)):
            print "Set " + str(i + 1);
            print "\tStart: " + str(self.start_goal_points[i][0]) + "\tGoal: " + str(self.start_goal_points[i][1]);
        print "---";
        self.print_grid();

    def visualize_graph(self):
        '''# visualization purposes, needs to be updates as it throws a conversion error
        #plt.figure();
        im = plt.imshow(self.grid,
                        interpolation='none', vmin=0, vmax=1, aspect='equal');
        fig, ax = plt.subplots()

        # Don't allow the axis to be on top of your data
        ax.set_axisbelow(True)

        # Customize the grid
        ax.grid(linestyle='-', linewidth='0.5', color='red')
        plt.show();
        for i in range(0, len(self.obstacles)):
            current_obstacle = self.obstacles[i];
            for j in range(0, current_obstacle.num_vertices):
                current_vertex = current_obstacle.vertex_list[j];
                x = current_vertex[0];
                y = current_vertex[1];

                x_offset = self.offset_from_center(x, y)[0];  # adjusting points for offsets
                y_offset = self.offset_from_center(x, y)[1];'''


        
        fig = plt.figure()
        for i in range(0, len(self.obstacles)):
            current_obstacle = self.obstacles[i];
            vertices_of_one_polygon = []
            for j in range (0, current_obstacle.num_vertices):
                current_vertex = current_obstacle.vertex_list[j];
                vertices_of_one_polygon.append(current_vertex)
            
            #print(vertices_of_one_polygon)
            new_polygon = Polygon(vertices_of_one_polygon)
            ax = fig.add_subplot(111)
            new_patch = PolygonPatch(new_polygon)
            ax.add_patch(new_patch)
        print(self.world_corners)
        xvals = []
        yvals = []
        for k in range (0, len(self.world_corners)):
            new_tuple = self.world_corners[k]
            xvals.append(new_tuple[0])
            yvals.append(new_tuple[1])
        xrange = [min(xvals), max(xvals)]
        yrange = [min(yvals), max(yvals)]
        print(yrange)


        ax.set_xlim(*xrange)
        ax.set_ylim(*yrange)
        ax.set_aspect(1)

        plt.show()



                




# Usage: right now it'll throw an error with connecting edges because of recursion depth
# we'll have to write a function to shove all of this together so that we can just call map.initialize(); or something like that
file_name = "map1.txt";
map_file = open(file_name, "r");
file_contents = str(map_file.read())
map = RobotMap(file_contents)
map.init_map()
map.compute_map_size()
map.generate_grid()
map.load_obstacles_into_grid()
#map.load_obstacle_outline(map.obstacles[0].vertex_list);
map.print_grid()

#print map.grid;