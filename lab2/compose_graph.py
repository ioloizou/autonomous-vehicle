import carla
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt

# Initialize CARLA client
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

class GlobalRoutePlanner(object):
    """
    This class provides a very high level route plan.
    """

    def __init__(self, wmap, sampling_resolution):
        self.sampling_resolution = sampling_resolution
        self.wmap = wmap
        self.topology = None
        self.graph = None
        self.id_map = None
        self.road_id_to_edge = None


        # Build the graph
        self.build_topology()
        self.build_graph()

    def build_topology(self):
        """
        This function retrieves topology from the server as a list of
        road segments as pairs of waypoint objects, and processes the
        topology into a list of dictionary objects with the following attributes

        - entry (carla.Waypoint): waypoint of entry point of road segment
        - entryxyz (tuple): (x,y,z) of entry point of road segment
        - exit (carla.Waypoint): waypoint of exit point of road segment
        - exitxyz (tuple): (x,y,z) of exit point of road segment
        - path (list of carla.Waypoint):  list of waypoints between entry to exit, separated by the resolution
        """
        self.topology = []
        # Retrieving waypoints to construct a detailed topology
        for segment in self.wmap.get_topology():
            wp1, wp2 = segment[0], segment[1]
            l1, l2 = wp1.transform.location, wp2.transform.location
            # Rounding off to avoid floating point imprecision
            x1, y1, z1, x2, y2, z2 = np.round([l1.x, l1.y, l1.z, l2.x, l2.y, l2.z], 0)
            wp1.transform.location, wp2.transform.location = l1, l2
            seg_dict = dict()
            seg_dict['entry'], seg_dict['exit'] = wp1, wp2
            seg_dict['entryxyz'], seg_dict['exitxyz'] = (x1, y1, z1), (x2, y2, z2)
            seg_dict['path'] = []
            endloc = wp2.transform.location
            if wp1.transform.location.distance(endloc) > self.sampling_resolution:
                w = wp1.next(self.sampling_resolution)[0]
                while w.transform.location.distance(endloc) > self.sampling_resolution:
                    seg_dict['path'].append(w)
                    w = w.next(self.sampling_resolution)[0]
            else:
                seg_dict['path'].append(wp1.next(self.sampling_resolution)[0])
            self.topology.append(seg_dict)

    def build_graph(self):
        """
        This function builds a networkx graph representation of topology, creating several class attributes:
        - graph (networkx.DiGraph): networkx graph representing the world map, with:
            Node properties:
                vertex: (x,y,z) position in world map
            Edge properties:
                entry_vector: unit vector along tangent at entry point
                exit_vector: unit vector along tangent at exit point
                net_vector: unit vector of the chord from entry to exit
                intersection: boolean indicating if the edge belongs to an  intersection
        - id_map (dictionary): mapping from (x,y,z) to node id
        - road_id_to_edge (dictionary): map from road id to edge in the graph
        """

        self.graph = nx.DiGraph()
        self.id_map = dict()  # Map with structure {(x,y,z): id, ... }
        self.road_id_to_edge = dict()  # Map with structure {road_id: {lane_id: edge, ... }, ... }

        for segment in self.topology:
            entry_xyz, exit_xyz = segment['entryxyz'], segment['exitxyz']
            path = segment['path']
            entry_wp, exit_wp = segment['entry'], segment['exit']
            intersection = entry_wp.is_junction
            road_id, section_id, lane_id = entry_wp.road_id, entry_wp.section_id, entry_wp.lane_id

            for vertex in entry_xyz, exit_xyz:
                # Adding unique nodes and populating id_map
                if vertex not in self.id_map:
                    new_id = len(self.id_map)
                    self.id_map[vertex] = new_id
                    self.graph.add_node(new_id, vertex=vertex)
            n1 = self.id_map[entry_xyz]
            n2 = self.id_map[exit_xyz]
            if road_id not in self.road_id_to_edge:
                self.road_id_to_edge[road_id] = dict()
            if section_id not in self.road_id_to_edge[road_id]:
                self.road_id_to_edge[road_id][section_id] = dict()
            self.road_id_to_edge[road_id][section_id][lane_id] = (n1, n2)
            
            entry_carla_vector = entry_wp.transform.rotation.get_forward_vector()
            exit_carla_vector = exit_wp.transform.rotation.get_forward_vector()

            # Adding edge with attributes
            self.graph.add_edge(
                n1, n2,
                length=len(path) + 1)

gp = GlobalRoutePlanner(client.get_world().get_map(), 1.0)
print(gp.graph.nodes.data())
print(gp.graph.edges.data())
nx.draw(gp.graph,pos=nx.spring_layout(gp.graph))
plt.show()

import pickle

# save graph object to file
pickle.dump(gp.graph.nodes.data(), open('Town01_nodes.pickle', 'wb'))
pickle.dump(gp.graph.edges.data(), open('Town01_edges.pickle', 'wb'))
