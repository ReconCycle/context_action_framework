import os
from matplotlib.pyplot import tight_layout
import numpy as np
import time
import cv2
from rich import print
import io
from enum import IntEnum
from typing import List, Optional, Tuple, Any, overload

from itertools import permutations
from shapely.geometry import Polygon

# from graph_tool.all import *

import matplotlib.pylab as plt
import networkx as nx

from context_action_framework.types import Detection, Action, Label


def compute_is_inside(poly1, poly2):
    """ is box1 inside box2? """
    if poly1.intersects(poly2): 
        # we use convex_hull because the inside shape may occlude the poly that is on top of it.
        intersect = poly1.convex_hull.intersection(poly2.convex_hull).area

        # stop division by 0
        if poly1.area < 0.001:
            return False
        
        ratio_p1_inside_p2 = intersect / poly1.area

        # if more than 90% of p1 is inside p2, then it is inside
        return ratio_p1_inside_p2 > 0.9

    else:
        return 0
    


def compute_iou(poly1, poly2):
    p1 = poly1
    p2 = poly2
    if isinstance(poly1, np.ndarray):
        p1 = Polygon(poly1)
    
    if isinstance(poly2, np.ndarray):
        p2 = Polygon(poly2)
    
    intersect = p1.intersection(p2).area

    # stop division by 0
    if p1.area < 0.000001 or p2.area < 0.000001:
        print("p1.area", p1.area, "p2.area", p2.area)
        return 0.0

    union = p1.union(p2).area
    iou = intersect / union
    return iou

def compute_is_next_to(poly1, poly2):
    dist = poly1.distance(poly2)
    # print("dist", dist)
    if dist == 0 and (compute_is_inside(poly1, poly2) or compute_is_inside(poly2, poly1)):
        return False

    if dist < 10: # pixels
        return True

def exists_detection(detections, det_or_label, label_face=None):
    if isinstance(det_or_label, Label):
        label = det_or_label
        
        found_dets = []
        for detection in detections:
            if label_face is None:
                if detection.label == label:
                    found_dets.append(detection)
            else:
                if detection.label == label and detection.label_face == label_face:
                    found_dets.append(detection)

        return found_dets
    else:
        # det_or_label is a detection
        det = det_or_label
        for detection in detections:
            if detection == det:
                return True

    return False



# todo: we can use enums for the relationships instead of strings
class Relation(IntEnum):
    next_to = 0
    inside = 1
    


class GraphRelations:
    def __init__(self, detections: List[Detection], use_valid_detections=False):
        self.detections = detections
        if use_valid_detections:
            self.valid_detections = [detection for detection in self.detections if detection.valid]
        else:
            self.valid_detections = self.detections

        self.dict_valid_dets = {}
        for valid_det in self.valid_detections:
            self.dict_valid_dets[valid_det.id] = valid_det
        
        self.tracking_ids = {}
        
        self.G = None
        
        self.edge_labels_dict = None
        self.inside_edges = None
        self.next_to_edges = None
        self.relations_groups = []
        
        self.list_wc_components = None
        self.list_wc_components_t = None
        self.groups = []
        
        self.get_relations()
        self.generate_network_x()
        self.make_groups()
        
    def get_relations(self):
        
        inside_edges = []
        next_to_edges = []
        for detection1, detection2 in permutations(self.valid_detections, 2):
            
            # todo: we can use the polygon for this
            inside = compute_is_inside(detection1.polygon_px, detection2.polygon_px) # not associative, a inside b
            next_to = compute_is_next_to(detection1.polygon_px, detection2.polygon_px) # associative

            if inside:
                # print(detection1.label, "inside", detection2.label)
                inside_edges.append((detection1.id, detection2.id))
            
            if next_to:
                # print(detection1.label, "next to", detection2.label)
                next_to_edges.append((detection1.id, detection2.id))

        edge_labels_dict = {}
        for inside_edge in inside_edges:
            edge_labels_dict[inside_edge] = "in" # inside
            
        for next_to_edge in next_to_edges:
            edge_labels_dict[next_to_edge] = "next to" #nextto
                
        self.edge_labels_dict = edge_labels_dict
        self.inside_edges = inside_edges
        self.next_to_edges = next_to_edges


    def generate_network_x(self):
        # draw graph
        self.G = nx.DiGraph()
        self.G.add_nodes_from([detection.id for detection in self.valid_detections])
        self.G.add_edges_from(self.inside_edges + self.next_to_edges)
        
        # ! do we use these tracking ids?
        self.tracking_ids = {}
        for detection in self.valid_detections:
            self.tracking_ids[detection.id] = detection.tracking_id
        

    
    def draw_network_x(self):
        
        plt.rcParams["figure.autolayout"] = True
        fig = plt.figure(1, figsize=(12, 9)) # , tight_layout={"pad": 20 }
        fig.clear(True)
        
        node_colors = []
        for detection in self.detections:
            if detection.label == Label.battery:
                node_colors.append("blue")
            else:
                node_colors.append("pink")
        # try:
        is_planar, P = nx.check_planarity(self.G)
        
        if is_planar:
            pos = nx.planar_layout(self.G) # nice and stays mostly the same per frame
        else:
            pos = nx.spring_layout(self.G, k=2) # nice, but is different for every frame
            # pos = nx.spectral_layout(self.G) # all nodes on top of each other for each connected component
        
        # print("edge_labels_dict", edge_labels_dict)
        # print("node labels", {id: self.valid_detections[id].label.name for id in G.nodes()})
        
        # add a margin
        ax1 = plt.subplot(111)
        ax1.margins(0.05)
        
        nx.draw(
            self.G, pos, ax=ax1, edge_color='black', width=1, linewidths=1,
            node_size=500, node_color=node_colors,
            labels={id: self.dict_valid_dets[id].label.name for id in self.G.nodes()},
            font_size=32
        )

        nx.draw_networkx_edge_labels(self.G, pos, self.edge_labels_dict,
                                    label_pos=0.5,
                                    font_size=32
                                    )
        
        io_buf = io.BytesIO()
        fig.savefig(io_buf, format='raw')
        io_buf.seek(0)
        img_arr = np.reshape(np.frombuffer(io_buf.getvalue(), dtype=np.uint8),
                            newshape=(int(fig.bbox.bounds[3]), int(fig.bbox.bounds[2]), -1))
        io_buf.flush()
        io_buf.close()
        
        return img_arr
        # except AttributeError as e:
        #     print("error with drawing graph!")
            
        # return None
    
    
    def make_groups(self):
        # all connected components in graph
        self.list_wc_components = sorted(nx.weakly_connected_components(self.G), key=len, reverse=True)
        # print("list_wc_components", self.list_wc_components)
        
        # set the group id on each detection
        for group_id, wc_components in enumerate(self.list_wc_components):
            for det_id in wc_components:
                det = self.get_detection_by_id(det_id)
                det.group_id = group_id #? do we ever use this group_id?
        
        # create list_wc_components but with tracking_ids
        self.list_wc_components_t = []
        for wc_component in self.list_wc_components:
            wc_component_t = []
            for id in wc_component:
                wc_component_t.append(self.tracking_ids[id])
            self.list_wc_components_t.append(wc_component_t)
        
        # create the groups as a list of lists
        self.groups = []
        for wc_component in self.list_wc_components:
            group = []
            for id in wc_component:
                group.append(self.get_detection_by_id(id))
            
            self.groups.append(group)
        
        
        # relations_groups = edge_labels_dict grouped by weakly connected components
        self.relations_groups = []
        for wc_component in self.list_wc_components:
            group = {}
            for id in wc_component: 
                # add all dict key:value pairs that contain this id
                for key, val in self.edge_labels_dict.items():
                    if id in key:
                        group[key] = val
                         

            self.relations_groups.append(group)
        
            
    def get_detection_by_id(self, id):
        return self.dict_valid_dets[id]
        
    
    def is_inside(self, det_or_label1, det_or_label2):
        if isinstance(det_or_label1, Label):
            label1, label2 = det_or_label1, det_or_label2
            for key, val in self.edge_labels_dict.items():
                a, b = key
                detection_pair = (self.dict_valid_dets[a].label.value, self.dict_valid_dets[b].label.value)
                if detection_pair == (label1.value, label2.value) and val == "in":
                    return True, self.dict_valid_dets[a], self.dict_valid_dets[b]

            return False, None, None
        else:
            # det_or_label1 is type Detection
            det1, det2 = det_or_label1, det_or_label2
            for key, val in self.edge_labels_dict.items():
                a, b = key
                detection_pair = (self.dict_valid_dets[a], self.dict_valid_dets[b])
                if detection_pair == (det1, det2) and val == "in":
                    return True

            return False

        
    def is_next_to(self, det_or_label1, det_or_label2):
        if isinstance(det_or_label1, Label):
            label1 = det_or_label1
            label2 = det_or_label2
            for key, val in self.edge_labels_dict.items():
                a, b = key
                detection_pair = (self.dict_valid_dets[a].label.value, self.dict_valid_dets[b].label.value)
                if detection_pair == (label1.value, label2.value) and val == "next to":
                    return True, self.dict_valid_dets[a], self.dict_valid_dets[b]

            return False, None, None
        else:
            det1 = det_or_label1
            det2 = det_or_label2
            for key, val in self.edge_labels_dict.items():
                a, b = key
                detection_pair = (self.dict_valid_dets[a], self.dict_valid_dets[b])
                if detection_pair == (det1, det2) and val == "next to":
                    return True

            return False
    
    def get_all_next_to(self, det: Detection):
        next_to_dets_ids = [det_pair[0] for det_pair in self.next_to_edges if det_pair[1] == det.id]
        next_to_dets = [detection for detection in self.valid_detections if detection.id in next_to_dets_ids]
        
        return next_to_dets
    
    def exists(self, det_or_label, label_face=None):
        return exists_detection(self.valid_detections, det_or_label, label_face)

    def get_all_inside(self, det: Detection):
        inside_dets_ids = [det_pair[0] for det_pair in self.inside_edges if det_pair[1] == det.id]
        inside_dets = [detection for detection in self.valid_detections if detection.id in inside_dets_ids]
        
        return inside_dets

    
    @staticmethod
    def get(detections, label):
        found_dets = []
        for detection in detections:
            if detection.label == label:
                found_dets.append(detection)

        return found_dets

    @staticmethod
    def get_first(detections, label, label_face=None):
        for detection in detections:
            if label_face is None:
                if detection.label == label:
                    return detection
            else:
                if detection.label == label and detection.label_face == label_face:
                    return detection

        return None
    
    
    def to_text(self):
        """
        converts graph_relations to natural language
        """
        
        text_groups = ""
        text_single_items = ""
        single_items_list = []
        
        for idx, relations_group in enumerate(self.relations_groups):
            
            if relations_group: # check if there are relations

                # print("relations_group", relations_group)
                # print("self.groups[idx]", self.groups[idx])
                # print("list_wc_components[idx]", self.list_wc_components[idx])

                # get main item from: hca_back, hca_front, firealarm_front, firealarm_back
                # for detection in self.groups[idx]:
                #     if detection.label in [Label.hca_back, Label.hca_front, Label.firealarm_front, Label.firealarm_back]:
                #         print("main device:", detection.label.name)

                text_groups += f"Device {idx + 1}: "

                text_relations = []
                for relation_ids, relation_type in relations_group.items():
                    relation_id_1, relation_id_2 = relation_ids
                    relation1 = self.get_detection_by_id(relation_id_1)
                    relation2 = self.get_detection_by_id(relation_id_2)

                    # check that "x next to x" isn't added.
                    if relation1.label.name != relation2.label.name:

                        # only print one relation, "x next to y", or "y next to x"
                        if relation_type == "in" or (relation_type == "next to" and relation1.label.name < relation2.label.name):
                        
                            text_relations.append(f"{relation1.label.name} {relation_type} {relation2.label.name}")

                # print("text_relations asdf", text_relations)
                if len(text_relations) > 0:
                    text_groups += ", ".join(text_relations) + ". "
                        
            else:
                # group with 1 element, so no relations
                group = self.groups[idx]
                if len(group) == 1:
                    detection = group[0]
                    # text_groups += f"{detection.label.name}"
                    single_items_list.append(detection.label.name)
        
        # list also all the items that don't have relations
        if len(single_items_list) == 1:
            text_single_items = "Loose component: " + ', '.join(single_items_list)
        elif len(single_items_list) > 1:
            text_single_items = "Loose components: " + ', '.join(single_items_list)

        text_groups += text_single_items

        
        return text_groups
    
