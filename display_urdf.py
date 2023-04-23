#!/usr/bin/env python

import sys
import argparse

from urdf_parser_py.urdf import URDF

parser = argparse.ArgumentParser(usage='Load an URDF file')
parser.add_argument('file', type=argparse.FileType('r'), nargs='?',
                    default=None, help='File to load. Use - for stdin')
parser.add_argument('-o', '--output', type=argparse.FileType('w'),
                    default=None, help='Dump file to XML')
args = parser.parse_args()

robot = URDF.from_xml_string(args.file.read())

# logfile = open("urdf_log.txt", "w")
# print(robot, file=logfile)
# logfile.close()

# xml = robot.to_xml()

# def printxml(node):
#     for child in node:
#         print(child.tag, child.attrib)
#         printxml(child)

# printxml(xml)

# logdataf = open("robotjointlog.txt", "w")

# for i, data in enumerate(robot.joints):
#     print(f"info for {data.name}", file=logdataf)
#     print("--------------------------------------", file=logdataf)
#     print(data, file=logdataf)
#     print("", file=logdataf)
    
# logdataf.close()

for i, data in enumerate(robot.parent_map):
    print(f"info for {data}")
    print("--------------------------------------")
    # # for visual in robot.link_map[data].visuals:
    #     # print(visual.geometry.filename)
    # if robot.link_map[data].visuals:
    #     print(robot.link_map[data].visuals[0])
    print(robot.parent_map[data])
    print()

if args.output is not None:
    args.output.write(robot.to_xml_string())
