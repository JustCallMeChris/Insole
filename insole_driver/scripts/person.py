#!/usr/bin/env python

import time

class Person(object):

    def __init__(self, name=None, position=None, alignment=None):
        self.name = name
        self.position = position
        self.alignment = alignment
        self.allPositions = list()

    def get_name(self):
        return self.name

    def get_postion(self):
        return self.position

    def get_alignment(self):
        return self.alignment

    def set_name(self, name):
        self.name = name

    def set_postion(self, position):
        self.position = position
        self.allPositions.append(position)

    def set_alignment(self, alignment):
        self.alignment = alignment





