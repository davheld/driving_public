#  Stanford Driving Software
#  Copyright (c) 2011 Stanford University
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with
#  or without modification, are permitted provided that the
#  following conditions are met:
#
# * Redistributions of source code must retain the above
#   copyright notice, this list of conditions and the
#   following disclaimer.
# * Redistributions in binary form must reproduce the above
#   copyright notice, this list of conditions and the
#   following disclaimer in the documentation and/or other
#   materials provided with the distribution.
# * The names of the contributors may not be used to endorse
#   or promote products derived from this software
#   without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
#  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
#  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
#  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
#  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
#  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
#  DAMAGE.

# -*- coding: utf-8 -*-

import track_file_io.msg
import geometry_msgs.msg
import rosbag
import rospy


class TrackFileWriter:
    '''A class to write tracks into a trk file, which is really a bag file.

    The velodyne_pose must be set prior to calling write.
    '''
    def __init__(self, filename):
        self.bag = rosbag.Bag(filename, 'w')
        self.velodyne_pose_saved = False
        self.velodyne_pose = None

    def write(self, track):
        '''Writes a track into the trk file.'''
        if not self.velodyne_pose_saved:
            self.bag.write('/velpose', self.velodyne_pose, t=track.stamp)
            self.velodyne_pose_saved = True
        self.bag.write('/tracks', track, t=track.stamp)


class TrackFileReader:
    '''A class to read tracks from a trk file, which is really a bag file.

    It's implemented as a generator.
    '''
    def __init__(self, filename):
        self.bag = rosbag.Bag(filename, 'r')
        self.velodyne_pose = None
        for topic, msg, t in self.bag.read_messages(topics=['/velpose']):
            self.velodyne_pose = msg
            break
        self.reader = self.bag.read_messages(topics=['/tracks'])

    def __iter__(self):
        return self

    # Python 3 compatibility
    def __next__(self):
        return self.next()

    def next(self):
        topic, msg, t = self.reader.next()
        return msg


def save(filename, tracks):
    '''Saves the tracks as a trk file'''
    writer = TrackFileWriter(filename)
    writer.velodyne_pose = tracks.velodyne_pose
    for t in tracks.tracks:
        writer.write(t)


def load(filename):
    '''Loads a trk file. Returns the tracks.'''
    reader = TrackFileReader(filename)
    tracks = track_file_io.msg.Tracks()
    tracks.velodyne_pose = reader.velodyne_pose
    for t in reader:
        tracks.tracks.append(t)
    return tracks
