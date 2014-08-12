#!/usr/bin/env python

from __future__ import print_function
import os
import rospkg
import subprocess

WIKI_URI="git@github.com:StanfordDrivingTeam/driving.wiki.git"

def do_cmd(cmd, err, **kwargs):
    status = subprocess.call(cmd, **kwargs)
    if status != 0:
        raise Exception(err)

def get_dir():
    """Get the location of the local checkout of the wiki"""
    return os.path.join(rospkg.get_ros_home(), "driving_wiki")

def update():
    """Do a pull --rebase to update to the latest version of the wiki
    This will also do a checkout if necessary
    """
    d = get_dir()

    # if our directory doesn't exist, clone it
    if not os.path.exists(d):
        do_cmd(['git', 'clone', WIKI_URI, d], "unable to checkout driving wiki")
        # we did a clean checkout. we're done
        return
    if not os.path.isdir(d):
        raise Exception("wiki checkout is not a directory")
    do_cmd(['git', 'pull', '--rebase'], "Unable to update wiki", cwd=d)

def push():
    """Do a push to upload local changes to the wiki"""
    do_cmd(['git', 'push'], "Unable to upload changes to wiki", cwd=get_dir())

def commit(message):
    """Commit outstanding changes to the wiki"""
    do_cmd(['git', 'commit', '-m', message], "Unable to commit changes",
            cwd=get_dir())

def add(path):
    do_cmd(['git', 'add', path], "Unable to add file to wiki", cwd=get_dir())
