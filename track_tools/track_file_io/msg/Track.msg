# the class label, e.g. car, bicycle, pedestrian, etc.
string label

# the sequence of frames that make the track
Frame[] frames

# the track id. Corresponds to the track id given by the tracker. Only useful
# to associate with the result of perception, when constructing the data set.
# meaningless otherwise
uint64 id
