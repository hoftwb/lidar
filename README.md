# lidar

Current progress:
  - Code reads in depth images or pointclouds (depth images preferred, because they have a consistent pixel count), and outputs a graph of each individual pixel's depth over time (frames) along with a set of statistics (25th/50th/75th percentile, mean, range, etc)
  - A single 2-million pixel image is processed in <~60ms, all compute times are displayed in debug. The code uses numpy builtins whenever possible to increase efficiency.
  - Has been tested on a rendering of water physics simulated in Blender, various cases for debugging, and depth images pulled from a real-world lidar scan of Oxford's campus
  - Vaguely supports live (realtime) adding of new frames and discarding of old ones - this feature was kind of just for fun, but it's ~50% faster than reconstructing all of the frames to add a new one.

I wasn't sure what the best intake method for the pixel-array based lidars' info streams would be so I designed the new code around depth images because I figured that would be simpler to test while still being relatively similar to actual live data, with the way it's set up it's trivial to add new processing to take in a different type of data.

Note: due to the way I set things up you have to switch from Main to Master to see the actual contents. "code_flood" is the most recent and only really relevany script, the others are either old systems or testing grounds.
