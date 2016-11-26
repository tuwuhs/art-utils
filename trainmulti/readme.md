# trainmulti

(c) 2016 Tuwuh Sarwoprasojo

Train multimarker .dat file from a scene. 

All markers must be visible on one image frame. It uses the transforms calculated by ARToolKit for each of detected markers,
which are then referenced to one marker as the base.

Usage: ```trainmulti <image_file> <square_size> [<base_id>]```

Accuracy is not the best as each transforms are deduced only from four marker corners. 
Better approaches would be to use multi-view geometry, but I'm just too lazy . . . :D
