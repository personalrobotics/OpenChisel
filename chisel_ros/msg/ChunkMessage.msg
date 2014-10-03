Header header

#Chunk ID spatial position
int32 ID_x
int32 ID_y
int32 ID_z

#Chunk spatial hash value
int64 spatial_hash

#Size of a voxel in meters
float32 resolution_meters

#number of voxels in each dimension
int32 num_voxels_x
int32 num_voxels_y
int32 num_voxels_z

#Binary serialization of distances/weights
uint32[] distance_data

#Binary serialization of colors/weights
uint32[] color_data
