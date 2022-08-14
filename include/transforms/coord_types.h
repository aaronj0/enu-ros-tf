#ifndef LAZY_THETA_STAR_3D_INCLUDE_LAZY_THETA_STAR_3D_COORD_TYPES_H
#define LAZY_THETA_STAR_3D_INCLUDE_LAZY_THETA_STAR_3D_COORD_TYPES_H

struct coordsM {
	int x, y, z;
};

struct coordsW {
	double x, y, z;
};

struct obstacle_container {
	double height;
	std::vector<std::vector<double>> poly;
};

enum waypt_enum {
	is_fly_through_false = 0,
	is_fly_through_true = 1,
	airdrop_loc = 2,
	cvrg_start = 3,
	cvrg_end = 4,
};

#endif 
