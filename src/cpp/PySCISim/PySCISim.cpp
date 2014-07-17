/*
 * A Python wrapper to SCISim library.
 */

#include "PySCISim.h"

void test(const std::string& xml_fname) {
	int argc = 2;
	char* argv[] = {"PySCISim",
			"/Users/livne/Research/SCISim/assets/3DRigidBodyScene/Balls/BernoullisProblem.xml"};

	main(argc, argv);
};

