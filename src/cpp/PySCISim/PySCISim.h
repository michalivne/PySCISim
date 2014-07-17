/*
 * A Python wrapper to SCISim library.
 */

#ifndef __PYSCISIM_H__
#define __PYSCISIM_H__

#include "ThreeDRigidBodies/ThreeDRigidBodySim.h"
#include <cstring>

std::string generateOutputConfigurationDataFileName(const std::string& prefix);
void printCompileInfo(std::ostream& output_stream);
bool loadXMLScene(const std::string& xml_file_name);
std::string generateTimeString();
void saveState();
void serializeSystem();
void deserializeSystem(const std::string& file_name);
void exportConfigurationData();
std::string generateOutputConstraintForceDataFileName();
void stepSystem();
void executeSimLoop();
void exitCleanup();
void printUsage(const std::string& executable_name);
bool parseCommandLineOptions(int* argc, char*** argv, bool& help_mode_enabled,
		scalar& end_time_override, unsigned& output_frequency,
		std::string& serialized_file_name);
int main(int argc, char** argv);

void test(const std::string& xml_fname);

#endif // __PYSCISIM_H__
