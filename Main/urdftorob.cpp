/*
 * urdftorob.cpp
 *
 *  Created on: Oun 07, 2013
 *      Author: jingru
 */
#include <Modeling/Robot.h>
#include <string.h>
#include <utils/stringutils.h>
int main(int argc, char** argv) {
	if (argc < 2 || argc > 3) {
		printf(
				"USAGE: URDFtoRob robot_file.urdf [optional filename for .rob]\n");
		return 0;
	}

	string filename;
	const char* fname;
	if(argc == 2)
		filename.assign(argv[1]);
	else if(argc == 3)
		filename.assign(argv[2]);
	StripExtension(filename);
	filename.append(".rob");
	const char* ext = FileExtension(argv[1]);
	string path = GetFilePath(argv[1]);
	if (0 == strcmp(ext, "urdf")) {
		Robot robot;

		robot.LoadURDF(argv[1]);
		robot.Save(filename.c_str(), path.c_str());
		robot.SaveGeometry(path.c_str(),"tri");
	} else {
		printf("Unknown file extension %s on file %s!\nOnly converts URDF to Rob", ext, argv[1]);
		return 1;
	}
	cout<<"Converted "<<argv[1]<<" to "<< filename<<endl;
	cout << "Done!" << endl;
}
