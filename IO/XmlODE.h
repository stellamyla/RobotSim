#ifndef XML_ODE_H
#define XML_ODE_H

#include <tinyxml.h>
#include "Simulation/ODESimulator.h"
#include "Simulation/WorldSimulation.h"

class XmlODEGeometry
{
 public:
  XmlODEGeometry(TiXmlElement* element);
  bool Get(ODETriMesh& tri);

  TiXmlElement* e;
};

class XmlODESettings
{
 public:
  XmlODESettings(TiXmlElement* element);
  bool GetSettings(ODESimulator& sim);

  TiXmlElement* e;
};

class XmlSimulationSettings
{
 public:
  XmlSimulationSettings(TiXmlElement* element);
  bool GetSettings(WorldSimulation& sim);

  TiXmlElement* e;
};

#endif
