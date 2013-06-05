#ifndef XML_WORLD_H
#define XML_WORLD_H

#include <tinyxml/tinyxml.h>
#include "Modeling/World.h"

class XmlRobot
{
 public:
  XmlRobot(TiXmlElement* element);
  bool GetRobot(Robot& robot);
  //void GetController();

  TiXmlElement* e;
};

class XmlRigidObject
{
 public:
  XmlRigidObject(TiXmlElement* element);
  bool GetObject(RigidObject& object);
  //void GetGraspGenerator();

  TiXmlElement* e;
};

class XmlTerrain
{
 public:
  XmlTerrain(TiXmlElement* element);
  bool GetTerrain(Environment& env);

  TiXmlElement* e;
};

class XmlWorld
{
 public:
  XmlWorld();
  bool Load(const string& fn);
  bool Load(TiXmlElement* e);
  bool GetWorld(RobotWorld& world);
  TiXmlElement* GetElement(const string& name);
  TiXmlElement* GetElement(const string& name,int index);
  TiXmlElement* GetRobot(int index) { return GetElement("robot",index); }
  TiXmlElement* GetRigidObject(int index) { return GetElement("rigidObject",index); }
  TiXmlElement* GetTerrain(int index) { return GetElement("terrain",index); }

  TiXmlDocument doc;
  TiXmlElement* elem;
  Vector3 goals[10];
  int goalCount;
};

#endif

