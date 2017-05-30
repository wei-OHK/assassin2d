// Copyright (C) 2015-2016 Wei@OHK, Hiroshima University.
// This file is part of the "assassin2d".
// For conditions of distribution and use, see copyright notice in assassin2d.h

#include "actor.h"
#include "scenemgr.h"

namespace assa2d {
Component::Component(Configuration* conf) : bul::dynamics::Actor::Component(conf),
			_Rigid_Attachment(static_cast<assa2d::SceneMgr*>(GetSceneMgr())->GetWorld()) { }

Object::Object(Configuration* conf) : bul::dynamics::Object(conf),
			_Rigid_Attachment(static_cast<assa2d::SceneMgr*>(GetSceneMgr())->GetWorld()) { }

} /* namespace assa2d */

