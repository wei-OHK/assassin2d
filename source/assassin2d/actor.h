// Copyright (C) 2015-2016 Wei@OHK, Hiroshima University.
// This file is part of the "assassin2d".
// For conditions of distribution and use, see copyright notice in assassin2d.h

#ifndef _ASSA2D_ACTOR_H
#define _ASSA2D_ACTOR_H

#include <bulwark/bulwark.h>

#include "node.h"

namespace assa2d {
/// Component is the basic unit of an actor (e.g. robot).
class Component : public bul::dynamics::Actor::Component, public _Rigid_Attachment, public _Contact_Attachment {
public:
	/// A component configuration holds basic data needed to construct a component.
	typedef typename bul::dynamics::Actor::Component::Configuration Configuration;

	Component(Configuration* conf);
	virtual ~Component() { }

protected:
	/// Always called even if the component is inactive.
	virtual void Act_Anyway() override { }
};

/// Actor stands for all active elements which take some actions during each time step.
class Actor : public bul::dynamics::Actor {
public:
	/// An actor configuration holds basic data needed to construct an actor.
	typedef typename bul::dynamics::Actor::Configuration Configuration;

	Actor(Configuration* conf) : bul::dynamics::Actor(conf) {
		m_main_component = nullptr;
	}
	virtual ~Actor() { }

	/// Get the main component.
	assa2d::Component* GetMainComponent() {
		return m_main_component;
	}

	const assa2d::Component* GetMainComponent() const {
		return m_main_component;
	}

protected:
	/// Called before all components act.
	virtual void PreAct() override { }

	/// Called after all components act.
	virtual void PostAct() override { }

	/// Set the main component.
	void SetMainComponent(assa2d::Component* component) {
		m_main_component = component;
	}

private:
	template<typename>
	friend class Accessor;

	assa2d::Component* m_main_component;
};

/// Object stands for those elements which do not perform any action during the simulation (e.g. obstacle).
class Object : public bul::dynamics::Object, public _Rigid_Attachment, public _Contact_Attachment {
public:
	/// An object configuration holds basic data needed to construct an object.
	typedef typename bul::dynamics::Object::Configuration Configuration;

	Object(Configuration* conf);
	virtual ~Object() { }
};

} /* namespace assa2d */

#endif /* _ASSA2D_ACTOR_H */
