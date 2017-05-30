// Copyright (C) 2015-2016 Wei@OHK, Hiroshima University.
// This file is part of the "assassin2d".
// For conditions of distribution and use, see copyright notice in assassin2d.h

#ifndef _ASSA2D_NODE_H
#define _ASSA2D_NODE_H

#include <stdexcept>

#include <bulwark/bulwark.h>
#include <Box2D/Box2D.h>

#include "types.h"

namespace assa2d {
/// Forward-declaration.
class ContactMgr;

/// Rigid body attributes / methods.
class _Rigid_Attachment {
public:
	_Rigid_Attachment(b2World* world) : m_world(world) {
		m_body = nullptr;
	}
	virtual ~_Rigid_Attachment() { }

	/// Get current position.
	virtual b2Vec2 const& GetPosition() const {
		if(m_body == nullptr) {
			throw std::runtime_error("assa2d::_Rigid_Attachment::GetPosition() : b2Body nullptr.");
		}
		return m_body -> GetPosition();
	}

	/// Get current angle.
	virtual float32 GetAngle() const {
		if(m_body == nullptr) {
			throw std::runtime_error("assa2d::_Rigid_Attachment::GetAngle() : b2Body nullptr.");
		}
		return m_body -> GetAngle();
	}

	/// Get mass.
	virtual float32 GetMass() const {
		if(m_body == nullptr) {
			throw std::runtime_error("assa2d::_Rigid_Attachment::GetMass() : b2Body nullptr.");
		}
		return m_body -> GetMass();
	}

	/// Get the pointer.
	b2World* GetWorld() {
		return m_world;
	}

	const b2World* GetWorld() const {
		return m_world;
	}

	b2Body* GetBody() {
		return m_body;
	}

	const b2Body* GetBody() const {
		return m_body;
	}

protected:
	/// Set the pointer.
	void SetBody(b2Body* body) {
		m_body = body;
	}

private:
	b2World* const m_world;

	b2Body* m_body;
};

/// Contact attributes / methods.
class _Contact_Attachment {
public:
	_Contact_Attachment() { }
	virtual ~_Contact_Attachment() { }

protected:
	/// Called when two objects begin to touch.
	virtual void BeginContact(Node* node, b2Contact* contact) { }

	/// Called when two objects cease to touch.
	virtual void EndContact(Node* node, b2Contact* contact) { }

	/// Called before the contact goes to the solver.
	virtual void PreSolve(Node* node, b2Contact* contact, const b2Manifold* oldManifold) { }

	/// This lets you inspect a contact after the solver is finished.
	virtual void PostSolve(Node* node, b2Contact* contact, const b2ContactImpulse* impulse) { }

private:
	friend class ContactMgr;
};

} /* namespace assa2d */

#endif /* _ASSA2D_NODE_H */
