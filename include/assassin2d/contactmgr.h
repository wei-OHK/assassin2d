// Copyright (C) 2015-2016 Wei@OHK, Hiroshima University.
// This file is part of the "assassin2d".
// For conditions of distribution and use, see copyright notice in assassin2d.h

#ifndef _ASSA2D_CONTACTMGR_H
#define _ASSA2D_CONTACTMGR_H

#include <stdexcept>
#include <functional>

#include <bulwark/bulwark.h>
#include <Box2D/Box2D.h>

#include "actor.h"

namespace assa2d {
/// Forward-declaration.
class ContactMgr;

/// Read-Only b2ContactListener.
class ContactListener {
public:
	ContactListener() { }
	virtual ~ContactListener() { }

protected:
	/// Called when two fixtures begin to touch.
	virtual void BeginContact(const b2Contact* contact) { }

	/// Called when two fixtures cease to touch.
	virtual void EndContact(const b2Contact* contact) { }

	/// This is called after a contact is updated. This allows you to inspect a
	/// contact before it goes to the solver.
	/// A copy of the old manifold is provided so that you can detect changes.
	/// Note: this is called only for awake bodies.
	/// Note: this is called even when the number of contact points is zero.
	/// Note: this is not called for sensors.
	virtual void PreSolve(const b2Contact* contact, const b2Manifold* oldManifold) { }

	/// This lets you inspect a contact after the solver is finished. This is useful
	/// for inspecting impulses.
	/// Note: the contact manifold does not include time of impact impulses, which can be
	/// arbitrarily large if the sub-step is small. Hence the impulse is provided explicitly
	/// in a separate data structure.
	/// Note: this is only called for contacts that are touching, solid, and awake.
	virtual void PostSolve(const b2Contact* contact, const b2ContactImpulse* impulse) { }

private:
	friend class ContactMgr;
};

/// ContactMgr delivers contacts to corresponding nodes, and also forward contacts to external contact listeners if there are.
class ContactMgr final : public b2ContactListener {
public:
	/// A contact manager configuration holds basic data needed to construct a contact manager.
	struct Configuration {
		bool ExternalContactListenerEnabled = true;
	};

	ContactMgr(Configuration* conf) {
		m_contact_listener_enabled = conf->ExternalContactListenerEnabled;
	}
	virtual ~ContactMgr() { }

	/// Add an external contact listener.
	void AddContactListener(ContactListener* listener) {
		m_contact_listener.insert(listener);
	}

	/// Remove an external contact listener.
	void RemoveContactListener(ContactListener* listener) {
		m_contact_listener.erase(listener);
	}

	/// Return true if external contact listeners are enabled, otherwise false.
	bool IsContactListenerEnabled() const {
		return m_contact_listener_enabled;
	}

	/// Enable / disable external contact listeners.
	void SetContactListenerEnabled(bool flag) {
		m_contact_listener_enabled = flag;
	}

protected:
	/// Forward the contact to the corresponding nodes.
	virtual void BeginContact(b2Contact* contact) override final {
		std::function<void(_Contact_Attachment*, Node*)> function =
				std::bind(_M_BeginContact_Wrapper, std::placeholders::_1, std::placeholders::_2, contact);
		_M_Contact_Solver(contact, function);
		if(IsContactListenerEnabled()) {
			for(auto listener : m_contact_listener) {
				listener -> BeginContact(contact);
			}
		}
	}

	virtual void EndContact(b2Contact* contact) override final {
		std::function<void(_Contact_Attachment*, Node*)> function =
				std::bind(_M_EndContact_Wrapper, std::placeholders::_1, std::placeholders::_2, contact);
		_M_Contact_Solver(contact, function);
		if(IsContactListenerEnabled()) {
			for(auto listener : m_contact_listener) {
				listener -> EndContact(contact);
			}
		}
	}

	virtual void PreSolve(b2Contact* contact, const b2Manifold* oldManifold) override final {
		std::function<void(_Contact_Attachment*, Node*)> function =
				std::bind(_M_PreSolve_Wrapper, std::placeholders::_1, std::placeholders::_2, contact, oldManifold);
		_M_Contact_Solver(contact, function);
		if(IsContactListenerEnabled()) {
			for(auto listener : m_contact_listener) {
				listener -> PreSolve(contact, oldManifold);
			}
		}
	}

	virtual void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) override final {
		std::function<void(_Contact_Attachment*, Node*)> function =
				std::bind(_M_PostSolve_Wrapper, std::placeholders::_1, std::placeholders::_2, contact, impulse);
		_M_Contact_Solver(contact, function);
		if(IsContactListenerEnabled()) {
			for(auto listener : m_contact_listener) {
				listener -> PostSolve(contact, impulse);
			}
		}
	}

	/// Helper method for forwarding.
	void _M_Contact_Solver(b2Contact* contact, std::function<void(_Contact_Attachment*, Node*)>& function) {
		b2Body* bA = contact->GetFixtureA()->GetBody();
		b2Body* bB = contact->GetFixtureB()->GetBody();
		Node* nA = static_cast<Node*>(bA->GetUserData());
		Node* nB = static_cast<Node*>(bB->GetUserData());
		if(nA == nullptr || nB == nullptr) {
			throw std::runtime_error("assa2d::ContactMgr::_M_Contact_Solver() : uncaught b2Body*.");
		}

		_Contact_Attachment* contact_nodeA = CastType(nA);
		if(contact_nodeA) {
			function(contact_nodeA, nB);
		}

		_Contact_Attachment* contact_nodeB = CastType(nB);
		if(contact_nodeB) {
			function(contact_nodeB, nA);
		}
	}

	/// Wrappers as helper methods.
	static void _M_BeginContact_Wrapper(_Contact_Attachment* contact_node, Node* node, b2Contact* contact) {
		contact_node ->BeginContact(node, contact);
	}

	static void _M_EndContact_Wrapper(_Contact_Attachment* contact_node, Node* node, b2Contact* contact) {
		contact_node ->EndContact(node, contact);
	}

	static void _M_PreSolve_Wrapper(_Contact_Attachment* contact_node, Node* node, b2Contact* contact, const b2Manifold* oldManifold) {
		contact_node ->PreSolve(node, contact, oldManifold);
	}

	static void _M_PostSolve_Wrapper(_Contact_Attachment* contact_node, Node* node, b2Contact* contact, const b2ContactImpulse* impulse) {
		contact_node ->PostSolve(node, contact, impulse);
	}

	/// Return a proper pointer.
	static _Contact_Attachment* CastType(Node* node) {
		switch(node->GetType()) {
		case bul::dynamics::Node_Type::Actor_Component: {
				Component* component = static_cast<Component*>(node);
				return static_cast<_Contact_Attachment*>(component);
			}
			break;
		case bul::dynamics::Node_Type::Object: {
				Object* object = static_cast<Object*>(node);
				return static_cast<_Contact_Attachment*>(object);
			}
			break;
		default: {
				return nullptr;
			}
			break;
		}
	}

private:
	bool m_contact_listener_enabled;
	std::set<ContactListener*> m_contact_listener;
};

} /* namespace assa2d */

#endif /* _ASSA2D_CONTACTMGR_H */
