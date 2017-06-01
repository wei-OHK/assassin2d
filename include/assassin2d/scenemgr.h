// Copyright (C) 2015-2016 Wei@OHK, Hiroshima University.
// This file is part of the "assassin2d".
// For conditions of distribution and use, see copyright notice in assassin2d.h

#ifndef _ASSA2D_SCENEMGR_H
#define _ASSA2D_SCENEMGR_H

#include <stdexcept>

#include <bulwark/bulwark.h>
#include <Box2D/Box2D.h>

#include "contactmgr.h"

namespace assa2d {
/// Scene manager is the main part which runs and controls the simulation.
class SceneMgr : public bul::manager::SceneMgr {
public:
	/// A scene manager configuration holds basic data needed to construct a scene manager.
	struct Configuration : public bul::manager::SceneMgr::Configuration {
		float32 TimeStep = 1.0f / 60.0f;
		int32 VelocityIterations = 8;
		int32 PositionIterations = 3;

		b2World* World = nullptr;

		ContactMgr::Configuration ContactManager;
	};

	SceneMgr(Configuration* conf) : bul::manager::SceneMgr(conf),
			m_timestep(conf->TimeStep), m_velocity_iterations(conf->VelocityIterations),
			m_position_iterations(conf->PositionIterations), m_world(conf->World),
			m_contact_manager(&conf->ContactManager) {
		if(m_world == nullptr) {
			throw std::runtime_error("assa2d::SceneMgr::SceneMgr() : b2World nullptr.");
		}

		m_world -> SetContactListener(&m_contact_manager);

		b2BodyDef bd;
		bd.type = b2_staticBody;
		bd.position.Set(0.0f, 0.0f);
		m_ground = m_world->CreateBody(&bd);
	}

	virtual ~SceneMgr() {
		m_world -> DestroyBody(m_ground);
		m_ground = nullptr;

		m_world -> SetContactListener(nullptr);
	}

	/// Getters.
	float32 GetTimeStep() const {
		return m_timestep;
	}

	int32 GetVelocityIterations() const {
		return m_velocity_iterations;
	}

	int32 GetPositionIterations() const {
		return m_position_iterations;
	}

	b2World* GetWorld() {
		return m_world;
	}

	const b2World* GetWorld() const {
		return m_world;
	}

	ContactMgr & GetContactMgr() const {
		return const_cast<ContactMgr&>(m_contact_manager);
	}

	b2Body* GetGround() {
		return m_ground;
	}

	const b2Body* GetGround() const {
		return m_ground;
	}

protected:
	/// Called before all elements take their actions.
	virtual void PreStep() override { }

	/// Called after all elements take their actions.
	virtual void PostStep() override final {
		m_world -> Step(m_timestep, m_velocity_iterations, m_position_iterations);
	}

private:
	template<typename>
	friend class Accessor;

	float32 const m_timestep;
	int32 const m_velocity_iterations;
	int32 const m_position_iterations;

	b2World* const m_world;

	ContactMgr m_contact_manager;

	b2Body* m_ground;
};

} /* namespace assa2d */

#endif /* _ASSA2D_SCENEMGR_H */
