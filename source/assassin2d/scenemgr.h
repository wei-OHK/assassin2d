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
			_M_timestep(conf->TimeStep), _M_velocity_iterations(conf->VelocityIterations),
			_M_position_iterations(conf->PositionIterations), _M_world(conf->World),
			_M_contact_manager(&conf->ContactManager) {
		if(_M_world == nullptr) {
			throw std::runtime_error("assa2d::SceneMgr::SceneMgr() : b2World nullptr.");
		}

		_M_world -> SetContactListener(&_M_contact_manager);

		b2BodyDef bd;
		bd.type = b2_staticBody;
		bd.position.Set(0.0f, 0.0f);
		_M_ground = _M_world->CreateBody(&bd);
	}

	virtual ~SceneMgr() {
		_M_world -> DestroyBody(_M_ground);
		_M_ground = nullptr;

		_M_world -> SetContactListener(nullptr);
	}

	/// Getters.
	float32 GetTimeStep() const {
		return _M_timestep;
	}

	int32 GetVelocityIterations() const {
		return _M_velocity_iterations;
	}

	int32 GetPositionIterations() const {
		return _M_position_iterations;
	}

	b2World* GetWorld() {
		return _M_world;
	}

	const b2World* GetWorld() const {
		return _M_world;
	}

	ContactMgr & GetContactMgr() const {
		return const_cast<ContactMgr&>(_M_contact_manager);
	}

	b2Body* GetGround() {
		return _M_ground;
	}

	const b2Body* GetGround() const {
		return _M_ground;
	}

protected:
	/// Called before all elements take their actions.
	virtual void PreStep() override { }

	/// Called after all elements take their actions.
	virtual void PostStep() override final {
		_M_world -> Step(_M_timestep, _M_velocity_iterations, _M_position_iterations);
	}

private:
	template<typename>
	friend class Accessor;

	float32 const _M_timestep;
	int32 const _M_velocity_iterations;
	int32 const _M_position_iterations;

	b2World* const _M_world;

	ContactMgr _M_contact_manager;

	b2Body* _M_ground;
};

} /* namespace assa2d */

#endif /* _ASSA2D_SCENEMGR_H */
