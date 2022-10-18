/*
novodex.h - part of PhysX physics engine implementation
Copyright (C) 2022 SNMetamorph

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
*/

#include "PxPhysicsAPI.h"
#include "PxSimulationEventCallback.h"
#include "PxScene.h"
#include "PxActor.h"
#include "NxUserStream.h"
#include "NxErrorStream.h"
#include "PxTriangleMeshDesc.h"
//#include "PxTriangleMeshShapeDesc.h"
//#include "PxForceFieldLinearKernel.h"
//#include "NxBoxForceFieldShapeDesc.h"
//#include "NxForceFieldDesc.h"
//#include "NxDebugRenderable.h"
//#include "NxConvexShapeDesc.h"
#include "PxConvexMeshDesc.h"
//#include "PxBoxShapeDesc.h"
//#include "PxBoxShape.h"
#include "PxMaterial.h"
//#include "PxActorDesc.h"
#include "PxCooking.h"
#include "PxTriangle.h"
//#include "PhysXLoader.h"

#define DENSITY_FACTOR		0.0013f
#define PADDING_FACTOR		0.49f
#define SOLVER_ITERATION_COUNT	16
#define CONVEYOR_SCALE_FACTOR		((1.0f / gpGlobals->frametime) * 0.5f)
	
class CPhysicNovodex : public IPhysicLayer
{
private:
	physx::PxPhysics	*m_pPhysics;	// pointer to the PhysX engine
	physx::PxFoundation	*m_pFoundation;
	physx::PxScene		*m_pScene;	// pointer to world scene
	model_t				*m_pWorldModel;	// pointer to worldmodel

	char		m_szMapName[256];
	BOOL		m_fLoaded;	// collision tree is loaded and actual
	BOOL		m_fDisableWarning;	// some warnings will be swallowed
	BOOL		m_fWorldChanged;	// world is changed refresh the statics in case their scale was changed too
	BOOL		m_fNeedFetchResults;

	physx::PxTriangleMesh	*m_pSceneMesh;
	physx::PxActor		*m_pSceneActor;	// scene with installed shape
	physx::PxBounds3		worldBounds;
	physx::PxMaterial	*m_pDefaultMaterial;
	physx::PxMaterial	*m_pConveyorMaterial;

	char		p_speeds_msg[1024];	// debug message

	ErrorCallback	m_ErrorCallback;
	physx::PxCooking	*m_pCooking;
	physx::PxDefaultAllocator m_Allocator;
	physx::PxPvd		*m_pVisualDebugger;
	//NxUtilLib		*m_pUtils;

	cvar_t		*fps_max;
public:
	void		InitPhysic( void );
	void		FreePhysic( void );
	void		*GetUtilLibrary( void );
	void		Update( float flTime );
	void		EndFrame( void );
	void		RemoveBody( edict_t *pEdict );
	void		*CreateBodyFromEntity( CBaseEntity *pEntity );
	void		*CreateBoxFromEntity( CBaseEntity *pObject );
	void		*CreateKinematicBodyFromEntity( CBaseEntity *pEntity );
	void		*CreateStaticBodyFromEntity( CBaseEntity *pObject );
	void		*CreateVehicle( CBaseEntity *pObject, string_t scriptName = 0 );
	void		*RestoreBody( CBaseEntity *pEntity );
	void		SaveBody( CBaseEntity *pObject );
	bool		Initialized( void ) { return (m_pPhysics != NULL); }
	void		SetOrigin( CBaseEntity *pEntity, const Vector &origin );
	void		SetAngles( CBaseEntity *pEntity, const Vector &angles );
	void		SetVelocity( CBaseEntity *pEntity, const Vector &velocity );
	void		SetAvelocity( CBaseEntity *pEntity, const Vector &velocity );
	void		MoveObject( CBaseEntity *pEntity, const Vector &finalPos );
	void		RotateObject( CBaseEntity *pEntity, const Vector &finalAngle );
	void		AddImpulse( CBaseEntity *pEntity, const Vector &impulse, const Vector &position, float factor );
	void		AddForce( CBaseEntity *pEntity, const Vector &force );
	void		EnableCollision( CBaseEntity *pEntity, int fEnable );
	void		MakeKinematic( CBaseEntity *pEntity, int fEnable );
	void		UpdateVehicle( CBaseEntity *pObject );
	int		FLoadTree( char *szMapName );
	int		CheckBINFile( char *szMapName );
	int		BuildCollisionTree( char *szMapName );
	bool		UpdateEntityPos( CBaseEntity *pEntity );
	void		UpdateEntityAABB( CBaseEntity *pEntity );
	bool		UpdateActorPos( CBaseEntity *pEntity );
	void		SetupWorld( void );	
	void		DebugDraw( void );
	void		DrawPSpeeds( void );
	void		FreeAllBodies( void );

	void		TeleportCharacter( CBaseEntity *pEntity );
	void		TeleportActor( CBaseEntity *pEntity );
	void		MoveCharacter( CBaseEntity *pEntity );
	void		MoveKinematic( CBaseEntity *pEntity );
	void		SweepTest( CBaseEntity *pTouch, const Vector &start, const Vector &mins, const Vector &maxs, const Vector &end, struct trace_s *tr );
	void		SweepEntity( CBaseEntity *pEntity, const Vector &start, const Vector &end, struct gametrace_s *tr );
	bool		IsBodySleeping( CBaseEntity *pEntity );
	void		*GetCookingInterface( void ) { return m_pCooking; }
	void		*GetPhysicInterface( void ) { return m_pPhysics; }
private:
	// misc routines
	int		ConvertEdgeToIndex( model_t *model, int edge );
	physx::PxConvexMesh	*ConvexMeshFromBmodel( entvars_t *pev, int modelindex );
	physx::PxConvexMesh	*ConvexMeshFromStudio( entvars_t *pev, int modelindex );
	physx::PxConvexMesh	*ConvexMeshFromEntity( CBaseEntity *pObject );
	physx::PxTriangleMesh	*TriangleMeshFromBmodel( entvars_t *pev, int modelindex );
	physx::PxTriangleMesh	*TriangleMeshFromStudio( entvars_t *pev, int modelindex );
	physx::PxTriangleMesh	*TriangleMeshFromEntity( CBaseEntity *pObject );
	physx::PxActor		*ActorFromEntity( CBaseEntity *pObject );
	CBaseEntity	*EntityFromActor( physx::PxActor *pObject );
	bool	CheckCollision(physx::PxRigidBody *pActor);
	void	ToggleCollision(physx::PxRigidBody *pActor, bool enabled);

	int		CheckFileTimes( const char *szFile1, const char *szFile2 );
	void		HullNameForModel( const char *model, char *hullfile, size_t size );
	void		MeshNameForModel( const char *model, char *hullfile, size_t size );

	void		StudioCalcBoneQuaterion( mstudiobone_t *pbone, mstudioanim_t *panim, Vector4D &q );
	void		StudioCalcBonePosition( mstudiobone_t *pbone, mstudioanim_t *panim, Vector &pos );

	bool		P_SpeedsMessage( char *out, size_t size );
};
