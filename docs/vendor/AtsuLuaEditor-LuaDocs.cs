namespace AtsuLuaEditor;

public static class LuaDocs
{

public static string[] GetAll()
{
	return new[] {
		BasicsEntries,
		LibrariesEntries,
		AIEntries,
		BlueprintEntries,
		ComponentsEntries,
		FleetEntries,
		FriendliesEntries,
		LoggingEntries,
		MathsEntries,
		MiscEntries,
		MissileGuidanceEntries,
		MissileWarningEntries,
		PropulsionEntries,
		ResourceZonesEntries,
		SelfAwareEntries,
		SpinnersEntries,
		SubConstructsEntries,
		TargetsEntries,
		WeaponsEntries,
		KeybindsEntries,
		GenericEntries,
		VarEntries,
	};
}

private const string BlueprintEntries = @"
. Note: this section is part of FTD's Lua environment and implemented as shown, though not documented in vanilla.
.
$ BlueprintSpawners (read-only)
^ [BlueprintSpawnerInfo[]]: An array containing information about each blueprint spawner. See below.
,
% BlueprintSpawnerInfo
- IsSpawning [bool]: is it currently spawning a blueprint?
- SpawnName [string]: the name of the blueprint to spawn.
- MaxAllowedLifetime [int]: the maximum number of blueprints to spawn during its lifetime.
- MaxAllowedActive [int]: the maximum number of blueprints to allow active at once.
- HoldTime [float]: the time to hold the blueprint (seconds).
- Spawn(): start spawning the blueprint. Each call attempts to start a new spawn.
";

private const string BasicsEntries = @"
# Getting started
. A basic code file needs a function called Update that takes an input called I. It should look like this:  ::::  String_AFunctionCalledUpdate
|function Update(I)
|	-- put your code here
|end
. 'I' is the interface to the game and contains all the function calls you see in this manual.  ::::  String_FunctionCalls
. The code in Update will be executed every single physics step, which happens forty times per second.  ::::  String_CodeInUpdate
. Comments in code are for explaining to yourself and others how your code functions. Two consecutive hyphens (--) create a comment line, so the second line of the format above is ignored by Lua.  ::::  String_CreatesACommentLine
. Here is a simple example of a function that uses one of the interface functions to write 'Hello' to the HUD:  ::::  String_IsASimple
|function Update(I)
|	I:Log ""Hello""
|end
. To learn more about Lua as a language, see
@ https://www.lua.org/pil/contents.html
. or view its reference manual at
@ https://www.lua.org/manual/5.1/manual.html
";

private const string FleetEntries = @"
. The Fleet Awareness API provides scripts basic information about the fleet the craft is in.  ::::  String_ScriptsBasicInformation
.
$ FleetIndex (read-only)
^ [int]: Index of the ship in the fleet, starting from 0.  ::::  Field_FleetIndex_Outputs
,
$ IsFlagship (read-only)
^ [bool]: Is the craft the fleet's flagship?  ::::  Field_IsFlagship_Outputs
,
$ Fleet (read-only)
^ [FleetInfo]: Information about the fleet.  ::::  Field_Fleet_Outputs
,
% FleetInfo
- Name [string]: Name of the fleet.  ::::  Field_FleetInfo_Name
- ID [int]: The unique ID of the fleet.
- Flagship [FriendlyInfo]: Information about the flagship of the fleet.  ::::  Field_FleetInfo_Flagship
- Members [FriendlyInfo[]]: A table of information regarding the fleet's members. See chapter <i>Friendlies</i> for more information. <i>May contain nils!</i>  ::::  Field_FleetInfo_Members
- GetMemberIndex(friendlyInfo): Get the fleet-index [int] of a friendly construct by its FriendlyInfo. Returns -1 if not found.
";

private const string ResourceZonesEntries = @"
. Scripts can use the following fields to get information about known resource zones.  May change in the future to require a detector.  ::::  String_RequireADetector
.
$ ResourceZones (read-only)
^ [ResourceZoneInfo[]]: List of known resource zones.  ::::  Field_ResourceZones_Outputs
,
$ Resources (read-only)
^ [ResourceInfo]: Information about any available resource zone.  ::::  Field_Resources_Outputs
,
% ResourceZoneInfo
- Name [string]: Name of the resource zone.  ::::  Field_ResourceZoneInfo_Name
- Position [Vector3]: Position of the resource zone.  ::::  Field_ResourceZoneInfo_Position
- Radius [float]: Radius of the resource zone.  ::::  Field_ResourceZoneInfo_Radius
- Resources [ResourceInfo]: Available resources of the resource zone.  ::::  Field_ResourceZoneInfo_Resources
- Id [int]: The unique ID of the resource zone.
,
% ResourceInfo
- NaturalTotal [float]: Total resources.  ::::  Field_ResourceInfo_NaturalTotal
- NaturalMax [float]: Max resources.  ::::  Field_ResourceInfo_NaturalMax
";

private const string FriendliesEntries = @"
. The following API will provide you with the states of friendly vehicles.  ::::  String_PositionsOfFriendlyVehiclesIn
.
I:GetFriendlyCount()
> [int]: the number of friendlies spawned into the world.  ::::  Field_GetFriendlyCount_Outputs
,
I:GetFriendlyInfo(index)
< index [int]: 0 is the first construct  ::::  Field_GetFriendlyInfo_Inputs
> [FriendlyInfo]: the FriendlyInfo object  ::::  Field_GetFriendlyInfo_Outputs
? Returns a friendly info object for a friendly vehicle.  ::::  Field_GetFriendlyInfo_Functionality
,
I:GetFriendlyInfoById(id)
< id [int]: the ID you want  ::::  Field_GetFriendlyInfoByID_Inputs
> [FriendlyInfo]: the FriendlyInfo object  ::::  Field_GetFriendlyInfoByID_Outputs
? Returns a friendly info object for an ID  ::::  Field_GetFriendlyInfoByID_Functionality
,
% FriendlyInfo
- Valid [bool]: false if this table doesn't contain valid data. Ignore if so.
- Rotation [Quaternion]: the rotation of the friendly construct.  ::::  Field_FriendlyInfo_Rotation
- ReferencePosition [Vector3]: the position of the construct (in-world, East-Up-North) from which PositiveSize and NegativeSize are referenced.  ::::  Field_FriendlyInfo_ReferencePosition
- PositiveSize [Vector3]: the extent of the construct in the right-up-forwards direction relative to ReferencePostion.  ::::  Field_FriendlyInfo_PositiveSize
- NegativeSize [Vector3]: the extent of the construct in the left-down-back direction relative to ReferencePosition.  ::::  Field_FriendlyInfo_NegativeSize
- CenterOfMass [Vector3]: the center of mass of the construct (in-world, East-Up-North).  ::::  Field_FriendlyInfo_CenterOfMass
- Velocity [Vector3]: the velocity of the construct (in-world, East-Up-North).  ::::  Field_FriendlyInfo_Velocity
- UpVector [Vector3]: The construct's up vector (in-world, East-Up-North).  ::::  Field_FriendlyInfo_UpVector
- RightVector [Vector3]: the construct's up vector (in-world, East-Up-North).  ::::  Field_FriendlyInfo_RightVector
- ForwardVector [Vector3]: the construct's forward vector (in-world, East-Up-North).  ::::  Field_FriendlyInfo_ForwardVector
- HealthFraction [float]: the construct's fraction of health (including subobjects).  ::::  Field_FriendlyInfo_HealthFraction
- SparesFraction [float]: always 1 (obsolete).
- AmmoFraction [float]: the fraction of materials present on craft, relative to its maximum storage.
- FuelFraction [float]: equal to AmmoFraction.
- EnergyFraction [float]: the energy fraction. Returns 1 if no batteries present.  ::::  Field_FriendlyInfo_EnergyFraction
- PowerFraction [float]: the power fraction. Returns 1 if no fuel storage present.  ::::  Field_FriendlyInfo_PowerFraction
- ElectricPowerFraction [float]: the electric power fraction. Returns 1 if no fuel storage present.  ::::  Field_FriendlyInfo_ElectricPowerFraction
- AxisAlignedBoundingBoxMinimum [Vector3]: the minimum extent of the construct (in-world, East-Up-North).  ::::  Field_FriendlyInfo_AxisAlignedBoundingBoxMinimum
- AxisAlignedBoundingBoxMaximum [Vector3]: the maximum extent of the construct (in-world, East-Up-North).  ::::  Field_FriendlyInfo_AxisAlignedBoundingBoxMaximum
- BlueprintName [string]: the name of the blueprint.  ::::  Field_FriendlyInfo_BlueprintName
- Id [int]: the unique ID of the construct.  ::::  Field_FriendlyInfo_ID
- FleetId [int]: the unique ID of the construct's fleet.
- Type [int]: structure = 0, fortress = 1, vehicle = 2.
";

private const string SpinnersEntries = @"
. Spin blocks spinners have their own interface because they use 'SubConstruct' identifiers.  ::::  String_SpinBlocksSpinners
.
I:SetSpinBlockSpeedFactor(subconstruct_uid, speed_factor)
< subconstruct_uid [int]: the persistent identifier of the SubConstruct.
< speed_factor [float]: 0 to 1, the fractional power output.
? Set the speed factor. In continuous mode spinners this allows some blades to spin slower than others, in insta-spin blades this is related to the speed they are spinning at (1 is max speed, 0 is no speed), and in rotation spinners this does nothing.  ::::  Field_SetSpinBlockSpeedFactor_Functionality
,
I:SetSpinBlockPowerDrive(subconstruct_uid, drive)
< subconstruct_uid [int]: the persistent identifier of the subconstruct.
< drive [float]: the relative power use of the spinner (0 to 10).
? Sets the power drive. this allows heliblades to produce more force. Requires engine power. 0 removes engine use. 10 is maximum power use.  ::::  Field_SetSpinBlockPowerDrive_Functionality
,
I:SetSpinBlockRotationAngle(subconstruct_uid, angle)
< subconstruct_uid [int]: the persistent identifier of the subconstruct.
< angle [float]: angle in degrees to turn to.
? Sets the angle of rotation. Changes the spinner into Rotate mode. 'Rotatebackwards' is not available through this interface, but you shouldn't need it.  ::::  Field_SetSpinBlockRotationAngle_Functionality
,
I:SetSpinBlockContinuousSpeed(subconstruct_uid, speed)
< subconstruct_uid [int]: the persistent identifier of the subconstruct.
< speed [float]: speed to rotate at in radians per second, for <i>-30</i> ≤ <i>ω</i> ≤ <i>30</i>.
? Sets the speed of rotation. Changes the spinner into continuous mode. 'ContinuouseReverse' mode is not available through this interface so set the speed negative to facilitate reverse spinning.  ::::  Field_SetSpinBlockContinuousSpeed_Functionality
,
I:SetSpinBlockInstaSpin(subconstruct_uid, value)
< subconstruct_uid [int]: the persistent identifier of the subconstruct.
< value [float]: -1 means spin backwards full speed, whereas +1 means spin forwards full speed
? Spins the blades in a direction and speed determined by value. Will set the spinner into instaspin forwards mode and will affect speed factor variable of the spinner.  ::::  Field_SetSpinBlockInstaSpin_Functionality
. Pistons have their own interface because they use 'SubConstruct' identifiers  ::::  String_UseSubconstructIdentifiers
,
I:GetPistonExtension(subconstruct_uid)
< subconstruct_uid [int]: the persistent identifier of the subconstruct.
> [float]: the extension distance of the piston in meters, or -1 if not found  ::::  Field_GetPistonExtension_Outputs
,
I:GetPistonVelocity(subconstruct_uid)
< subconstruct_uid [int]: the persistent identifier of the SubConstruct.
> [float]: the absolute velocity of the piston in meters per second, or -1 if not found  ::::  Field_GetPistonVelocity_Outputs
,
I:SetPistonExtension(subconstruct_uid, distance)
< subconstruct_uid [int]: the persistent identifier of the SubConstruct.
< distance [float]: the extension distance of the piston (in meters, will be clamped if necessary)
? Set the extension of the piston.  ::::  Field_SetPistonExtension_Functionality
,
I:SetPistonVelocity(subconstruct_uid, velocity)
< subconstruct_uid [int]: the persistent identifier of the SubConstruct.
< velocity [float]: the velocity of the piston in meters per second (between 0.1 and 2)
? Set the velocity of the piston.  ::::  Field_SetPistonVelocity_Functionality
";

private const string SubConstructsEntries = @"
. SubConstructs (turrets and spin blocks) have their own interface dedicated to work with stacked SubConstructs. They all have a unique persistent index which will never be modified in the blueprint, which starts at 1.  ::::  String_AUniquePersistentIndex
.
I:GetAllSubconstructsCount()
> [int]: the total number of SubConstructs on the vehicle, including stacked SubConstructs  ::::  Field_GetAllSubConstructsCount_Outputs
,
I:GetSubConstructIdentifier(index)
< index [int]: 0 is the first SubConstruct  ::::  Field_GetSubconstructIdentifier_Inputs
> [int]: the persistent identifier of the SubConstruct  ::::  Field_GetSubconstructIdentifier_Outputs
? Returns the identifier of the SubConstruct. The indices start at 0 and are in no particular order  ::::  Field_GetSubconstructIdentifier_Functionality
,
I:GetSubconstructsChildrenCount(subconstruct_uid)
< subconstruct_uid [int]: the persistent identifier of the SubConstruct  ::::  Field_GetSubconstructsChildrenCount_Inputs
> [int]: all the number of SubConstructs directly placed on the given SubConstruct  ::::  Field_GetSubconstructsChildrenCount_Outputs
,
I:GetSubConstructChildIdentifier(subconstruct_uid, index)
< subconstruct_uid [int]: the persistent identifier of the parent SubConstruct
< index [int]: 0 is the first child SubConstruct
> [int]: the persistent identifier of the SubConstruct. The indices start at 0 and are in no particular order  ::::  Field_GetSubconstructChildIdentifier_Outputs
,
I:GetParent(subconstruct_uid)
< subconstruct_uid [int]: the persistent identifier of the SubConstruct  ::::  Field_GetParent_Inputs
> [int]: the persistent index of the parent SubConstruct of the given SubConstruct. 0 for the main construct, or -1 if not found  ::::  Field_GetParent_Outputs
,
I:IsTurret(subconstruct_uid)
< subconstruct_uid [int]: the persistent identifier of the SubConstruct  ::::  Field_IsTurret_Inputs
> [bool]: is the given SubConstruct a turret?  ::::  Field_IsTurret_Outputs
,
I:IsSpinBlock(subconstruct_uid)
< subconstruct_uid [int]: the persistent identifier of the SubConstruct  ::::  Field_IsSpinBlock_Inputs
> [bool]: is the given SubConstruct a spin block?  ::::  Field_IsSpinBlock_Outputs
,
I:IsPiston(subconstruct_uid)
< subconstruct_uid [int]: the persistent identifier of the SubConstruct  ::::  Field_IsPiston_Inputs
> [bool]: is the given SubConstruct a piston?  ::::  Field_IsPiston_Outputs
,
I:IsAlive(subconstruct_uid)
< subconstruct_uid [int]: the persistent identifier of the SubConstruct  ::::  Field_IsAlive_Inputs
> [bool]: is the given SubConstruct not yet completely destroyed?  ::::  Field_IsAlive_Outputs
,
I:IsSubConstructOnHull(subconstruct_uid)
< subconstruct_uid [int]: the persistent identifier of the SubConstruct  ::::  Field_IsSubConstructOnHull_Inputs
> [bool]: is the given SubConstruct on the hull?  ::::  Field_IsSubConstructOnHull_Outputs
,
I:GetSubConstructInfo(subconstruct_uid)
< subconstruct_uid [int]: the persistent identifier of the SubConstruct  ::::  Field_GetSubConstructInfo_Inputs
> [BlockInfo]: a BlockInfo object for the SubConstruct active block (the SpinBlock block, the piston or the turret block). May be invalid  ::::  Field_GetSubConstructInfo_Outputs
,
I:GetSubConstructIdleRotation(subconstruct_uid)
< subconstruct_uid [int]: the persistent identifier of the SubConstruct  ::::  Field_GetSubConstructIdleRotation_Inputs
> [Quaternion]: the rotation of the subconstruct relative to its parent as it was first placed  ::::  Field_GetSubConstructIdleRotation_Outputs
";

private const string LibrariesEntries = @"
# Mathf
. The Mathf Library is full of functions available through Unity's online documentation. Call these like Mathf.Min(1, 2). See:  ::::  String_FunctionIsStaticallyCalled
@ https://docs.unity3d.com/6000.0/Documentation/ScriptReference/Mathf.html
. Note that Lua's built-in math library, among others, is also fully functional, through <i>math</i>.
,
# Vector3
. Unity's Vector3 library is fully exposed to you. You can create a new Vector3 with v = Vector3(x, y, z), and call functions such as Vector3.Angle(v1, v2). See:  ::::  String_MoreInformationOnThisLibrary
@ https://docs.unity3d.com/6000.0/Documentation/ScriptReference/Vector3.html
,
# Quaternion
. Unity's Quaternion library is fully exposed to you also. See:  ::::  String_FindOutMore
@ https://docs.unity3d.com/6000.0/Documentation/ScriptReference/Quaternion.html
,
# Builtin
. <i>AtsuLuaEditor</i> specially exposes also the functions <i>assert</i>, <i>collectgarbage</i>, <i>error</i>, <i>getfenv</i>, <i>rawget</i>, <i>rawset</i>, <i>require</i>, <i>select</i>, <i>select</i>, <i>setfenv</i>, <i>xpcall</i>, and the tables <i>debug</i>, <i>package</i>. Use of tables <i>io</i> and <i>os</i> require explicit opt-in through the configuration file. Find out more about these items in the Lua 5.1 reference manual:
@ https://www.lua.org/manual/5.1/index.html#index
";

private const string MathsEntries = @"
. Note: these functions have their equivalents in the Vector3 and Quaternion libraries. See previous chapter <i>Libraries</i>.
.
I:Maths_AngleBetweenVectors(v1x, v1y, v1z, v2x, v2y, v2z)
< [floats]: don't worry about normalisation  ::::  Field_MathsAngleBetweenVectors_Inputs
> [float]: angle between v1 and v2 in degrees  ::::  Field_MathsAngleBetweenVectors_Outputs
? Return the angle between two vectors v1 and v2. Has an overload for accepting two Vector3 structures instead. Functionally equivalent to <i>Vector3.Angle(v1, v2)</i>.  ::::  Field_MathsAngleBetweenVectors_Functionality
,
I:Maths_CreateVector3(x, y, z)
< x, y, z [floats]: the values of x y and z for your Vector3 object  ::::  Field_MathsCreateVector_Inputs
> [Vector3]: your Vector3 is returned  ::::  Field_MathsCreateVector_Outputs
? Returns an 'official' Vector3, which you may need as an input to some functions. Functionally identical to <i>Vector3(x, y, z)</i>.  ::::  Field_MathsCreateVector_Functionality
,
I:Maths_CreateQuaternion(forward, up)
< forwards [Vector3]: the forwards-pointing vector for this rotation
< up [Vector3]: the up-pointing vector for this rotation.
> [Quaternion]: a quaternion structure for which you can use for rotations  ::::  Field_MathsCreateQuaternion_Outputs
? Returns a quaternion which you can use to rotate vectors. Functionally identical to <i>Quaternion.LookRotation(forwards, up)</i>.  ::::  Field_MathsCreateQuaternion_Functionality
,
I:Maths_RotateVector3(v, q)
< v [Vector3]: your vector.
< q [Quaternion]: your rotation quaternion.
> [Vector3]: your rotated vector  ::::  Field_MathsRotateVector3_Outputs
? Rotates a vector <i>v</i> by a rotation <i>q</i>. Functionally equal to <i>q * v</i> (note the order of multiplication).  ::::  Field_MathsRotateVector3_Functionality
,
I:QuaternionToEulerAngles(q)
< q [Quaternion]: the quaternion you want the Euler angles for  ::::  Field_QuaternionToEulerAngles_Inputs
> [Vector3]: the Euler Angles (pitch, yaw, roll)  ::::  Field_QuaternionToEulerAngles_Outputs
? Returns the Euler Angles for your Quaternion. Functionally identical to <i>q.eulerAngles</i>.  ::::  Field_QuaternionToEulerAngles_Functionality
";

private const string MissileGuidanceEntries = @"
. Connect Lua Transceivers to your missile blocks to allow missiles from those missile blocks to be sent Lua Guidance points.
.
I:GetLuaTransceiverCount()
> [int]: the number of LuaTransceivers  ::::  Field_GetLuaTranscieverCount_Outputs
? Return the number of LuaTransceivers. Each transceiver can have a number of missiles which are controllable.  ::::  Field_GetLuaTranscieverCount_Functionality
,
I:GetLuaControlledMissileCount(transceiver_idx)
< transceiver_idx [int]: the index of the LuaTransceiver where 0 is the first one  ::::  Field_GetLuaControlledMissileCount_Inputs
> [int]: the number of missiles associated with that LuaTransceiver  ::::  Field_GetLuaControlledMissileCount_Outputs
? Returns the number of missiles which that luaTransceiver has communications links to.  ::::  Field_GetLuaControlledMissileCount_Functionality
,
I:GetLuaTransceiverInfo(transceiver_idx)
< transceiver_idx [int]: the index of the LuaTransceiver, starting from 0  ::::  Field_GetLuaTransceiverInfo_Inputs
> [BlockInfo]: a BlockInfo object for the LuaTransceiver's Launchpad  ::::  Field_GetLuaTransceiverInfo_Outputs
? Returns a BlockInfo object for the LuaTransceiver's Launchpad. If no Launch pad exists, it'll return it for the LuaTransceiver.  ::::  Field_GetLuaTransceiverInfo_Functionality
. See chapter <i>Components</i> for the BlockInfo structure  ::::  String_ComponentsTab
,
I:GetLuaControlledMissileInfo(transceiver_idx, missile_idx)
< transceiver_idx [int]: 0 is the first one.
< missile_idx [int]: 0 is the first missile.
> [MissileWarningInfo]: Get a MissileWarningInfo object for your missile.  ::::  Field_GetLuaControlledMissileInfo_Outputs
? Returns a MissileWarningInfo structure for your missile. You can tell where it is and how fast it is going from this  ::::  Field_GetLuaControlledMissileInfo_Functionality
. See the chapter <i>Missile Warning</i> for the MissileWarningInfo structure.  ::::  String_MissileWarningTab
,
I:SetLuaControlledMissileAimPoint(transceiver_idx, missile_idx, x, y, z)
< transceiver_idx [int]: as above.
< missile_idx [int]: as above.
< x, y, z [floats]: global coordinates of the aim point.
? Sets the aim point. No guidance modules will help achieve this aim point so do your own predictive guidance. Needs a lua receiver component ON the launcher to work.  ::::  Field_SetLuaControlledMissileAimPoint_Functionality
,
I:DetonateLuaControlledMissile(transceiver_idx, missile_idx)
< transceiver_idx [int]: as above.
< missile_idx [int]: as above.
? Explodes the missile. Needs a lua receiver component ON the launcher to work.  ::::  Field_DetonateLuaControlledMissile_Functionality
,
I:IsLuaControlledMissileAnInterceptor(transceiver_idx, missile_idx)
< transceiver_idx [int]: 0 is the first one.
< missile_idx [int]: 0 is the first one.
> [bool]: true means the missile has an interceptor module, otherwise false is returned. If the missile has no lua receiver, false will be returned.  ::::  Field_IsLuaControlledMissileAnInterceptor_Outputs
? Find out if the missile has an interceptor capability.  ::::  Field_IsLuaControlledMissileAnInterceptor_Functionality
,
I:SetLuaControlledMissileInterceptorTarget(transceiver_idx, missile_idx, target_idx)
< transceiver_idx [int]: 0 is the first one
< missile_idx [int]: 0 is the first one
< target_idx [int]: 0 is the first missile which that mainframe has a warning for
? Set the target of an interceptor missile to be a specific missile for which a warning exists. This is enough to get the interceptor missile to behave normally but if you want to actually guide it yourself use SetLuaControlledMissileInterceptorStandardGuidanceOnOff to turn the guidance off.  ::::  Field_SetLuaControlledMissileInterceptorTarget_Functionality
,
I:SetLuaControlledMissileInterceptorStandardGuidanceOnOff(luaTranceiver, missile_idx, onOff)
< transceiver_idx [int]: 0 is the first one
< missile_idx [int]: 0 is the first one
< onOff [bool]: true will use standard missile guidance to aim at the interceptors target, false will rely on SetLuaControlledMissileAimPoint for aiming coordinates.  ::::  Field_SetLuaControlledMissileInterceptorStandardGuidanceOnOff_Inputs
? Turns standard guidance for the missile on and off. Turn it off if you're going to guide the missile in yourself.  ::::  Field_SetLuaControlledMissileInterceptorStandardGuidanceOnOff_Functionality
.
. The following section may be used to, for example, change the missile thrust mid-flight. You can't change its parts.
. <color=orange>Note: the following section is technically deprecated and should be considered unstable.
,
I:GetMissileInfo(transceiver_idx, missile_idx)
< transceiver_idx [int]: 0 is the first one
< missile_idx [int]: 0 is the first one
> [MissileInfo]: See below.
? Note that this function is different from <i>I:GetLuaControlledMissileInfo()</i>.
,
% MissileInfo
- Parts [MissilePartInfo[]]: The only field. See below.
,
% MissilePartInfo
- Name [string]: The name of the component.
- Registers [float[2]]: Read-only list of the part's settings registers. Only the first value is usable at most, which corresponds with the top setting.
- SendRegister(int index, float value): Send an update to the corresponding register. Index must be 1 or 2.
";

private const string MissileWarningEntries = @"
I:GetNumberOfWarnings()
> [int]: the number of missiles being warned on  ::::  Field_GetNumberOfWarnings_Outputs
? Return the number of missiles the construct has warnings for.  ::::  Field_GetNumberOfWarnings_Functionality
,
I:GetMissileWarning(missile_idx)
< missile_idx [int]: the index of the missile  ::::  Field_GetMissileWarning_Inputs
> [MissileWarningInfo]: information on the missile.  ::::  Field_GetMissileWarning_Outputs
? Request information on a specific missile warning.  ::::  Field_GetMissileWarning_Functionality
,
% MissileWarningInfo
- Valid [bool]: false if this table doesn't contain valid data. Ignore if so.
- Position [Vector3]: the position of the missile.  ::::  Field_MissileWarningInfo_Position
- Velocity [Vector3]: the velocity of the missile in meters per second.  ::::  Field_MissileWarningInfo_Velocity
- Range [float]: the distance from centre of mass of your construct to the missile (meters).  ::::  Field_MissileWarningInfo_Range
- Azimuth [float]: the azimuth angle between your construct's forward direction and the missile (degrees).  ::::  Field_MissileWarningInfo_Azimuth
- Elevation [float]: the elevation angle between your construct's forward direction and the missile (degrees).  ::::  Field_MissileWarningInfo_Elevation
- TimeSinceLaunch [float]: the time since missile launch.  ::::  Field_MissileWarningInfo_TimeSinceLaunch
- Id [int]: the unique ID of the missile.  ::::  Field_MissileWarningInfo_ID
";

private const string WeaponsEntries = @"
I:GetWeaponCount()
> [int]: the number of weapons on the hull- doesn't include weapons on the turrets but does include the turrets themselves.  ::::  Field_GetWeaponCount_Outputs
? Get the number of weapons on the hull. Knowing this number is useful for when you want to call GetWeaponInfo(i) to find out weapon information.  ::::  Field_GetWeaponCount_Functionality
,
I:GetWeaponInfo(weapon_idx)
< weapon_idx [int]: the index of the weapon you want information on. 0 is the first weapon.  ::::  Field_GetWeaponInfo_Inputs
> [WeaponInfo]: information on the weapon.  ::::  Field_GetWeaponInfo_Outputs
? Gets weapon information for a specific weapon. Useful to figure out what sort of weapon is present.  ::::  Field_GetWeaponInfo_Functionality
,
% WeaponInfo
- Valid [bool]: false if this table doesn't contain valid data. Ignore if so.
- LocalPosition [Vector3]: the local position in the vehicle of the weapon. x is right, y is up and z is forwards.  ::::  Field_WeaponInfo_LocalPosition
- GlobalPosition [Vector3]: the global position of the weapon. x is East, y is Up and Z is North.  ::::  Field_WeaponInfo_GlobalPosition
- LocalFirePoint [Vector3]: the local position in the vehicle where the projectile or laser will be created.  ::::  Field_WeaponInfo_LocalFirePoint
- GlobalFirePoint [Vector3]: the global position in the world where the projectile or laser will be created.  ::::  Field_WeaponInfo_GlobalFirePoint
- Speed [float]: the launch velocity in meters per second of the weapon- approximately correct for most weapon types.  ::::  Field_WeaponInfo_Speed
- CurrentDirection [Vector3]: the direction in global coordinate system that the weapon is facing  ::::  Field_WeaponInfo_CurrentDirection
- WeaponType [int]: the type of the weapon. Cannon = 0, missile = 1, laser = 2, harpoon = 3, turret = 4, missilecontrol = 5, fireControlComputer = 6  ::::  Field_WeaponInfo_WeaponType
- WeaponSlot [int]: the weapon slot of the weapon itself. 0 -> 5.  ::::  Field_WeaponInfo_WeaponSlot
- WeaponSlotMask [int]: the weapon slot bit mask. The rightmost bit represents 'ALL' and is always on, and the second bit represents slot 1, etc. (for example 100111 will respond to slots All, 1, 2, and 5)  ::::  Field_WeaponInfo_WeaponSlotMask
- PlayerCurrentlyControllingIt [bool]: true if the player is controlling this weapon at the moment  ::::  Field_WeaponInfo_PlayerCurrentlyControllingIt
,
I:GetWeaponConstraints(weapon_idx)
< weapon_idx [int]: the index of the weapon you want the constraints off. 0 is the first weapon.  ::::  Field_GetWeaponConstraints_Inputs
> [WeaponConsraints]: information on the field-of-fire constraints of the weapon.  ::::  Field_GetWeaponConstraints_Outputs
? Gets field-of-fire constrains information for a specific weapon.  ::::  Field_GetWeaponConstraints_Functionality
,
% WeaponConstraints
- Valid [bool]: false if this table doesn't contain valid data. Ignore if so.
- MinAzimuth [float]: the minimum azimuth angle in degrees.  ::::  Field_WeaponConstraints_MinAzimuth
- MaxAzimuth [float]: the maximum azimuth angle in degrees.  ::::  Field_WeaponConstraints_MaxAzimuth
- MinElevation [float]: the minimum elevation angle in degrees.  ::::  Field_WeaponConstraints_MinElevation
- MaxElevation [float]: the maximum elevation angle in degrees.  ::::  Field_WeaponConstraints_MaxElevation
- FlipAzimuth [bool]: true if the 'Flip azimuth constraints' toggle is selected.  ::::  Field_WeaponConstraints_FlipAzimuth
- InParentConstructSpace [bool]: true if the 'Set the restrictions in the parent construct space' toggle is selected.  ::::  Field_WeaponConstraints_InParentConstructSpace
,
I:GetWeaponBlockInfo(weapon_idx)
< weapon_idx [int]: the index of the weapon you want information on. 0 is the first weapon.  ::::  Field_GetWeaponBlockInfo_Inputs
> [BlockInfo]: the block information of the main component of the weapon. See 'Components' for information on BlockInfo.  ::::  Field_GetWeaponBlockInfo_Outputs
? Gets the block information for a specific weapon.  ::::  Field_GetWeaponBlockInfo_Functionality
,
I:AimWeaponInDirection(weapon_idx, x, y, z, slot)
< weapon_idx [int]: 0 is the first weapon.
< x, y, z [floats]: the world coordinate scheme direction components to point in. They don't need to be normalised.
< slot [int]: 0 for all, otherwise 1 to 5.
> [int]: the number of weapons that can fire in this direction. 0 for none.  ::::  Field_AimWeaponInDirection_Outputs
? Aims a weapon in a specific direction. For a turret this will aim all weapons on the turret as well as the turret itself.  ::::  Field_AimWeaponInDirection_Functionality
,
I:FireWeapon(weapon_idx, slot)
< weapon_idx [int]: 0 is the first weapon.
< slot [int]: 0 will control all weapons. Only goes up to 5.
> [bool]: has any weapon fired? If so, true.  ::::  Field_FireWeapon_Outputs
? Fires a specific weapon. It's important for most weapons that you aim them first as they won't fire if they can't fire in the direction they are aimed.  ::::  Field_FireWeapon_Functionality
,
I:GetWeaponCountOnSubConstruct(subconstruct_uid)
< subconstruct_uid [int]: This identifier never change in the blueprint, use the SubConstructs-related functions to get it.  ::::  Field_GetWeaponCountOnSubConstruct_Inputs
> [int]: the number of weapons on this turret or spinner, not including the turret itself  ::::  Field_GetWeaponCountOnSubConstruct_Outputs
? return the number of weapons on the turret or spinner. If you wanted to control the turret itself then note that it is treated as a hull mounted weapon.  ::::  Field_GetWeaponCountOnSubConstruct_Functionality
,
I:GetWeaponInfoOnSubConstruct(subconstruct_uid, weapon_idx)
< subconstruct_uid [int]: This identifier never change in the blueprint, use the SubConstructs-related functions to get it.
< weapon_idx [int]: the index of the weapon. 0 is the first one.
> [WeaponInfo]: a WeaponInfo object. See above for the definition of this structure. Note that changes to this structure in Lua do not affect the weapon itself.  ::::  Field_GetWeaponInfoOnSubConstruct_Outputs
? Get weapon info of a weapon on a turret or spinner.  ::::  Field_GetWeaponInfoOnSubConstruct_Functionality
,
I:GetWeaponConstraintsOnSubConstruct(subconstruct_uid, weapon_idx)
< subconstruct_uid [int]: This identifier never change in the blueprint, use the SubConstructs-related functions to get it.
< weapon_idx [int]: the index of the weapon. 0 is the first one.
> [WeaponConsraints]: information on the field-of-fire constraints of the weapon.  ::::  Field_GetWeaponConstraintsOnSubConstruct_Outputs
? Gets field-of-fire constrains information for a specific weapon.  ::::  Field_GetWeaponConstraintsOnSubConstruct_Functionality
,
I:GetWeaponBlockInfoOnSubConstruct(subconstruct_uid, weapon_idx)
< subconstruct_uid [int]: This identifier never change in the blueprint, use the SubConstructs-related functions to get it.
< weapon_idx [int]: the index of the weapon. 0 is the first one.
> [BlockInfo]: the block inforamation of the main component of the weapon. See 'Components' for information on BlockInfo.  ::::  Field_GetWeaponBlockInfoOnSubConstruct_Outputs
? Gets the block information for a specific weapon.  ::::  Field_GetWeaponBlockInfoOnSubConstruct_Functionality
,
I:AimWeaponInDirectionOnSubConstruct(subconstruct_uid, weapon_idx, x, y, z, slot)
< subconstruct_uid is the SubConstruct identifier. For the other parameters, see 'AimWeaponInDirection'.
> as per AimWeaponInDirection  ::::  Field_AimWeaponInDirectionOnSubConstruct_Outputs
? Aims a specific weapon on the turret without aiming the turret  ::::  Field_AimWEaponInDirectionOnSubConstruct_Functionality
,
I:FireWeaponOnSubConstruct(subconstruct_uid, weapon_idx, slot)
< subconstruct_uid is the SubConstruct identifier. For the other parameters, see 'FireWeapon'.
> [bool]: has any weapon fired? If so, true.  ::::  Field_FireWeaponOnSubConstruct_Outputs
? Fires a specific weapon. It's important for most weapons that you aim them first as they won't fire if they can't fire in the direction they are aimed.  ::::  Field_FireWeaponOnSubConstruct_Functionality
";

private const string ComponentsEntries = @"
% Component types and their logic.!  ::::  Field_Types
- 0 = balloon deployer  ::::  Field_Types_BalloonDeployer
- 	bool 0  	<i>get     </i>	Get whether the balloon is deployed.  ::::  Field_Types_BalloonDeployer_1
-      		<i>set     </i>	True to deploy the balloon. False to sever the balloon.  ::::  Field_Types_BalloonDeployer_2
- 1 = drive maintainer  ::::  Field_Types_BalloonDeployer_3
- 	int 0   	<i>get/set </i>	Get/set the channel of the drive maintainer (0 = Primary, 1 = Secondary, 2 = Tertiary).  ::::  Field_Types_BalloonDeployer_4
- 	float 0 	<i>get/set </i>	Get/set the drive that the drive maintainer is set to.  ::::  Field_Types_BalloonDeployer_5
- 2 = all pumps (helium and air)  ::::  Field_Types_AllPump
- 	bool 0  	<i>get     </i>	Get whether the pump is on (buoyancy fraction > 0).  ::::  Field_Types_AllPump_1
-      		 <i>set     </i>	False sets buoyancy fraction to 0. True sets buoyancy fraction to 1.  ::::  Field_Types_AllPump_2
- 	float 0 	<i>get/set </i>	Get/set buoyancy fraction.  ::::  Field_Types_AllPump_3
- 	float 1 	<i>get     </i>	Get fraction of volume that is flooded.  ::::  Field_Types_AllPump_4
- 3 = resource gatherer  ::::  Field_Types_ResourceGatherer
- 	bool 0  	<i>get/set </i>	On/off.  ::::  Field_Types_ResourceGatehrer_1
- 4 = oil drill.  ::::  Field_Types_OilDrill
- 	bool 0  	<i>get/set </i>	On/off.  ::::  Field_Types_OilDrill_1
- 5 = ammo processor  ::::  Field_Types_AmmoProcessor
- 	bool 0  	<i>get/set </i>	On/off.  ::::  Field_Types_AmmoProcessor_1
- 6 = fuel refinery.  ::::  Field_Types_FuelRefinery
- 	bool 0  	<i>get/set </i>	On/off.  ::::  Field_Types_FuelRefinery_1
- 	float 0 	<i>get     </i>	Get dangerous gas level.  ::::  Field_Types_FuelRefinery_2
- 	float 1 	<i>get     </i>	Get refining efficiency.  ::::  Field_Types_FuelRefinery_3
- 	float 2 	<i>get     </i>	Get time per batch.  ::::  Field_Types_FuelRefinery_4
- 7 = tractor beams  ::::  Field_Types_TractorBeams
- 	bool 0  	<i>get/set </i>	On/off.  ::::  Field_Types_TractorBeams_1
- 	int 0   	<i>get     </i>	Get the unique id of the selected vehicle. -1 if nothing is selected.  ::::  Field_Types_TractorBeams_2
-        	 	<i>set     </i>	Select the vehicle with the specified unique id.  ::::  Field_Types_TractorBeams_3
- 	float 0 	<i>get/set </i>	Get/set hold distance.  ::::  Field_Types_TractorBeams_4
- 	float 1 	<i>get/set </i>	Get/set hold azimuth.  ::::  Field_Types_TractorBeams_5
- 	float 2 	<i>get/set </i>	Get/set hold elevation.  ::::  Field_Types_TractorBeams_6
- 8 = hydrofoils  ::::  Field_Types_Hydrofoils
- 	float 0 	<i>get/set </i>	Get/set angle.  ::::  Field_Types_Hydrofoils_1
- 9 = propulsion  ::::  Field_Types_Propulsion
- 	float 0 	<i>get/set </i>	Get/set drive fraction.  ::::  Field_Types_Propulsion_1
- 	float 1 	<i>get     </i>	Get propulsion requests to this component so far this frame.  ::::  Field_Types_Propulsion_2
-      	 	<i>set     </i>	Add a propulsion request to this component.  ::::  Field_Types_Propulsion_3
- 	float 2 	<i>get     </i>	Get total propulsion request to this component last frame.  ::::  Field_Types_Propulsion_4
- 	float 3 	<i>get     </i>	Max force of this component.  ::::  Field_Types_Propulsion_5
- 10 = shield projector  ::::  Field_Types_ShieldProjector
- 	bool 0  	<i>get     </i>	Get whether the shield is functioning.  ::::  Field_Types_ShielfProjector_1
- 	int 0   	<i>get/set </i>	Get/set the shield type (0 = off, 1 = reflect, 2 = disrupt, 3 = laser absorb).  ::::  Field_Types_ShieldProjector_2
- 	float 0 	<i>get/set </i>	Get/set strength.  ::::  Field_Types_ShieldProjector_3
- 	float 1 	<i>get/set </i>	Get/set azimuth.  ::::  Field_Types_ShieldProjector_4
- 	float 2 	<i>get/set </i>	Get/set elevation.  ::::  Field_Types_ShieldProjector_5
- 	float 3 	<i>get/set </i>	Get/set range.  ::::  Field_Types_ShieldProjector_6
- 	float 4 	<i>get/set </i>	Get/set width.  ::::  Field_Types_ShieldProjector_7
- 	float 5 	<i>get/set </i>	Get/set height.  ::::  Field_Types_ShieldProjector_8
- 	float 6 	<i>get/set </i>	Get/set alpha.  ::::  Field_Types_ShieldProjector_9
- 	float 7 	<i>get/set </i>	Get/set red.  ::::  Field_Types_ShieldProjector_10
- 	float 8 	<i>get/set </i>	Get/set green.  ::::  Field_Types_ShieldProjector_11
- 	float 9 	<i>get/set </i>	Get/set blue.  ::::  Field_Types_ShieldProjector_12
- 11 = helium pump  ::::  Field_Types_HeliumPump
- 	bool 0  	<i>get     </i>	Get whether the airpump is on (buoyancy fraction > 0).  ::::  Field_Types_HeliumPump_1
-      	 	<i>set     </i>	False sets buoyance fraction to 0. True sets buoyancy fraction to 1.  ::::  Field_Types_HeliumPump_2
- 	float 0 	<i>get/set </i>	Get/set buoyancy fraction.  ::::  Field_Types_HeliumPump_3
- 	float 1 	<i>get     </i>	Get fraction of volume that is flooded.  ::::  Field_Types_HeliumPump_4
- 12 = spotlight  ::::  Field_Types_Spotlight
- 	float 0 	<i>get/set </i>	Get/set cone angle.  ::::  Field_Types_Spotlight_1
- 	float 1 	<i>get/set </i>	Get/set azimuth angle.  ::::  Field_Types_Spotlight_2
- 	float 2 	<i>get/set </i>	Get/set elevation angle.  ::::  Field_Types_Spotlight_3
- 	float 3 	<i>get/set </i>	Get/set intensity.  ::::  Field_Types_Spotlight_4
- 	float 4 	<i>get/set </i>	Get/set range.  ::::  Field_Types_Spotlight_5
- 	float 5 	<i>get/set </i>	Get/set red.  ::::  Field_Types_Spotlight_6
- 	float 6 	<i>get/set </i>	Get/set green.  ::::  Field_Types_Spotlight_7
- 	float 7 	<i>get/set </i>	Get/set blue.  ::::  Field_Types_Spotlight_8
- 13 = advanced cannon laser targetter  ::::  Field_Types_AdvancedCannonLaserTargetter
- 	int 0   	<i>get     </i>	Get the weapon index of the firing piece it is attached to. -1 otherwise.  ::::  Field_Types_AdvancedCannonLaserTargetter_1
- 	float 0 	<i>get/set </i>	Get/set timed fuse time.  ::::  Field_Types_AdvancedCannonLaserTargetter_2
- 	float 1 	<i>get     </i>	Get altitude fuse 'low' altitude.  ::::  Field_Types_AdvancedCannonLaserTargetter_3
-        	 	<i>set     </i>	Set altitude fuse altitude.  ::::  Field_Types_AdvancedCannonLaserTargetter_4
- 	float 2 	<i>get     </i>	Get altitude fuse 'high' altitude.  ::::  Field_Types_AdvancedCannonLaserTargetter_5
- 14 = CRAM cannon laser targetter  ::::  Field_Types_CramCannonLaserTargetter
- 	int 0   	<i>get     </i>	Get the weapon index of the firing piece it is attached to. -1 otherwise.  ::::  Field_Types_CramCannonLaserTargetter_1
- 	float 0 	<i>get/set </i>	Get/set timed fuse time.  ::::  Field_Types_CramCannonLaserTargetter_2
- 	float 1 	<i>get     </i>	Get altitude fuse 'low' altitude.  ::::  Field_Types_CramCannonLaserTargetter_3
-        	 	<i>set     </i>	Set altitude fuse altitude.  ::::  Field_Types_CramCannonLaserTargetter_4
- 	float 2 	<i>get     </i>	Get altitude fuse 'high' altitude.  ::::  Field_Types_CramCannonLaserTargetter_5
- 15 = warp drive  ::::  Field_Types_WarpDrive
- 	bool 0  	<i>get     </i>	Is charging.  ::::  Field_Types_WarpDrive_1
- 	int 0   	<i>set     </i>	0 = start charging, 1 = warp.  ::::  Field_Types_WarpDrive_2
- 	float 0 	<i>get     </i>	Right jump distance.  ::::  Field_Types_WarpDrive_3
- 	float 1 	<i>get     </i>	Up jump distance.  ::::  Field_Types_WarpDrive_4
- 	float 2 	<i>get     </i>	Forwards jump distance.  ::::  Field_Types_WarpDrive_5
- 	float 3 	<i>get     </i>	Right jump factor based on vehicle size and warp rod size.  ::::  Field_Types_WarpDrive_6
- 	float 4 	<i>get     </i>	Up jump factor.  ::::  Field_Types_WarpDrive_7
- 	float 5 	<i>get     </i>	Forwards jump factor.  ::::  Field_Types_WarpDrive_8
- 	float 6 	<i>get     </i>	Total length of attached chargers.  ::::  Field_Types_WarpDrive_9
- 	float 7 	<i>get     </i>	Charge duration.  ::::  Field_Types_WarpDrive_10
- 16 = particle cannon lense  ::::  Field_Types_ParticleCannonLense
- 	int 0   	<i>get     </i>	Get the weapon index of the firing piece it is attached to. -1 otherwise.  ::::  Field_Types_ParticleCannonLense_1
- 17 = steam boiler controller  ::::  Field_Types_SteamBoilerController
- 	float 0 	<i>get/set </i>	Get/set burn rate.  ::::  Field_Types_SteamBoilerController_1
- 	float 1 	<i>get     </i>	Volume of attached boiler.  ::::  Field_Types_SteamBoilerController_2
- 	float 2 	<i>get     </i>	Pressure of attached boiler.  ::::  Field_Types_SteamBoilerController_3
- 18 = fuel engine generator  ::::  Field_Types_FuelEngineGenerator
- 	float 0 	<i>get/set </i>	Get/set max relative rpm.  ::::  Field_Types_FuelEngineGenerator_1
- 	float 1 	<i>get/set </i>	Get/set battery charge drive.  ::::  Field_Types_FuelEngineGenerator_2
- 	float 2 	<i>get     </i>	Current relative rpm.  ::::  Field_Types_FuelEngineGenerator_3
- 	float 3 	<i>get     </i>	Estimate of maximum power.  ::::  Field_Types_FuelEngineGenerator_4
- 	float 4 	<i>get     </i>	Fuel usage per second.  ::::  Field_Types_FuelEngineGenerator_5
- 19 = sail main block  ::::  Field_Types_SailMainBlock
- 	int 0   	<i>get     </i>	Get the sail type (0 = three point sail, 1 = square rigged sail).  ::::  Field_Types_SailMainBlock_1
- 	float 0 	<i>get     </i>	Mast winch setting.  ::::  Field_Types_SailMainBlock_2
-        	 	<i>set     </i>	Winch mast towards this setting.  ::::  Field_Types_SailMainBlock_3
- 	float 1 	<i>get     </i>	Boom winch setting.  ::::  Field_Types_SailMainBlock_4
-        	 	<i>set     </i>	Winch boom towards this setting.  ::::  Field_Types_SailMainBlock_5
- 	float 2 	<i>get     </i>	Mast height.  ::::  Field_Types_SailMainTypes_6
- 	float 3 	<i>get     </i>	Boom length.  ::::  Field_Types_SailMainTypes_7
- 20 = advanced cannon ammo intake  ::::  Field_Types_AdvancedCannonAmmOIntake
- 	int 0   	<i>get     </i>	Get unique id of the associated ammo controller. -1 otherwise.  ::::  Field_Types_AdvancedCannonAmmoIntake_1
-        	 	<i>set     </i>	Select an ammo controller by its unique id.  ::::  Field_Types_AdvancedCannonAmmoIntake_2
- 	int 1   	<i>get     </i>	Get component index of the associated ammo controller. -1 otherwise.  ::::  Field_Types_AdvancedCannonAmmoIntake_3
-        	 	<i>set     </i>	Select an ammo controller by its component index.  ::::  Field_Types_AdvancedCannonAmmoIntake_4
- 	int 2   	<i>get     </i>	Get the weapon index of the firing piece it is attached to. -1 otherwise.  ::::  Field_Types_AdvancedCannonAmmoIntake_5
- 21 = advanced cannon ammo controller  ::::  Field_Types_AdvancedCannonAmmoController
- 	int 0   	<i>get     </i>	Get unique id.  ::::  Field_Types_AdvancedCannonAmmoController_1
- 22 = AI wireless transmitter  ::::  Field_Types_AIWirelessTransmitter
- 	int 0   	<i>get/set </i>	Get/set channel.  ::::  Field_Types_AiWirelessTransmitter_1
- 23 = AI wireless receiver  ::::  Field_Types_AIWirelessReceiver
- 	int 0   	<i>get/set </i>	Get/set channel.  ::::  Field_Types_AIWirelessReceiver_1
- 24 = AI aimpoint selection card  ::::  Field_Types_AIAimpointSelectionCard
- 	int 0   	<i>get/set </i>	Get/set aimpoint selection type (0 = random, 1 = ammo and AI).  ::::  Field_Types_AIAimpointSelectionCard_1
- 	float 0 	<i>get/set </i>	Get/set retarget time.  ::::  Field_Types_AIAimpointSelectionCard_2
- 25 = detection component  ::::  Field_Types_DetectionComponent
- 	bool 0  	<i>get/set </i>	On/off.  ::::  Field_Types_DetectionComponent_1
- 26 = AI mainframe  ::::  Field_Types_AIMainframe
- 	int 0   	<i>get/set </i>	AI movement mode (0 = Off, 1 = Manual, 2 = Automatic, 3 = Fleet).  ::::  Field_Types_AIMainframe_1
- 	int 1   	<i>get/set </i>	AI firing mode (0 = Off, 1 = On).  ::::  Field_Types_AIMainframe_2
- 27 = laser colorer  ::::  Field_Types_LaserColorer
- 	float 0 	<i>get/set </i>	Get/set red.  ::::  Field_Types_LaserColorer_1
- 	float 1 	<i>get/set </i>	Get/set green.  ::::  Field_Types_LaserColorer_2
- 	float 2 	<i>get/set </i>	Get/set blue.  ::::  Field_Types_LaserColorer_3
- 28 = laser missile defence  ::::  Field_Types_LaserMissileDefence
- 	float 0 	<i>get/set </i>	Get/set red.  ::::  Field_Types_LaserMissileDefence_1
- 	float 1 	<i>get/set </i>	Get/set green.  ::::  Field_Types_LaserMissileDefence_2
- 	float 2 	<i>get/set </i>	Get/set blue.  ::::  Field_Types_LaserMissileDefence_3
- 29 = particle cannon  ::::  Field_Types_ParticleCannon
- 	float 0 	<i>get/set </i>	Get/set red.  ::::  Field_Types_ParticleCannon_1
- 	float 1 	<i>get/set </i>	Get/set green.  ::::  Field_Types_ParticleCannon_2
- 	float 2 	<i>get/set </i>	Get/set blue.  ::::  Field_Types_ParticleCannon_3
- 30 = light fitting  ::::  Field_Types_LightFitting
- 	float 0 	<i>get/set </i>	Get/set intensity.  ::::  Field_Types_LightFitting_1
- 	float 1 	<i>get/set </i>	Get/set range.  ::::  Field_Types_LightFitting_2
- 	float 2 	<i>get/set </i>	Get/set red.  ::::  Field_Types_LightFitting_3
- 	float 3 	<i>get/set </i>	Get/set green.  ::::  Field_Types_LightFitting_4
- 	float 4 	<i>get/set </i>	Get/set blue.  ::::  Field_Types_LightFitting_5
- 31 = simple laser  ::::  Field_Types_SimpleLaser
- 	float 0 	<i>get/set </i>	Get/set red.  ::::  Field_Types_SimpleLaser_1
- 	float 1 	<i>get/set </i>	Get/set green.  ::::  Field_Types_SimpleLaser_2
- 	float 2 	<i>get/set </i>	Get/set blue.  ::::  Field_Types_SimpleLaser_3
- 32 = smoke generator  ::::  Field_Types_SmokeGenerator
- 	float 0 	<i>get/set </i>	Get/set particle speed.  ::::  Field_Types_SmokeGenerator_1
- 	float 1 	<i>get/set </i>	Get/set particle size.  ::::  Field_Types_SmokeGenerator_2
- 	float 2 	<i>get/set </i>	Get/set red.  ::::  Field_Types_SmokeGenerator_3
- 	float 3 	<i>get/set </i>	Get/set green.  ::::  Field_Types_SmokeGenerator_4
- 	float 4 	<i>get/set </i>	Get/set blue.  ::::  Field_Types_SmokeGenerator_5
- 33 = hologram projector  ::::  Field_Types_HologramProjector
- 	bool 0  	<i>get/set </i>	On/off.  ::::  Field_Types_HologramProjector_1
- 	bool 1  	<i>get/set </i>	Get/set whether the image is mirrored.  ::::  Field_Types_HologramProjector_2
- 	float 0 	<i>get/set </i>	Get/set width.  ::::  Field_Types_HologramProjector_3
- 	float 1 	<i>get/set </i>	Get/set height.  ::::  Field_Types_HologramProjector_4
- 	float 2 	<i>get/set </i>	Get/set distance.  ::::  Field_Types_HologramProjector_5
- 	float 3 	<i>get/set </i>	Get/set right translation.  ::::  Field_Types_HologramProjector_6
- 	float 4 	<i>get/set </i>	Get/set up translation.  ::::  Field_Types_HologramProjector_7
- 	float 5 	<i>get/set </i>	Get/set azimuth.  ::::  Field_Types_HologramProjector_8
- 	float 6 	<i>get/set </i>	Get/set elevation.  ::::  Field_Types_HologramProjector_9
- 	float 7 	<i>get/set </i>	Get/set rotation.  ::::  Field_Types_HologramProjector_10
- 34 = poster holder  ::::  Field_Types_PosterHolder
- 	float 0 	<i>get/set </i>	Get/set width.  ::::  Field_Types_PosterHolder_1
- 	float 1 	<i>get/set </i>	Get/set height.  ::::  Field_Types_PosterHolder_2
- 35 = electric engine  ::::  Field_Types_ElectricEngine
- 	float 0 	<i>get/set </i>	Get/set power fraction.  ::::  Field_Types_ElectricEngine_1
- 36 = steam boiler controller  ::::  Field_Types_SteamBoilerController_4
- 	float 0 	<i>get/set </i>	Get/set burn rate.  ::::  Field_Types_SteamBoilerController_5
- 	float 1 	<i>get     </i>	Get storage module pressure.  ::::  Field_Types_SteamBoilerController_6
- 37 = air pump  ::::  Field_Types_AirPump
- 	bool 0  	<i>get     </i>	Get whether the airpump is on (buoyancy fraction > 0).  ::::  Field_Types_AirPump_1
-        		<i>set     </i>	False sets buoyance fraction to 0. True sets buoyancy fraction to 1.  ::::  Field_Types_AirPump_2
- 	float 0 	<i>get/set </i>	Get/set buoyancy fraction.  ::::  Field_Types_AirPump_3
- 	float 1 	<i>get     </i>	Get fraction of volume that is flooded.  ::::  Field_Types_AirPump_4
,
% BlockInfo!
- Valid [bool]: false if this table doesn't contain valid data. Ignore if so.
- Position [Vector3]: position (in-world, East-Up-North).  ::::  Field_BlockInfo_Position
- LocalPosition [Vector3]: position (in-construct, right-up-forwards).  ::::  Field_BlockInfo_LocalPosition
- LocalPositionRelativeToCom [Vector3]: local position relative to the center of mass.  ::::  Field_BlockInfo_LocalPositionRelativeToCom
- Forwards [Vector3]: forwards direction (in-world, East-Up-North).  ::::  Field_BlockInfo_Forwards
- LocalForwards [Vector3]: forward direction (in-construct, right-up-forwards).  ::::  Field_BlockInfo_LocalForwards
- Rotation [Quaternion]: the rotation of the block in-world.  ::::  Field_BlockInfo_Rotation
- LocalRotation [Quaternion]: the rotation of the block in the vehicle's (or turret's) coordinate system.  ::::  Field_BlockInfo_LocalRotation
- SubConstructIdentifier [int]: The subconstruct identifier of the subconstruct the block is part of.
- CustomName [string]: The custom name assigned to the block.  ::::  Field_BlockInfo_CustomName
,
# Interface functions and methods!

I:Component_GetCount(type)
< type [int]: the type of component you want the count of.  ::::  Field_ComponentGetCount_Type
> [int]: the number of components of this type.  ::::  Field_ComponentGetCount_Outputs
~? Returns the number of components of this type  ::::  Field_ComponentGetCount_Functionality
,
I:Component_GetLocalPosition(type, block_idx)
< type [int]: the type of component you want the local position of.
< block_idx [int]: the index of the component you want the position of.
> [Vector3i]: a Vector3i is a Vector3 where .x .y and .z are integers.
? Returns the local position in the vehicle of this component.  ::::  Field_ComponentGetLocalPosition_Functionality
,
I:Component_GetBlockInfo(type, block_idx)
< type [int]: the type of component you want information on.
< block_idx [int]: the index of the component you want block info for.
> [BlockInfo]: a BlockInfo structure relating to the component.  ::::  Field_ComponentGetBlockInfo_Outputs
? Returns an extensive BlockInfo object for the component.  ::::  Field_ComponentGetBlockInfo_Functionality
,
I:SetHologramProjectorURL(block_idx, url)
< block_idx [int]: the index of the hologram projector.
< url [string]: the url to set the hologram projector to as a string.
? Sets the url of the specified hologram projector.  ::::  Field_SetHologramProjectorURL_Functionality
,
I:SetPosterHolderURL(block_idx, url)
< block_idx [int]: the index of the poster holder.
< url [string]: the url to set the poster holder to as a string.
? Sets the url of the specified poster holder.  ::::  Field_SetPosterHolderURL_Functionality
,
# Booleans!

I:Component_GetBoolLogic_1(type, block_idx, prop_idx)
< type [int]: the type of component you want boolean logic for.
< block_idx [int]: the index of the component you want boolean logic for.
< prop_idx [int]: the index of the index of the boolean logic you want.
> [bool]: the specified boolean logic for this component. Default: false.  ::::  Field_ComponentGetBoolLogic1_Outputs
? Returns a boolean (true/false) for a component.  ::::  Field_ComponentGetBoolLogic1_Functionality
,
I:Component_GetBoolLogic(type, block_idx)
? Equal to I:Component_GetBoolLogic_1(type, block_idx, 0)
,
I:Component_SetBoolLogic_1(type, block_idx, prop_idx_1, bool1)
I:Component_SetBoolLogic_2(type, block_idx, prop_idx_1, bool1, prop_idx_2, bool2)
I:Component_SetBoolLogic_3(type, block_idx, prop_idx_1, bool1, prop_idx_2, bool2, prop_idx_3, bool3)
< type [int]: the type of component you want to set boolean logic for.
< block_idx [int]: the index of the component you want to set boolean logic for.
< prop_idx* [int]: the index of the boolean logic you want to set.
< bool* [bool]: the true/false you want to set.
? Sets the specified boolean logic for a component. Depending on the type of this component this means different things (or nothing at all).  ::::  Field_ComponentSetBoolLogic1_Functionality
,
I:Component_SetBoolLogic(type, block_idx, bool)
? Equal to I:Component_SetBoolLogic_1(type, block_idx, 0, bool)
,
I:Component_SetBoolLogicAll_1(type, prop_idx_1, bool1)
I:Component_SetBoolLogicAll_2(type, prop_idx_1, bool1, prop_idx_2, bool2)
I:Component_SetBoolLogicAll_3(type, prop_idx_1, bool1, prop_idx_2, bool2, prop_idx_3, bool3)
< type [int]: the type of component you want to set boolean logic for.
< prop_idx* [int]: the index of the boolean logic you want to set.
< bool* [bool]: the true/false you want to set.
? Sets the specified boolean logic for all components of a specific type. Depending on the type of this component this means different things (or nothing at all).  ::::  Field_ComponentSetBoolLogicAll1_Functionality
,
I:Component_SetBoolLogicAll(type, bool)
? Equal to I:Component_SetBoolLogicAll_1(type, 0, bool)
,
# Floats!

I:Component_GetFloatLogic_1(type, block_idx, prop_idx)
< type [int]: the type of component you want float logic for.
< block_idx [int]: the index of the component you want float logic for.
< prop_idx [int]: the index of the index of the float logic you want.
> [float]: the specified float logic for this component. Default: 0.  ::::  Field_ComponentGetFloatLogic1_Outputs
? Returns a floating point value for a component. Depending on the type of this component this means different things (or nothing at all). Default return is 0.  ::::  Field_ComponentGetFloatLogic1_Functionality
,
I:Component_GetFloatLogic(type, block_idx)
? Equal to I:Component_GetFloatLogic_1(type, block_idx, 0)
,
I:Component_SetFloatLogic_1(type, block_idx, prop_idx_1, float1)
I:Component_SetFloatLogic_2(type, block_idx, prop_idx_1, float1, prop_idx_2, float2)
I:Component_SetFloatLogic_3(type, block_idx, prop_idx_1, float1, prop_idx_2, float2, prop_idx_3, float3)
< type [int]: the type of component you want to set float logic for.
< block_idx [int]: the index of the component you want to set float logic for.
< prop_idx* [int]: the index of the float logic you want to set.
< float* [float]: the floating point value you want to set.
? Sets the specified float logic for a component. Depending on the type of this component this means different things (or nothing at all).  ::::  Field_ComponentSetFloatLogic1_Functionality
,
I:Component_SetFloatLogic(type, block_idx, float)
? Equal to I:Component_SetFloatLogic_1(type, block_idx, 0, float)
,
I:Component_SetFloatLogicAll_1(type, prop_idx_1, float1)
I:Component_SetFloatLogicAll_2(type, prop_idx_1, float1, prop_idx_2, float2)
I:Component_SetFloatLogicAll_3(type, prop_idx_1, float1, prop_idx_2, float2, prop_idx_3, float3)
< type [int]: the type of component you want to set float logic for.
< prop_idx* [int]: the index of the float logic you want to set.
< float* [float]: the floating point value you want to set.
? Sets the specified floating point logic for all components of a specific type.  ::::  Field_ComponentSetFloatLogicAll1_Functionality
,
I:Component_SetFloatLogicAll(type, float)
? Equal to I:Component_SetFloatLogicAll_1(type, 0, float)
,
# Integers!

I:Component_GetIntLogic_1(type, block_idx, prop_idx)
< type [int]: the type of component you want int logic for.
< block_idx [int]: the index of the component you want int logic for.
< prop_idx [int]: the index of the index of the int logic you want.  ::::  Field_ComponentGetIntLogic1_Inputs
> [int]: the specified int logic for this component. Default: 0.  ::::  Field_ComponentGetIntLogic1_Outputs
? Returns a integer number for a component.  ::::  Field_ComponentGetIntLogic1_Functionality
,
I:Component_GetIntLogic(type, block_idx)
? Equal to I:Component_GetIntLogic_1(type, block_idx, 0)
,
I:Component_SetIntLogic_1(type, block_idx, prop_idx_1, int1)
I:Component_SetIntLogic_2(type, block_idx, prop_idx_1, int1, prop_idx_2, int2)
I:Component_SetIntLogic_3(type, block_idx, prop_idx_1, int1, prop_idx_2, int2, prop_idx_3, int3)
< type [int]: the type of component you want to set int logic for.
< block_idx [int]: the index of the component you want to set int logic for.
< prop_idx* [int]: the index of the int logic you want to set.
< int* [int]: the integer number you want to set.  ::::  Field_ComponentSetIntLogic1_Inputs
? Sets the specified int logic for a component.  ::::  Field_ComponentSetIntLogic1_Functionality
,
I:Component_SetIntLogic(type, block_idx, int)
? Equal to I:Component_SetIntLogic_1(type, block_idx, 0, int)
,
I:Component_SetIntLogicAll_1(type, prop_idx_1, int1)
I:Component_SetIntLogicAll_2(type, prop_idx_1, int1, prop_idx_2, int2)
I:Component_SetIntLogicAll_3(type, prop_idx_1, int1, prop_idx_2, int2, prop_idx_3, int3)
< type [int]: the type of component you want to set int logic for.
< prop_idx* [int]: the index of the int logic you want to set.
< int* [int]: the integer number you want to set.  ::::  Field_ComponentSetIntLogicAll1_Inputs
? Sets the specified integer logic for all components of a specific type.  ::::  Field_ComponentSetIntLogicAll1_Functionality
,
I:Component_SetIntLogicAll(type, int)
? Equal to I:Component_SetIntLogicAll_1(type, 0, int)
";

private const string LoggingEntries = @"
I:Log(message)
< message [string]: the message you want to write to the log.  ::::  Field_Log_Inputs
? Writes a message to the log. Log is visible when editing the Lua box and appears in the 'Output' panel. The last 100 log messages are maintained.
,
I:ClearLogs()
? Clears your log. Pretty harmless!  ::::  Field_ClearLogs_Functionality
,
I:LogToHud(message)
< message [string]: the message you want to write to the HUD.  ::::  Field_LogToHud_Inputs
? Writes a message to the HUD. HUD messages are visible during normal play and when on the map.  ::::  Field_LogToHud_Functionality
,
I:LogToHud(""draw""..message, args) *
? AtsuLuaEditor adds a soft override to I:LogToHud, accessed through prefixing the first parameter with ""draw"" and special syntax. As Lua is often used for computational and dimensional problems, this override allows you to draw shapes and lines in-world. Useful for debugging 3D scripts.
? Examples below.
?
# Syntax!
p Any message is a collection of function calls. Functions and arguments are seperated by whitespace, in the form of <i>func [arg1 [arg2 [arg3]]]</i>. Functions are divided into two categories: <i>Commands</i>, which define <i>what</i> to draw; and <i>Attributes</i>, which define <i>how</i> to draw what comes after, such as colour. Redefining an Attribute will override its previous value. Default values are in place.
. Any argument may be a literal of the corresponding type (e.g. 10.7, green) or referenced
.  ➤ by <i>$Name</i>, which returns <i>args[""Name""]</i>, or
.  ➤ by <i>^Name</i>, which returns <i>args[""Name""]</i> converted from local to global coordinates where it makes sense.
. Any Vector3 argument may be a single referenced Vector3, or any set of literal or referenced Numbers following the form
.  ➤ <i>(x y z)</i> when using global coordinates, or
.  ➤ <i><x y z></i> when using local coordinates.
. Null vectors may be abbreviated as an empty () or <>. Similar rules exist for quaternions, though these must contain a fourth component, <i>w</i>. () and <> abbreviate for an identity quaternion.
.
# Commands!
. These commands define <i>what</i> to draw. The <i>pos</i> argument, for example, determines where to draw it.
{ sphere -> pos%_vec3_% %| radius%_number_%>|
{ line -> from%_vec3_% %| to%_vec3_%>|
{ arrow -> from%_vec3_% %| to%_vec3_%>|
{ dash -> from%_vec3_% %| to%_vec3_% %| on%_number_% %| off%_number_%>|
{ gimbal -> pos%_vec3_% %| rot%_quat_% %| radius%_number_%>|
{ circle -> pos%_vec3_% %| rot%_quat_% %| radius%_number_%>|
{ pin -> pos%_vec3_% %| rot%_quat_% %| radius%_number_%>|
{ clear

# Attributes!
. These attributes define <i>how</i> to draw the commands that come after. For example, <i>duration 100</i> lets shapes be drawn for 100 ticks (2.5 seconds) and <i>fade</i> makes them fade out over that time.
{ color -> value%_name / code / vec3_%>|
{ width -> width%_number_%>|
{ duration -> ticks>|
{ fade
{ nofade

. Note that <i>color</i> accepts either a Unity color name (""yellow""), an RGB[A] hexadecimal code (""#ffff00"" or ""#ff0"") or a Vector3 (1.0, 1.0, 0.0). Since v2.0.2, colors may be given freely without mentioning `color`.
.
# Examples
. This snippet draws an arrow from the center of mass extending along the craft's current velocity.
|function Update(I)
|	local com = I:GetConstructCenterOfMass()
|	I:LogToHud(""draw arrow $com $to"", {
|		com = com,
|		to  = com + I:GetVelocityVector() })
|end
. This snippet draws an arrow from the craft to the AI's primary target's aimpoint.
|function Update(I)
|	if I:GetNumberOfTargets(0) == 0 then
|		return
|	end
|	I:LogToHud(""draw sphere <> 3 arrow <> $aim"", {
|		aim = I:GetTargetInfo(0, 0).AimPointPosition })
|end
. This snippet draws four identical gimbals of varying size.
|function Update(I)
|	I:LogToHud([[draw
|		gimbal <>   <>   1
|		gimbal $ctr ^loc 2
|		gimbal <>   $glo 5
|		gimbal $ctr $eul 9
|	]], {
|		ctr = I:GetConstructPosition(),
|		loc = Quaternion.identity,
|		glo = I:Maths_CreateQuaternion(
|			I:GetConstructForwardVector(),
|			I:GetConstructUpVector() ),
|		eul = Quaternion.Euler(
|			I:GetConstructPitch(),
|			I:GetConstructYaw(),
|			I:GetConstructRoll()) })
|end
";

private const string MiscEntries = @"
I:GetTerrainAltitudeForPosition(x, y, z)
< x, y, z [floats]: in-world east/west, vertical and north/south positions in meters.
> [float]: the terrain altitude in meters where 0 is sea level.  ::::  Field_GetTerrainAltitudeForPosition_Outputs
? Returns altitude of the terrain at a position in the world. Can be overloaded with a single Vector3 rather than x,y,z components.  ::::  Field_GetTerrainAltitudeForPosition_Functionality
,
I:GetTerrainAltitudeForLocalPosition(x, y, z)
< x, y, z [floats]: right, up and forwards offsets from construct position in meters.
> [float]: the terrain altitude in meters where 0 is sea level.  ::::  Field_GetTerrainAltitudeForLocalPosition_Outputs
? Returns altitude of the terrain at a position relative to the construct. Can be overloaded with a single Vector3 rather than x,y,z components.  ::::  Field_GetTerrainAltitudeForLocalPosition_Functionality
,
I:GetGravityForAltitude(alt)
< alt [float]: altitude (0 is sea level)  ::::  Field_GetGravityForAltitude_Inputs
> [Vector3]: gravity vector  ::::  Field_GetGravityForAltitude_Outputs
? Returns gravity vector for an altitude. gravity.y is the component of interest.  ::::  Field_GetGravityForAltitude_Functionality
,
I:GetTime()
> [float]: the time in seconds with an arbitrary offset (and will thus seldom be 0).  ::::  Field_GetTime_Outputs
,
I:GetTimeSinceSpawn()
> [float]: the time in seconds since the construct spawned.  ::::  Field_GetTimeSinceSpawn_Outputs
,
I:GetGameTime()
> [float]: The time since the instance started in seconds.  ::::  Field_GetGameTime_Outputs
,
I:GetWindDirectionAndMagnitude()
> [Vector3]: the direction and the magnitude of the wind.  ::::  Field_GetWindDirectionAndMagnitude_Outputs
? Get the direction and magnitude of the current wind.  ::::  Field_GetWindDirectionAndMagnitude_Functionality
,
I:Crash()
? Crash the environment. Causes Lua to exit and print out a stacktrace.
";

private const string SelfAwareEntries = @"
I:GetConstructPosition()
> [Vector3]: the position of the construct's origin  ::::  Field_GetConstructPosition_Outputs
,
I:GetConstructForwardVector()
> [Vector3]: the normalized forward vector of the construct  ::::  Field_GetConstructForwardVector_Outputs
,
I:GetConstructRightVector()
> [Vector3]: the normalized right vector of the construct  ::::  Field_GetConstructRightVector_Outputs
,
I:GetConstructUpVector()
> [Vector3]: the normalized up vector of the construct  ::::  Field_GetConstructUpVector_Outputs
,
I:GetConstructMaxDimensions()
> [Vector3]: The size of the vehicle right, up and forwards of its origin, local to the construct  ::::  Field_GetConstructMaxDimensions_Outputs
,
I:GetConstructMinDimensions()
> [Vector3]: The size of the vehicle left, down and back of its origin, local to the construct  ::::  Field_GetConstructMinDimensions_Outputs
,
I:GetConstructRoll()
> [float]: the roll angle in degrees  ::::  Field_GetConstructRoll_Outputs
,
I:GetConstructPitch()
> [float]: the pitch angle in degrees, between -180° and 180°  ::::  Field_GetConstructPitch_Outputs
,
I:GetConstructYaw()
> [float]: the yaw angle in degrees  ::::  Field_GetConstructYaw_Outputs
,
I:GetConstructCenterOfMass()
> [Vector3]: the position of the construct's CoM, in-world  ::::  Field_GetConstructCenterOfMass_Outputs
,
I:GetConstructLocalCenterOfMass()
> [Vector3]: the position of the construct's CoM, local to the construct, typically rounded to increments of 50 cm to make balancing more easily achievable, but this can be changed in the 'V' menu  ::::  Field_GetConstructLocalCenterOfMass_Outputs
,
I:GetAiPosition(mainframe_idx)
< mainframe_idx [int]: 0 is the first mainframe.  ::::  Field_GetAIPosition_Inputs
> [Vector3]: the position of the mainframe, in-world, or a null vector if no such mainframe exists  ::::  Field_GetAIPosition_Outputs
,
I:GetVelocityMagnitude()
> [float]: magnitude of your velocity in meters per second.  ::::  Field_GetVelocityMagnitude_Outputs
,
I:GetForwardsVelocityMagnitude()
> [float]: magnitude of your forwards velocity in meters per second. Negative if you're moving backwards  ::::  Field_GetForwardsVelocityMagnitude_Outputs
,
I:GetVelocityVector()
> [Vector3]: your construct's global velocity vector in meters per second (in-world, East-Up-North)  ::::  Field_GetVelocityVector_Outputs
,
I:GetVelocityVectorNormalized()
> [Vector3]: your construct's normalized velocity vector in meters per second (in-world, East-Up-North)  ::::  Field_GetVelocityVectorNormalized_Outputs
,
I:GetAngularVelocity()
> [Vector3]: your construct's angular velocity in world space  ::::  Field_GetAngularVelocity_Outputs
? Returns your angular velocity. x is speed of turn around the east->west axis, y is around the vertical axis and z is around the north south axis. You're probably going to want the next function instead of this one...  ::::  Field_GetAngularVelocity_Functionality
,
I:GetLocalAngularVelocity()
> [Vector3]: Your construct's angular velocity in local space  ::::  Field_GetLocalAngularVelocity_Outputs
? Returns your angular velocity. x is pitch, y is yaw and z is roll.  ::::  Field_GetLocalAngularVelocity_Functionality
,
I:GetAmmoFraction()
> [float]: fraction of materials present on craft, relative to its maximum storage.
,
I:GetFuelFraction()
> [float]: Equal to I:GetAmmoFraction().
,
I:GetEnergyFraction()
> [float]: fraction of energy this construct has left, between 0 and 1. 1 if no batteries are available  ::::  Field_GetEnergyFraction_Outputs
,
I:GetPowerFraction()
> [float]: fraction of power this construct has left, between 0 to 1. 0 if no power can be generated  ::::  Field_GetPowerFraction_Outputs
,
I:GetHealthFraction()
> [float]: fraction of health this construct, including SubConstructs, has left, between 0 and 1. 1 if full health  ::::  Field_GetHealthFraction_Outputs
,
I:IsDocked()
> [bool]: is this construct docked?  ::::  Field_IsDocked_Outputs
,
I:GetHealthFractionDifference(time)
< time[float]: the time you want the difference measured over. Time will be limited to be between 1 and 30.  ::::  Field_GetHealthFractionDifference_Inputs
> [float]: difference in health points as a fraction (0 to 1) over the given time  ::::  Field_GetHealthFractionDifference_Outputs
,
I:GetBlueprintName()
> [string]: name of this blueprint.  ::::  Field_GetBlueprintName_Outputs
,
I:GetUniqueId()
> [int]: this construct's unique ID. No other construct has it  ::::  Field_GetUniqueID_Outputs
";

private const string PropulsionEntries = @"
I:TellAiThatWeAreTakingControl()
? Will stop the AI from issuing propulsion commands for the next second, after which it will assume control again. This is exactly what happens when the player presses a control key on an AI controlled vehicle.  ::::  Field_TellAIThatWeAreTakingControl_Functionality
.
% Propulsion axis numbers
- Main		= 0
- Secondary	= 1
- Tertiary		= 2
- Roll		= 3
- Pitch		= 4
- Yaw		= 5
- Forwards	= 6
- Up		= 7
- Right		= 8
- A			= 9
- B			= 10
- C			= 11
- D			= 12
- E			= 13
,
I:AddPropulsionRequest(type, value)
< type [int]: see above
< value [float]:</b> the amount to add to the axis  ::::  Field_AddPropulsionRequest_Inputs
? Adds a propulsion request to the specified axis. This is additive to any other requests made to the axis in the same frame and is clamped between -1 and 1  ::::  Field_AddPropulsionRequest_Functionality
,
I:SetPropulsionRequest(type, value)
< type [int]: see above
< value [float]:</b> the amount the axis is set to  ::::  Field_SetPropulsionRequest_Inputs
? Sets the propulsion request to the specified axis. This overwrites any other requests made to the axis in the same frame and is clamped between -1 and 1  ::::  Field_SetPropulsionRequest_Functionality
,
I:GetPropulsionRequest(type)
< type [int]: see above  ::::  Field_GetPropulsionRequest_Inputs
? Gets the sum of all requests made to the specified axis in the previous frame or reads the value that the drive is set to if the type is Main, Secondary or Tertiary  ::::  Field_GetPropulsionRequest_Functionality
,
I:RequestComplexControllerStimulus(stim)
< stim: none = 0, <b>T</b> = 1, <b>G</b> = 2, <b>Y</b> = 3, <b>H</b> = 4, <b>U</b> = 5, <b>J</b> = 6, <b>I</b> = 7, <b>K</b> = 8, <b>O</b> = 9, <b>L</b> = 10, <b>up</b> = 11, <b>down</b> = 12, <b>left</b> = 13, <b>right</b> = 14.  ::::  Field_RequestComplexControllerStimulus_Inputs
? Requests a stimuli as per the complex controller block, ignoring invalid stims.  ::::  Field_RequestComplexControllerStimulus_Functionality
,
I:MoveFortress(direction)
< direction [Vector3]:  direction to move the fortress in. Limited to 1 meter.  ::::  Field_MoveFortress_Inputs
? Move fortress in any direction. Limited to 1 meter.  ::::  Field_MoveFortress_Functionality
,
I:RequestCustomAxis(name, value)
< name [string]: name of axis to create/use. Limited to 32 characters.
< value [float]: value to add to the axis on this frame.  ::::  RequestCustomAxis
? Creates or uses an axis with a custom name. Adds a value to the axis. Axes values are limited to between -1 and 1. Axis names are limited to 32 characters.  ::::  RequestCustomAxis_Functionality
,
I:GetCustomAxis(name)
< name [string]: name of axis to get value for.  ::::  Field_GetCustomAxis_Inputs
> The value of the axis as a float. 0 if axis not created yet.  ::::  Field_GetCustomAxis_Outputs
? Returns the value of the named axis that it had the previous frame, or 0 if axis not created yet.  ::::  Field_RequestCustomAxis_Functionality
.
";

private const string AIEntries = @"
$ Waypoint (read-only)
^ [Vector3]: The waypoint location of the AI mainframe. To write this, you should use a breadboard manoeuvre link, or manage movement manually.
,
$ AIMode (read-only)
^ [string]: ""off"" if the movement of the AI mainframe is disabled, else ""on"" (i.e. automatic, fleet).
,
I:GetAIMovementMode(mainframe_idx)
< mainframe_idx [int]: index of the AI mainframe  ::::  Field_GetAIMovementMode_Inputs
> [string]: movement mode of the AI mainframe. Possible modes: Off, Manual, Automatic, Fleet  ::::  Field_GetAIMovementMode_Outputs
~? returns the movement mode of the AI mainframe specified by the index  ::::  Field_GetAIMovementMode_Functionality
,
I:GetAIFiringMode(mainframe_idx)
< mainframe_idx [int]: index of the AI mainframe  ::::  Field_GetAIFiringMode_Inputs
> [string]: firing mode of the AI mainframe. Either ""on"" or ""off"".  ::::  Field_GetAIFiringMode_Outputs
";

private const string TargetsEntries = @"
I:GetNumberOfMainframes()
> The number of mainframes on your vehicle.  ::::  Field_GetNumberOfMainframes_Outputs
? The mainframe count of your vehicle is useful for requesting targets.  ::::  Field_GetNumberOfMainframes_Functionality
,
I:GetNumberOfTargets(mainframe_idx)
< mainframe_idx [int]: 0 being the first mainframe. Use GetNumberOfMainframes() to find out how many there are.  ::::  Field_GetNumberOfTargets_Inputs
> The number of targets in this particular mainframe. Returns 0 if such a mainframe does not exist.  ::::  Field_GetNumberOfTargets_Outputs
? The target count is important when calling I:GetTargetInfo, below.  ::::  Field_GetNumberOfTargets_Functionality
,
I:GetTargetInfo(mainframe_idx, target_idx)
< mainframe_idx: 0 being the first mainframe. Use GetNumberOfMainframes() to find out how many there are.
< target_idx: 0 being the first target. If target prioritisation card is in use 0 is the highest priority target.  ::::  Field_GetTargetInfo_Inputs
> A TargetInfo object  ::::  Field_GetTargetInfo_Outputs
? The TargetInfo object contains many interesting variables relating to the target. Valid will be false if the target has died but the AI has not yet cleared it.  ::::  Field_GetTargetInfo_Functionality
,
% TargetInfo
- Priority [int]: 0 is highest priority  ::::  Field_TargetInfo_Priority
- Score [float]: high is a good score- taken from target prioritisation card  ::::  Field_TargetInfo_Score
- AimPointPosition [Vector3]: position in game world of aim point (this is the current position of the block that's being aimed for)  ::::  Field_TargetInfo_AimPointPosition
- Team [int]: team of target  ::::  Field_TargetInfo_Team
- Protected [bool]: is it salvage? Will be false for salvage.  ::::  Field_TargetInfo_Protected
- Position [Vector3]: position in game world of target object.  ::::  Field_TargetInfo_Position
- Velocity [Vector3]: velocity in game world in meters per second  ::::  Field_TargetInfo_Velocity
- PlayerTargetChoice [bool]: has the player set this as the target?  ::::  Field_TargetInfo_playerTargetChoice
- Id [int]: the unique integer Id of the target.  ::::  Field_TargetInfo_ID
,
I:GetTargetPositionInfo(mainframe_idx, target_idx)
< mainframe_idx: 0 being the first mainframe. Use GetNumberOfMainframes() to find out how many there are.
< target_idx: 0 being the first target. If target prioritisation card is in use 0 is the highest priority target.  ::::  Field_GetTargetPositionInfo_Inputs
> A TargetPositionInfo object  ::::  Field_GetTargetPositionInfo_Outputs
? The TargetPositionInfo object contains many interesting variables relating to the target. Valid will be false if the target has died but the AI has not yet cleared it.  ::::  Field_GetTargetPositionInfo_Functionality
,
I:GetTargetPositionInfoForPosition(mainframe_idx, x, y, z)
< mainframe_idx [int]: 0 being the first mainframe. Use GetNumberOfMainframes() to find out how many there are.
< x, y, z [floats]: east-west, up-down, north-south in meters.  ::::  Field_GetTargetPositionInfoForPosition_Inputs
> A TargetPositionInfo object (see below) for this point in space. Velocity will be 0.  ::::  Field_GetTargetPositionInfoForPosition_Outputs
? Only useful to defer some calculations.  ::::  Field_GetTargetPositionInfoForPosition_Functionality
,
% TargetPositionInfo
- Valid [bool]: false if this table doesn't contain valid data. Ignore if so.
- Azimuth [float]: degrees off nose of our vehicle where positive is clockwise  ::::  Field_TargetPositionInfo_Azimuth
- Elevation [float]: degrees off nose of our vehicle where positive is downwards. This often has dodgy values  ::::  Field_TargetPositionInfo_Elevation
- ElevationForAltitudeComponentOnly [float]: the elevation off nose of the target's altitude. Robustly calculated  ::::  Field_TargetPositionInfo_ElevationForAltitudeComponentOnly
- Range [float]: the range to the target  ::::  Field_TargetPositionInfo_Range
- Direction [Vector3]: the direction to the target (absolute, not normalised)  ::::  Field_TargetPositionInfo_Direction
- GroundDistance [float]: the distance along the ground (ignoring vertical component) to the target  ::::  Field_TargetPositionInfo_GroundDistance
- AltitudeAboveSeaLevel [float]: in metres.  ::::  Field_TargetPositionInfo_AltitudeAboveSeaLevel
- Position [Vector3]: position of target.  ::::  Field_TargetPositionInfo_Position
- Velocity [Vector3]: meters per second.  ::::  Field_TargetPositionInfo_Velocity
";

private const string KeybindsEntries = @"
. <i>AtsuLuaEditor</i> adds a handful of keyboard shortcuts for your convenience. A key preceded by a caret indicates the control key; thus ^K is ctrl-K.
.
{  <b>Tab</b> -> Indent at the current cursor position.>|
{  <b>^Tab</b> -> Indent the entire line or selected lines. Ctrl-Shift: dedent.>|
{  <b>^F</b> -> Select the search bar.>|
{  <b>^G</b> -> Select gotoline bar.>|
{  <b>^N</b> -> Go to next search term, if any.>|
{  <b>^B</b> -> Execute or abort script.>|
{  <b>^R</b> -> Restart script execution.>|
{  <b>^=</b> -> Increase font size by 1 pt.>|
{  <b>^-</b> -> Decrease font size by 1 pt.>|
{  <b>^E</b> -> Move down the script view by one line. Ctrl-Shift: half a page.>|
{  <b>^Y</b> -> Move up the script view by one line. Ctrl-Shift: half a page.>|
{  <b>^,</b> -> Edit configuration.>|
{
{  <b>Esc</b> -> Unfocus script or exit.>|
{  <b>^L</b> -> Toggle script focus.>|
{  <b>^T</b> -> Focus menu-tools.>|
{  <b>^M</b> -> Focus menu-manual.>|
{  <b>^Up</b> -> Go to previous manual chapter.>|
{  <b>^Down</b> -> Go to next manual chapter.>|
{
{  <b>^S</b> -> Open dialog to save to disk.>|
{  <b>^O</b> -> Open dialog to load from disk.>|
";

private const string GenericEntries = @"
p Note: <i>AtsuLuaEditor</i> allows access to generic block setters and getters in Lua. These functions cannot be used without the mod. You can check for its presence by confirming _ALE ~= nil, which when installed contains a string with the mod's version similar to _VERSION.
p
p The workflow is as such: when prototyping, query a list of possible blocks with I:Gb_Query, then query a list of settable/gettable variables with I:Gb_Query. Then, when initialising, construct and cache a list of targets with I:Gbs, I:Gbg or I:Gbsg. Finally within Update, use set() or get() on the object. Code in this section will error for invalid inputs.
p
~. Examples below.

# Functions!
I:Gb_Query(block, which) *
< block [string]: '?' to get all relevant blocks, or either the GUID or the full name of the block to get its variables as provided by the previous option.
< which [string]: 'set' or 'get' for either settable or gettable variables.
> [string[]]: a list of all usable, relevant names.
? Use this function to either get the names of all settable/gettable blocks, or (when you have a name) their respective variables.
,
I:Gbs(block, filter, set) *
I:Gbg(block, filter, get) *
I:Gbsg(block, filter, set, get) *
< block [string]: the GUID or the full name of the block.
< filter [string | nil]: the custom name (or part thereof) to filter on.
< set [int | string]: nth variable as provided by I:Gb_Query(block, 'set'), starting at 1. If it's a string, it may also be a regular expression (or a literal when prefixed with @), for which the first (partial) match with the variable name will be selected. Will error if not found.
< get [int | string]: As <i>set</i> prior, but for 'get' variables. May also be '&' to copy <i>set</i> in I:Gbsg.
> [GenericTarget]: a cached GBS and/or GBG. Described below. Either three of the functions work, but use the last if you must both read and write.
,
% GenericTarget (readonly)
- Block [string]: the kind of block.
- Guid [string]: the GUID of the block in hexadecimal 8-4-4-4-12.
- Filter [string]: the custom name to filter on.
- VariableName [string]: the name of this variable.
- Type [string]: the type of this value.
- set(string), get(string): see below.
.
! GenericTarget:set(a?, b?, c?, b?) *
< ... [bool | number | Vector3 | Quaternion | string]: value(s) to set, where each nth value is the nth GBS input (up to four).
,
! GenericTarget:get(select) *
< select [string]: see <i>Select</i> below.
> [number | Vector3 | Quaternion | nil]: retrieved value, or nil if there are no alive matches.
,
% Select (enum)
- '1'	the first matching block's value.
- '~'	mean value of all matching blocks.
- '<'	minimum value of all matching blocks.
- '>'	maximum value of all matching blocks.
- '+'	sum value of all matching blocks.
- '#'	the number of alive matching blocks.
,
# Examples!
. This snippet simply prints all gettable blocks and all variables of the first block in a bulleted list.
|function Update(I)
|	local function list(t) return table.concat(t, ',\n - ') end
|	local blocks = I:Gb_Query('?', 'get')
|	local blockname = blocks[1]:gsub('.+ - ', '')
|	local vars = I:Gb_Query(blockname, 'get')
|
|	I:ClearLogs()
|	I:Log('Gettable blocks:\n - '..list(blocks))
|	I:Log('Gettable vars from '..blockname..':\n\n - '..list(vars))
|
|	Update = nil -- run once
|end
. This snippet turns all smoke generators named 'party' into rainbow clouds.
|smoke = nil
|function Update(I)
|	if not smoke then
|		smoke = I:Gbs('Smoke generator', 'party', 'Color')
|	end
|	local t = 4 * I:GetTime()
|	smoke:set(
|		1 + math.sin(t + 2/3 * math.pi),
|		1 + math.sin(t - 2/3 * math.pi),
|		1 + math.sin(t), 1)
|end
. This snippet vectors a pair of forward-facing CJEs, each named 'L' or 'R'.
|regs = {}
|function init(I)
|	local BLOCK = 'Jet controller'
|	regs.pitch_l = I:Gbsg(BLOCK, 'L', 'Pitch angle', '&')
|	regs.pitch_r = I:Gbs(BLOCK, 'R', 'Pitch angle')
|	regs.yaw     = I:Gbsg(BLOCK, '', 'Yaw angle', '&')
|end
|
|function Update(I)
|	I:ClearLogs()
|	if init then init = init(I) end -- execute only once
|
|	local roll  = 15 * I:GetPropulsionRequest(3)
|	local pitch = 15 * I:GetPropulsionRequest(4)
|	local yaw   = 15 * I:GetPropulsionRequest(5)
|
|	regs.pitch_l:set(pitch - roll)
|	regs.pitch_r:set(pitch + roll)
|	regs.yaw:set(-yaw)
|
|	local t = I:GetTime()
|	I:Log(string.format('[%.2f] Yawing at %.2f degrees. (%d)', t, regs.yaw:get'~', regs.yaw:get'#'))
|	I:Log(string.format('[%.2f] Pitching L at %.2f degrees.', t, regs.pitch_l:get'1'))
|	I:Log(tostring(regs.pitch_l))
|end
";

private const string VarEntries = @"
p Note: <i>AtsuLuaEditor</i> allows setting construct- and fleet variables -- those accessible through a breadboard's variable readers and writers, now in Lua too. These functions cannot be used without the mod. You can check for its presence by confirming _ALE ~= nil, which when installed contains a string with the mod's version similar to _VERSION.
p
% Domains (enum)
- 'v' 	Vehicle
- 'm'	Mothership and drones
- 'f'  	Fleet
- 't'  	Team
- 'e'  	Everyone
,
I:Var_Write(domain, name, index, value) *
< domain [string]: see above.
< name [string]: the name of the variable.
< index [int]: the index to write to. Wraps between [0, 1000].
< value [number | string | Vector3 | Quaternion]: the value to write.
,
I:Var_Read(domain, name, index) *
< domain [string]: see above.
< name [string]: the name of the variable.
< index [int]: the index to read from. Wraps between [0, 1000].
> [number | string | Vector3 | Quaternion | nil]: the variable's value, or nil if not found.
,
I:Var_GetCount(domain, name) *
< domain [string]: see above.
< name [string]: the name of the variable array.
> [int]: the number of items in the variable array, or -1 if <i>name</i> does not exist.
,
I:Var_Clear(domain, name) *
< domain [string]: see above.
< name [string]: the name of the variable.
";

}
