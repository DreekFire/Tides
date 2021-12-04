-- WIP, for use in ship identification. Currently only contains data for Marauder and Kitakami. Hopefully I figure out some way to automate this.

local enemyData = {}

enemyData.types = {
  STATIC = 0, -- includes both fortresses and structures
  SHIP = 1,
  PLANE = 2,
  THRUSTER = 3,
  SUBMARINE = 4,
  HELICOPTER = 5,
  AIRSHIP = 6,
  SPACECRAFT = 7,
  TANK = 8
}

enemyData.facing = {
  VELOCITY = 0, -- ship generally travels in the same direction it is pointing
  TARGET = 1 -- for frontsiders, which point at the target while moving in different direcitons
}

enemyData = {
  {
    name = "Marauder",
    type = enemyData.types.SHIP,
    facing = enemyData.facing.TARGET,
    stats = {
      BlockPriority = 2781,
      CRAMPriority = 19.72,
      APSPriority = 0,
      PACPriority = 0,
      MissilePriority = 0,
      LaserPriority = 0,
      AIPriority = 1,
      PropulsionPriority = 42,
      TopSpeed = 6
    }
  },
  {
    name = "Kitakami",
    type = enemyData.types.SHIP,
    facing = enemyData.facing.VELOCITY,
    stats = {
      BlockPriority = 4136,
      CRAMPriority = 0,
      APSPriority = 26.15,
      PACPriority = 24.89,
      MissilePriority = 58.62,
      LaserPriority = 0,
      AIPriority = 1,
      PropulsionPriority = 1053,
      TopSpeed = 33
    }
  }
}