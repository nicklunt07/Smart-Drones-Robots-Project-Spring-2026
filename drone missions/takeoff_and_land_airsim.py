#!/usr/bin/env python3
import asyncio
from mavsdk import System

async def run():
    print("creating System", flush=True)
    drone = System()                                   # embedded server
    print("connecting", flush=True)
    await drone.connect(system_address="udpin://0.0.0.0:14540") # PX4's mavlink port
    print("waiting for connection state", flush=True)

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!", flush=True)
            break

    print("waiting for global position", flush=True)
    async for health in drone.telemetry.health():
        print(f"  gps_ok={health.is_global_position_ok}, home_ok={health.is_home_position_ok}", flush=True)
        if health.is_global_position_ok and health.is_home_position_ok:
            break

    print("-- Arming", flush=True)
    await drone.action.arm()
    print("-- Taking off", flush=True)
    await drone.action.takeoff()
    await asyncio.sleep(10)
    print("-- Landing", flush=True)
    await drone.action.land()
    await asyncio.sleep(10)

if __name__ == "__main__":
    asyncio.run(run())
