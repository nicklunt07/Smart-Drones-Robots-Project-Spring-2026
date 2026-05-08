import asyncio
import math
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

# Shared current target — updated by the main task, streamed by the streamer task
current_setpoint = PositionNedYaw(0.0, 0.0, -30.0, 0.0)

async def setpoint_streamer(drone):
    """Continuously send the current target at 10 Hz so PX4 stays happy."""
    while True:
        await drone.offboard.set_position_ned(current_setpoint)
        await asyncio.sleep(0.1)

async def goto_ned(drone, n, e, d, tolerance=1.5):
    """Update target and wait until the drone gets there."""
    global current_setpoint
    current_setpoint = PositionNedYaw(n, e, d, 0.0)  # yaw fixed at 0
    async for pos in drone.telemetry.position_velocity_ned():
        dn = pos.position.north_m - n
        de = pos.position.east_m - e
        dd = pos.position.down_m - d
        dist = math.sqrt(dn*dn + de*de + dd*dd)
        if dist < tolerance:
            return

async def run():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
    print("waiting for global position...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            break

    print("arming, taking off")
    await drone.param.set_param_float("MPC_XY_VEL_MAX", 3.0)  # 3 m/s
    await drone.param.set_param_float("MPC_XY_CRUISE", 3.0)

    await drone.action.arm()
    await drone.action.takeoff()
    await asyncio.sleep(8)

    # Prime offboard with initial setpoint, then start streaming
    await drone.offboard.set_position_ned(current_setpoint)
    streamer = asyncio.create_task(setpoint_streamer(drone))

    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"offboard start failed: {e._result.result}")
        streamer.cancel()
        await drone.action.land()
        return

    print("flying survey")
    waypoints = [
        (  0.0,  0.0, -8.0),
        ( 50.0,  0.0, -8.0),
        ( 50.0, 20.0, -8.0),
        (  0.0, 20.0, -8.0),
        (  0.0, 40.0, -8.0),
        ( 50.0, 40.0, -8.0),
        (  0.0,  0.0, -8.0),
    ]
    for n, e, d in waypoints:
        print(f"  -> ({n}, {e}, {d})")
        await goto_ned(drone, n, e, d)

    await drone.offboard.stop()
    streamer.cancel()
    print("landing")
    await drone.action.land()

if __name__ == "__main__":
    asyncio.run(run())